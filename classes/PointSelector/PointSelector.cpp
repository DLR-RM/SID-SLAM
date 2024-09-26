//
// Created by font_al on 08/10/21.
//

#include "../../include/PointSelector.h"
#include <limits>


bool IDNav::PointSelector::extractPoints(typeFrame& frame_,  unordered_map<size_t,typeFrame>& keyframes_){
    try {
#ifdef COUT_COMPLETE_PIPELINE
        extractPointsProfiler.begin();
#endif

        // Init an image mask if not semantic segmentation was provided
        if (frame_->maskImg.empty())
            frame_->maskImg = Mat(frame_->grayImgDist.rows, frame_->grayImgDist.cols, CV_8UC1, Scalar(10));

#ifdef ACTIVE_FEATURES_TRACKING
        reuse_observations(frame_, keyframes_, frame_->maskImg);
#endif
#ifdef ACTIVE_FEATURES
        vecFt allFeatures_{}, features_{};
        extract_features(features_, allFeatures_, frame_->keypoints, frame_->grayImgG[0], frame_->depthImg,
                         frame_->maskImg);
#endif
#ifdef ACTIVE_FEATURES_TRACKING
#ifdef COUT_COMPLETE_PIPELINE
        featureAssociationProfiler.begin();
#endif
#endif
#ifdef ACTIVE_FEATURES
        frame_->associate_featuresToFrame(features_, allFeatures_, solveOptionsPlaneEstimate, summaryPlaneEstimate);
#endif
#ifdef ACTIVE_FEATURES_TRACKING
#ifdef COUT_COMPLETE_PIPELINE
        featureAssociationProfiler.end();
#endif
#endif

        // Extract and associate high gradient pixels
        vecPt hgp_{};
#ifndef DEACTIVATE_HGP
        extract_hgp(hgp_, frame_->imgGradient, frame_->grayImgDist, frame_->depthImg, frame_->maskImg);
#endif
#ifdef COUT_COMPLETE_PIPELINE
        hgpAssociationProfiler.begin();
#endif
        frame_->associate_hgpToFrame(hgp_, solveOptionsPlaneEstimate, summaryPlaneEstimate);
#ifdef COUT_COMPLETE_PIPELINE
        hgpAssociationProfiler.end();
#endif
        // Select points with information criteria
        selectInformativePoints(frame_);
        trackingInformationBits = 0.0;

        // Project all points in world coordinates
        for (typePt &pt: frame_->hgp0)
            frame_->pose.xynlambda_2_XYZ(pt);
        int ft_id = 0;
        for (typeFt &ft: frame_->features) {
            frame_->pose.xynlambda_2_XYZ(ft);
            ft->id = ft_id;
            ++ft_id;
        }
#ifdef COUT_COMPLETE_PIPELINE
        extractPointsProfiler.end();
#endif
        return true;
    }
    catch (const PointSelector::PointSelectorFail &t) {
        static int numExceptions{0};
        ++numExceptions;
        std::cout << LIGTH_RED_COUT << "    [PointSelector] exception (#"<< numExceptions<<"): "<<t.errorMessage() << RESET_COUT << endl;
        return false;
    }
}

bool IDNav::PointSelector::thereIsDepthKinect(float& depthOutput_,
                                              const int& uRef_, const int&vRef_,
                                              const Mat& depthImg_,
                                              const float& maxDepthKinect_){

    static const float minDepthKinect{0.1};
    static const int kinectSearchPath_size{1};
    static const float incDepth_th{0.1};

    float depth0 = depthImg_.at<DEPHT_VALUE_TYPE>(vRef_, uRef_);
    if((depth0 > minDepthKinect)&&(depth0 < maxDepthKinect_)){
        depthOutput_ = depth0;
    }else return false;

    float depthi{}, incDepth{};
    int upRef{}, vpRef{};
    for (int u_patch{-kinectSearchPath_size}; u_patch <= kinectSearchPath_size; ++u_patch) {
        for (int v_patch{-kinectSearchPath_size}; v_patch <= kinectSearchPath_size; ++v_patch) {
            upRef = uRef_ + u_patch;
            vpRef = vRef_ + v_patch;
            //depthi = depthImg_.at<float>(vpRef, upRef);
            depthi = depthImg_.at<DEPHT_VALUE_TYPE>(vpRef, upRef);
            if((depthi > minDepthKinect)&&(depthi < maxDepthKinect_)&&(incDepth > incDepth_th)){
                incDepth = (depth0-depthi);
                if(incDepth > incDepth_th) {
                    depthOutput_ = depthi;
                }
            }
        }
    }

    return true;
}

// This functions filter the points relative to a keyframe attending to the pose information.
void IDNav::PointSelector::selectInformativePoints(typeFrame& frame_){
#ifdef COUT_COMPLETE_PIPELINE
    informativePointSelectionProfiler.begin();
#endif
    frame = frame_;
    cam = frame_->cam;
    for (typePt &pt: frame_->hgp) {
        cam->uv_2_xyn(pt);
    }
    for (typeFt &ft: frame_->features) {
        cam->uv_2_xyn(ft);
    }

    if(frame_->hgp.size() + frame_->features.size() < 7){
        std::cout << BOLDRED_COUT <<  "[Point Selector](Select Informative Points) : Number of points is smaller than needed ( min points = " << 7 << " )" << RESET_COUT << std::endl;
        frame_->hgpInf = frame_->hgp;
        return;
    }

    if(frame_->hgp.size() + frame_->features.size() < 1.05*numPointsToSelectWithInformation){
        std::cout << BOLDRED_COUT <<  "[Point Selector](Select Informative Points) : Informative Selection skipped because "
                                      "the low number of points ( num points = " << frame_->hgp.size() + frame_->features.size() << " )" << RESET_COUT << std::endl;
        frame_->hgpInf = frame_->hgp;
        return;
    }

    temp_hgp = frame_->hgp;
    temp_features = frame_->features;
    frame_->hgp.clear();
    frame_->features.clear();
    computePoseRefJacobiansMatrix();
    selectInitialPoints();
    for(int indexPt{}; (indexPt < numPointsToSelectWithInformation - 7); ++indexPt) {
        if((temp_hgp.empty())&&(temp_features.empty())) break;
        computeInformationContribution();
        findBestPoint();
        addPoint(bestPtRow);
    }
    frame_->hgpInf = frame_->hgp;

#ifdef COUT_COMPLETE_PIPELINE
    informativePointSelectionProfiler.end();
#endif
}

void IDNav::PointSelector::findBestPoint() {
    scores = (informationContribution/maxEntropyContribution)  + balanced_entropy_similarity*(similarity/maxSimilarityContribution) + observability;
    int emptyPointer{};
    (scores.array()).maxCoeff(&bestPtRow, &emptyPointer);
}

void IDNav::PointSelector::computeInformationContribution() {

    const size_t numHgpAvailable = poseJacobiansMatrix_hgp.rows();
    const size_t numFtAvailable = poseJacobiansMatrix_u.rows();
    const size_t ftStartRow = numHgpAvailable;
    const size_t scoresSize = numHgpAvailable + numFtAvailable;
    informationContribution = matX::Zero(scoresSize,1);

    poseHessianInverse =  poseHessian.inverse();
    informationContribution.block(0,0,numHgpAvailable,1) =
            ((poseJacobiansMatrix_hgp * poseHessianInverse).array() *(poseJacobiansMatrix_hgp.array())).rowwise().sum();
    for(size_t iRow{0}; iRow < numHgpAvailable; ++ iRow){
        informationContribution(iRow) = 0.5*log2(1.0 + informationContribution(iRow));
    }

    for(size_t iRow{0}; iRow < numFtAvailable; ++ iRow){
        informationContribution(iRow + ftStartRow) =
                ((poseJacobiansMatrix_u.row(iRow).transpose()*poseJacobiansMatrix_u.row(iRow) +
                poseJacobiansMatrix_v.row(iRow).transpose()*poseJacobiansMatrix_v.row(iRow)).array()*poseHessianInverse.array()).array().sum();
        informationContribution(iRow + ftStartRow) = 0.5*log2(1.0 + informationContribution(iRow + ftStartRow));
    }
}

void IDNav::PointSelector::computePoseRefJacobiansMatrix() {

    int poseJacobianSize_hgp = (int)temp_hgp.size();
    int poseJacobianSize_ft = (int)temp_features.size();

    poseJacobiansMatrix_hgp = matX::Zero(poseJacobianSize_hgp,6);
    poseJacobiansMatrix_u = matX::Zero(poseJacobianSize_ft,6);
    poseJacobiansMatrix_v = matX::Zero(poseJacobianSize_ft,6);

    { // Compute photometric jacobians
        const CovCalibration* photoCal = cam->get_photoDef2_param();
        vec6 dI_dpose{};
        int row{0};
        for (typePt &pt: temp_hgp) {
            cam->computeJPhotometricRef(dI_dpose, pt);
            //dI_dpose *= pt->G_mean/pt->G_ref;
            //dI_dpose /=  photoCal->imgStd;
            dI_dpose /= pt->G_ref;
            poseJacobiansMatrix_hgp.row(row) = dI_dpose;
            ++row;
        }
    }

    { // Compute geometric jacobians
        const CovCalibration* geoCal = cam->get_geoDef2_param();
        vec6 du_dpose{},dv_dpose{};
        int row{0};
        for (typeFt &ft: temp_features) {
            cam->computeJGeometricRef(du_dpose, dv_dpose, ft);
            //du_dpose /=  geoCal->imgStd;
            //dv_dpose /=  geoCal->imgStd;
            //du_dpose *=  10.0;
            //dv_dpose *=  10.0;
            //if(ft->reUsedFeature){
                //du_dpose *=  100*ft->numObservations;
                //dv_dpose *=  100*ft->numObservations;
            //}
            poseJacobiansMatrix_u.row(row) = du_dpose;
            poseJacobiansMatrix_v.row(row) = dv_dpose;
            ++row;
        }
    }
}

void IDNav::PointSelector::selectInitialPoints(){
    int emptyPointer{};
    size_t similaritySize = temp_hgp.size() + temp_features.size();
    similarity = (640*640 + 480*480)*matX::Ones(similaritySize,1);
    observability  = matX::Zero(similaritySize,1);
    matX observability_aux  = matX::Zero(similaritySize,1);
    maxSimilarityContribution = 1.0;
    maxEntropyContribution = 1.0;

    int indexFt = temp_hgp.size();
    int maxValue = 1;
    for(typeFt& ft: temp_features) {
        observability(indexFt) = ft->numObservations;
        observability_aux(indexFt) = 0.5;
        ++indexFt;
        if(ft->numObservations > maxValue){
            maxValue =  ft->numObservations;
        }
    }
    observability = observability/maxValue + observability_aux;

    poseHessian =10000.0*Eigen::Matrix<dataType, 6, 6>::Identity();
    for(int indexPt{}; (indexPt < 1); ++indexPt) {
        if((temp_hgp.empty())&&(temp_features.empty())) break;
        computeInformationContribution();
        (informationContribution.array()).maxCoeff(&bestPtRow, &emptyPointer);
        addPoint(bestPtRow);
    }
    for(int indexPt{}; (indexPt < 5); ++indexPt) {
        if((temp_hgp.empty())&&(temp_features.empty())) break;
        computeInformationContribution();
        findBestPoint();
        addPoint(bestPtRow);
    }
    poseHessian -=10000.0*Eigen::Matrix<dataType, 6, 6>::Identity();

    for(int indexPt{}; (indexPt < 1); ++indexPt) {
        if((temp_hgp.empty())&&(temp_features.empty())) break;
        computeInformationContribution();
        (informationContribution.array()).maxCoeff(&bestPtRow, &emptyPointer);
        maxEntropyContribution = informationContribution(bestPtRow);
        findBestPoint();
        addPoint(bestPtRow);
    }

}

void IDNav::PointSelector::addPoint(const int& row){
    size_t ftStartRow = temp_hgp.size();
    if(row < ftStartRow){ // Point is hgp
        row6 Jpose = poseJacobiansMatrix_hgp.row(row);
        poseHessian += Jpose.transpose()*Jpose;
        frame->hgp.push_back(temp_hgp[row]);
        updateSimilarity(temp_hgp[row]);
        removePoint(row);
    }else{ // Point is feature
        int row0 = row - ftStartRow;
        row6 Jpose = poseJacobiansMatrix_u.row(row0);
        poseHessian += Jpose.transpose()*Jpose;
        Jpose = poseJacobiansMatrix_v.row(row0);
        poseHessian += Jpose.transpose()*Jpose;
        frame->features.push_back(temp_features[row0]);
        updateSimilarity(temp_features[row0]);
        removePoint(row);
    }
}

void IDNav::PointSelector::removePoint(const int& row){
    size_t numHgp = temp_hgp.size();
    size_t ftStartRow = numHgp;
    if(row < ftStartRow){ // Point is hgp
        int numRowsJacobianMatrixForTracking = (int) poseJacobiansMatrix_hgp.rows()-1;
        poseJacobiansMatrix_hgp.block(row,0,numRowsJacobianMatrixForTracking-row,6) = poseJacobiansMatrix_hgp.block(row+1,0,numRowsJacobianMatrixForTracking-row,6);
        poseJacobiansMatrix_hgp.conservativeResize(numRowsJacobianMatrixForTracking,colsEigen);
        temp_hgp.erase(temp_hgp.begin() + row);
    }else{
        int row0 = row - ftStartRow;
        int numRowsJacobianMatrixForTracking = (int) poseJacobiansMatrix_u.rows()-1;
        poseJacobiansMatrix_u.block(row0,0,numRowsJacobianMatrixForTracking-row0,6) = poseJacobiansMatrix_u.block(row0+1,0,numRowsJacobianMatrixForTracking-row0,6);
        poseJacobiansMatrix_u.conservativeResize(numRowsJacobianMatrixForTracking,colsEigen);
        poseJacobiansMatrix_v.block(row0,0,numRowsJacobianMatrixForTracking-row0,6) = poseJacobiansMatrix_v.block(row0+1,0,numRowsJacobianMatrixForTracking-row0,6);
        poseJacobiansMatrix_v.conservativeResize(numRowsJacobianMatrixForTracking,colsEigen);
        temp_features.erase(temp_features.begin() + row0);
    }
    int numRowsSimilarity = (int) similarity.rows()-1;
    similarity.block(row,0,numRowsSimilarity-row,1) = similarity.block(row+1,0,numRowsSimilarity-row,1);
    similarity.conservativeResize(numRowsSimilarity,colsEigen);
    observability.block(row,0,numRowsSimilarity-row,1) = observability.block(row+1,0,numRowsSimilarity-row,1);
    observability.conservativeResize(numRowsSimilarity,colsEigen);
}

void IDNav::PointSelector::updateSimilarity(const typeIdNavPoint pt_) {
    double distance{};
    int indexPt{0};
    maxSimilarityContribution = 0.0;
    for(typePt& ptTemp: temp_hgp){
        distance = pow(ptTemp->uRef - pt_->uRef,2) + pow(ptTemp->vRef - pt_->vRef,2);
        if(distance < similarity(indexPt))
            similarity(indexPt) = distance;
        if(similarity(indexPt) > maxSimilarityContribution)
            maxSimilarityContribution = similarity(indexPt);
        ++indexPt;
    }
    for(typeFt& ftTemp: temp_features){
        distance = pow(ftTemp->uRef - pt_->uRef,2) + pow(ftTemp->vRef - pt_->vRef,2);
        if(distance < similarity(indexPt))
            similarity(indexPt) = distance;
        if(similarity(indexPt) > maxSimilarityContribution)
            maxSimilarityContribution = similarity(indexPt);
        ++indexPt;
    }
}
