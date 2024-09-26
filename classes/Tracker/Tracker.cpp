//
// Created by font_al on 1/31/20.
//

#include "../../include/Tracker/Tracker.h"
#include "../../utils/informationFunctions.h"

void IDNav::Tracker::trackFrame(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_,
                            const Pose& pose0_ , const vec6& speed_, const mat6& speedCovInv_){
    try {

        trackFrameProfiler.begin();
        setInitialState(trackedFrame_, keyframes_ , refKeyframe_, pose0_, speed_, speedCovInv_);
        tracking_ceres(1e-3, 3);

        setRefineState();
        for(int it{0}; it < 2; ++it){
            correctCovariancesFromResiduals()->eraseOutliers();
            refine_tracking_ceres(1e-2,1000);
        }
        correctCovariancesFromResiduals()->eraseOutliers();

        checkTrackingSucceed();

        finishOptimization();

        trackingSucceed = true;
        trackFrameProfiler.end();
        //*(covCorr_Log) << std::setprecision(16) << onlinePhotoCorr << " " << onlineReprojCorr << "\n";

        //*(covCorr_Log) << std::setprecision(16) << onlinePhotoCorr << "\n";

        return;
    }
    catch (const Tracker::TrackerFail &t) {
        static int numExceptions{0};
        ++numExceptions;
        std::cout << LIGTH_RED_COUT << "    [Tracker] exception (#"<< numExceptions<<"): "<<t.errorMessage() << RESET_COUT << endl;
        cout << LIGTH_RED_COUT << "        hgp inliers = " << numHgpInliers_refKey << "/" << numHgpAvailable_refKey << " = " << float(numHgpInliers_refKey)/float(numHgpAvailable_refKey)<< RESET_COUT << endl;
#ifdef ACTIVE_FEATURES
        cout << LIGTH_RED_COUT << "        ft inliers = " << numFtInliers_refKey << "/" <<  numFtAvailable_refKey << " = " << float(numFtInliers_refKey)/float(numFtAvailable_refKey)<< RESET_COUT << endl;
#endif
        cout << LIGTH_RED_COUT << "        point inliers = " << numHgpInliers_refKey +  numFtInliers_refKey << "/" <<  numHgpAvailable_refKey + numFtAvailable_refKey << " = " <<
             float(numHgpInliers_refKey + numFtInliers_refKey )/float(numHgpAvailable_refKey + numFtAvailable_refKey) << " < " << minInliersRatio<< RESET_COUT << endl;
        cout << LIGTH_RED_COUT << "        photo error mean = " << abs(photoErrorMean) << " < " << photoErrorMeanMax << RESET_COUT << endl;

        finishOptimization();
        resetOptimization();
        trackFrameProfiler.end();
    }
}

// This function initializes all variables for the optimization process
void IDNav::Tracker::setInitialState(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_,
                                    const Pose& pose0_ ,
                                    const vec6& speed_, const mat6& speedCovInv_ ){

    // Set Tracking members
    trackedFrame = trackedFrame_;
    refKeyframe = refKeyframe_;
    keyframes = keyframes_;

    // Cluster visible points respect their reference keyframe
    refKeyframe->hgp = refKeyframe->hgp0;

    hgpForTracking.clear();
    hgpForTrackingMap.clear();
    featuresForTracking.clear();
    featuresForTrackingMap.clear();

    unordered_map<size_t,typeFrame> keyframes_aux{};
    keyframes_aux.insert({refKeyframe->keyId,refKeyframe});
    clusterPoints( keyframes_aux);

    // Init point covariances
    initProjectionCovariances();
    ceresTracker->set_tstudent2(tstudent2_p_init, numDofPhoto, numDofReproj);

    // Save input state in case reset is needed
    saveInputState();

    // Set initial pose estimation
    trackedFrame->setPose(pose0_);

    // Set kinematic prior
    speedPrior = speed_;
    covInvPrior = speedCovInv_;

    // Set tracking state
    trackingSucceed = false;
}

void IDNav::Tracker::clusterPoints(unordered_map<size_t,typeFrame>& keyframes_){
    clustered_hgpForTracking.clear();
    clustered_featuresForTracking.clear();
    numHgpAvailable = 0;
    numFtAvailable = 0;
    vecPt hgp_aux{};
    vecFt features_aux{};
    for(auto& [keyId,keyframe]: keyframes_){
        keyframe->getVisiblePointsIn(trackedFrame, hgp_aux, features_aux);

        for(typePt& pt: hgp_aux){
            clustered_hgpForTracking[keyframe].push_back(pt);
        }
        numHgpAvailable += hgp_aux.size();

        for(typeFt& ft: features_aux){
            clustered_featuresForTracking[keyframe].push_back(ft);
        }
        numFtAvailable += features_aux.size();
    }
}

void IDNav::Tracker::initProjectionCovariances(){
    dataType photoStd_ = trackedFrame->cam->get_imgPhotoStd();
    dataType geoInf_ = 1.0/trackedFrame->cam->get_imgGeoStd();
    dataType geoInf;
    for(auto&[keyframe, hgp]: clustered_hgpForTracking) {
        for(typePt& pt: hgp){
            pt->photoStd =  photoStd_;
            pt->photoStd0 =  pt->photoStd;
        }
    }
    for(auto&[keyframe, features]: clustered_featuresForTracking){
        for(typeFt& ft: features){
            geoInf = geoInf_*ft->minSize*2.0/(ft->keyPtRef.size + ft->matchedKeypt->size);
            ft->geoInf =  geoInf * mat2::Identity();
            ft->geoInf0 = ft->geoInf;
        }
    }
}

void IDNav::Tracker::saveInputState(){
    poseInput.copyPoseFrom(trackedFrame->pose);
    phScalar_input = trackedFrame->phScalar;
    phBias_input = trackedFrame->phBias;
}

void IDNav::Tracker::setRefineState(){
    if((emergencySettings)||(keyframes.size() < 3)){
        refKeyframe->hgp = refKeyframe->hgp0;
    }

    clusterPoints(keyframes);
    estimateCovariancesWithModel();
    ceresTracker->set_tstudent2(tstudent2_p_refine,numDofPhoto,numDofReproj);
}

void IDNav::Tracker::estimateCovariancesWithModel(){
    const dataType fx{trackedFrame->cam->getFocalLengthX()},fy{trackedFrame->cam->getFocalLengthY()};
    const CovCalibration* photoCal = trackedFrame->cam->get_photoDef2_param();
    const CovCalibration* geoCal = trackedFrame->cam->get_geoDef2_param();
    mat3 R{};

    for (auto &[keyframe,hgp] : clustered_hgpForTracking) {
        R = trackedFrame->pose.Rcw * keyframe->pose.Rwc;
        for(typePt& pt: hgp) {
            pt->photoStd = pt->estimatePhotoStd(fx,fy,R,photoCal);
            pt->photoStd0 = pt->photoStd;
        }
    }

    for (auto &[keyframe,features] : clustered_featuresForTracking) {
        R = trackedFrame->pose.Rcw * keyframe->pose.Rwc;
        for(typeFt& ft: features) {
            ft->geoInf = ft->estimateGeoInf(fx,fy,R,geoCal,*ft->matchedKeypt);
            ft->geoInf0 = ft->geoInf;
        }
    }
}
IDNav::Tracker* IDNav::Tracker::correctCovariancesFromResiduals(){
    dataType photoStdCorr{1.0},reprojStdCorr{1.0};
    estimateCovariancesFromResiduals(photoStdCorr, reprojStdCorr);

    for (auto &[keyframe, hgp]: clustered_hgpForTracking) {
        for (typePt &pt: hgp) {
            pt->photoStd = photoStdCorr * pt->photoStd0;
        }
    }

    for (auto &[keyframe,features] : clustered_featuresForTracking ) {
        for(typeFt& ft: features){
            ft->geoInf = ft->geoInf0/reprojStdCorr;
        }
    }
    return this;
}

void IDNav::Tracker::estimateCovariancesFromResiduals(dataType& photoStdOnline_, dataType& reprojStdOnline_) {

    photoStdOnline_ = 1.0;
    reprojStdOnline_ = 1.0;

    {
        vector<dataType> pixelPhotoErrors{};
        vector<vecX> patchPhotoErrors{};
        vecX e = vecX::Zero(PATCH_SIZE);
        vecPt visibleHgp{};
        for (typePt &pt: clustered_hgpForTracking[refKeyframe]) {
            trackedFrame->XYZ_2_uv(pt);
            if (trackedFrame->cam->isPointIn(pt)) {
                visibleHgp.push_back(pt);
                trackedFrame->extract_I_and_G_BilinInt(pt);
                trackedFrame->get_PhotometricError(e, pt, 0);
                patchPhotoErrors.push_back(e);
                for(int iPatch{0}; iPatch < pt->ptSize; ++iPatch){
                    pixelPhotoErrors.push_back(e(iPatch));
                }
            }
        }
        photoErrorMean = vectorMean(pixelPhotoErrors);

        double chi2Photo;
        int indexPt{0};
        vector<dataType> chi2Errors{};
        for(typePt& pt: visibleHgp){
            chi2Photo = (patchPhotoErrors[indexPt].array() - photoErrorMean).square().sum()/ pow(pt->photoStd0, 2);
            chi2Errors.push_back(chi2Photo);
            ++indexPt;
        }

        if (chi2Errors.size() > 40){
            std::sort (chi2Errors.begin(), chi2Errors.end());
            dataType testChi2 =  chi2Errors[int(tstudent2_p_refine*chi2Errors.size()/100.0)];
            dataType test50 =  chi2Errors[int(50.0*chi2Errors.size()/100.0)];
            dataType testChi2_est = tstudent2FromMedian(numDofPhoto,tstudent2_p_refine,test50) ;
            dataType correction_1 = sqrt(testChi2/tstudent2(numDofPhoto,tstudent2_p_refine));
            dataType correction_2 = sqrt(testChi2_est/tstudent2(numDofPhoto,tstudent2_p_refine));

            if(correction_2 > correction_1){
                photoStdOnline_ = correction_1;
            }else{
                photoStdOnline_ = correction_2;
            }
        }
        else photoStdOnline_ = 1.0;
        onlinePhotoCorr = photoStdOnline_;
    }

    {
        vector<dataType> chi2Errors{};
        vec2 e_geo_i = vec2::Zero();
        dataType chi2Reproj{};
        for (typeFt &ft: clustered_featuresForTracking[refKeyframe]) {
            trackedFrame->XYZ_2_uv(ft);
            e_geo_i(0) = ft->matchedKeypt->pt.x - ft->u[0];
            e_geo_i(1) = ft->matchedKeypt->pt.y - ft->v[0];
            chi2Reproj = e_geo_i.dot(ft->geoInf0 * ft->geoInf0 * e_geo_i);
            chi2Errors.push_back(chi2Reproj);
        }

        if (chi2Errors.size() > 40){
            std::sort (chi2Errors.begin(), chi2Errors.end());
            dataType testChi2 =  chi2Errors[int(tstudent2_p_refine*chi2Errors.size()/100.0)];
            dataType test50 =  chi2Errors[int(50.0*chi2Errors.size()/100.0)];
            dataType testChi2_est = tstudent2FromMedian(numDofReproj,tstudent2_p_refine,test50) ;
            dataType correction_1 = sqrt(testChi2/tstudent2(numDofReproj,tstudent2_p_refine));
            dataType correction_2 = sqrt(testChi2_est/tstudent2(numDofReproj,tstudent2_p_refine));

            if(correction_2 > correction_1){
                reprojStdOnline_ = correction_1;
            }else{
                reprojStdOnline_ = correction_2;
            }
        }
        else reprojStdOnline_ = 1.0;
        onlineReprojCorr = reprojStdOnline_;
    }
}

void IDNav::Tracker::eraseOutliers() {

    { // Erase photometric outliers
        hgpForTracking.clear();
        hgpForTrackingMap.clear();
        numHgpAvailable = 0;
        numHgpAvailable_refKey = 0;
        numHgpInliers_refKey = 0;

        dataType chi2Photo;
        vecX e = vecX::Zero(PATCH_SIZE);
        dataType photoThreshold = tstudent2(numDofPhoto,tstudent2_p_refine);
        for(auto& [keyframe,hgp]: clustered_hgpForTracking){
            for(typePt& pt: hgp){
                trackedFrame->XYZ_2_uv(pt);
                if (trackedFrame->cam->isPointIn(pt)) {
                    ++numHgpAvailable;
                    numHgpAvailable_refKey += (pt->idKey == refKeyframe->keyId);
                    trackedFrame->extract_I_and_G_BilinInt(pt);
                    trackedFrame->get_PhotometricError(e, pt, 0);

                    chi2Photo = (e.array()-photoErrorMean).square().sum() / pow(pt->photoStd, 2);

                    //*(covCorr_Log) << std::setprecision(16) << (e.array()-photoErrorMean).square().sum()/trackedFrame->cam->get_imgPhotoCov() << " " << chi2Photo << "\n";

                    if (chi2Photo < photoThreshold) {
                        hgpForTracking.push_back(pt);
                        hgpForTrackingMap.insert({pt, true});
                        numHgpInliers_refKey += (pt->idKey == refKeyframe->keyId);
                    }
                }
            }
        }
    }

    { // Erase reprojection outliers
        featuresForTracking.clear();
        featuresForTrackingMap.clear();
        numFtAvailable = 0;
        numFtAvailable_refKey = 0;
        numFtInliers_refKey = 0;
        vec2 e_geo_i = vec2::Zero();
        dataType reprojChi2{};
        dataType geothreshold = tstudent2(numDofReproj,tstudent2_p_refine);
        for (auto &[keyframe,features] : clustered_featuresForTracking ) {
            for(typeFt& ft: features){
                trackedFrame->XYZ_2_uv(ft);
                if(trackedFrame->cam->isPointIn(ft)){
                    ++numFtAvailable;
                    numFtAvailable_refKey += (keyframe->keyId == refKeyframe->keyId);
                    e_geo_i(0) = ft->matchedKeypt->pt.x - ft->u[0];
                    e_geo_i(1) = ft->matchedKeypt->pt.y - ft->v[0];
                    reprojChi2 = e_geo_i.dot(ft->geoInf*ft->geoInf*e_geo_i);

                    if(reprojChi2 < geothreshold){
                        featuresForTracking.push_back(ft);
                        featuresForTrackingMap.insert({ft,true});
                        numFtInliers_refKey += (keyframe->keyId == refKeyframe->keyId);
                    }
                }
            }
        }
    }
}

void IDNav::Tracker::finishOptimization(){
    trackedFrame->set_photo_parameters(0.0,0.0);
    speedPrior = vec6::Zero();
    covInvPrior = mat6::Zero();
    clustered_hgpForTracking.clear();
    clustered_featuresForTracking.clear();

    // Erase ouliers from trackedFeatures and observations
    trackedFrame->observations.clear();
    for(auto& [keId,keyframe]: keyframes){
        for(typeFt& ft: keyframe->features){
            if(featuresForTrackingMap.find(ft) != featuresForTrackingMap.end()){
                trackedFrame->observations.insert({ft,*ft->matchedKeypt});
           }
        }
    }

    trackedFrame->cam->setResolution(0);
    refKeyframe->hgp = refKeyframe->hgpInf;
}

void IDNav::Tracker::checkTrackingSucceed(){

    // Check inliers ratio
    size_t numPointsAvailable = numHgpAvailable_refKey + numFtAvailable_refKey;
    size_t numInliers = numHgpInliers_refKey + numFtInliers_refKey;
    inliersRatio = float(numInliers)/float(numPointsAvailable);
    inliersRatio_hgp = float(numHgpInliers_refKey)/float(numHgpAvailable_refKey);
    inliersRatio_features = float(numFtInliers_refKey)/float(numFtAvailable_refKey);
    if(inliersRatio < minInliersRatio){
        throw Tracker::TrackerFail{"Very few inliers"};
    }

    if(abs(photoErrorMean) > photoErrorMeanMax){
        throw Tracker::TrackerFail{"Big photo error mean"};
    }

    // Check kinematic prior
    //vec6 speed_i = poseInput.relativeMovement(trackedFrame->pose);
    //if((speed_i-speed_).transpose()*speedCovInv_*(speed_i-speed_)  > 0.352){
    //    throw Tracker::TrackingErrorFail{"Tracking ceres: FAILED",0,0,0,0};
    //}
}

void IDNav::Tracker::resetOptimization(){
    trackedFrame->pose.set_T_cw(poseInput.tcw,poseInput.Rcw);
    trackedFrame->set_photo_parameters(phScalar_input,phBias_input);
    trackedFrame->cam->patchSize = PATCH_SIZE;
    trackedFrame->cam->setResolution(0);
    trackedFrame->observations.clear();

    for (auto &[keyframe,hgp] : clustered_hgpForTracking ) {
        for (typePt &pt: hgp) {
            keyframe->uv_2_XYZ(pt);
        }
    }

    // Tracking state
    trackingSucceed = false;

    // Tracking members
    trackedFrame.reset();
    refKeyframe.reset();
    keyframes.clear();

    // Map members
    hgpForTracking.clear();
    clustered_hgpForTracking.clear();
    hgpForTrackingMap.clear();
    featuresForTracking.clear();
    clustered_featuresForTracking.clear();
    featuresForTrackingMap.clear();
    numHgpAvailable = 0;
    numFtAvailable = 0;

    // Input variables
    poseInput = Pose();
    phScalar_input = 0.0;
    phBias_input = 0.0;

    // Kinematic prior
    speedPrior = vec6::Zero();
    covInvPrior = mat6::Zero();
}

bool IDNav::Tracker::isTrackingSuccessful(){
    return trackingSucceed;
}

void IDNav::Tracker::computeTrackingHessian(mat6& totalHessian_){

    int numHgp{0};
    int numHgp_refKey;
    mat6 totalHessian_hgp = mat6::Zero();
    {
        mat16 dI_dx{};
        for(auto&[keyId, keyframe]: keyframes){
            int numHgp_i{0};
            for(typePt& pt: keyframe->hgp){
                if(hgpForTrackingMap.find(pt) != hgpForTrackingMap.end()){
                    trackedFrame->compute_inf_ph_poseJacobian(pt,dI_dx);
                    totalHessian_hgp += dI_dx.transpose() * dI_dx;
                    ++numHgp_i;
                }
            }
            numHgp += numHgp_i;
            if(keyId == refKeyframe->keyId){
                numHgp_refKey = numHgp_i;
            }
        }
    }

    int numFt{0};
    int numFt_refKey;
    mat6 totalHessian_features = mat6::Zero();
    {
        row26 duv_dx{};
        for(auto&[keyId, keyframe]: keyframes){
            int numFt_i{0};
            for(typeFt& ft: keyframe->features){
                if(featuresForTrackingMap.find(ft) != featuresForTrackingMap.end()){
                    trackedFrame->computeInformativeGeoJacobian(ft,duv_dx);
                    totalHessian_features += duv_dx.transpose() * duv_dx;
                    ++numFt_i;
                }
            }
            numFt += numFt_i;
            if(keyId == refKeyframe->keyId){
                numFt_refKey = numFt_i;
            }
        }
    }

    totalHessian_ = (numHgp_refKey + numFt_refKey)*(totalHessian_hgp + totalHessian_features)/(numHgp + numFt);
}


