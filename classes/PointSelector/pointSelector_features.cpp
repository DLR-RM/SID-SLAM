
#include "../../include/PointSelector.h"


// This function is just an interface to the extract hgp function: extract_features_grid_kinect(...)
void IDNav::PointSelector::extract_features(vecFt& features_ ,
                                            vecFt& allFeatures_,
                                            const vecKeyPt& keypoints0_ ,
                                            const Mat& grayImg_,
                                            const Mat& depthImg_,
                                            Mat& maskImg_){
#ifdef ACTIVE_FEATURES_TRACKING
#ifdef COUT_COMPLETE_PIPELINE
    featureExtractionProfiler.begin();
#endif
#endif
#ifdef ACTIVE_FEATURES
    extract_features_grid_kinect(features_, allFeatures_, keypoints0_, grayImg_, depthImg_, maskImg_,
                                 numPointsToExtract,maxDepthKinect);
#endif
#ifdef ACTIVE_FEATURES_TRACKING
#ifdef COUT_COMPLETE_PIPELINE
    featureExtractionProfiler.end();
#endif
#endif
}

void IDNav::PointSelector::extract_features_grid_kinect(vecFt& features_,
                                                        vecFt& allFeatures_,
                                                        const vecKeyPt& keypoints0_,
                                                        const Mat& grayImg_,
                                                        const Mat& depthImg_,
                                                        Mat& maskImg_,
                                                        const int numFeaturesToExtract_,
                                                        const float& maxDepthKinect_){

    vecKeyPt keypointsTmp{keypoints0_};
    filterFeaturesWithDepth(allFeatures_ , keypointsTmp, maskImg_, depthImg_);
    computeAndAssociateDescriptors(allFeatures_,keypointsTmp);
#ifdef ACTIVE_FEATURES_TRACKING
#ifndef DEACTIVATE_HGP
    filterFeaturesWithResponse(features_, allFeatures_ , maskImg_);
    updateMask(features_ , maskImg_);
#else
    features_ = allFeatures_;
#endif
#endif
}

void IDNav::PointSelector::filterFeaturesWithDepth(vecFt& allFeatures_,
                                                   vecKeyPt& keypoints_,
                                                   const Mat& maskImg_,
                                                   const Mat& depthImg_,
                                                   const float& maxDepthKinect_) const{
    int uRef, vRef;
    dataType uDist,vDist;
    Mat point(1,2,CV_64F);
    float depthRef{};
    allFeatures_.clear();
    vecKeyPt keypointsTmp = keypoints_;
    keypoints_.clear();
    for(KeyPoint& keyPt: keypointsTmp){
        cam->distortPt(uDist,vDist, keyPt.pt.x, keyPt.pt.y);
        uRef = (int)(uDist);
        vRef = (int)(vDist);
        if(thereIsDepthKinect(depthRef, uRef, vRef, depthImg_,MAX_DEPTH_VALUE)){
            keypoints_.push_back(keyPt);
            allFeatures_.push_back(createFt((dataType) keyPt.pt.x, (dataType) keyPt.pt.y, (dataType) depthRef, keyPt));
        }
    }
}

void IDNav::PointSelector::computeDescriptors(vecKeyPt& keypoints_, Mat& allDescriptors_){
#ifdef ACTIVE_FEATURES
#ifdef FEATURE_BIAS
    for(int iKeyPt{0}; iKeyPt < keypoints_.size(); ++iKeyPt){
        keypoints_[iKeyPt].pt.x += featureBias;//FEATURE_BIAS;
        keypoints_[iKeyPt].pt.y += featureBias;// FEATURE_BIAS;
    }
#endif
    akazeEvolution->Compute_Descriptors(keypoints_,allDescriptors_);
#ifdef FEATURE_BIAS
    for(int iKeyPt{0}; iKeyPt < keypoints_.size(); ++iKeyPt){
        keypoints_[iKeyPt].pt.x -= featureBias;// FEATURE_BIAS;
        keypoints_[iKeyPt].pt.y -= featureBias;// FEATURE_BIAS;
    }
#endif
#endif
}

void IDNav::PointSelector::computeAndAssociateDescriptors(const vecFt& features_, vecKeyPt& keypoints_){
#ifdef ACTIVE_FEATURES
    Mat allDescriptors_;
    computeDescriptors(keypoints_,allDescriptors_);
    akazeEvolution->Compute_Descriptors(keypoints_,allDescriptors_);
    for(int iFt{0}; iFt < features_.size(); ++iFt){
        features_[iFt]->descriptor = allDescriptors_.row(iFt);
    }
#endif
}

void IDNav::PointSelector::filterFeaturesWithResponse(vecFt& features_ , vecFt& allFeatures_, Mat& maskImg_){

    map<std::pair<int,int>, vecFt> features_map;
    features_.clear();
    int uCell,vCell;
    int uRef,vRef;
    for(typeFt& ft: allFeatures_){
        uRef = (int)round(ft->keyPtRef.pt.x);
        vRef = (int)round(ft->keyPtRef.pt.y);
        if(int(maskImg_.at<uchar>(vRef,uRef)) != 0) {
            if (ft->keyPtRef.response > minResponse0) {
                if (ft->keyPtRef.octave < 2) {
                    imageGrid.get_cell(uCell, vCell, int(ft->uRef), int(ft->vRef));
                    features_map[std::make_pair(uCell, vCell)].push_back(ft);
                }
            }
        }
    }

    // Filter clusters
    dataType maxResponse;
    int maxIndex{0};
    int index{0};
    for(auto & cell : features_map){
        maxResponse = 0.0;
        maxIndex = 0;
        index = 0;
        for(typeFt& ft: cell.second){
            if(ft->keyPtRef.response > maxResponse){
                maxResponse = ft->keyPtRef.response;
                maxIndex = index;
            }
            ++index;
        }
        features_.push_back(cell.second[maxIndex]);
    }
}

void IDNav::PointSelector::updateMask(vecFt& features_ , Mat& maskImg_){
    int up,vp,ui,vi;
    for(typeFt& ft: features_){
        up =(int) round(ft->uRef);
        vp =(int)round(ft->vRef);
        for(int i{-ft->ptArea}; i < ft->ptArea; ++i){
            ui = up + i;
            if((ui < 0)||(ui > maskImg_.cols - 1)) continue;
            for(int j{-ft->ptArea}; j < ft->ptArea; ++j){
                vi = vp + j;
                if(!((vi < 0)||(vi > maskImg_.rows - 1))){
                    maskImg_.at<uchar>(vi,ui) = 0;
                }
            }
        }
    }
}

void remove_outliers_by_distance(const std::vector<cv::KeyPoint> & kp0_,
                                 const std::vector<cv::KeyPoint> & kp1_,
                                 const float threshold_,
                                 std::vector<cv::DMatch> & matches_,
                                 std::vector<cv::DMatch> & outliers_)
{
    if (matches_.empty())
        return;

    // Remove matches if the best-matched distance is too far
    matches_.erase(std::remove_if(std::begin(matches_), std::end(matches_),
                                  [threshold_, &outliers_](cv::DMatch &m)
                                  {
                                      if (m.distance > threshold_) {
                                          outliers_.push_back(m);
                                          return true;
                                      }
                                      return false;
                                  }),
                   matches_.end());
}

void IDNav::PointSelector::trackFeatures(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_, vec6& speedPrior_){
    trackFeaturesSuccesful = true;
#ifdef ACTIVE_FEATURES_TRACKING
    static int consecutiveTrackFeaturesFailures{0};
    trackedFrame_->observations.clear();

#ifdef COUT_COMPLETE_PIPELINE
    featureTrackingProfiler.begin();
#endif

    // Extract keypoints
    if(trackedFrame_->keypoints.empty()){
#ifdef COMPLETE_PROFILING
        extractKeypointsProfiler.begin();
#endif
        extractKeypoints(trackedFrame_->keypoints);
        trackedFrame_->descriptorsObject =  Mat::zeros(trackedFrame_->keypoints.size(), DESCRIPTOR_SIZE, CV_8UC1);
        trackedFrame_->isDescriptorComputed  = vector<bool>(trackedFrame_->keypoints.size(), false);

        computeDescriptors(trackedFrame_->keypoints, trackedFrame_->descriptorsObject);
        trackedFrame_->isDescriptorComputed  = vector<bool>(trackedFrame_->keypoints.size(), true);

#ifdef COMPLETE_PROFILING
        extractKeypointsProfiler.end();
#endif
    }

#ifdef COMPLETE_PROFILING
    trackFeaturesProfiler.begin();
#endif

    // Set speedPrior
    Pose pose0 = trackedFrame_->pose;
    trackedFrame_->pose.update_T_cw_left(speedPrior_);

    size_t numAvailableFeatures = trackedFrame_->keypoints.size();
    availableFeaturesClustered.clear();

    // Search for visible features
    size_t numAllFeatures = 0;
    size_t numGeoVisibleFeatures = 0;
    for(auto& [keyId, keyframe_]: keyframes_){
        keyframe_->projectFeaturesTo(trackedFrame_);
        numAllFeatures += keyframe_->features.size();
        keyframe_->numTrackedFeatures = 0;
        for(typeFt& ft: keyframe_->features){
            ft->x0 = ft->u[0]; // Save projection to compute Optical Flow
            ft->y0 = ft->v[0];
            if(trackedFrame_->isGeoVisible(ft, maxLambdaVis, minLambdaVis, minCosVis)){
                availableFeaturesClustered[keyframe_].push_back(ft);
                ++numGeoVisibleFeatures;
            }
        }
    }
    size_t numGeoVisibleFeatures_refKeyframe = availableFeaturesClustered[refKeyframe_].size();

    // Track features
    dataType windowSize = 2.0*opticalFlow;
    dataType akazeTh = akazeTh0;
    dataType distanceTh = distanceTh0;

    size_t numTrackedFeatures{};
    size_t numTrackedFeatures_refKeyframe{};

    float ratioGlobal{0.0};
    float ratioGlobal_refKeyframe{0.0};

    for(size_t it{0}; it < 2; ++it){
        trackFeatures(windowSize, akazeTh, distanceTh, trackedFrame_);

        numTrackedFeatures = trackedFrame_->observations.size();
        numTrackedFeatures_refKeyframe =  refKeyframe_->numTrackedFeatures;

        ratioGlobal = float(numTrackedFeatures)/float(numGeoVisibleFeatures);
        ratioGlobal_refKeyframe =  float(numTrackedFeatures_refKeyframe)/float(numGeoVisibleFeatures_refKeyframe);
        if((ratioGlobal > succesfulRatioFeaturesTracked)||(ratioGlobal_refKeyframe > succesfulRatioFeaturesTracked)) break;

        windowSize += 8;
    }

    // Reset input pose
    trackedFrame_->pose.copyPoseFrom(pose0);

    // Check track features succeed
    float ratioRefKey =   float(refKeyframe_->hgp.size() + refKeyframe_->numTrackedFeatures)/float(refKeyframe_->hgp.size() + numGeoVisibleFeatures_refKeyframe);
    trackFeaturesSuccesful = (ratioRefKey > succesfulRatioFeaturesTracked);
    if(ratioRefKey < succesfulRatioFeaturesTracked){
        ++consecutiveTrackFeaturesFailures;
        cout <<GREEN_COUT << "        [PointSelector]: " << LIGTH_RED_COUT << "trackFeatures()"<< endl;
        cout <<LIGTH_RED_COUT << "            # trackPts / # visPts (refKey) = "<< refKeyframe_->hgp.size() + refKeyframe_->numTrackedFeatures << " / "<< refKeyframe_->hgp.size() + numGeoVisibleFeatures_refKeyframe << " = "<< ratioRefKey  << RESET_COUT<<endl;
        cout <<LIGTH_RED_COUT << "            # trackFts / # visFts = "<< numTrackedFeatures << "/"<< numGeoVisibleFeatures << " = "<< ratioGlobal << RESET_COUT<<endl;
        cout <<LIGTH_RED_COUT << "            # trackFts / # visFts (refKey) = "<< numTrackedFeatures_refKeyframe << "/"<< numGeoVisibleFeatures_refKeyframe << " = "<< ratioGlobal_refKeyframe << RESET_COUT<<endl;
        cout <<LIGTH_RED_COUT << "            # available keypoints = "<< numAvailableFeatures <<RESET_COUT<<endl;
        cout <<LIGTH_RED_COUT << "            # consecutive trackFeaturesFailures = "<< consecutiveTrackFeaturesFailures <<RESET_COUT<<endl;
    }else{
        consecutiveTrackFeaturesFailures = 0;
#ifdef COUT_COMPLETE_PIPELINE
        cout <<GREEN_COUT << "        [PointSelector]: trackFeatures()"<< endl;
        cout <<LIGTH_GREEN_COUT << "            # trackPts / # visPts (refKey) = "<< refKeyframe_->hgp.size() + refKeyframe_->numTrackedFeatures << " / "<< refKeyframe_->hgp.size() + numGeoVisibleFeatures_refKeyframe << " = "<< ratioRefKey  << RESET_COUT<<endl;
        cout <<LIGTH_GREEN_COUT << "            # trackFts / # visFts = "<< numTrackedFeatures << "/"<< numGeoVisibleFeatures << " = "<< ratioGlobal << RESET_COUT<<endl;
        cout <<LIGTH_GREEN_COUT << "            # trackFts / # visFts (refKey) = "<< numTrackedFeatures_refKeyframe << "/"<< numGeoVisibleFeatures_refKeyframe << " = "<< ratioGlobal_refKeyframe << RESET_COUT<<endl;
#endif
    }

#ifdef COUT_COMPLETE_PIPELINE
    featureTrackingProfiler.end();
#endif
#ifdef COMPLETE_PROFILING
    trackFeaturesProfiler.end();
#endif
#endif
}

void IDNav::PointSelector::extractKeypoints(vecKeyPt& keypoints_){
#ifdef ACTIVE_FEATURES
    keypoints_.clear();
    float numKeyPoints_ratio{0.0};

    // Function parameters
    static const float maxRatio{1.20f},minRatio{0.80f};
    static const size_t maxNumIt{2};

    // Extract Keypoints
    size_t it{0};
    akazeEvolution->Feature_Detection(keypoints_);
    numKeyPoints_ratio = float(keypoints_.size()) / float(AKAZE_NUM_POINTS);

    while((numKeyPoints_ratio < minRatio)||(numKeyPoints_ratio > maxRatio)){
        tune_akaze_threshold(keypoints_.size());
        akazeEvolution->Feature_Detection(keypoints_);
        numKeyPoints_ratio = float(keypoints_.size()) / float(AKAZE_NUM_POINTS);
        ++it;

        if(akazeEvolution->getThreshold() <= AKAZE_THRESHOLD_MIN) break;
        if(akazeEvolution->getThreshold() >= AKAZE_THRESHOLD_MAX) break;
        if (it > maxNumIt) break;

    }
    if(numKeyPoints_ratio < minRatio){
        akazeEvolution->setThreshold(AKAZE_THRESHOLD_MIN);
        akazeEvolution->Feature_Detection(keypoints_);
    }
#ifdef FEATURE_BIAS
    for(KeyPoint& keyPt: keypoints_){
        keyPt.pt.x -=  featureBias;//FEATURE_BIAS;
        keyPt.pt.y -=  featureBias;//FEATURE_BIAS;
    }
#endif

#ifdef COUT_COMPLETE_PIPELINE
    if(numKeyPoints_ratio < minRatio){
        std::cout << GREEN_COUT <<  "        [PointSelector] " << BOLDRED_COUT <<  "extractKeypoints() : Very few keypoints extracted" << std::endl;
        std::cout << LIGTH_RED_COUT <<  "                # Keypoints extracted = " <<    keypoints_.size() << "/" << AKAZE_NUM_POINTS << RESET_COUT << std::endl;
        std::cout << LIGTH_RED_COUT <<  "                Akaze threshold = " << akazeEvolution->getThreshold() << " (" << AKAZE_THRESHOLD_MIN << " - "<< AKAZE_THRESHOLD_MAX << ")" << RESET_COUT << std::endl;
    }else{
        if(numKeyPoints_ratio > maxRatio){
            std::cout << GREEN_COUT <<  "        [PointSelector] " << BOLDRED_COUT <<  "extractKeypoints() : Too many keypoints extracted" << std::endl;
            std::cout << LIGTH_RED_COUT <<  "                # Keypoints extracted = " <<    keypoints_.size() << "/" << AKAZE_NUM_POINTS << RESET_COUT << std::endl;
            std::cout << LIGTH_RED_COUT <<  "                Akaze threshold = " << akazeEvolution->getThreshold() << " (" << AKAZE_THRESHOLD_MIN << " - "<< AKAZE_THRESHOLD_MAX << ")" << RESET_COUT << std::endl;
        }
        else{
            std::cout  <<  GREEN_COUT << "        [PointSelector] extractKeypoints() :" << RESET_COUT << endl;
            std::cout  <<  LIGTH_GREEN_COUT << "            # Keypoints extracted = " <<
                       keypoints_.size() << "/" << AKAZE_NUM_POINTS  << RESET_COUT << endl;
            std::cout  <<  LIGTH_GREEN_COUT <<"            Akaze threshold = " <<
                       akazeEvolution->getThreshold() << " (" << AKAZE_THRESHOLD_MIN << " - "<< AKAZE_THRESHOLD_MAX << ")" << RESET_COUT << endl;
        }
    }
#endif
#endif
}

#ifdef ACTIVE_FEATURES
void IDNav::PointSelector::tune_akaze_threshold(const int& last_nkp_){
    static double x_pre{0.0};
    static double y_pre{0.0};
    static double slope{-1.0}; // Some negative number; closer to 0 means finer and slower to approach the target

    /*
      By converting the parameters as y = log10(nkp+1), x = log10(threshold),
      a simple fitting line, y = a * x + b, can be assumed to find out
      the threshold to give the target nkp
    */

    const double target_nkp = AKAZE_NUM_POINTS;
    const double target_y = log10(target_nkp);


    double x = log10(akazeEvolution->getThreshold());
    double y = log10(last_nkp_ + 1.0);

    x = x + slope * (target_y - y);

    double threshold = exp(x * log(10.0));

    if (threshold > AKAZE_THRESHOLD_MAX)
        threshold = AKAZE_THRESHOLD_MAX; // The aperture is closed
    else
    if (threshold < AKAZE_THRESHOLD_MIN)
        threshold = AKAZE_THRESHOLD_MIN; // The aperture is fully open

    akazeEvolution->setThreshold(threshold);

    x = log10(akazeEvolution->getThreshold());
    y =  log10(last_nkp_ + 1.0);
    if(abs(y_pre - y) > 0.01){
        slope = -(x_pre-x)/(y_pre - y);
    }
    if(slope > -0.5) slope = -0.5;
    if(slope < -2.0)  slope = -2.0;

    x_pre = x;
    y_pre = y;
}
#endif

namespace IDNav{
    struct Match{
        cv::KeyPoint* keypt;
        double distance{};
        size_t matchedKeyPtIndex{};
        Match(cv::KeyPoint* keypt, double& distance, size_t& matchedKeyPtIndex):keypt(keypt),distance(distance),matchedKeyPtIndex(matchedKeyPtIndex){}
    };
}

#ifdef ACTIVE_FEATURES_TRACKING
void IDNav::PointSelector::trackFeatures( dataType& windowSize_, dataType& akazeTh_, dataType& distanceTh_,
                                          typeFrame& trackedFrame_){

    // Tmp variables
    unordered_map<typeFrame,vecFt> lostFeatures{};
    vecKeyPt potentialKeypoints{};
    vector<size_t> indexKeypoints{};
    int keyPtindex{};
    mat3 R{};
    dataType perspDef{};
    dataType searchArea{};
    dataType imgDistance{};
    mat2 Cleft{};
    dataType fx = trackedFrame_->cam->getFocalLengthX();
    dataType fy = trackedFrame_->cam->getFocalLengthY();
    vector<IDNav::Match> matches;
    float dist1{}, dist2{};
    int idx{},idx_i{};

    // Track features loop
    for(auto& [keyframe_, features_]: availableFeaturesClustered){
        lostFeatures.clear();
        R = trackedFrame_->pose.Rcw * keyframe_->pose.Rwc;
        for (typeFt &ft: features_) {
            // Search for nearest features
            potentialKeypoints.clear();
            indexKeypoints.clear();
            ft->estimateProjectionDeformation2D(Cleft,fx,fy,R);
            perspDef = sqrt(Cleft.determinant());
            searchArea = (ft->keyPtRef.size/ft->minSize)*(windowSize_ + perspDef);
            keyPtindex = 0;
            for (KeyPoint &keypt: trackedFrame_->keypoints) {
                imgDistance = sqrt(pow(ft->u[0] - keypt.pt.x, 2) + pow(ft->v[0] - keypt.pt.y, 2));
                if (imgDistance < searchArea) {
                    potentialKeypoints.push_back(keypt);
                    indexKeypoints.push_back(keyPtindex);
                    if (!trackedFrame_->isDescriptorComputed[keyPtindex]) {
                        vecKeyPt keyPtTemp{keypt};
                        Mat descriptorTemp = Mat::zeros(1, DESCRIPTOR_SIZE, CV_8UC1);
                        computeDescriptors(keyPtTemp, descriptorTemp);
                        descriptorTemp.row(0).copyTo(trackedFrame_->descriptorsObject.row(keyPtindex));
                        trackedFrame_->isDescriptorComputed[keyPtindex] = true;
                    }
                }
                ++keyPtindex;
            }

            if (!potentialKeypoints.empty()) { // Search area CHECK
                // Look for matches with the HAMMING NORM
                matches.clear();
                for (size_t i{0}; i < potentialKeypoints.size(); ++i) {
                    double dist = norm(ft->descriptor,trackedFrame_->descriptorsObject.row((int)indexKeypoints[i]),NORM_HAMMING);
                    if(dist < akazeTh_*MATCH_HAMMING_RADIUS){
                        KeyPoint* keyPointTmp =  &trackedFrame_->keypoints[(int)indexKeypoints[i]];
                        matches.emplace_back(IDNav::Match(keyPointTmp,dist,indexKeypoints[i]));
                    }
                }

                if (!matches.empty()) { // Matcher CHECK
                    // Search for two minimum distances dist1 and dist2
                    dist1 = 4 * MATCH_HAMMING_RADIUS;
                    dist2 = 4 * MATCH_HAMMING_RADIUS;
                    idx = 0;
                    idx_i = 0;
                    for(auto& match: matches){
                        if(match.distance < dist1){
                            dist2 = dist1;
                            dist1 = match.distance;
                            idx = idx_i;
                        }else{
                            if(match.distance < dist2){
                                dist2 = match.distance;
                            }
                        }
                        ++idx_i;
                    }

                    // Look for unambiguously unique matches
                    if (dist1 < distanceTh_*dist2) { // Distance CHECK
                        //if(abs(log10(ft->keyPtRef.response) - log10(matches[idx].keypt.response)) < 1) {
                        //if(ft->keyPtRef.octave == matches[idx].keypt.octave){
                        ft->matchedKeypt = matches[idx].keypt;
                        ft->matchedKeyPtIndex =  matches[idx].matchedKeyPtIndex;
                        //keyframe_->trackedFeatures.push_back(ft);
                        ++keyframe_->numTrackedFeatures;
                        trackedFrame_->observations.insert({ft, *matches[idx].keypt});
                        //}
                        //}
                    }
                    else{ // Distance CHECK
                        ft->matchedKeypt = nullptr;
                        lostFeatures[keyframe_].push_back(ft);
                    }
                }
                else{ // Matcher CHECK
                    ft->matchedKeypt = nullptr;
                    lostFeatures[keyframe_].push_back(ft);
                }
            }else{ // Search area CHECK
                ft->matchedKeypt = nullptr;
                lostFeatures[keyframe_].push_back(ft);
            }
        }
    }
    availableFeaturesClustered = lostFeatures;
}
#endif

void IDNav::PointSelector::retrackFeatures(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_){
#ifdef ACTIVE_FEATURES_TRACKING
    const size_t windowSize{1};
    const double akazeTh{akazeTh0*1.25};

    size_t retrackedFeatures = 0;
    size_t lostFeatures = 0;

    dataType imgDistance{};
    dataType searchArea{};
    vecKeyPt potentialKeypoints{};
    vector<size_t> indexKeypoints{};
    size_t keyPtindex{};
    vector<IDNav::Match> matches;

    mat3 R;
    mat2 Cleft;
    dataType fx = trackedFrame_->cam->getFocalLengthX();
    dataType fy = trackedFrame_->cam->getFocalLengthY();
    dataType perpsDef;
    dataType dist1{486.0}, dist2{486.0};
    int idx{},idx_i{};
    vecFt features_aux{};
    for(auto& [keyId, keyframe]: keyframes_){
        R = trackedFrame_->pose.Rcw * keyframe->pose.Rwc;
        keyframe->numTrackedFeatures = 0;
        keyframe->projectFeaturesTo(trackedFrame_);
        for(typeFt& ft: keyframe->features) {
            if(trackedFrame_->observations.find(ft) != trackedFrame_->observations.end()){
                ++keyframe->numTrackedFeatures;
                continue;
            }
            if(!trackedFrame_->cam->isPointIn(ft)){
                continue;
            }

            ft->estimateProjectionDeformation2D(Cleft,fx,fy,R);
            perpsDef = sqrt(Cleft.determinant());
            searchArea = (ft->keyPtRef.size/ft->minSize)*(windowSize + perpsDef);

            potentialKeypoints.clear();
            indexKeypoints.clear();
            keyPtindex = 0;
            for (KeyPoint &keypt: trackedFrame_->keypoints) {
                imgDistance = sqrt(pow(ft->u[0] - keypt.pt.x, 2) + pow(ft->v[0] - keypt.pt.y, 2));
                if (imgDistance < searchArea) {
                    potentialKeypoints.push_back(keypt);
                    indexKeypoints.push_back(keyPtindex);
                    if (!trackedFrame_->isDescriptorComputed[keyPtindex]) {
                        vecKeyPt keyPtTemp{keypt};
                        Mat descriptorTemp = Mat::zeros(1, DESCRIPTOR_SIZE, CV_8UC1);
                        computeDescriptors(keyPtTemp, descriptorTemp);
                        descriptorTemp.row(0).copyTo(trackedFrame_->descriptorsObject.row(keyPtindex));
                        trackedFrame_->isDescriptorComputed[keyPtindex] = true;
                    }
                }
                ++keyPtindex;
            }

            if (!potentialKeypoints.empty()) { // Search area CHECK
                matches.clear();
                for (size_t i{0}; i < potentialKeypoints.size(); ++i) {
                    double dist = norm(ft->descriptor, trackedFrame_->descriptorsObject.row((int) indexKeypoints[i]), NORM_HAMMING);
                    if (dist < akazeTh * MATCH_HAMMING_RADIUS) {
                        KeyPoint* keyPointTmp =  &trackedFrame_->keypoints[(int)indexKeypoints[i]];
                        matches.emplace_back(IDNav::Match(keyPointTmp,dist,indexKeypoints[i]));
                    }
                }

                if (!matches.empty()) { // Matcher CHECK
                    dist1 = 486;
                    dist2 = 486;
                    idx = 0;
                    idx_i = 0;
                    for (auto &match: matches) { // Search for two minimum distances dist1 and dist2
                        if (match.distance < dist1) {
                            dist2 = dist1;
                            dist1 = match.distance;
                            idx = idx_i;
                        } else {
                            if (match.distance < dist2) {
                                dist2 = match.distance;
                            }
                        }
                        ++idx_i;
                    }

                    if(ft->keyPtRef.octave == matches[idx].keypt->octave){
                        //if (dist1 < distanceTh*dist2) { // Distance Check 2
                            ft->matchedKeypt = matches[idx].keypt;
                            ft->matchedKeyPtIndex =  matches[idx].matchedKeyPtIndex;
                            trackedFrame_->observations.insert({ft, *matches[idx].keypt});
                            ++retrackedFeatures;
                            ++keyframe->numTrackedFeatures;
                        //}
                    }else{
                        ft->matchedKeypt = matches[idx].keypt;
                        ft->matchedKeyPtIndex =  matches[idx].matchedKeyPtIndex;
                        trackedFrame_->observations.insert({ft, *matches[idx].keypt});
                        ++retrackedFeatures;
                        ++keyframe->numTrackedFeatures;
                    }

                }
            }
        }
    }
#ifdef COUT_COMPLETE_PIPELINE
    cout << "    [PointSelector] retrackFeatures() : # retracked features = "<< retrackedFeatures <<"/"<< lostFeatures <<endl;
#endif
#endif
}

namespace IDNav{
struct ObservationAux{
    vecFt features;
    vecFrame keyframes{};
    const KeyPoint matchedKeyPt;
    int numObservations{1};
    ObservationAux(typeFt ft_, KeyPoint matchedKeyPt_, typeFrame& keyframe_): matchedKeyPt(matchedKeyPt_){
        keyframes.push_back(keyframe_);
        features.push_back(ft_);
    }
};
}

void IDNav::PointSelector::reuse_observations(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, Mat& maskImg_){
#ifdef ACTIVE_FEATURES_TRACKING

    trackedFrame_->features.clear();
    trackedFrame_->featuresForLC.clear();

    if(trackedFrame_->observations.empty()) return;

    vecFt reusedFt{};


    int uRef,vRef;
    float depthRef{0.0};
    unordered_map<size_t,ObservationAux>observationsAux{};
    for(auto& [ft,obs]: trackedFrame_->observations){
        auto it = observationsAux.find(ft->matchedKeyPtIndex);
        typeFrame refKeyframe = keyframes_[ft->idKey];
        if(it == observationsAux.end()){
            ObservationAux observationsAux_i(ft,obs,refKeyframe);
            observationsAux.insert({ft->matchedKeyPtIndex, observationsAux_i});
        }else{
            it->second.features.push_back(ft);
            it->second.keyframes.push_back(refKeyframe);
            ++(it->second.numObservations);
        }
    }

    for(auto& [index,obs]: observationsAux){
        if((obs.matchedKeyPt.response < minResponse0)||(obs.matchedKeyPt.octave > 1))continue;

        uRef = int(round(obs.matchedKeyPt.pt.x));
        vRef = int(round(obs.matchedKeyPt.pt.y));
        if(thereIsDepthKinect(depthRef,uRef,vRef,trackedFrame_->depthImg,MAX_DEPTH_VALUE)){
            typeFt reusedFt_i = createFt(uRef,vRef,depthRef,obs.matchedKeyPt);
            reusedFt_i->descriptor =  trackedFrame_->descriptorsObject.row(index);
            reusedFt_i->reUsedFeature = true;
            reusedFt_i->numObservations = obs.numObservations;
            reusedFt.push_back(reusedFt_i);
            int indexFt{0};
            for(typeFrame& keyframe: obs.keyframes){
                keyframe->observations.insert({reusedFt_i,obs.features[indexFt]->keyPtRef});
                ++indexFt;
            }
        }
    }


    trackedFrame_->features = reusedFt;
    updateMask(reusedFt , maskImg_);

#ifdef COUT_COMPLETE_PIPELINE
    cout <<GREEN_COUT <<"    |   |   [PointSelector] " <<  "reuse_observations() : # features = " << trackedFrame_->features.size() << RESET_COUT<< endl;
#endif
#endif
}
