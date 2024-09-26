
#include "../../include/trackingFrontEnd.h"

bool IDNav::TrackingFrontEnd::loadImage(Mat& rgbImg_, dataType& grayImgTs_,
                                        Mat& depthImage_, dataType& validDepth_,
                                        size_t& id_,
                                        Mat& maskImg_){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "\n"<< endl;
#endif
    processImageProfiler.begin();
    Mat grayImg_;
    bool loadImageSucceed =  trackedFrame->loadImage(rgbImg_, grayImg_ , grayImgTs_, depthImage_,  validDepth_, id_, maskImg_);

    // Create Akaze Non Linear Space
#ifdef ACTIVE_FEATURES
    Mat grayImgCV32;
    grayImg_.convertTo(grayImgCV32,CV_32F,1.0/255.0,0);
    pointSelector->akazeEvolution->Create_Nonlinear_Scale_Space(grayImgCV32);
#endif

    // Create Resolution Pyramid
    trackedFrame->grayImgG.clear();
    Mat Mat_gray_temporal;
    for(size_t iPyr{0}; iPyr < trackedFrame->cam->getNumPyrLevelsUsed(); ++iPyr){
        resize(grayImg_, Mat_gray_temporal, Size(trackedFrame->cam->get_w(iPyr),trackedFrame->cam->get_h(iPyr)));
        trackedFrame->grayImgG.push_back(Mat_gray_temporal.clone());

    }

    processImageProfiler.end();

    return loadImageSucceed;
}

bool IDNav::TrackingFrontEnd::initFrontEnd(){
   pointSelector->extractKeypoints(trackedFrame->keypoints);
   if(!addKeyframe(trackedFrame)){
       return false;
   }
   getTrackingMap();
   getKeyframesForTracking();
   return true;
}

void IDNav::TrackingFrontEnd::initAddKeyframe(){
    addKeyframeProfiler.begin();
}

void IDNav::TrackingFrontEnd::updateFeatureObservations(typeFrame& newKeyframe_){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "            updateFeatureObservations()" <<  endl;
#endif

    for(typeFt& ft: newKeyframe_->features){
        for(auto& [keyId,keyframe_hc]: keyframesLocalWindow){
            auto it = keyframe_hc->observations.find(ft);
            if(it != keyframe_hc->observations.end()){
                (*keyframes)[keyId]->observations.insert({it->first,it->second});
            }
        }
    }

    unordered_map<typeFt,KeyPoint> observationsTmp{newKeyframe_->observations};
    newKeyframe_->observations.clear();
    typeFrame keyframe;
    typeFt ft;
    for(auto& [keyId,keyframe_hc]: keyframesLocalWindow){
        keyframe = (*keyframes)[keyId];
        for(typeFt& ft_hc: keyframe_hc->features){
            auto it = observationsTmp.find(ft_hc);
            if(it != observationsTmp.end()){
                ft = keyframe->features[ft_hc->id];
                ft->matchedKeyPtIndex = ft_hc->matchedKeyPtIndex;
                newKeyframe_->observations.insert({ft,it->second});
            }
        }
    }
}

void IDNav::TrackingFrontEnd::addKeyframeToWindow(typeFrame& newKeyframe_){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "        [TrackingFrontend] addKeyframeToWindow() "<< endl;
#endif

    if(keyframes->empty()){
        keyframes->insert({newKeyframe_->keyId,newKeyframe_});
        keyframesToOptimize.insert({newKeyframe_->keyId,newKeyframe_});
        setNewKeyframeAsReferenceKeyframe(newKeyframe_);
        return;
    }
    // Add newkeyframe to keyframes
    keyframes->insert({newKeyframe_->keyId,newKeyframe_});
    newKeyframe_->refKeyId = refKeyframe->keyId;

    // Force connection between newkeyframe and the previous refKeyframe
    connectFrames(refKeyframe,newKeyframe_);

    // Try to connect newkeyframe with keyframes connected to the previous refKeyframe
    for(auto [idKey,cov]: refKeyframe->keyframes){
        connectFramesConditioned((*keyframes)[idKey],newKeyframe_);
    }
    for(auto [idKey,cov]: refKeyframe->keyframes){
        typeFrame keyframe = (*keyframes)[idKey];
        for(auto [idKey_,cov_]: keyframe->keyframes){
            connectFramesConditioned((*keyframes)[idKey_],newKeyframe_);
        }
    }

    keyframesToOptimize.clear();
    IDNav::TrackingFrontEnd::get_N_covisibleKeyframes_toOptimize(
            keyframesToOptimize, newKeyframe_, 0.5,0.5, numKeyframesWindowOptimization);

    for(auto [keyId_i,keyframe_i]: keyframesToOptimize){
        for(auto [keyId_j,keyframe_j]: keyframesToOptimize){
            connectFramesConditioned(keyframe_i, keyframe_j);
        }
    }

    setNewKeyframeAsReferenceKeyframe(newKeyframe_);

}

void IDNav::TrackingFrontEnd::setNewKeyframeAsReferenceKeyframe(typeFrame& newKeyframe_){

    static int numKeyframes{0};
    ++numKeyframes;
#ifdef COUT_COMPLETE_PIPELINE
    cout << "            setReferenceKeyframe() = " << numKeyframes << " , keyId = " << newKeyframe_->keyId << " , frameId = " << newKeyframe_->frameId <<  endl;
#endif
    refKeyframe = newKeyframe_;
    Rrel_refKeyframe = mat3::Identity();
    trel_refKeyframe = vec3::Zero();
    Rrel_previousFrame_refKeyframe = mat3::Identity();
    trel_previousFrame_refKeyframe = vec3::Zero();
    switchedKeyframe = false;
    createdNewKeyframe = true;
}

void IDNav::TrackingFrontEnd::connectFrames(typeFrame& frame1_, typeFrame& frame2_){

    if(frame1_->keyId == frame2_->keyId) return;
    if(frame1_->keyframes.find(frame2_->keyId) != frame1_->keyframes.end()) return;

    mat4 Tji{mat4::Identity()};
    Tji.block<3,3>(0,0) = frame2_->pose.Rwc * frame1_->pose.Rcw;
    Tji.block<3,1>(0,3) = frame2_->pose.Rwc * frame1_->pose.tcw + frame2_->pose.twc;

    float covRatio = frame1_->getCovisibilityRatio(frame2_);
    Covisibility cov1(Tji, covRatio);
    frame1_->keyframes.insert({frame2_->keyId,cov1});

    covRatio = frame2_->getCovisibilityRatio(frame1_);
    Covisibility cov2(Tji.inverse(), covRatio);
    frame2_->keyframes.insert({frame1_->keyId,cov2});

}

void IDNav::TrackingFrontEnd::connectFramesConditioned(typeFrame& frame1_, typeFrame& frame2_){

    if(frame1_->keyId == frame2_->keyId) return;
    if(frame1_->keyframes.find(frame2_->keyId) != frame1_->keyframes.end()) return;

    mat4 Tji{mat4::Identity()};
    Tji.block<3,3>(0,0) = frame2_->pose.Rwc * frame1_->pose.Rcw;
    Tji.block<3,1>(0,3) = frame2_->pose.Rwc * frame1_->pose.tcw + frame2_->pose.twc;

    float covRatio = frame1_->getCovisibilityRatio(frame2_);
    Covisibility cov1(Tji, covRatio);
    if((cov1.distance < maxKeyframeDistance)&&(cov1.inliersRatio > minVisiblePointsRatio)) {
        frame1_->keyframes.insert({frame2_->keyId, cov1});
        covRatio = frame2_->getCovisibilityRatio(frame1_);
        Covisibility cov2(Tji.inverse(), covRatio);
        frame2_->keyframes.insert({frame1_->keyId, cov2});
    }

}

void IDNav::TrackingFrontEnd::connectFrames(const typeFrame& frame1, const typeFrame& frame2, mat4& Tji){

    if(frame1->keyId == frame2->keyId) return;
    if(frame1->keyframes.find(frame2->keyId) != frame1->keyframes.end()) return;

    float covRatio = frame1->getCovisibilityRatio(frame2);
    Covisibility cov1(Tji, covRatio);
    frame1->keyframes.insert({frame2->keyId,cov1});

    covRatio = frame2->getCovisibilityRatio(frame1);
    Covisibility cov2(Tji.inverse(), covRatio);
    frame2->keyframes.insert({frame1->keyId,cov2});
}

void IDNav::TrackingFrontEnd::initTracking(){
    trackingProfiler.begin();
    static int emergencyTracksTotal{0};
    static int emergencyTracks{0};
    if(tracker->emergencySettings){
//#ifdef COUT_COMPLETE_PIPELINE
        ++emergencyTracksTotal;
        ++emergencyTracks;
        cout <<LIGTH_RED_COUT << "    [TrackingFrontend] Emergency tracking activated " << RESET_COUT <<  endl;
        cout <<"        Consecutive emergency tracks = " << emergencyTracks << RESET_COUT <<  endl;
        cout <<"        Total emergency tracks       = " << emergencyTracksTotal << RESET_COUT <<  endl;
//#endif
    }else{
        emergencyTracks = 0;
    }
}

void IDNav::TrackingFrontEnd::selectReferenceKeyframe(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "    [TrackingFrontend] selectReferenceKeyframe()"<< endl;
#endif
#ifdef COMPLETE_PROFILING
    selectReferenceKeyframeProfiler.begin();
#endif

    if(createdNewKeyframe) return;

    vecFt feat_aux;
    vecPt hgp_aux;
    int numVisiblePoints;
    float ratioVisibility;
    dataType distance;
    dataType minDistance = 0.9*refKeyframe_hc->distanceToFrame(trackedFrame);
    size_t indexRefKeyframe{refKeyframe_hc->keyId};
    for(auto& [idKey, cov]: keyframesLocalWindow){
        auto keyframe = keyframesLocalWindow[idKey];
        keyframe->getVisiblePointsIn(trackedFrame, hgp_aux,feat_aux);
        numVisiblePoints = int(hgp_aux.size() + feat_aux.size());
        ratioVisibility = float(numVisiblePoints)/float(keyframe->hgp.size() + keyframe->features.size());
        if(ratioVisibility > 0.0){
            distance = keyframe->distanceToFrame(trackedFrame);
            if(distance < minDistance){
                minDistance = distance;
                indexRefKeyframe = idKey;
            }
        }
    }

    if(refKeyframe->keyId != indexRefKeyframe){
        auto nextRefKeyframe = (*keyframes)[indexRefKeyframe];
        refKeyframe ->trackingInformationBitsMax =  0.5*(refKeyframe ->trackingInformationBitsMax + nextRefKeyframe->trackingInformationBitsMax);
        nextRefKeyframe->trackingInformationBitsMax = refKeyframe->trackingInformationBitsMax;

        refKeyframe = nextRefKeyframe;
        refKeyframe_hc = keyframesLocalWindow[indexRefKeyframe];

        updateTrackingInformationThreshold();

        switchedKeyframe = true;
    }

    getRelativePoseToReferenceKeyframe();
#ifdef COMPLETE_PROFILING
    selectReferenceKeyframeProfiler.end();
#endif
}

void IDNav::TrackingFrontEnd::getRelativePoseToReferenceKeyframe(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "        getRelativePoseToReferenceKeyframe()" << endl;
#endif

    Rrel_refKeyframe = trackedFrame->pose.Rwc*refKeyframe_hc->pose.Rcw;
    trel_refKeyframe = trackedFrame->pose.Rwc*refKeyframe_hc->pose.tcw + trackedFrame->pose.twc;

    if(previousFrame != nullptr){
        Rrel_previousFrame_refKeyframe = previousFrame->pose.Rwc*refKeyframe_hc->pose.Rcw;
        trel_previousFrame_refKeyframe = previousFrame->pose.Rwc*refKeyframe_hc->pose.tcw + previousFrame->pose.twc;
    }

    // Normalize rotation to avoid numerical error failures
    //vec3 v{},w{};
    //log_lie(v,w,trel_refKeyframe,Rrel_refKeyframe);
    //exp_lie(trel_refKeyframe,Rrel_refKeyframe,v,w);
}

void IDNav::TrackingFrontEnd::setRelativePoseToReferenceKeyframe(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "        setRelativePoseToReferenceKeyframe()" << endl;
#endif
    {
        mat3 Rwc = Rrel_refKeyframe*refKeyframe->pose.Rwc;
        vec3 twc = Rrel_refKeyframe*refKeyframe->pose.twc + trel_refKeyframe;
        trackedFrame->pose.set_T_wc(twc, Rwc);
        if(previousFrame != nullptr){
            Rwc = Rrel_previousFrame_refKeyframe*refKeyframe->pose.Rwc;
            twc = Rrel_previousFrame_refKeyframe*refKeyframe->pose.twc + trel_previousFrame_refKeyframe;
            previousFrame->pose.set_T_wc(twc, Rwc);
        }
    }
}

void IDNav::TrackingFrontEnd::getKeyframesForTracking(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "    [TrackingFrontend] getKeyframesForTracking()"<< endl;
#endif
#ifdef COMPLETE_PROFILING
    getKeyframesForTrackingProfiler.begin();
#endif

    int numKeyframesForTracking = 5;//2 + float(pointSelector->numPointsToExtract)/float(pointSelector->numPointsToSelectWithInformation);
    get_N_covisibleKeyframes(keyframesForTracking,refKeyframe_hc,trackedFrame,keyframesLocalWindow,
                             refKeyframe_hc->keyframes,2.0,0.1, numKeyframesForTracking);

#ifdef COMPLETE_PROFILING
    getKeyframesForTrackingProfiler.end();
#endif
}

void IDNav::TrackingFrontEnd::getTrackingMap(){
#ifdef COMPLETE_PROFILING
    getMapProfiler.begin();
#endif
    std::mutex mMutex;
    unique_lock<mutex> lock(mMutex);
    {
        if(switchedKeyframe){
#ifdef COUT_COMPLETE_PIPELINE
            cout << "    [TrackingFrontend] getTrackingMap(): switchedKeyframe" << endl;
#endif
            for(auto& [keyId,cov]: refKeyframe_hc->keyframes){
                if(keyframesLocalWindow.find(keyId) == keyframesLocalWindow.end()){
                    typeFrame keyframe_i = (*keyframes)[keyId];
                    typeFrame keyframeTmp = hardCopyFrame(keyframe_i);
                    keyframeTmp->cam = cameraTracking;
                    keyframesLocalWindow.insert({keyId,keyframeTmp});
                }
            }
            unordered_map<size_t,typeFrame> keyframesLocalWindow_aux{keyframesLocalWindow};
            for(auto& [keyId,keyframe]: keyframesLocalWindow_aux){
                if((refKeyframe->keyframes.find(keyId) == refKeyframe->keyframes.end())&&(refKeyframe->keyId != keyId)){
                    keyframesLocalWindow.erase(keyId);
                }
            }
        }

        if((createdNewKeyframe)||(triggeredLoop)){
#ifdef COUT_COMPLETE_PIPELINE
            cout << "    [TrackingFrontend] getTrackingMap(): createdNewKeyframe" << endl;
#endif
            keyframesLocalWindow.clear();

            refKeyframe_hc = hardCopyFrame(refKeyframe);
            refKeyframe_hc->cam = cameraTracking;
            keyframesLocalWindow[refKeyframe_hc->keyId] = refKeyframe_hc;

            for(auto& [idKey,cov]: refKeyframe_hc->keyframes){
                typeFrame keyframeTmp = hardCopyFrame((*keyframes)[idKey]);
                keyframeTmp->cam = cameraTracking;
                keyframesLocalWindow[keyframeTmp->keyId] = keyframeTmp;
            }
        }
    }

    switchedKeyframe = false;
    createdNewKeyframe = false;
    triggeredLoop = false;

#ifdef COMPLETE_PROFILING
    getMapProfiler.end();
#endif
}

void IDNav::TrackingFrontEnd::bufferFrame(){
    previousFrame = IDNav::bufferFrame(trackedFrame);
    //previousFrame.reset(new Frame{trackedFrame});
}

void IDNav::TrackingFrontEnd::finishAddKeyframe(std::shared_ptr<IDNav::Frame>& newKeyframe){
    previousFrame = nullptr;
    Rrel_previousFrame_refKeyframe = mat3::Identity();
    trel_previousFrame_refKeyframe = vec3::Zero();

    numTrackedFrames = 0;
    trackingInformationBitsVector.clear();

    // Release useless data
    for(size_t iPyr{newKeyframe->grayImgG.size() -1 }; iPyr > 0; --iPyr){
        newKeyframe->grayImgG[iPyr].release();
        newKeyframe->grayImgG.erase(newKeyframe->grayImgG.begin() + iPyr);
    }
#ifdef GEO_DEPTH_OPTIMIZATION

#else
    newKeyframe->depthImg.release();
#endif

    //newKeyframe->keypoints.clear();
    newKeyframe->featuresForLC.clear();
    newKeyframe->grayImgDist.release();

    if(trajectory.find(newKeyframe->frameId) != trajectory.end()){
        trajectory.find(newKeyframe->frameId)->second->setKeyframe(newKeyframe);
    }else{
        trajectory.insert({newKeyframe->frameId, make_shared<RelativePose>(newKeyframe,newKeyframe)});
    }

    addKeyframeProfiler.end();
}

void IDNav::TrackingFrontEnd::finishTracking(){
    trackedFrame->keypoints.clear();
    trackedFrame->descriptorsObject.release();
    trackedFrame->isDescriptorComputed.clear();
    trackingProfiler.end();

    if(!visualization) return;

    { // Just with visualization purposes.
        if(trackedFrame->frameId == refKeyframe_hc->frameId){
            KeyPoint keyPt0;
            for(typeFt& ft: refKeyframe_hc->features){
                trackedFrame->observations.insert({ft,keyPt0});
            }
        }
        for(auto& [idKey,keyframe_]: keyframesLocalWindow){
            for(typePt& pt: keyframe_->hgp){
                trackedFrame->XYZ_2_uv(pt);
            }
            for(typeFt& ft: keyframe_->features){
                trackedFrame->XYZ_2_uv(ft);
            }
        }
    }
}

void IDNav::TrackingFrontEnd::computeOpticalFlow(){
#ifdef ACTIVE_FEATURES
    { // Compute Optical Flow
        dataType opticalFlowMax{0.0};
        dataType opticalFlow{0.0};
        vecFt features_aux{};
        for(auto& [keyId,keyframe]: keyframesForTracking){
            keyframe->getVisibleFeaturesIn(trackedFrame,features_aux);
            for(typeFt& ft: features_aux){
                opticalFlow = sqrt(pow(ft->matchedKeypt->pt.x - ft->x0,2) + pow(ft->matchedKeypt->pt.y - ft->y0,2));
                if(opticalFlow > opticalFlowMax){
                    opticalFlowMax = opticalFlow;
                }
            }
        }
        pointSelector->setOpticalFlow(opticalFlowMax);
    }
#endif
}

void IDNav::TrackingFrontEnd::trackingSucceed(){
#ifdef COUT_COMPLETE_PIPELINE
    cout <<"    [TrackingFrontend]: " << LIGTH_GREEN_COUT << "Succesful tracking" << RESET_COUT <<endl;
#endif
#ifdef COMPLETE_PROFILING
    trackingSucceedProfiler.begin();
#endif

    saveFrameInTrajectory();
    computeOpticalFlow();
    bufferFrame();
    ++numTrackedFrames;

#ifdef COMPLETE_PROFILING
    trackingSucceedProfiler.end();
#endif
}
void IDNav::TrackingFrontEnd::get_N_covisibleKeyframes(unordered_map<size_t,typeFrame>& covKeyframes,
                                                       typeFrame& refKeyframe_,
                                                       unordered_map<size_t,typeFrame>& keyframes_,
                                                       unordered_map<size_t,Covisibility>& keyframesCov_,
                                                       const dataType maxDistance_,
                                                       const dataType minVisiblePointsRatio_,
                                                       int maxCovKeyframes){

    //
    covKeyframes.clear();
    covKeyframes.insert({refKeyframe_->keyId,refKeyframe_});

    //
    unordered_map<size_t,Covisibility*> keyframesAux{};
    for(auto& [idKey,cov]: keyframesCov_){
        auto it = keyframes_.find(idKey);
        if(it != keyframes_.end()){
            if((cov.distance < maxDistance_)&&(cov.inliersRatio > minVisiblePointsRatio_)){
                keyframesAux.insert({idKey, &(cov)});
            }
        }
    }

    //
    dataType keyframeDistance{0.0};
    typeFrame windowKeyframe_i;
    vector<size_t>keyIds{};
    while((keyframeDistance < maxDistance_)&&(!keyframesAux.empty())){
        keyIds.clear();
        for(auto& [idKey,cov]: keyframesAux){
            if(covKeyframes.size() == maxCovKeyframes){
                keyframeDistance = maxDistance_;
                break;
            }
            if(cov->distance < keyframeDistance){
                covKeyframes.insert({idKey, keyframes_[idKey]});
                keyIds.push_back(idKey);
            }
        }
        for(size_t& idKey: keyIds){
            keyframesAux.erase(idKey);
        }
        keyframeDistance += (maxDistance_/100.0);
    }
}

void IDNav::TrackingFrontEnd::get_N_covisibleKeyframes_toOptimize(unordered_map<size_t,typeFrame>& covKeyframes,
                                                                  typeFrame& refKeyframe_,
                                                                  const dataType maxDistance_,
                                                                  const dataType minVisiblePointsRatio_,
                                                                  int maxCovKeyframes_){

    //
    covKeyframes.clear();
    covKeyframes.insert({refKeyframe_->keyId,refKeyframe_});
    unordered_map<size_t,typeFrame> covKeyframes_i{};

    // Geometric search around ref keyframe
    {
        map<size_t, Covisibility *> keyframesAux{};
        for (auto&[idKey, cov]: refKeyframe_->keyframes) {
            if ((cov.distance < maxDistance_) && (cov.inliersRatio > minVisiblePointsRatio_)) {
                keyframesAux.insert({idKey, &cov});
            }
        }
        if (keyframesAux.size() + covKeyframes.size() < maxCovKeyframes_) {
            for (auto&[idKey, cov]: keyframesAux) {
                covKeyframes.insert({idKey, (*keyframes)[idKey]});
                covKeyframes_i.insert({idKey, (*keyframes)[idKey]});
            }
        } else {
            int numKeyframesToAdd = maxCovKeyframes_ - covKeyframes.size();
            size_t keyId_i;
            for (int it{0}; it < numKeyframesToAdd; ++it) {
                double minDistance{10000.0};
                for (auto&[idKey, cov]: keyframesAux) {
                    if (cov->distance < minDistance) {
                        minDistance = cov->distance;
                        keyId_i = idKey;
                    }
                }
                covKeyframes.insert({keyId_i, (*keyframes)[keyId_i]});
                keyframesAux.erase(keyId_i);
            }
            return;
        }
    }

    if(covKeyframes_i.empty()){
        covKeyframes.insert({refKeyframe_->refKeyId,(*keyframes)[refKeyframe_->refKeyId]});
        covKeyframes_i.insert({refKeyframe_->refKeyId,(*keyframes)[refKeyframe_->refKeyId]});
    }

    //First breath search
    int numIt{0};
    while((covKeyframes.size() <maxCovKeyframes_)&&(numIt < numKeyframesWindowOptimization))
    {
        map<size_t, typeFrame> keyframesAux{};
        for (auto&[idKey, cov]: covKeyframes_i) {
            auto keyframe = (*keyframes)[idKey];
            size_t refKeyId = keyframe->refKeyId;
            if((keyframesAux.find(refKeyId) == keyframesAux.end())&&(covKeyframes.find(refKeyId) == covKeyframes.end())){
                keyframesAux.insert({refKeyId,(*keyframes)[refKeyId]});
            }
        }

        covKeyframes_i.clear();
        if (keyframesAux.size() + covKeyframes.size() < maxCovKeyframes_) {
            for (auto&[idKey, cov]: keyframesAux) {
                covKeyframes.insert({idKey, (*keyframes)[idKey]});
                covKeyframes_i.insert({idKey, (*keyframes)[idKey]});
            }
        } else {
            int numKeyframesToAdd = maxCovKeyframes_ - covKeyframes.size();
            size_t keyId_i;
            for (int it{0}; it < numKeyframesToAdd; ++it) {
                double minDistance{10000.0};
                double distance{};
                for (auto&[idKey, cov]: keyframesAux) {
                    distance = refKeyframe_->distanceToFrame((*keyframes)[idKey]);
                    if (distance < minDistance) {
                        minDistance = distance;
                        keyId_i = idKey;
                    }
                }
                covKeyframes.insert({keyId_i, (*keyframes)[keyId_i]});
                keyframesAux.erase(keyId_i);
            }
            return;
        }
        ++numIt;
    }

}

void IDNav::TrackingFrontEnd::get_N_covisibleKeyframes(unordered_map<size_t,typeFrame>& covKeyframes,
                                                       typeFrame& refKeyframe_,
                                                       typeFrame& trackedFrame_,
                                                       unordered_map<size_t,typeFrame>& keyframes_,
                                                       unordered_map<size_t,Covisibility>& keyframesCov_,
                                                       const double maxDistance_,
                                                       const double minVisiblePointsRatio_,
                                                       int maxCovKeyframes){

    //
    covKeyframes.clear();
    covKeyframes.insert({refKeyframe_->keyId,refKeyframe_});

    //
    unordered_map<size_t,Covisibility*> keyframesAux{};
    for(auto& [idKey,cov]: keyframesCov_){
        if((cov.distance < maxDistance_)&&(cov.inliersRatio > minVisiblePointsRatio_)){
            auto it = keyframes_.find(idKey);
            if(it != keyframes_.end()){
                keyframesAux.insert({idKey, &(cov)});
            }
        }
    }

    //
    dataType keyframeDistance{0.0};
    typeFrame windowKeyframe_i;
    vector<size_t>keyIds{};
    dataType distance;
    while((keyframeDistance < maxDistance_)&&(!keyframesAux.empty())){
        keyIds.clear();
        for(auto& [idKey,cov]: keyframesAux){
            if(covKeyframes.size() == maxCovKeyframes){
                keyframeDistance = maxDistance_;
                break;
            }
            distance = trackedFrame_->distanceToFrame(keyframes_[idKey]);
            if(distance < keyframeDistance){
                covKeyframes.insert({idKey, keyframes_[idKey]});
                keyIds.push_back(idKey);
            }
        }
        for(size_t& idKey: keyIds){
            keyframesAux.erase(idKey);
        }
        keyframeDistance += (maxDistance_/100.0);
    }
}

void IDNav::TrackingFrontEnd::performRelocalization(){
    triggerLoops();

    /*static bool first{true};
    if(!first) return;
    dataType distance{};
    dataType covisibilityRatio{};
    for(auto& [idKey,keyframe]: *keyframes){
        if(refKeyframe->keyId == idKey) continue;
        if(refKeyframe->keyframes.find(keyframe) == refKeyframe->keyframes.end()){
            distance = refKeyframe->distanceToFrame(keyframe);
            if(distance < 0.05){
                covisibilityRatio = refKeyframe->getCovisibilityRatio(keyframe);
                if(covisibilityRatio > 0.5){
                    if(refKeyframe->keyId - keyframe->keyId > 50){
                        cout << "Potential keyframe to relocalize "<< endl;
                        mat4 Tji;
                        cout << "distance 1 = "<< refKeyframe->distanceToFrame(keyframe) << endl;
                        ria->relocalizeFrame(keyframe,keyframesForTracking,refKeyframe,keyframe->pose,speed,speedCovInv,Tji);
                        if(ria->isTrackingSuccessful()){
                            conectFrames(refKeyframe,keyframe, Tji);
                            for(auto& [keyframe_relocalize,Tkj]: keyframe->keyframes){
                                mat4 Tki = Tkj * Tji;
                                conectFrames(refKeyframe, keyframe_relocalize,Tki);
                            }
                            poseGraphOptimizer->optimizePoseGraph(keyframes);
                            BA->resetOptimization();
                            BA->prepareOptimization(*keyframes,"iterativeOptimization");
                            BA->optimize(100);
                            cout << "distance 3 = "<< refKeyframe->distanceToFrame(keyframe) << endl;
                            first = false;
                        }
                    }

                }
            }
        }
    }*/
}

void IDNav::TrackingFrontEnd::setKeyframeCovisibility(const dataType& maxKeyframeDistance_ , const dataType& minVisiblePointsRatio_){
    maxKeyframeDistance = maxKeyframeDistance_;
    minVisiblePointsRatio = minVisiblePointsRatio_;
    if(maxKeyframeDistance > 2.0)
        maxKeyframeDistance = 2.0;
    if(maxKeyframeDistance < 0.1)
        maxKeyframeDistance = 0.1;
    if(minVisiblePointsRatio > 1.0)
        minVisiblePointsRatio = 1.0;
    if(minVisiblePointsRatio < 0.0)
        minVisiblePointsRatio = 0.0;
}

void IDNav::TrackingFrontEnd::set_numKeyframesWindowOptimization(const int& numKeyframesWindowOptimization_){
    numKeyframesWindowOptimization = numKeyframesWindowOptimization_;
    if(numKeyframesWindowOptimization > 50) numKeyframesWindowOptimization = 50;
    if(numKeyframesWindowOptimization < 2) numKeyframesWindowOptimization = 2;
}

// SAVE TRAJECTORY FUNCTIONS

void IDNav::TrackingFrontEnd::saveFrameInTrajectory(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "    [TrackingFrontend] saveFrameInTrajectory()" << endl;
#endif
    {
        trajectory.insert({trackedFrame->frameId, make_shared<RelativePose>(trackedFrame,refKeyframe_hc)});
    }
}