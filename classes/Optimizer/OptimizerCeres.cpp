
#include "../../include/Optimizer/OptimizerCeres.h"
#include "../../include/trackingFrontEnd.h"

void IDNav::OptimizerCeres::resetOptimization(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << BLUE_COUT << spaces + "[OptimizerCeres] resetOptimization()" << RESET_COUT << endl;
#endif
    optimizationPrepared = false;
    optimizationConverged = false;
    optimizationConverged_pose = false;
    optimizationConverged_depth_photo = false;
    optimizationConverged_depth_reproj = false;
    beginOptimization = false;
}

void IDNav::OptimizerCeres::prepareOptimization(unordered_map<size_t,typeFrame>& keyframesToOptimize_ ,
                                                const std::string& optimizationMode_){

    if(optimizationPrepared) return;
    if(keyframesToOptimize_.size() >= minNumKeyframesForOptimization){
        prepareOptProfiler.begin();
#ifdef COUT_COMPLETE_PIPELINE
        cout <<  spaces + "        optimizationMode = "<< optimizationMode_ << RESET_COUT << endl;
        cout <<  spaces + "        # keyframesToOptimize = "<< keyframesToOptimize_.size() << RESET_COUT << endl;
#endif
        setInitialState(keyframesToOptimize_  ,optimizationMode_);
        setProjections();
        setFixedKeyframes();
        setObservations();
        buildProblem();
        optimizationPrepared = true;
        prepareOptProfiler.end();
#ifdef OPTIMIZER_DEBUG
        optimizerDebug();
#endif
    }
}
#ifdef OPTIMIZER_DEBUG
void IDNav::OptimizerCeres::optimizerDebug() {
    cout << LIGTH_YEL_COUT << "[OptimizerCeres] optimizerDebug()" << RESET_COUT << endl;
    cout  << "    # num keyframes to optimize = "<< keyframesToOptimize.size() << endl;
    cout  << "    # num fixed keyframes  = "<< fixedKeyframes.size() << endl;
    cout  << "    # num map keyframes  = "<< windowKeyframes.size() -  keyframesToOptimize.size() << endl;

    covisibilityWindow_hgp = matXi::Zero(keyframesToOptimize.size() + 1, keyframesToOptimize.size() + 1);
    covisibilityWindow_features = matXi::Zero(keyframesToOptimize.size() + 1, keyframesToOptimize.size() + 1);

    int indexKey{0};
    keyframeIndexes.clear();
    for(auto& [iKey, keyframe]: keyframesToOptimize){
        covisibilityWindow_hgp(0,indexKey + 1) = iKey;
        covisibilityWindow_hgp(indexKey + 1,0) = iKey;
        covisibilityWindow_features(0,indexKey + 1) = iKey;
        covisibilityWindow_features(indexKey + 1,0) = iKey;
        keyframeIndexes.insert({iKey,indexKey});
        ++indexKey;
    }
    for(typeProj_hgp & proj : hgpProjections){
        ++covisibilityWindow_hgp(keyframeIndexes[proj->refKeyId] + 1,keyframeIndexes[proj->projKeyId] + 1);
    }
    for(typeProj_ft & proj : ftProjections){
        ++covisibilityWindow_features(keyframeIndexes[proj->refKeyId] + 1,keyframeIndexes[proj->projKeyId] + 1);
    }
    cout << LIGTH_YEL_COUT "covisibility Window hgp  = " << RESET_COUT << endl;
    cout << covisibilityWindow_hgp << endl;
    cout << LIGTH_YEL_COUT "covisibility Window features  = " << RESET_COUT << endl;
    cout << covisibilityWindow_features << endl;

    int numAllKeyframes = windowKeyframes.size();
    covisibilityComplete_hgp = matXi::Zero(numAllKeyframes + 1,numAllKeyframes + 1);
    covisibilityComplete_features = matXi::Zero(numAllKeyframes + 1,numAllKeyframes + 1);
    indexKey = keyframeIndexes.size();
    for(auto& [iKey, keyframe]: windowKeyframes){
        if(keyframesToOptimize.find(iKey) == keyframesToOptimize.end()){
            covisibilityComplete_hgp(0,indexKey + 1) = iKey;
            covisibilityComplete_hgp(indexKey + 1,0) = iKey;
            covisibilityComplete_features(0,indexKey + 1) = iKey;
            covisibilityComplete_features(indexKey + 1,0) = iKey;
            keyframeIndexes.insert({iKey,indexKey});
            ++indexKey;
        }
    }
    covisibilityComplete_hgp.block(0,0,covisibilityWindow_hgp.rows(),covisibilityWindow_hgp.cols()) = covisibilityWindow_hgp;
    covisibilityComplete_features.block(0,0,covisibilityWindow_features.rows(),covisibilityWindow_features.cols()) = covisibilityWindow_features;
    for(typeProj_hgp & proj : hgpProjections_map){
        ++covisibilityComplete_hgp(keyframeIndexes[proj->refKeyId] + 1,keyframeIndexes[proj->projKeyId] + 1);
    }
    for(typeProj_ft & proj : ftProjections_map){
        ++covisibilityComplete_features(keyframeIndexes[proj->refKeyId] + 1,keyframeIndexes[proj->projKeyId] + 1);
    }
    cout << LIGTH_YEL_COUT  "\ncovisibility Complete hgp = " << RESET_COUT << endl;
    cout << covisibilityComplete_hgp << endl;
    cout << LIGTH_YEL_COUT  "\ncovisibility Complete features = " << RESET_COUT << endl;
    cout << covisibilityComplete_features << endl;
    showCorrelations();
}
void IDNav::OptimizerCeres::showCorrelation(int& refKeyId_ , int& projKeyId_ ){
    windowKeyframes[refKeyId_]->drawPoints(windowKeyframes[projKeyId_]);
}

void IDNav::OptimizerCeres::showCorrelations(){
    std::string input;

    std::cout << LIGTH_YEL_COUT " \nRefKeyframe id = " << RESET_COUT << endl;
    std::cin >> input;
    int refKeyId = stoi(input);
    if(refKeyId < 0) return;
    std::cout << LIGTH_YEL_COUT " ProjKeyframe id = " << RESET_COUT << endl;
    std::cin >> input;
    int projKeyId = stoi(input);
    while (refKeyId >= 0) {
        showCorrelation(refKeyId,projKeyId);
        std::cout << LIGTH_YEL_COUT " RefKeyframe id = " << RESET_COUT << endl;
        std::cin >> input;
        refKeyId = stoi(input);
        if(refKeyId < 0) return;
        std::cout << LIGTH_YEL_COUT " ProjKeyframe id = " << RESET_COUT << endl;
        std::cin >> input;
        projKeyId = stoi(input);
    }
}
#endif

void IDNav::OptimizerCeres::optimize(int numIt_, double maxTime_){

    if((!optimizationPrepared)||(optimizationConverged)) return;

    beginOptimization = true;
    solverOptions_pose->max_num_iterations = numIt_;
    solverOptions_pose->max_solver_time_in_seconds = maxTime_;

    optimizeProfiler.begin();

    if(optimizeDepths){
        optimizeDepthPhoto();
    }
    optimizationConverged_depth_photo = true;

    if(optimizeDepths){
        optimizeDepthGeo();
    }
    optimizationConverged_depth_reproj = true;

    ceres::Solve(*solverOptions_pose, &(*problem_pose), &(*summary_pose));
    optimizationConverged_pose = (summary_pose->termination_type ==  ceres::CONVERGENCE);

    if(BA_type == "global"){
        cout << summary_pose->FullReport()<< endl;
    }

    optimizationConverged = (optimizationConverged_pose)&&(optimizationConverged_depth_photo)&&(optimizationConverged_depth_reproj);

    optimizeProfiler.end();

}

void IDNav::OptimizerCeres::updateVariables(){
    if(!beginOptimization) return;

#ifdef COUT_OPTIMIZER_PIPELINE
    if(optimizationConverged){
        cout << BLUE_COUT << spaces + "[OptimizerCeres] updateVariables(): " <<LIGTH_GREEN_COUT << "Optimization Convergence"<< RESET_COUT << endl;
    }else{
        cout << BLUE_COUT << spaces + "[OptimizerCeres] updateVariables(): " <<LIGTH_RED_COUT << "Optimization NO Convergence"<< RESET_COUT << endl;
        cout  << spaces + "       optimizationConverged_pose         = " << optimizationConverged_pose << endl;
        cout  << spaces + "       optimizationConverged_depth_photo  = " << optimizationConverged_depth_photo << endl;
        cout  << spaces + "       optimizationConverged_depth_reproj = " << optimizationConverged_depth_reproj << endl;
    }
#endif
    if(optimizeDepths){
        // Depth update
        //if(numHgpToOptimize > 0) {
        //double covariance_xx[1 * 1];
        //covariance_photo.reset(new ceres::Covariance(covarianceOptions));
        //CHECK(covariance_photo->Compute(covariance_blocks_photo, &(*problem_depth_photo)));
        for (auto&[pt, lambdaRef]: deltaLambdaHgp) {
            //if(hgpObservations[pt][0]->optDepth){
            //covariance_photo->GetCovarianceBlock(&deltaLambdaHgp[pt], &deltaLambdaHgp[pt], covariance_xx);
            //if(*covariance_xx < pt->lambdaRefCov){
            //pt->updateLambdaRef(lambdaRef, *covariance_xx);
            pt->updateLambdaRef(lambdaRef, pt->lambdaRefCov);
            //}
            //optimizeDepthGeo(hgpObservations[pt]);
            /*if(hgpObservations[pt].size() < 2){
                cout << "clara"<< endl;
                cout << hgpObservations[pt].size() << endl;
                cout <<" pt->idKey =  " <<  pt->idKey << endl;
                cout <<" pt->uRef =  " <<  pt->uRef << endl;
                cout <<" pt->vRef =  " <<  pt->vRef << endl;
                cout <<" pt->lambdaRef =  " <<  pt->lambdaRef << endl;
                cout <<" pt->planeParameters =  " <<  pt->planeParameters.transpose() << endl;
            }*/
            //numProjections_cl +=hgpObservations[pt].size();
            //}
        }
        //}

        //if(numFtToOptimize > 0) {
        //double covariance_xx[1 * 1];
        //covariance_reproj.reset(new ceres::Covariance(covarianceOptions));
        //CHECK(covariance_reproj->Compute(covariance_blocks_reproj, &(*problem_depth_reproj)));
        for (auto&[ft, lambdaRef]: deltaLambdaFeatures) {
            //if(featureObservations[ft][0]->optDepth) {
            //covariance_reproj->GetCovarianceBlock(&deltaLambdaFeatures[ft], &deltaLambdaFeatures[ft],covariance_xx);
            //if(*covariance_xx < ft->lambdaRefCov) {
            //ft->updateLambdaRef(lambdaRef, *covariance_xx);
            //dataType cov = 0.0;
            //for(auto& obs: featureObservations[ft]){
            //cov += obs->duv_dlambdaRef;
            //if(obs->thereIsDepthObs){
            //cout << "obs->lambdaRefObs = "<< obs->lambdaRefObs << endl;
            //}
            // optimizeDepthGeo(featureObservations[ft]);
            //cout << " "<< endl;
            //}
            //cov = 1.0/cov;
            //cout << "cov = "<<  cov << " = "<< *covariance_xx<< endl;
            ft->updateLambdaRef(lambdaRef, ft->lambdaRefCov);
            //}
            //}
        }
        //}
    }

#ifdef OPTIMIZER_DEBUG
    cout << LIGTH_YEL_COUT << "[OptimizerCeres] updateVariables()" << RESET_COUT << endl;
#endif

    // Pose update
    vec3 v{}, w{}, delta_t{}, tcw_updated{};
    mat3 delta_R{}, Rcw_updated{};
    for (auto&[keyId, keyframe]: (keyframesToOptimize)) {
        if (fixedKeyframes.find(keyId) == fixedKeyframes.end()) {
            dataType *incPose = deltaPoses[keyId];
            v << incPose[0], incPose[1], incPose[2];
            w << incPose[3], incPose[4], incPose[5];
#ifdef OPTIMIZER_DEBUG
            cout  <<setprecision(10)<< "    keyframe id = "<< keyId <<
                " , v = " <<  v.transpose() << " , w = " << w.transpose() << endl;
            cout  <<setprecision(10)<< "    keyframe id = "<< keyId <<
                  " , phScalar = "<< deltaPhotoScalar[keyId] <<" , phExpScalar = "<< deltaPhotoExpScalar[keyId] << " , phBias = " << deltaPhotoBias[keyId] << endl;
#endif
            if(BA_type == "global"){
                cout  <<setprecision(10)<< "    keyframe id = "<< keyId <<
                      " , v = " <<  v.transpose() << " , w = " << w.transpose() << endl;
                cout  <<setprecision(10)<< "    keyframe id = "<< keyId <<
                      " , phScalar = "<< deltaPhotoScalar[keyId] <<" , phExpScalar = "<< deltaPhotoExpScalar[keyId] << " , phBias = " << deltaPhotoBias[keyId] << endl;
            }

            exp_lie(delta_t, delta_R, v, w);
            Rcw_updated = delta_R * keyframe->pose.Rcw;
            tcw_updated = delta_R * keyframe->pose.tcw + delta_t;
            keyframe->pose.set_T_cw(tcw_updated, Rcw_updated);
        }
        keyframe->pose.xynlambda_2_XYZ(keyframe->hgp);
    }

    for (auto&[keyId_i, keyframe_i]: (keyframesToOptimize)) {
        for (auto&[keyId_j, keyframe_j]: (keyframesToOptimize)) {
            if(keyId_i == keyId_j) continue;
            if(keyframe_i->keyframes.find(keyId_j) != keyframe_i->keyframes.end()){
                auto it = keyframe_i->keyframes.find(keyId_j);
                it->second.Tji.block<3,3>(0,0) =  keyframe_j->pose.Rwc * keyframe_i->pose.Rcw;
                it->second.Tji.block<3,1>(0,3) = keyframe_j->pose.Rwc * keyframe_i->pose.tcw + keyframe_j->pose.twc;
            }
        }
    }
}

void IDNav::OptimizerCeres::setInitialState(unordered_map<size_t,typeFrame>& keyframesToOptimize_,
                                            const std::string& optimizationMode_){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "    [OptimizerCeres] setInitialState()"<< RESET_COUT << endl;
#endif

    keyframesToOptimize = keyframesToOptimize_;
    optimizationMode = optimizationMode_;

    windowKeyframes.clear();
    for(auto& [idKey,keyframe]: keyframesToOptimize){
        windowKeyframes.insert({idKey,keyframe});
    }

    Rrel.clear();
    updateRelativeRotations();
}

void IDNav::OptimizerCeres::updateRelativeRotations(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << spaces + "        updateRelativeRotations()" << endl;
#endif
    mat3 R{};
    if(Rrel.empty()){
        for(auto& [idHost,hostKey]: windowKeyframes){
            for(auto& [idProj,projKey]: windowKeyframes){
                if(Rrel.find(std::make_pair(hostKey,projKey)) != Rrel.end()){
                    continue;
                }
                if(idHost != idProj){
                    R = (projKey->pose.Rcw) * (hostKey->pose.Rwc);
                    Rrel[std::make_pair(hostKey,projKey)] = R;
                    inv_R(Rrel[std::make_pair(projKey,hostKey)],R);
                }
            }
        }
    }else{
        for(auto& R_: Rrel){
            if(R_.first.first->keyId != R_.first.second->keyId){
                R = (R_.first.second->pose.Rcw) * ( R_.first.first->pose.Rwc);
                Rrel[std::make_pair(R_.first.first,R_.first.second)] = R;
                inv_R(Rrel[std::make_pair(R_.first.first,R_.first.second)],R);
            }else{
                Rrel[std::make_pair(R_.first.first,R_.first.second)] = mat3::Identity();
                Rrel[std::make_pair(R_.first.second,R_.first.first)] = mat3::Identity();
            }

        }
    }
}

void IDNav::OptimizerCeres::setProjections(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "    [OptimizerCeres] setProjections()"<< RESET_COUT << endl;
#endif
    setProjections_hgp();
    setProjections_features();
    numProjections = numHgpProjections + numFtProjections;
    maxGeoNumProjections = maxGeoNumHgpProjections + maxGeoNumFtProjections;
    maxNumProjections = maxNumHgpProjections + maxNumFtProjections;
    numMapProjections = numHgpMapProjections + numFtMapProjections;

#ifdef COUT_OPTIMIZER_PIPELINE
    cout << spaces + "        # Hgp Projections = "   <<
         numHgpProjections << "/" << maxGeoNumHgpProjections << "/" << maxNumHgpProjections  << endl;
#ifdef ACTIVE_FEATURES_TRACKING
    cout << spaces + "        # Ft Projections = "    <<
         numFtProjections  << "/" << maxGeoNumFtProjections  << "/" << maxNumFtProjections << endl;
    cout << spaces + "        # Total Projections = " <<
         numProjections    << "/" << maxGeoNumProjections    << "/" << maxNumProjections << endl;
#endif
    cout << spaces + "        # Hgp map Projections = "   << numHgpMapProjections  << endl;
#ifdef ACTIVE_FEATURES_TRACKING
    cout << spaces + "        # Features map Projections = "   <<
         numFtMapProjections   << endl;
    cout << spaces + "        # Total map Projections = "   <<
         numMapProjections   << endl;
#endif
    cout << spaces + "        # Hgp to optimize = "   <<
         hgpObservations.size()   << endl;
#ifdef ACTIVE_FEATURES_TRACKING
    cout << spaces + "        # Features to optimize = "   <<
         featureObservations.size()   << endl;
#endif
#endif
}

void IDNav::OptimizerCeres::setProjections_hgp(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << spaces + "        setProjections_hgp()"<< endl;
#endif

    // Reset variables
    hgpObservations.clear();
    hgpProjections.clear();
    hgpProjections_map.clear();
    numHgpProjections = 0;
    maxGeoNumHgpProjections = 0;
    maxNumHgpProjections = 0;
    numHgpMapProjections = 0;

    // Get projections between keyframesToOptimize
    vecPt hgp_aux{};
    typeProj_hgp projection_temp{};
    for (auto& [hostKeyId, hostKey]: keyframesToOptimize) {
        for (auto& [projKeyId, projKey]: keyframesToOptimize) {
            if(hostKeyId == projKeyId) continue;
            if(hostKey->keyframes.find(projKeyId) == hostKey->keyframes.end()) continue; // If projKey is not connected to hostKey in the covisibility graph

            hostKey->getVisibleHgpIn(projKey,hgp_aux);
            maxNumHgpProjections += hostKey->hgp.size();
            maxGeoNumHgpProjections += hgp_aux.size();
            for(typePt& pt: hgp_aux){
                projection_temp.reset( new IDNav::Projection_hgp(
                        pt, hostKey, projKey, &(Rrel[std::make_pair(hostKey,projKey)])));
                projection_temp->computeResidualCov();
                projection_temp->getDepthObservation();
                hgpProjections.push_back(projection_temp);
                hgpObservations[pt].push_back(projection_temp);
            }
        }
    }

    // Get projections between keyframesToOptimize and mapKeyframes
    typeFrame mapKey;
    for (auto& [projKeyId, projKey]: keyframesToOptimize) {
        unordered_map<size_t,Covisibility> keyframesMap_aux{};
        for (auto& [mapKeyId, cov]: projKey->keyframes){
            if(keyframesToOptimize.find(mapKeyId) == keyframesToOptimize.end()) { // mapKey is not a keyframe to be optimized
                keyframesMap_aux.insert({mapKeyId,cov});
            }
        }
        unordered_map<size_t,typeFrame> keyframesMap{};
        IDNav::TrackingFrontEnd::get_N_covisibleKeyframes(keyframesMap, projKey, (*keyframes) , keyframesMap_aux, 2.0,0.5, 10);

        for (auto& [mapKeyId, emptyBool]: keyframesMap) {
       // for (auto& [mapKeyId, emptyBool]: projKey->keyframes) {
            mapKey = (*keyframes)[mapKeyId];
            if(keyframesToOptimize.find(mapKeyId) == keyframesToOptimize.end()){ // mapKey is not a keyframe to be optimized
                if(windowKeyframes.find(mapKeyId) == windowKeyframes.end()){ // If mapKey is not yet in the optimization window
                    windowKeyframes.insert({mapKeyId,mapKey});
                }

                if(Rrel.find(std::make_pair(mapKey,projKey)) == Rrel.end()){ // Update Rrel with a new keyframe correlation
                    Rrel[std::make_pair(mapKey,projKey)] = projKey->pose.Rcw * mapKey->pose.Rwc;
                    Rrel[std::make_pair(projKey,mapKey)] = Rrel[std::make_pair(mapKey,projKey)].inverse();
                }

                mapKey->getVisibleHgpIn(projKey,hgp_aux);
                for(typePt& pt: hgp_aux){
                    projection_temp.reset( new IDNav::Projection_hgp(
                            pt, mapKey, projKey,
                            &(Rrel[std::make_pair(mapKey,projKey)])));
                    projection_temp->computeResidualCov();
                    projection_temp->optRefPose = false;
                    projection_temp->optDepth = false;
                    hgpProjections_map.push_back(projection_temp);
                    hgpObservations[pt].push_back(projection_temp);
                }

                projKey->getVisibleHgpIn(mapKey,hgp_aux);
                for(typePt& pt: hgp_aux){
                    projection_temp.reset( new IDNav::Projection_hgp(
                            pt,  projKey, mapKey,
                            &(Rrel[std::make_pair(projKey,mapKey)])));
                    projection_temp->computeResidualCov();
                    projection_temp->getDepthObservation();
                    projection_temp->optProjPose = false;
                    hgpProjections_map.push_back(projection_temp);
                    hgpObservations[pt].push_back(projection_temp);
                }
            }
        }
    }
    numHgpProjections = hgpProjections.size();
    numHgpMapProjections = hgpProjections_map.size();
}

void IDNav::OptimizerCeres::setProjections_features(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << spaces + "        setProjections_features()"<< endl;
#endif
#ifdef ACTIVE_FEATURES
    // Reset variables
    featureObservations.clear();
    ftProjections.clear();
    ftProjections_map.clear();
    numFtProjections = 0;
    maxGeoNumFtProjections = 0;
    maxNumFtProjections = 0;
    numFtMapProjections = 0;

    // Get projections between keyframesToOptimize
    vecFt ft_aux{};
    typeProj_ft projection_temp{};
    for (auto& [hostKeyId, hostKey]: keyframesToOptimize) {
        for (auto& [projKeyId, projKey]: keyframesToOptimize) {
            if(hostKeyId == projKeyId) continue;
            if(hostKey->keyframes.find(projKeyId) == hostKey->keyframes.end()) continue; // If projKey is not connected to hostKey in the covisibility graph

            hostKey->getVisibleFeaturesIn(projKey,ft_aux);
            maxNumFtProjections += hostKey->features.size();
            maxGeoNumFtProjections += ft_aux.size();
            for(typeFt& ft: ft_aux){
                auto obs = projKey->observations.find(ft);
                projection_temp.reset( new IDNav::Projection_ft(
                        ft, hostKey, projKey,
                        &(Rrel[std::make_pair(hostKey,projKey)]),
                        obs->second));
                projection_temp->computeResidualCov();
                projection_temp->getDepthObservation();
                ftProjections.push_back(projection_temp);
                featureObservations[ft].push_back(projection_temp);
            }
        }
    }

    // Get projections between keyframesToOptimize and mapKeyframes
    typeFrame mapKey;
    for (auto& [projKeyId, projKey]: keyframesToOptimize) {
        unordered_map<size_t,Covisibility> keyframesMap_aux{};
        for (auto& [mapKeyId, cov]: projKey->keyframes){
            if(keyframesToOptimize.find(mapKeyId) == keyframesToOptimize.end()) { // mapKey is not a keyframe to be optimized
                keyframesMap_aux.insert({mapKeyId,cov});
            }
        }
        unordered_map<size_t,typeFrame> keyframesMap{};
        IDNav::TrackingFrontEnd::get_N_covisibleKeyframes(keyframesMap, projKey, (*keyframes) , keyframesMap_aux, 2.0,0.5, 10);

        for (auto& [mapKeyId, emptyBool]: keyframesMap) {
        //for (auto&[mapKeyId, emptyBool]: projKey->keyframes) {
            mapKey = (*keyframes)[mapKeyId];
            if(keyframesToOptimize.find(mapKeyId) == keyframesToOptimize.end()){ // mapKey is not a keyframe to be optimized
                if(windowKeyframes.find(mapKeyId) == windowKeyframes.end()) { // If mapKey is not yet in the optimization window
                    windowKeyframes.insert({mapKeyId,mapKey});
                }

                if(Rrel.find(std::make_pair(mapKey,projKey)) == Rrel.end()){ // Update Rrel with a new keyframe correlation
                    Rrel[std::make_pair(mapKey,projKey)] = projKey->pose.Rcw * mapKey->pose.Rwc;
                    Rrel[std::make_pair(projKey,mapKey)] = Rrel[std::make_pair(mapKey,projKey)].inverse();
                }

                mapKey->getVisibleFeaturesIn(projKey,ft_aux);
                for(typeFt& ft: ft_aux){
                    auto obs = projKey->observations.find(ft);
                    projection_temp.reset( new IDNav::Projection_ft(
                            ft, mapKey, projKey,
                            &(Rrel[std::make_pair(mapKey,projKey)]),
                            obs->second));
                    projection_temp->computeResidualCov();
                    projection_temp->optRefPose = false;
                    projection_temp->optDepth = false;
                    ftProjections_map.push_back(projection_temp);
                    featureObservations[ft].push_back(projection_temp);
                }

                projKey->getVisibleFeaturesIn(mapKey,ft_aux);
                for(typeFt& ft: ft_aux){
                    auto obs = mapKey->observations.find(ft);
                    projection_temp.reset( new IDNav::Projection_ft(
                            ft,  projKey, mapKey,
                            &(Rrel[std::make_pair(projKey,mapKey)]),
                            obs->second));
                    projection_temp->computeResidualCov();
                    projection_temp->getDepthObservation();
                    projection_temp->optProjPose = false;
                    ftProjections_map.push_back(projection_temp);
                    featureObservations[ft].push_back(projection_temp);
                }
            }
        }
    }
#endif
    numFtProjections = ftProjections.size();
    numFtMapProjections = ftProjections_map.size();
}

void IDNav::OptimizerCeres::setFixedKeyframes(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "    [OptimizerCeres] setFixedKeyframes()"<< RESET_COUT << endl;
#endif

    fixedKeyframes.clear();
    if(numMapProjections < minNumMapProjections){
        map<size_t,size_t> idsTemp{};
        for(auto& [idKey,keyframe]: keyframesToOptimize){
            idsTemp.insert({idKey,idKey});
#ifdef COUT_OPTIMIZER_PIPELINE
            cout  << spaces + "        optimize Keyframe id : "<< idKey << endl;
#endif
        }

        int numFixedKeyframes_ = numFixedKeyframes;
        if(numFixedKeyframes_ >= keyframesToOptimize.size())
            numFixedKeyframes_ = keyframesToOptimize.size() - 1;
        for(auto& [idKey,emptySize_t]: idsTemp){
            fixedKeyframes.insert({idsTemp[idKey],true});
#ifdef COUT_OPTIMIZER_PIPELINE
            cout  << spaces + "        fixed Keyframe id : "<< idsTemp[idKey]  << endl;
#endif
            if(fixedKeyframes.size() == numFixedKeyframes_)
                break;
        }

        // Update projections
        vecProj_hgp projectionsMap_hgp_tmp = hgpProjections;
        hgpProjections.clear();
        for(typeProj_hgp& proj: projectionsMap_hgp_tmp){
            if(fixedKeyframes.find(proj->projKeyId) != fixedKeyframes.end()){
                proj->optProjPose = false;
            }
            if(fixedKeyframes.find(proj->refKeyId) != fixedKeyframes.end()){
                proj->optRefPose = false;
            }
            if((proj->optProjPose)||(proj->optRefPose)){
                hgpProjections.push_back(proj);
            }
        }

        projectionsMap_hgp_tmp = hgpProjections_map;
        hgpProjections_map.clear();
        for(typeProj_hgp& proj: projectionsMap_hgp_tmp){
            if(proj->optProjPose){
                if(fixedKeyframes.find(proj->projKeyId) == fixedKeyframes.end()){
                    hgpProjections_map.push_back(proj);
                }
            }
            if(proj->optRefPose){
                if(fixedKeyframes.find(proj->refKeyId) == fixedKeyframes.end()){
                    hgpProjections_map.push_back(proj);
                }
            }
        }

        vecProj_ft projectionsMap_ft_tmp = ftProjections;
        ftProjections.clear();
        for(typeProj_ft& proj: projectionsMap_ft_tmp){
            if(fixedKeyframes.find(proj->projKeyId) != fixedKeyframes.end()){
                proj->optProjPose = false;
            }
            if(fixedKeyframes.find(proj->refKeyId) != fixedKeyframes.end()){
                proj->optRefPose = false;
            }
            if((proj->optProjPose)||(proj->optRefPose)){
                ftProjections.push_back(proj);
            }
        }

        projectionsMap_ft_tmp = ftProjections_map;
        ftProjections_map.clear();
        for(typeProj_ft& proj: projectionsMap_ft_tmp){
            if(proj->optRefPose){
                if(fixedKeyframes.find(proj->refKeyId) == fixedKeyframes.end()){
                    ftProjections_map.push_back(proj);
                }
            }
            if(proj->optProjPose){
                if(fixedKeyframes.find(proj->projKeyId) == fixedKeyframes.end()){
                    ftProjections_map.push_back(proj);
                }
            }
        }
    }
}

void IDNav::OptimizerCeres::setObservations(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "    [OptimizerCeres] setObservations()"<< RESET_COUT << endl;
#endif
    numHgpToOptimize = 0;
    for(auto& [pt, obs]: hgpObservations){
        if(obs.size() >= minNumObservations){
            ++numHgpToOptimize;
            for(typeProj_hgp & proj: obs)
                proj->optDepth = true;
        }
    }

    numFtToOptimize = 0;
    for(auto& [ft, obs]: featureObservations){
        if(obs.size() >= minNumObservations){
            ++numFtToOptimize;
            for(typeProj_ft & proj: obs)
                proj->optDepth = true;
        }
    }
}

void IDNav::OptimizerCeres::buildProblem(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "    [OptimizerCeres] buildProblem()"<< RESET_COUT << endl;
#endif
    addVariables();
    if(optimizationMode == "iterativeOptimization"){
        addResiduals_poseOptimization();
        addResiduals_depthOptimization();
        //addVariableConstraints();
    }
}

void IDNav::OptimizerCeres::addVariables(){
#ifdef COUT_OPTIMIZER_PIPELINE
    cout << BLUE_COUT << spaces + "        [OptimizerCeres] addVariables()"<< RESET_COUT << endl;
#endif
    addDepths();
    addPoses();
    addProblemOptions();
    addEvaluationCallback();
}

void IDNav::OptimizerCeres::addDepths() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "            [OptimizerCeres] addDepths()" << RESET_COUT << endl;
#endif
    deltaLambdaHgp.clear();
    deltaLambdaFeatures.clear();
    for(auto& [pt,projections]: hgpObservations){
        deltaLambdaHgp[pt] = pt->lambdaRef;
    }
    for(auto& [ft,projections]: featureObservations){
        deltaLambdaFeatures[ft] = ft->lambdaRef;
    }
}

void IDNav::OptimizerCeres::addPoses() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "            [OptimizerCeres] addPoses()" << RESET_COUT << endl;
#endif

    for(auto[index,deltaPose_i] : deltaPoses){
        delete [] deltaPose_i;
    }
    deltaPoses.clear();
    twc.clear();
    Rwc.clear();
    tcw.clear();
    Rcw.clear();
    deltaPhotoScalar.clear();
    deltaPhotoExpScalar.clear();
    deltaPhotoBias.clear();

    for(auto& [keyId,keyframe]: windowKeyframes){
        twc.insert({keyId,keyframe->pose.twc});
        Rwc.insert({keyId,keyframe->pose.Rwc});
        tcw.insert({keyId,keyframe->pose.tcw});
        Rcw.insert({keyId,keyframe->pose.Rcw});
        //deltaPhotoScalar.insert({keyId,keyframe->phScalar});
        //deltaPhotoExpScalar.insert({keyId,keyframe->phScalarExp});
        //deltaPhotoBias.insert({keyId,keyframe->phBias});
        deltaPhotoScalar.insert({keyId,0.0});
        deltaPhotoExpScalar.insert({keyId,1.0});
        deltaPhotoBias.insert({keyId,0.0});
        if(keyframesToOptimize.find(keyId) != keyframesToOptimize.end()){
            if (fixedKeyframes.find(keyId) == fixedKeyframes.end()){
                deltaPoses[keyId] = new dataType[6]{0.0,0.0,0.0,0.0,0.0,0.0};
                //deltaPhotoScalar[keyId] = 0.0;
                //deltaPhotoExpScalar[keyId] = 1.0;
                //deltaPhotoBias[keyId] = 0.0;
            }
        }
    }
}

void IDNav::OptimizerCeres::addProblemOptions() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "            [OptimizerCeres] addOptions()" << RESET_COUT << endl;
#endif
    problemOptions_pose.reset(new ceres::Problem::Options{});
    problemOptions_depth_photo.reset(new ceres::Problem::Options{});
    problemOptions_depth_reproj.reset(new ceres::Problem::Options{});
}

void IDNav::OptimizerCeres::addEvaluationCallback() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "            [OptimizerCeres] addEvaluationCallback()" << RESET_COUT << endl;
#endif
    auto *eval_function = new OptimizerCeres::evaluationCallback(
            &keyframesToOptimize,
            &deltaPoses,
            &tcw , &Rcw,
            &twc , &Rwc,
            &fixedKeyframes,
            &Rrel,
            &hgpObservations, &featureObservations,
            &deltaLambdaHgp,  &deltaLambdaFeatures,
            &deltaPhotoScalar, &deltaPhotoExpScalar, &deltaPhotoBias
    );
    problemOptions_pose->evaluation_callback = eval_function;
    problemOptions_depth_photo->evaluation_callback = eval_function;
    problemOptions_depth_reproj->evaluation_callback = eval_function;
}

void IDNav::OptimizerCeres::addResiduals_poseOptimization() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "        [OptimizerCeres] addResiduals_poseOptimization()" << RESET_COUT << endl;
#endif

    problem_pose.reset(new ceres::Problem{*problemOptions_pose});
    summary_pose.reset(new ceres::Solver::Summary{});

    for (typeProj_hgp &proj: hgpProjections) {
        if (proj->optRefPose && proj->optProjPose) {
            ceres::CostFunction *cost_function = new PhotoResidual_poseOpt(proj,
                                                                           &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                           &deltaPhotoExpScalar[proj->refKeyId],&deltaPhotoScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                           &deltaPhotoExpScalar[proj->projKeyId],&deltaPhotoScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
                                                                           );
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                           deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
        }else {
            if (proj->optRefPose) {
                ceres::CostFunction *cost_function = new PhotoResidual_poseOpt_ref(proj,
                                                                                   &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                                   &deltaPhotoExpScalar[proj->refKeyId],&deltaPhotoScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                                   &deltaPhotoExpScalar[proj->projKeyId],&deltaPhotoScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
                );
                problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                               deltaPoses[proj->refKeyId]);
            }
            if(proj->optProjPose) {
                ceres::CostFunction *cost_function = new PhotoResidual_poseOpt_proj(proj,
                                                                                    &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                                    &deltaPhotoExpScalar[proj->refKeyId],&deltaPhotoScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                                    &deltaPhotoExpScalar[proj->projKeyId],&deltaPhotoScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
                );
                problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                               deltaPoses[proj->projKeyId]);
            }
        }
    }

    for (typeProj_ft &proj: ftProjections) {
        if (proj->optRefPose && proj->optProjPose) {
            ceres::CostFunction *cost_function = new ReprojResidual_poseOpt(proj,
                                                                            &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                           deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
        }else {
            if (proj->optRefPose) {
                ceres::CostFunction *cost_function = new ReprojResidual_poseOpt_ref(proj,
                                                                                    &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
                problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                               deltaPoses[proj->refKeyId]);
            }
            if(proj->optProjPose) {
                ceres::CostFunction *cost_function = new ReprojResidual_poseOpt_proj(proj,
                                                                                     &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
                problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                               deltaPoses[proj->projKeyId]);
            }
        }
    }

    for (typeProj_hgp &proj: hgpProjections_map) {
        if (proj->optRefPose) {
            ceres::CostFunction *cost_function = new PhotoResidual_poseOpt_ref(proj,
                                                                               &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                               &deltaPhotoExpScalar[proj->refKeyId],&deltaPhotoScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                               &deltaPhotoExpScalar[proj->projKeyId],&deltaPhotoScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
            );
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                           deltaPoses[proj->refKeyId]);
        }
        if (proj->optProjPose) {
            ceres::CostFunction *cost_function = new PhotoResidual_poseOpt_proj(proj,
                                                                                &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                                &deltaPhotoExpScalar[proj->refKeyId],&deltaPhotoScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                                &deltaPhotoExpScalar[proj->projKeyId],&deltaPhotoScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
            );
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                           deltaPoses[proj->projKeyId]);
        }
    }

    for (typeProj_ft &proj: ftProjections_map) {
        if (proj->optRefPose) {
            ceres::CostFunction *cost_function = new ReprojResidual_poseOpt_ref(proj,
                                                                                &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                           deltaPoses[proj->refKeyId]);
        }
        if (proj->optProjPose) {
            ceres::CostFunction *cost_function = new ReprojResidual_poseOpt_proj(proj,
                                                                                 &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            problem_pose->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                           deltaPoses[proj->projKeyId]);
        }
    }
}

void IDNav::OptimizerCeres::addVariableConstraints(){
    for(auto& [keyId,keyframe]: windowKeyframes){
        if(keyframesToOptimize.find(keyId) != keyframesToOptimize.end()){
            if (fixedKeyframes.find(keyId) == fixedKeyframes.end()){
                deltaPoses[keyId] = new dataType[6]{0.0,0.0,0.0,0.0,0.0,0.0};
                deltaPhotoScalar[keyId] = 0.0;
                deltaPhotoExpScalar[keyId] = 1.0;
                deltaPhotoBias[keyId] = 0.0;

                /*problem_pose->SetParameterLowerBound(&deltaPhotoScalar[keyId], 0, -2.5);
                problem_pose->SetParameterUpperBound(&deltaPhotoScalar[keyId], 0, 2.5);
                problem_pose->SetParameterLowerBound(&deltaPhotoBias[keyId], 0, -1.0);
                problem_pose->SetParameterUpperBound(&deltaPhotoBias[keyId], 0, 1.0);*/

            }
        }
    }
}

void IDNav::OptimizerCeres::addResiduals_depthOptimization() {
#ifdef COUT_OPTIMIZER_PIPELINE
    cout  << BLUE_COUT << spaces + "        [OptimizerCeres] addResiduals_depthOptimization()" << RESET_COUT << endl;
#endif

    problem_depth_photo.reset(new ceres::Problem{*problemOptions_depth_photo});
    summary_depth_photo.reset(new ceres::Solver::Summary{});

    problem_depth_reproj.reset(new ceres::Problem{*problemOptions_depth_reproj});
    summary_depth_reproj.reset(new ceres::Solver::Summary{});

    covariance_blocks_photo.clear();
    covariance_blocks_reproj.clear();

    dataType lambdaRefStd;
    dataType minLambda;
    dataType maxLambda;

    for(auto& [pt,projections]: hgpObservations){
        if(!projections[0]->optDepth) continue;
        for(typeProj_hgp& proj: projections){
            ceres::CostFunction *cost_function = new PhotoResidual_depthOpt(proj,
                                                                            &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]),
                                                                            &deltaPhotoExpScalar[proj->refKeyId], &deltaPhotoBias[proj->refKeyId],
                                                                            &deltaPhotoExpScalar[proj->projKeyId], &deltaPhotoBias[proj->projKeyId]
                                                                            );
            problem_depth_photo->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentPhoto),
                                                  &deltaLambdaHgp[pt]);
        }
        covariance_blocks_photo.emplace_back(make_pair(&deltaLambdaHgp[pt], &deltaLambdaHgp[pt]));
        problem_depth_photo->AddParameterBlock(&deltaLambdaHgp[pt], 1);

        lambdaRefStd = 3.0*sqrt(pt->lambdaRefCov);
        minLambda = pt->lambdaRef - lambdaRefStd;
        maxLambda = pt->lambdaRef + lambdaRefStd;
        //problem_depth_photo->SetParameterLowerBound(&deltaLambdaHgp[pt],0, minLambda);
        //problem_depth_photo->SetParameterUpperBound(&deltaLambdaHgp[pt],0, maxLambda);
    }

    for(auto& [ft,projections]: featureObservations){
        if(!projections[0]->optDepth) continue;
        for(typeProj_ft& proj: projections){
            ceres::CostFunction *cost_function = new ReprojResidual_depthOpt(proj,
                                                                             &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            problem_depth_reproj->AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                                   &deltaLambdaFeatures[ft]);
        }
        covariance_blocks_reproj.emplace_back(&deltaLambdaFeatures[ft], &deltaLambdaFeatures[ft]);
        problem_depth_reproj->AddParameterBlock(&deltaLambdaFeatures[ft], 1);

        lambdaRefStd = 3.0*sqrt(ft->lambdaRefCov);
        minLambda = ft->lambdaRef - lambdaRefStd;
        maxLambda = ft->lambdaRef + lambdaRefStd;
        //problem_depth_reproj->SetParameterLowerBound(&deltaLambdaFeatures[ft],0, minLambda);
        //problem_depth_reproj->SetParameterUpperBound(&deltaLambdaFeatures[ft],0, maxLambda);
    }
}

void IDNav::OptimizerCeres::setProblemOptions() {
    solverOptions_pose.reset(new ceres::Solver::Options{});
    solverOptions_pose->linear_solver_type =  ceres::SPARSE_NORMAL_CHOLESKY;
    solverOptions_pose->minimizer_type = ceres::TRUST_REGION;
    solverOptions_pose->minimizer_progress_to_stdout = false;
    solverOptions_pose->ceres::Solver::Options::update_state_every_iteration = true;
    solverOptions_pose->use_inner_iterations = false;
    solverOptions_pose->check_gradients = false;
    solverOptions_pose->num_threads = numThreads;
    solverOptions_pose->use_nonmonotonic_steps = true;

    solverOptions_depth_photo.reset(new ceres::Solver::Options{});
    solverOptions_depth_photo->minimizer_progress_to_stdout = false;
    solverOptions_depth_photo->ceres::Solver::Options::update_state_every_iteration = true;
    solverOptions_depth_photo->use_inner_iterations = false;
    solverOptions_depth_photo->check_gradients = false;
    solverOptions_depth_photo->num_threads = numThreads;

    solverOptions_depth_reproj.reset(new ceres::Solver::Options{});
    solverOptions_depth_reproj->minimizer_progress_to_stdout = false;
    solverOptions_depth_reproj->ceres::Solver::Options::update_state_every_iteration = true;
    solverOptions_depth_reproj->use_inner_iterations = false;
    solverOptions_depth_reproj->check_gradients = false;
    solverOptions_depth_reproj->num_threads = numThreads;
}

void IDNav::OptimizerCeres::set_tstudent2_photo(const dataType &p_) {
    tstudent2ThPhoto = tstudent2(PATCH_SIZE, p_);
    tstudentPhoto = sqrt(tstudent2ThPhoto);
}

void IDNav::OptimizerCeres::set_tstudent2_reproj(const dataType &p_) {
    tstudent2ThReproj = chi2Value(2, p_);
    tstudentReproj = sqrt(tstudent2ThReproj);
}

void IDNav::OptimizerCeres::set_tstudent2(const dataType &p_) {
    set_tstudent2_photo(p_);
    set_tstudent2_reproj(p_);
}

void IDNav::OptimizerCeres::setOptimizeDepths(const bool& optimizeDepths_) {
    optimizeDepths = optimizeDepths_;
}

void IDNav::OptimizerCeres::set_tstudent2_p(const dataType& tstudent2_p_){
    tstudent2_p = tstudent2_p_;
    set_tstudent2(tstudent2_p);
}

void IDNav::OptimizerCeres::updateGlobalObservations(std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>& obs_hgp_,
                                                     std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>& obs_features_){
    if(!beginOptimization) return;
    {
        for(typeProj_hgp & proj: hgpProjections){
            proj->refKey->uv_2_XYZ(proj->pt);
            proj->projKey->XYZ_2_uv(proj->pt);

            Point_<dataType> keyPt{};
            keyPt.x = proj->pt->u[0];
            keyPt.y = proj->pt->v[0];
            auto it_pt = obs_hgp_.find(proj->pt);
            if(it_pt != obs_hgp_.end()){
                auto it_key = it_pt->second.find(proj->projKeyId);
                if(it_key != it_pt->second.end()){
                    it_key->second = keyPt;
                }else{
                    it_pt->second.insert({proj->projKeyId,keyPt});
                }
            }else{
                std::unordered_map<size_t,Point_<dataType>> proj_;
                proj_.insert({proj->projKeyId,keyPt});
                obs_hgp_.insert({proj->pt,proj_});
            }
        }

        for(typeProj_hgp & proj: hgpProjections_map){
            proj->refKey->uv_2_XYZ(proj->pt);
            proj->projKey->XYZ_2_uv(proj->pt);

            Point_<dataType> keyPt{};
            keyPt.x = proj->pt->u[0];
            keyPt.y = proj->pt->v[0];
            auto it_pt = obs_hgp_.find(proj->pt);
            if(it_pt != obs_hgp_.end()){
                auto it_key = it_pt->second.find(proj->projKeyId);
                if(it_key != it_pt->second.end()){
                    it_key->second = keyPt;
                }else{
                    it_pt->second.insert({proj->projKeyId,keyPt});
                }
            }else{
                std::unordered_map<size_t,Point_<dataType>> proj_;
                proj_.insert({proj->projKeyId,keyPt});
                obs_hgp_.insert({proj->pt,proj_});
            }
        }
    }

    {
        for(typeProj_ft & proj: ftProjections){
            proj->refKey->uv_2_XYZ(proj->ft);
            proj->projKey->XYZ_2_uv(proj->ft);

            Point_<dataType> keyPt{};
            keyPt.x = proj->ft->u[0];
            keyPt.y = proj->ft->v[0];
            auto it_pt = obs_features_.find(proj->ft);
            if(it_pt != obs_features_.end()){
                auto it_key = it_pt->second.find(proj->projKeyId);
                if(it_key != it_pt->second.end()){
                    it_key->second = keyPt;
                }else{
                    it_pt->second.insert({proj->projKeyId,keyPt});
                }
            }else{
                std::unordered_map<size_t,Point_<dataType>> proj_;
                proj_.insert({proj->projKeyId,keyPt});
                obs_features_.insert({proj->ft,proj_});
            }
        }

        for(typeProj_ft & proj: ftProjections_map){
            proj->refKey->uv_2_XYZ(proj->ft);
            proj->projKey->XYZ_2_uv(proj->ft);

            Point_<dataType> keyPt{};
            keyPt.x = proj->ft->u[0];
            keyPt.y = proj->ft->v[0];
            auto it_pt = obs_features_.find(proj->ft);
            if(it_pt != obs_features_.end()){
                auto it_key = it_pt->second.find(proj->projKeyId);
                if(it_key != it_pt->second.end()){
                    it_key->second = keyPt;
                }else{
                    it_pt->second.insert({proj->projKeyId,keyPt});
                }
            }else{
                std::unordered_map<size_t,Point_<dataType>> proj_;
                proj_.insert({proj->projKeyId,keyPt});
                obs_features_.insert({proj->ft,proj_});
            }
        }
    }
}