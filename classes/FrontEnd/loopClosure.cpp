#include "../../include/trackingFrontEnd.h"

std::mutex triggerLoopClosureMutex;
void IDNav::TrackingFrontEnd::triggerLoopClosure(std::vector<LCD::Result>& inconsistentLoops_){
    ///////////////////////////////////////////////////////
    const std::lock_guard<std::mutex> lock(triggerLoopClosureMutex);
    stopWindowOptimization = true;
    while((!windowOptimizationFinished)||(prepareWindowOptimizationInProgress)) usleep(1000);
    ///////////////////////////////////////////////////////
    // 1) Pose graph
    cout << "    Pose Graph Optimization " << endl;

    // CHECK RESIDUALS 0
    globalOptimizer->resetOptimization();
    globalOptimizer->prepareOptimization(*keyframes,"iterativeOptimization");
    globalOptimizer->optimize(1,1.0);
    wf("residuals0_photo");

    globalOptimizer->resetOptimization();
    globalOptimizer->geoBA(obs_hgp,obs_features);
    wf("residuals0_geo");

    // POSE GRAPH
    poseGraphOptimizer->optimizePoseGraph(keyframes);
    cout << " " << endl;
    wf("poseGraph");

    // CHECK RESIDUALS 1
    globalOptimizer->resetOptimization();
    globalOptimizer->prepareOptimization(*keyframes,"iterativeOptimization");
    globalOptimizer->optimize(1,1.0);
    wf("residuals1_photo");

    globalOptimizer->resetOptimization();
    globalOptimizer->geoBA(obs_hgp,obs_features);
    wf("residuals1_geo");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 2) Landmark-like pose BA

    cout << "    Landmark-like pose BA " << endl;
    globalOptimizer->geoBA(obs_hgp,obs_features);
    globalOptimizer->updateVariables();
    cout << " " << endl;
    wf("landmark");

    // 3) Update connections
    /*for(LCD::Result& loopResult: inconsistentLoops_){
        (*keyframes)[loopResult.database_id]->keyframes.erase(loopResult.query_id);
        (*keyframes)[loopResult.query_id]->keyframes.erase(loopResult.database_id);
        addConsistentLoop(loopResult);
    }*/

    // 4) Final BA
    cout << "    Final BA " << endl;
    globalOptimizer->resetOptimization();
    globalOptimizer->prepareOptimization(*keyframes,"iterativeOptimization");
    globalOptimizer->optimize(200,1000.0);
    globalOptimizer->updateVariables();
    obs_hgp.clear();
    obs_features.clear();
    globalOptimizer->updateGlobalObservations(obs_hgp,obs_features);

    cout << " " << endl;
    wf("BA");

    // 5) Final updates
    setRelativePoseToReferenceKeyframe();


    ///////////////////////////////////////////////////////
    stopWindowOptimization = false;
    usleep(1000);
    ///////////////////////////////////////////////////////

    //size_t database_id = loopClosure_.database_id;
    //size_t query_id = loopClosure_.query_id;
    //typeFrame dKeyframe = (*keyframes)[database_id];
    //typeFrame qKeyframe = (*keyframes)[query_id];

    //dataType imageError = getLoopImageError(dKeyframe,qKeyframe,loopClosure_);
    //cout  << "        imageError 0 = " << imageError  << endl;

    /*mat4 Td{mat4::Identity()};
    Td.block<3,3>(0,0) = dKeyframe->pose.Rwc;
    Td.block<3,1>(0,3) = dKeyframe->pose.twc;
    mat4 Tji = Td*loopClosure_.tf_q_db.cast<double>()*Td.inverse();
    dKeyframe->drawPoints(qKeyframe);*/

   // cout << Tji << endl;

    //mat4 Tji_2{mat4::Identity()};
    //Tji_2.block<3,3>(0,0) = qKeyframe->pose.Rwc * dKeyframe->pose.Rcw;
    //Tji_2.block<3,1>(0,3) = qKeyframe->pose.Rwc * dKeyframe->pose.tcw + qKeyframe->pose.twc;
    //cout << Tji_2 << endl;
    //terminate();
    //connectFrames(dKeyframe,qKeyframe,Tji);
    //poseGraphOptimizer->optimizePoseGraph(keyframes,database_id,query_id);
    //cout  << "        imageError 1 = " << getLoopImageError(dKeyframe,qKeyframe,loopClosure_)  << endl;

    //globalOptimizer->geoBA(obs_hgp,obs_features);
    //globalOptimizer->updateVariables();
    //cout  << "        imageError 2 = " << getLoopImageError(dKeyframe,qKeyframe,loopClosure_)  << endl;


    //wf(".");
    //usleep(100000);
    //wf(".");
    /*for(auto& [idKey, cov]: dKeyframe->keyframes){
        connectFramesConditioned(qKeyframe, (*keyframes)[idKey]);
    }*/
    /*for(auto& [idKey, cov]: qKeyframe->keyframes){
        connectFramesConditioned(dKeyframe, (*keyframes)[idKey]);
    }*/

    /*globalOptimizer->resetOptimization();
    globalOptimizer->prepareOptimization(*keyframes,"iterativeOptimization");
    globalOptimizer->optimize(100);
    globalOptimizer->updateVariables();
    globalOptimizer->updateGlobalObservations(obs_hgp,obs_features);*/

    //cout  << "        imageError 3 = " << getLoopImageError(dKeyframe,qKeyframe,loopClosure_)  << endl;
    //setRelativePoseToReferenceKeyframe();

//#ifdef COUT_COMPLETE_PIPELINE
    //cout  << "        imageError = " << imageError  << endl;
//#endif

}


bool IDNav::TrackingFrontEnd::isLoopConsistent(LCD::Result& loopResult_) {
//#ifdef COUT_COMPLETE_PIPELINE
    cout << BLUE_COUT << "    [TrackingFrontEnd] isLoopConsistent(): " << RESET_COUT << endl;
    cout  << "        database_id = "<< loopResult_.database_id  << endl;
    cout  << "        query_id = " << loopResult_.query_id  << endl;
//#endif

    size_t database_id = loopResult_.database_id;
    size_t query_id = loopResult_.query_id;
    typeFrame dKeyframe = (*keyframes)[database_id];
    typeFrame qKeyframe = (*keyframes)[query_id];

    cout << "    First question: were keyframes connected ? = " << (dKeyframe->keyframes.find(query_id) != dKeyframe->keyframes.end()) << " = " <<
                                                                (qKeyframe->keyframes.find(database_id) != qKeyframe->keyframes.end()) << endl;

    //mat4 Td{mat4::Identity()};
    //Td.block<3,3>(0,0) = dKeyframe->pose.Rwc;
    //Td.block<3,1>(0,3) = dKeyframe->pose.twc;
    //mat4 relPoseLcd = Td*loopResult_.tf_q_db.cast<double>()*Td.inverse();
    //mat4 relPose = dKeyframe->pose.relMovement(qKeyframe->pose);
    //mat4 relPoseError = relPoseLcd*relPose.inverse();*/

    dataType imageError = getLoopImageError(dKeyframe, qKeyframe, loopResult_);

//#ifdef COUT_COMPLETE_PIPELINE
    //cout  << "        imageError = " << imageError  << endl;
//#endif
    cout << "    Third question: how big is the initial bias  = " << imageError << " pixels = " << endl;
    return (imageError < 1.0);
    //return true;
}

IDNav::dataType IDNav::TrackingFrontEnd::getLoopImageError(typeFrame& dKeyframe_, typeFrame& qKeyframe_, LCD::Result& loopResult_) {

    mat4 Tr = loopResult_.tf_q_db.cast<double>().inverse();
    mat3 R = Tr.block<3,3>(0,0);
    vec3 t = Tr.block<3,1>(0,3);

    vecPt hgp_aux{};
    vecFt features_aux{};
    dKeyframe_->getVisiblePointsIn(qKeyframe_,hgp_aux,features_aux);

    cout << "    Second question: how many points to they share ? hgp = " << hgp_aux.size() << " , features = " << features_aux.size() << endl;

    dataType imageError{0.0};
    dataType u{},v{};
    dataType fx{dKeyframe_->cam->getFocalLengthX()},
                fy{dKeyframe_->cam->getFocalLengthY()} ,
                cx{dKeyframe_->cam->getCentrePointX()} ,
                cy{dKeyframe_->cam->getCentrePointY()};

    Point_<dataType> keyPt{};
    for(typePt& pt: hgp_aux){
        pt->project(u,v,t,R,fx,fy,cx,cy);
        keyPt.x = u;
        keyPt.y = v;
        imageError += sqrt(pow(pt->u[0] - u,2) + pow(pt->v[0] - v,2));
        obs_hgp[pt].insert({qKeyframe_->keyId,keyPt});
    }
    for(typeFt& ft: features_aux){
        ft->project(u,v,t,R,fx,fy,cx,cy);
        keyPt.x = u;
        keyPt.y = v;
        imageError += sqrt(pow(ft->u[0] - u,2) + pow(ft->v[0] - v,2));
        obs_features[ft].insert({qKeyframe_->keyId,keyPt});
    }
    imageError /= dataType(hgp_aux.size() + features_aux.size());

    return imageError;
}

void IDNav::TrackingFrontEnd::addInconsistentLoop(LCD::Result& inconsistentLoop_){
//#ifdef COUT_COMPLETE_PIPELINE
    cout << BLUE_COUT << "    [TrackingFrontEnd] addInconsistentLoop(): " << RESET_COUT << endl;
    cout  << "        database_id = "<< inconsistentLoop_.database_id  << endl;
    cout  << "        query_id = "   << inconsistentLoop_.query_id  << endl;
//#endif

    size_t database_id = inconsistentLoop_.database_id;
    size_t query_id = inconsistentLoop_.query_id;
    typeFrame dKeyframe = (*keyframes)[database_id];
    typeFrame qKeyframe = (*keyframes)[query_id];

    cout << "        Make connectiion " << endl;
    mat4 Td{mat4::Identity()};
    Td.block<3,3>(0,0) = dKeyframe->pose.Rwc;
    Td.block<3,1>(0,3) = dKeyframe->pose.twc;
    mat4 relPoseLcd = Td*inconsistentLoop_.tf_q_db.cast<double>()*Td.inverse();
    mat4 relPose = dKeyframe->pose.relMovement(qKeyframe->pose);
    mat4 relPoseError = relPoseLcd*relPose.inverse();
    cout << "relPoseLcd = º\n" << relPoseLcd << endl;
    cout << "relPose = º\n" << relPose << endl;
    cout << "relPoseError = º\n" << relPoseError << endl;

    connectFrames(dKeyframe,qKeyframe,relPoseLcd);

}

void IDNav::TrackingFrontEnd::addConsistentLoop(LCD::Result& consistentLoop_){
//#ifdef COUT_COMPLETE_PIPELINE
    cout << BLUE_COUT << "    [TrackingFrontEnd] addConsistentLoop(): " << RESET_COUT << endl;
    cout  << "        database_id = "<< consistentLoop_.database_id  << endl;
    cout  << "        query_id = " << consistentLoop_.query_id  << endl;
//#endif

    size_t database_id = consistentLoop_.database_id;
    size_t query_id = consistentLoop_.query_id;
    typeFrame dKeyframe = (*keyframes)[database_id];
    typeFrame qKeyframe = (*keyframes)[query_id];

    cout << "        Make all conections " << endl;

    connectFrames(dKeyframe, qKeyframe);
    for(auto& [idKey, cov]: dKeyframe->keyframes){
        connectFramesConditioned(qKeyframe, (*keyframes)[idKey]);
    }
    for(auto& [idKey, cov]: qKeyframe->keyframes){
        connectFramesConditioned(dKeyframe, (*keyframes)[idKey]);
    }
}

void IDNav::TrackingFrontEnd::triggerLoops() {
#ifdef ACTIVE_LC
    std::vector<LCD::Result> results;
    std::vector<LCD::Result> inconsistentLoops;
    bool loopNotConsistent = false;
    if (lcd->loopDetected(results)) {  // returns true if internal thread finished eval of the queue
        for(LCD::Result& loopResult: results){
            if(!isLoopConsistent(loopResult)){
                addInconsistentLoop(loopResult);
                inconsistentLoops.push_back(loopResult);
                loopNotConsistent = true;

            }else{
                addConsistentLoop(loopResult);
            }
        }
    }

    if(loopNotConsistent){
        triggerLoopClosure(inconsistentLoops);
        triggeredLoop = true;
    }
#endif

#ifdef LC
    static bool first{true};
    static vector<size_t> d_id{};
    static vector<size_t> q_id{};
    static vector<mat4> Tji_vector{};

    int centerRadius = (2*640/640);
    int centerThickness = 2*640/640;
    cv::Scalar color_features_tracked = cv::Scalar(255, 0, 0);
    cv::Scalar color_features_tracked_ric = cv::Scalar(0, 0, 255);
    cv::Scalar color_features_tracked_fin = cv::Scalar(0, 255, 255);

    std::vector<LCD::Result> results;
    if (lcd->loopDetected(results)) {  // returns true if internal thread finished eval of the queue
// do something with the results in the user thread
        for (int iResults{0}; iResults < results.size(); ++iResults) {
            size_t database_id = results[iResults].database_id;
            size_t query_id = results[iResults].query_id;
            if(query_id - database_id < 50) continue;
            //cout << " database_id = " << results[iResults].database_id << endl;
            //cout << " query_id = " << results[iResults].query_id << endl;
            typeFrame dKeyframe = (*keyframes)[results[iResults].database_id];
            typeFrame qKeyframe = (*keyframes)[results[iResults].query_id];

            mat4 Td{mat4::Identity()};
            Td.block<3,3>(0,0) = dKeyframe->pose.Rwc;
            Td.block<3,1>(0,3) = dKeyframe->pose.twc;

            mat4 relPoseRic = Td*results[iResults].tf_q_db.cast<double>()*Td.inverse();

            mat4 relPose = ((*keyframes)[results[iResults].database_id])->pose.relMovement(
                    ((*keyframes)[results[iResults].query_id])->pose);
            double normD = (relPose.block<3, 1>(0, 3) - relPoseRic.block<3, 1>(0, 3)).norm();


            /*cout << ((*keyframes)[results[iResults].database_id])->pose.relMovement(
                    ((*keyframes)[results[iResults].query_id])->pose) << endl;*/

            //cout << "NORM = " << normD << endl;
            //out << " RICCARDOS tf = " << endl;
            //cout << relPoseRic << endl;
            //cout << " OLD tf = " << endl;
            //cout << relPose << endl;

            mat4 Tq = relPose*Td;
            qKeyframe->pose.print();
            mat4 Tq_ric = relPoseRic*Td;

            /*Mat trackedImg{};
            cv::cvtColor(qKeyframe->grayImgG[0],trackedImg, cv::COLOR_GRAY2RGB);

            vecPt hgp_aux{};
            vecFt ft_aux{};
            dKeyframe->getVisiblePointsIn(qKeyframe,hgp_aux,ft_aux);
            for(typePt& pt: hgp_aux){
                Point pt0 = Point(pt->u[0], pt->v[0]);
                circle(trackedImg, pt0, centerRadius, color_features_tracked, centerThickness);
            }
            namedWindow("trackedFrame", cv::WINDOW_NORMAL);
            cv::resizeWindow("trackedFrame", 2*640,2*480);
            imshow("trackedFrame", trackedImg);    // Show our image inside it.*/
            /*
            Mat trackedImg2{};
            cv::cvtColor(qKeyframe->grayImgG[0],trackedImg2, cv::COLOR_GRAY2RGB);

            qKeyframe->pose.set_T_wc(Tq_ric);
            dKeyframe->getVisiblePointsIn(qKeyframe,hgp_aux,ft_aux);
            for(typePt& pt: hgp_aux){
                Point pt0 = Point(pt->u[0], pt->v[0]);
                circle(trackedImg2, pt0, centerRadius, color_features_tracked_ric, centerThickness);
            }
            namedWindow("trackedFrame2", cv::WINDOW_NORMAL);
            cv::resizeWindow("trackedFrame2", 2*640,2*480);
            imshow("trackedFrame2", trackedImg2);    // Show our image inside it.

            qKeyframe->pose.set_T_wc(Tq);
            dKeyframe->getVisiblePointsIn(qKeyframe,hgp_aux,ft_aux);
*/
            qKeyframe->pose.set_T_wc(Tq_ric);
            mat4 Tji;

            vecFrame keyframes_aux{};
            for(auto& [keyId,cov]: dKeyframe->keyframes){
                keyframes_aux.push_back((*keyframes)[keyId]);
            }
            ria->relocalizeFrame(qKeyframe,keyframes_aux,dKeyframe, qKeyframe->pose,speed,speedCovInv,Tji);

            /*Mat trackedImg3{};
            cv::cvtColor(qKeyframe->grayImgG[0],trackedImg3, cv::COLOR_GRAY2RGB);
            dKeyframe->getVisiblePointsIn(qKeyframe,hgp_aux,ft_aux);
            for(typePt& pt: hgp_aux){
                Point pt0 = Point(pt->u[0], pt->v[0]);
                circle(trackedImg3, pt0, centerRadius, color_features_tracked_ric, centerThickness);
            }
            namedWindow("trackedFrame3", cv::WINDOW_NORMAL);
            cv::resizeWindow("trackedFrame3", 2*640,2*480);
            imshow("trackedFrame3", trackedImg3);    // Show our image inside it.

            cout << "Tji = \n"<< Tji << endl;*/

            qKeyframe->pose.set_T_wc(Tq);

            if(ria->isTrackingSuccessful()){
                //d_id.push_back(database_id);
                //q_id.push_back(query_id);
                //Tji_vector.push_back(Tji);
                //if(d_id.size() > 3){
                    //for(int iMatch{}; iMatch < d_id.size(); ++ iMatch){
                        //dKeyframe = (*keyframes)[d_id[iMatch]];
                        //qKeyframe = (*keyframes)[q_id[iMatch]];
                        //Tji = Tji_vector[iMatch];
                        conectFrames(dKeyframe,qKeyframe, Tji);
                        for(auto& [idKey_relocalize,cov]: qKeyframe->keyframes){
                            mat4 Tki = cov.Tji * Tji;
                            conectFrames(dKeyframe, (*keyframes)[idKey_relocalize],Tki);
                        }
                    //}
                    poseGraphOptimizer->optimizePoseGraph(keyframes,database_id,query_id);
                    //BA->resetOptimization();
                    //BA->prepareOptimization(*keyframes,"iterativeOptimization");
                    //BA->optimize(100);
                    d_id.clear();
                    q_id.clear();
                    Tji_vector.clear();

                    /*Mat trackedImg4{};
                    cv::cvtColor(qKeyframe->grayImgG[0],trackedImg4, cv::COLOR_GRAY2RGB);
                    dKeyframe->getVisiblePointsIn(qKeyframe,hgp_aux,ft_aux);
                    for(typePt& pt: hgp_aux){
                        Point pt0 = Point(pt->u[0], pt->v[0]);
                        circle(trackedImg4, pt0, centerRadius, color_features_tracked_fin, centerThickness);
                    }
                    namedWindow("trackedFrame4", cv::WINDOW_NORMAL);
                    cv::resizeWindow("trackedFrame4", 2*640,2*480);
                    imshow("trackedFrame4", trackedImg4);    // Show our image inside it.
                    cv::waitKey(0);

                    cout << " RICCARDOS tf = " << endl;
                    cout << relPoseRic << endl;
                    cout << " OLD tf = " << endl;
                    cout << relPose << endl;
                    cout << "NORM = " << normD << endl;
                    cout << "Tji = \n"<< Tji << endl;
                    relPose = dKeyframe->pose.relMovement(qKeyframe->pose);
                    cout << "relPose final = \n"<< relPose << endl;
                    normD = (relPose.block<3, 1>(0, 3) - relPoseRic.block<3, 1>(0, 3)).norm();
                    cout << "NORM = " << normD << endl;
*/
                //cv::waitKey(0);
                //}
            }

            //terminate();
            //wf(".");
            /*if( refKeyframe->keyId != results[iResults].query_id) return;
            if(refKeyframe->keyframes.find((*keyframes)[results[iResults].query_id]) != refKeyframe->keyframes.end()) return;

            //wf(".");
            typeFrame keyframeToRelocalize = (*keyframes)[results[iResults].query_id];
            mat4 Twc_1{mat4::Identity()};
            Twc_1.block<3,3>(0,0) = (*keyframes)[results[iResults].database_id]->pose.Rwc;
            Twc_1.block<3,1>(0,3) = (*keyframes)[results[iResults].database_id]->pose.twc;

            cout << "Twc_1\n = " << Twc_1 << endl;

            mat4 Twc_2 = mat4_float_toDataType(results[iResults].tf_q_db)*Twc_1;
            mat3 Rwc_2 = Twc_2.block<3,3>(0,0);
            vec3 twc_2 = Twc_2.block<3,1>(0,3);

            cout << "Twc_2\n = " << Twc_2 << endl;

            keyframeToRelocalize->pose.print();
            //wf(".");
            Pose pose0;
            pose0.copyStampedPoseFrom(keyframeToRelocalize->pose);
            Pose pose;
            pose.set_T_wc(twc_2,Rwc_2);
            mat4 Tji;
            ria->relocalizeFrame(keyframeToRelocalize,keyframesForTracking,refKeyframe, pose,speed,speedCovInv,Tji);
            keyframeToRelocalize->setPose(pose0);
            if(ria->isTrackingSuccessful()){
                conectFrames(refKeyframe,keyframeToRelocalize, Tji);
                for(auto& [keyframe_relocalize,Tkj]: keyframeToRelocalize->keyframes){
                    mat4 Tki = Tkj * Tji;
                    conectFrames(refKeyframe, keyframe_relocalize,Tki);
                }
                poseGraphOptimizer->optimizePoseGraph(keyframes);
                //BA->resetOptimization();
                //BA->prepareOptimization(*keyframes,"iterativeOptimization");
                //BA->optimize(100);
                cout << "distance 3 = "<< refKeyframe->distanceToFrame(keyframeToRelocalize) << endl;
            }*/
        }
    }
#endif
}

void IDNav::TrackingFrontEnd::addKeyframeToLcd(typeFrame& newKeyframe) {
#ifdef ACTIVE_LC
    // Create ptr to keyframe
    mat4 T = mat4::Identity();
    T.block<3,3>(0,0) = newKeyframe->pose.Rwc;
    T.block<3,1>(0,3) = newKeyframe->pose.twc;

    mat4_float tf0 = T.cast<float>();
    size_t keyId = newKeyframe->keyId;
    Mat distCoeffs = Mat::zeros(1,5,CV_32F);
    std::shared_ptr<LCD::Keyframe> p_kf(new LCD::Keyframe(newKeyframe->grayImgG[0], newKeyframe->cam->cameraMatrixUndist, distCoeffs,
                                                          tf0, keyId)); // optionally, add a "true" as final argument to compute ORBs

    for (typeFt& ft: newKeyframe->featuresForLC){
        p_kf->addPoint(ft->keyPtRef.pt, ft->descriptor, float(1.0/ft->lambdaRef));
    }

    if (!newKeyframe->featuresForLC.empty()){
        std::list<size_t> no_match_list{};
        no_match_list.push_back(newKeyframe->keyId);
        for(auto& [keyId_,empty]: newKeyframe->keyframes){
            no_match_list.push_back(keyId_);
        }
        lcd->parseKeyframe(p_kf,no_match_list);
        //lcd->parseKeyframe(p_kf);
    }
#endif
}