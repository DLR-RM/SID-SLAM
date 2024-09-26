//
// Created by font_al on 1/11/19.
//

#ifndef IDNAV_SID_SLAM_H
#define IDNAV_SID_SLAM_H

#include "trackingFrontEnd.h"
#include "backEnd.h"
#include "Visualizer.h"

namespace IDNav{

    //void sidSlamTracking(IDNav::SID_SLAM* sidSlam_);
    //[[noreturn]] void sidSlamWindowOptimizationRun(IDNav::SID_SLAM* idSlam);
    //void statisticVisualizerRun(IDNav::StatisticVisualizer* statisticVisualizer);

    //void getRGBAndDepthImage(Mat& rgbImg_, dataType& grayImgTs ,Mat& depthImg ,dataType& validDepth,Mat& maskImg,IDNav::SID_SLAM* idSlam, const size_t& id);


    class SID_SLAM {
    public:

        // Pipeline flags
        bool initializedSystem{false};
        bool resetSystem{false};
        bool killSystem{false};
        bool rosActive{false};

        SequenceSettings seqSet{};
        SystemSettings sysSet{};

        unique_ptr<TrackingFrontEnd> trackingFrontEnd{};
        unique_ptr<BackEnd> backEnd{};
        unique_ptr<Visualizer> visualizer{};
        shared_ptr<Camera> camera{};
        shared_ptr<Camera> cameraTracking{};
        shared_ptr<Camera> cameraWindowOptimization{};

        unordered_map<size_t, typeFrame> keyframes{};
        unique_ptr<PoseGraphOptimizer> poseGraphOptimizer{};

        map<size_t,Pose> groundtruth{};

        SID_SLAM(const string& sequenceData_yamlFile_ , const string& datasetFolder_ , const SystemSettings& sysSet__){
            sysSet = sysSet__;
            seqSet.initialize(sequenceData_yamlFile_,datasetFolder_);
            initializeCameras();
            camera->show_calibration();
            seqSet.RGB_hz = camera->get_RGB_frequence_hz();
            seqSet.RGB_seconds = camera->get_RGB_frequence_secs();
            seqSet.depth_hz = camera->get_depth_frequence_hz();
            cout << "depthConstant    : " << seqSet.depthConstant   << " (m/unit) "<<endl;
            visualizer.reset(new Visualizer{&seqSet});
            visualizer->mapVisualizer->image_w = camera->get_w();
            visualizer->mapVisualizer->image_h = camera->get_h();
            visualizer->mapVisualizer->saveSequence = sysSet.saveSequence;
            visualizer->mapVisualizer->maxInformationLoss =  sysSet.maxInformationLoss;
            visualizer->mapVisualizer->maxKeyframeDistance =  sysSet.maxKeyframeDistance;
            visualizer->mapVisualizer->groundtruth = &groundtruth;

            trackingFrontEnd = std::make_unique<TrackingFrontEnd>(&keyframes,&seqSet, cameraTracking, cameraWindowOptimization);

            trackingFrontEnd->tracker->set_tstudent2Probability(sysSet.tstudent2Probability, PATCH_SIZE,2);
            trackingFrontEnd->tracker->set_minInliersRatio(sysSet.minInliersRatio);

            trackingFrontEnd->pointSelector->setNumPointsToSelect(sysSet.numPointsToSelectWithInformation, sysSet.numPointsToExtract);
            trackingFrontEnd->pointSelector->setFeaturesMinResponse(sysSet.minResponse);
            trackingFrontEnd->pointSelector->setFeaturesNumOctaves(sysSet.numOctaves);
            trackingFrontEnd->pointSelector->setFeaturesNumSublevels(sysSet.numSublevels);
            trackingFrontEnd->pointSelector->setFeaturesNumThreads(sysSet.numThreads);
            trackingFrontEnd->pointSelector->setAkazeEvolution();
            trackingFrontEnd->pointSelector->setFeatureTrackingParameters(sysSet.matchingThreshold,sysSet.matchingDistance);
            trackingFrontEnd->pointSelector->setFeatureBias(sysSet.featureBias);
            trackingFrontEnd->pointSelector->setMinPhotoGradient(sysSet.minPhotoGradient);

            trackingFrontEnd->setMaxInformationLoss(sysSet.maxInformationLoss);
            trackingFrontEnd->setKeyframeCovisibility(sysSet.maxKeyframeDistance, sysSet.minVisiblePointsRatio);
            trackingFrontEnd->set_numKeyframesWindowOptimization(sysSet.numKeyframesWindowOptimization);

            trackingFrontEnd->windowOptimizer->setOptimizeDepths(sysSet.optimizeDephts);
            trackingFrontEnd->globalOptimizer->setOptimizeDepths(sysSet.optimizeDephts);
            trackingFrontEnd->windowOptimizer->set_tstudent2_p(sysSet.tstudent2ProbabilityWindow);
            trackingFrontEnd->globalOptimizer->set_tstudent2_p(sysSet.tstudent2ProbabilityWindow);

            trackingFrontEnd->trackedFrame.reset(new Frame(cameraTracking));

            trackingFrontEnd->visualization = sysSet.visualization;

            IDNav::setVisibilityParameters(sysSet.maxLambdaVisParam);
            backEnd = std::make_unique<BackEnd>();
            poseGraphOptimizer = std::make_unique<PoseGraphOptimizer>();

        };

        void initialize(Mat& rgbImg_ , dataType& grayImgTs,
                        Mat& depthImg , dataType& validDepth,
                        size_t& id,
                        Mat& maskImg){
            if(trackingFrontEnd->loadImage(rgbImg_,grayImgTs,depthImg,validDepth,id,maskImg)){
                initializedSystem = trackingFrontEnd->initFrontEnd();
                if(!initializedSystem){
                    std::cout << LIGTH_RED_COUT << "[SID_SLAM] initialize(): failed \n"<< RESET_COUT << endl;

                }
            }
        };

        void processImage(Mat& rgbImg_, dataType& grayImgTs_,
                          Mat& depthImage_, dataType& validDepth_,
                          size_t& id_,
                          Mat& maskImg_);

        void loadGroundtruth(map<size_t,vec7>& gt_){
            vec3 t;
            Eigen::Quaterniond q{};
            mat3 R;

            q.x() = 0.0;//gt_[0](3);
            q.y() = 0.0;//gt_[0](4);
            q.z() = 0.0;//gt_[0](5);
            q.w() = 1.0;//gt_[0](6);
            mat3 R0 =  q.toRotationMatrix().transpose();
            vec3 t0;
            t0 << gt_[0](0), gt_[0](1), gt_[0](2);
            t0 = -R0*t0;

            for(auto&[keyId,values]: gt_){
                t << values(0),values(1),values(2);
                q.x() = values(3);
                q.y() = values(4);
                q.z() = values(5);
                q.w() = values(6);
                R = q.toRotationMatrix();
                R = R0 * R;
                t = R0 * t + t0;

                Pose pose;
                pose.set_T_wc(t,R);
                groundtruth.insert({keyId,pose});
            }
        }

        void updateKeyframesWithGroundtruth(){
            for(auto& [keyId,keyframe]: keyframes){
                keyframe->pose.copyPoseFrom(groundtruth[keyframe->frameId]);
                keyframe->uv_2_XYZ(keyframe->hgp);
                keyframe->uv_2_XYZ(keyframe->features);
            }
        }

        void initializeCameras();

        void reset(){
            cout <<LIGTH_RED_COUT << "Reset SID-SLAM" << RESET_COUT <<endl;
            {
                std::mutex resetMutex;
                const std::lock_guard<std::mutex> lock(resetMutex);
                trackingFrontEnd->stopWindowOptimization = true;
            }
            while((trackingFrontEnd->prepareWindowOptimizationInProgress)||(!trackingFrontEnd->windowOptimizationFinished)){
                usleep(1000);
            }

            keyframes.clear();
            trackingFrontEnd.release();
            backEnd.release();
            poseGraphOptimizer.release();

            trackingFrontEnd = std::make_unique<TrackingFrontEnd>(&keyframes,&seqSet, cameraTracking, cameraWindowOptimization);
            trackingFrontEnd->tracker->set_tstudent2Probability(sysSet.tstudent2Probability, PATCH_SIZE,2);
            trackingFrontEnd->tracker->set_minInliersRatio(sysSet.minInliersRatio);

            trackingFrontEnd->pointSelector->setNumPointsToSelect(sysSet.numPointsToSelectWithInformation, sysSet.numPointsToExtract);
            trackingFrontEnd->pointSelector->setFeaturesMinResponse(sysSet.minResponse);
            trackingFrontEnd->pointSelector->setFeaturesNumOctaves(sysSet.numOctaves);
            trackingFrontEnd->pointSelector->setFeaturesNumSublevels(sysSet.numSublevels);
            trackingFrontEnd->pointSelector->setFeaturesNumThreads(sysSet.numThreads);
            trackingFrontEnd->pointSelector->setAkazeEvolution();
            trackingFrontEnd->pointSelector->setFeatureTrackingParameters(sysSet.matchingThreshold,sysSet.matchingDistance);
            trackingFrontEnd->pointSelector->setFeatureBias(sysSet.featureBias);
            trackingFrontEnd->pointSelector->setMinPhotoGradient(sysSet.minPhotoGradient);

            trackingFrontEnd->setMaxInformationLoss(sysSet.maxInformationLoss);
            trackingFrontEnd->setKeyframeCovisibility(sysSet.maxKeyframeDistance, sysSet.minVisiblePointsRatio);
            trackingFrontEnd->set_numKeyframesWindowOptimization(sysSet.numKeyframesWindowOptimization);

            trackingFrontEnd->windowOptimizer->setOptimizeDepths(sysSet.optimizeDephts);
            trackingFrontEnd->globalOptimizer->setOptimizeDepths(sysSet.optimizeDephts);
            trackingFrontEnd->windowOptimizer->set_tstudent2_p(sysSet.tstudent2ProbabilityWindow);
            trackingFrontEnd->globalOptimizer->set_tstudent2_p(sysSet.tstudent2ProbabilityWindow);

            trackingFrontEnd->trackedFrame.reset(new Frame(cameraTracking));

            IDNav::setVisibilityParameters(sysSet.maxLambdaVisParam);

            trackingFrontEnd->visualization = sysSet.visualization;

            backEnd = std::make_unique<BackEnd>();
            poseGraphOptimizer = std::make_unique<PoseGraphOptimizer>();

            visualizer->mapVisualizer->trajectory.clear();

            initializedSystem = false;
            resetSystem = false;
        }

        void saveKeyframesTrajectoryTUM(){
            cout << "Save final trajectory TUM format = "<< trackingFrontEnd->trajectory.size() << endl;
            vec3 twc{};
            mat3 Rwc{};
            Eigen::Quaterniond qwc{};

            vec3 tcw0{keyframes[0]->pose.tcw};
            mat3 Rcw0{keyframes[0]->pose.Rcw};
            for(auto& relPose: trackingFrontEnd->trajectory){
                relPose.second->getAbsolutePoseInTUMformat(twc,Rwc,keyframes[relPose.second->refKeyId]);
                //Rwc = Rcw0 * Rwc;
                //twc = Rcw0 * twc + tcw0;
                qwc = Eigen::Quaterniond(Rwc);
                *(seqSet.cameraTrajectory_TUMformat_Log) <<
                    std::setprecision(16) << relPose.second->timestamp
                    << std::setprecision(10) <<
                    " " << twc(0) << " " << twc(1) << " " << twc(2) <<
                    " " << qwc.x() << " " << qwc.y() << " " << qwc.z() << " " << qwc.w() << "\n";

                if(relPose.second->isKeyframe){
                    *(seqSet.keyframeTrajectory_TUMformat_Log) <<
                        std::setprecision(16) << relPose.second->timestamp
                        << std::setprecision(10) <<
                        " " << twc(0) << " " << twc(1) << " " << twc(2) <<
                        " " << qwc.x() << " " << qwc.y() << " " << qwc.z() << " " << qwc.w() << "\n";
                }
            }
        }

        void saveKeyframesTrajectoryTUM(const int& experimentName ,const int& numExperiments, string resultsFolder){
            // Padding zeros
            int numZeros = to_string(numExperiments*10).size();
            string fileName = to_string(experimentName);
            string zeros = "0";
            for(int iZero{}; iZero < numZeros-fileName.size()-1; ++iZero){
                zeros += "0";
            }
            fileName = zeros + fileName;

            string path{};

            ofstream  cameraTrajectory_TUMformat_Log_;
            path = resultsFolder  +"/"+ fileName + "_CameraTrajectory.txt";
            cameraTrajectory_TUMformat_Log_.open(path, std::ios::trunc | std::ios::out);
            cameraTrajectory_TUMformat_Log_.precision(12);

            ofstream  keyframeTrajectory_TUMformat_Log_;
            path = resultsFolder  +"/"+ fileName + "_KeyFrameTrajectory.txt";
            keyframeTrajectory_TUMformat_Log_.open(path, std::ios::trunc | std::ios::out);
            keyframeTrajectory_TUMformat_Log_.precision(12);

            cout << "Save final trajectory TUM format = "<< trackingFrontEnd->trajectory.size() << endl;
            vec3 twc{};
            mat3 Rwc{};
            Eigen::Quaterniond qwc{};

            vec3 tcw0{keyframes[0]->pose.tcw};
            mat3 Rcw0{keyframes[0]->pose.Rcw};
            for(auto& relPose: trackingFrontEnd->trajectory){
                relPose.second->getAbsolutePoseInTUMformat(twc,Rwc,keyframes[relPose.second->refKeyId]);
                //Rwc = Rcw0 * Rwc;
                //twc = Rcw0 * twc + tcw0;
                qwc = Eigen::Quaterniond(Rwc);
                cameraTrajectory_TUMformat_Log_ <<
                                                         std::setprecision(16) << relPose.second->timestamp
                                                         << std::setprecision(10) <<
                                                         " " << twc(0) << " " << twc(1) << " " << twc(2) <<
                                                         " " << qwc.x() << " " << qwc.y() << " " << qwc.z() << " " << qwc.w() << "\n";

                if(relPose.second->isKeyframe){
                    keyframeTrajectory_TUMformat_Log_ <<
                                                               std::setprecision(16) << relPose.second->timestamp
                                                               << std::setprecision(10) <<
                                                               " " << twc(0) << " " << twc(1) << " " << twc(2) <<
                                                               " " << qwc.x() << " " << qwc.y() << " " << qwc.z() << " " << qwc.w() << "\n";
                }
            }
            cameraTrajectory_TUMformat_Log_.close();
            keyframeTrajectory_TUMformat_Log_.close();
        }

        void saveFinalReport(){
            cout << "Save final report  = "<< trackingFrontEnd->trajectory.size() << endl;
            *(seqSet.finalReport_Log) <<
            std::setprecision(5) <<
            keyframes.size() << " " <<
            trackingFrontEnd->numKeyframesCreatedWithInformation << " " <<
            trackingFrontEnd->numKeyframesCreatedBecauseTrackingLost << " " <<
            sysSet.maxInformationLoss << " " <<
            sysSet.numPointsToSelectWithInformation << " " <<
            trackingFrontEnd->processImageProfiler.getMeanTime() << " " <<
            trackingFrontEnd->tracker->trackFrameProfiler.getMeanTime() << " " <<
#ifdef COMPLETE_PROFILING
#ifdef ACTIVE_FEATURES
            trackingFrontEnd->pointSelector->extractKeypointsProfiler.getMeanTime() << " " <<
            trackingFrontEnd->pointSelector->trackFeaturesProfiler.getMeanTime() << " " <<
#endif
            trackingFrontEnd->trackingSucceedProfiler.getMeanTime() << " " <<
            trackingFrontEnd->selectReferenceKeyframeProfiler.getMeanTime() << " " <<
            trackingFrontEnd->updateTrackingInformationProfiler.getMeanTime() << " " <<
            trackingFrontEnd->getMapProfiler.getMeanTime() << " " <<
            trackingFrontEnd->getKeyframesForTrackingProfiler.getMeanTime() << " " <<
#endif
            trackingFrontEnd->trackingProfiler.getMeanTime() << " " <<
            trackingFrontEnd->addKeyframeProfiler.getMeanTime() << " " <<
            trackingFrontEnd->windowOptimizer->prepareOptProfiler.getMeanTime() << " " <<
            trackingFrontEnd->windowOptimizer->optimizeProfiler.getMeanTime() << "\n";

        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::mutex mapVisualizationMutex;
        void triggerMapVisualization(bool pangolinVisualization = true){

            if(pangolinVisualization)
                const std::lock_guard<std::mutex> lock(mapVisualizationMutex);

            // Reset flag
            resetSystem = visualizer->mapVisualizer->resetSystem;
            visualizer->mapVisualizer->resetSystem = false;

            //Camera poses
            visualizer->mapVisualizer->setCurrentCameraPose(trackingFrontEnd->trackedFrame);
            visualizer->mapVisualizer->setRefCameraPose(trackingFrontEnd->refKeyframe_hc);
            //visualizer->mapVisualizer->setRefCameraPose(keyframes[0]);
            visualizer->mapVisualizer->setKeyframesCameraPose(&trackingFrontEnd->keyframesLocalWindow,
                                                              &trackingFrontEnd->keyframesForTracking,
                                                              &trackingFrontEnd->keyframesToOptimize,
                                                              trackingFrontEnd->keyframes);

            //Menu Information
            visualizer->mapVisualizer->num_hgp = trackingFrontEnd->tracker->get_num_hgpForTracking();
            visualizer->mapVisualizer->num_features = trackingFrontEnd->tracker->get_num_featuresForTracking();
            visualizer->mapVisualizer->num_keyframes = keyframes.size();

            // Tracked Frame
            visualizer->mapVisualizer->trackedFrameImage = trackingFrontEnd->trackedFrame->Mat_gray_clone.clone();

            trackingFrontEnd->changeMaxInformationLoss(visualizer->mapVisualizer->maxInformationLoss);
            trackingFrontEnd->setKeyframeCovisibility(visualizer->mapVisualizer->maxKeyframeDistance, visualizer->mapVisualizer->minVisiblePointsRatio);

#ifdef SEMANTIC_SEGMENTATION
            threshold( trackingFrontEnd->trackedFrame->maskImg, visualizer->mapVisualizer->trackedFrameMask, 1, 122,THRESH_BINARY);
            //visualizer->mapVisualizer->trackedFrameMask = trackingFrontEnd->trackedFrame->maskImg.clone();
            cv::addWeighted(visualizer->mapVisualizer->trackedFrameImage,1.0,visualizer->mapVisualizer->trackedFrameMask,1.0,1.0,visualizer->mapVisualizer->trackedFrameImage);
            #endif

            //Keyframes window
            visualizer->mapVisualizer->trackedPoints.clear();
            visualizer->mapVisualizer->trackedFeatures.clear();
            visualizer->mapVisualizer->lostFeatures.clear();

            dataType maxCov_hgp = 0.0;
            dataType minCov_hgp = 1000000.0;
            //for(auto& [idKey,keyframe]: trackingFrontEnd->keyframesLocalWindow){
                //for(typePt& pt: keyframe->hgp0){
            for(typePt& pt: *(trackingFrontEnd->tracker->get_hgpForTracking())){
                trackingFrontEnd->trackedFrame->XYZ_2_uv(pt);
                IDNav::Point3D trackedPoint{};

                trackedPoint.u = (int)pt->u[0];
                trackedPoint.v = (int)pt->v[0];

                trackedPoint.X = (float)pt->X[0];
                trackedPoint.Y = (float)pt->Y[0];
                trackedPoint.Z = (float)pt->Z[0];

                trackedPoint.cov =  (float)pow(pt->photoStd,2);
                visualizer->mapVisualizer->trackedPoints.push_back(trackedPoint);
                if(trackedPoint.cov > maxCov_hgp)
                    maxCov_hgp = trackedPoint.cov;
                if(trackedPoint.cov < minCov_hgp)
                    minCov_hgp = trackedPoint.cov;
            }
           // }

            visualizer->mapVisualizer->maxCov_hgp = 1.0;// (float) maxCov_hgp;
            visualizer->mapVisualizer->minCov_hgp = 1.0;//(float) minCov_hgp;

            //for(auto& [idKey,keyframe]: trackingFrontEnd->keyframesLocalWindow){
                //for(typeFt& ft: keyframe->features){

                    for(auto& [ft,obs]: trackingFrontEnd->trackedFrame->observations){
                    trackingFrontEnd->trackedFrame->XYZ_2_uv(ft);
                    IDNav::Point3D trackedFeature{};

                    trackedFeature.u = (int) ft->u[0];
                    trackedFeature.v = (int) ft->v[0];
                    trackedFeature.X = (float) ft->X[0];
                    trackedFeature.Y = (float) ft->Y[0];
                    trackedFeature.Z = (float) ft->Z[0];
                    trackedFeature.reUsedFeature = ft->reUsedFeature;
                    //trackedFeature.cov = (float)pow(ft->photoStd,2);
                    visualizer->mapVisualizer->trackedFeatures.push_back(trackedFeature);
                }
            //}
            //Visualization flag
            visualizer->mapVisualizer->newFrameTracked = true;
        }

        std::mutex statisticsMutex;
        void triggerStatistics(bool pangolinVisualization = true){
            {
                if(pangolinVisualization)
                    const std::lock_guard<std::mutex> lock(statisticsMutex);

                int iLog{0};

                /*visualizer->statisticVisualizer->data[iLog][0] = float(trackingFrontEnd->tracker->get_num_featuresForTracking());
                visualizer->statisticVisualizer->thereIsNewData[iLog] = true;
                ++iLog;*/

                visualizer->statisticVisualizer->data[iLog].clear();
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->trackingInformationBits));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->refKeyframe_hc->trackingInformationBitsThreshold));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->informationMean));
                //visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->minInformation));
                visualizer->statisticVisualizer->thereIsNewData[iLog] = true;
                ++iLog;

                /*visualizer->statisticVisualizer->data[iLog].clear();
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->trackedFrame->cam->get_RGB_frequence_secs()*1000));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->trackingProfiler.getLastExecutionTime()));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->trackFrameProfiler.getLastExecutionTime()));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->addKeyframeProfiler.getLastExecutionTime()));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->windowOptimizer->prepareOptProfiler.getLastExecutionTime()));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->windowOptimizer->optimizeProfiler.getLastExecutionTime()));
                visualizer->statisticVisualizer->thereIsNewData[iLog] = true;
                ++iLog;*/

                /*visualizer->statisticVisualizer->data[iLog].clear();
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->get_onlinePhotoCorr()));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->get_onlineReprojCorr()));
                visualizer->statisticVisualizer->thereIsNewData[iLog] = true;
                ++iLog;*/

                /*visualizer->statisticVisualizer->data[iLog].clear();
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->inliersRatio));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->inliersRatio_hgp));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->inliersRatio_features));
                visualizer->statisticVisualizer->data[iLog].push_back(float(trackingFrontEnd->tracker->minInliersRatio));
                visualizer->statisticVisualizer->data[iLog].push_back(float(0.95f));
                visualizer->statisticVisualizer->thereIsNewData[iLog] = true;
                ++iLog;*/

            }
        }

        void writeTrajectoryTUM(string expId = "", string expType =""){
            /*int numExperiments = 100000;
            // Padding zeros
            int numZeros = to_string(numExperiments*10).size();
            string fileName = expId;
            string zeros = "0";
            for(int iZero{}; iZero < numZeros-fileName.size()-1; ++iZero){
                zeros += "0";
            }
            fileName = zeros + fileName;

            seqSet.finalTrajectory_TUMformat_Log = new ofstream();
            string path = seqSet.resultsFolder  +"/"+ seqSet.fileName + "_" + fileName + "finalTrajectory_TUMformat_Log.txt";
            seqSet.finalTrajectory_TUMformat_Log->open(path, std::ios::trunc | std::ios::out);
            seqSet.finalTrajectory_TUMformat_Log->precision(12);
            cout << "Create log file : " << path << endl;

            seqSet.keyframes_TUMformat_Log = new ofstream();
            path = seqSet.resultsFolder  +"/"+ seqSet.fileName + "_" + fileName + "_" + expType + "keyframes_TUMformat_Log.txt";
            seqSet.keyframes_TUMformat_Log->open(path, std::ios::trunc | std::ios::out);
            seqSet.keyframes_TUMformat_Log->precision(12);
            cout << "Create log file : " << path << endl;

            for(typeFrame& keyframe: keyframes){
                IDNav::mat4 T;
                T << keyframe->pose.Rwc[0],keyframe->pose.Rwc[1],keyframe->pose.Rwc[2],keyframe->pose.twc[0],
                     keyframe->pose.Rwc[3],keyframe->pose.Rwc[4],keyframe->pose.Rwc[5],keyframe->pose.twc[1],
                     keyframe->pose.Rwc[6],keyframe->pose.Rwc[7],keyframe->pose.Rwc[8],keyframe->pose.twc[2],
                        0.0,0.0,0.0,1.0;
                Eigen::Quaterniond q(T.block<3,3>(0,0));
                *seqSet.finalTrajectory_TUMformat_Log <<
                                                 std::setprecision(16) <<
                                                 double(keyframe->grayImgTs)
                                                 << std::setprecision(5) <<
                                                 " " << keyframe->pose.twc[0] <<
                                                 " " << keyframe->pose.twc[1] <<
                                                 " " << keyframe->pose.twc[2] <<
                                                 " " << q.x() <<
                                                 " " << q.y() <<
                                                 " " << q.z() <<
                                                 " " << q.w() << "\n";
                *seqSet.keyframes_TUMformat_Log <<
                                                      std::setprecision(16) <<
                                                      double(keyframe->grayImgTs)
                                                      << std::setprecision(5) <<
                                                      " " << keyframe->pose.twc[0] <<
                                                      " " << keyframe->pose.twc[1] <<
                                                      " " << keyframe->pose.twc[2] <<
                                                      " " << q.x() <<
                                                      " " << q.y() <<
                                                      " " << q.z() <<
                                                      " " << q.w() << "\n";
                for(int iPose{0} ;iPose < keyframe->numTrackedFrames - 1; ++iPose){
                    IDNav::mat4 Tframe = keyframe->trackedPoses[iPose]*T;
                    Eigen::Quaterniond q(Tframe.block<3,3>(0,0));
                    *seqSet.finalTrajectory_TUMformat_Log <<
                                                          std::setprecision(16) <<
                                                          double(keyframe->posesTs[iPose])
                                                          << std::setprecision(5) <<
                                                          " " << Tframe(0,3) <<
                                                          " " << Tframe(1,3) <<
                                                          " " << Tframe(2,3) <<
                                                          " " << q.x() <<
                                                          " " << q.y() <<
                                                          " " << q.z() <<
                                                          " " << q.w() << "\n";
                }
            }
            seqSet.finalTrajectory_TUMformat_Log->close();
            delete seqSet.finalTrajectory_TUMformat_Log;
            seqSet.keyframes_TUMformat_Log->close();
            delete seqSet.keyframes_TUMformat_Log;*/
        }
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Thread Functions
    void sidSlamWindowOptimizationRun(IDNav::SID_SLAM* sidSlam_);
    void mapVisualizerRun(std::shared_ptr<IDNav::MapVisualizer> mapVisualizer_);
    void statisticVisualizerRun(shared_ptr<IDNav::StatisticVisualizer> statisticVisualizer_);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
#endif //IDNAV_SID_SLAM_H
