//
// Created by font_al on 1/11/19.
//

#include "../include/SID_SLAM.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread Functions
//  - mapVisualizerRun(...)
//  - statisticVisualizerRun(...)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This function initializes the camera parameters from the given .yaml file and sets suitable values for the resolution
// pyramid and the grid extraction
void IDNav::SID_SLAM::initializeCameras(){
    cv::FileStorage fs_camera =  openFileStorage(seqSet.camera_yamlFile);
    cv::FileNode n;
    n = fs_camera["camera0"];
    string cameraModel = (string)(n["model"]);
    
    if(cameraModel == "pinhole"){
        camera = shared_ptr<Camera>{new CamPinhole((dataType)(n["fx"]), (dataType)(n["fy"]),
                                    (dataType)(n["cx"]), (dataType)(n["cy"]) ,
                                    (int)(n["w"]),(int)(n["h"]),
                                    (dataType)(n["stereoBaseline"]), (dataType)(n["associationThreshold"]))};
        seqSet.depthConstant = (dataType)n["depthConstant"];
        camera->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        camera->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                    (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        camera->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                    (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);

        cameraTracking = shared_ptr<Camera>{new CamPinhole((dataType)(n["fx"]), (dataType)(n["fy"]),
                                                   (dataType)(n["cx"]), (dataType)(n["cy"]) ,
                                                   (int)(n["w"]),(int)(n["h"]),
                                                   (dataType)(n["stereoBaseline"]), (dataType)(n["associationThreshold"]))};
        cameraTracking->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        cameraTracking->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                    (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        cameraTracking->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                  (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);

        cameraWindowOptimization = shared_ptr<Camera>{new CamPinhole((dataType)(n["fx"]), (dataType)(n["fy"]),
                                                           (dataType)(n["cx"]), (dataType)(n["cy"]) ,
                                                           (int)(n["w"]),(int)(n["h"]),
                                                           (dataType)(n["stereoBaseline"]), (dataType)(n["associationThreshold"]))};
        cameraWindowOptimization->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        cameraWindowOptimization->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                            (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        cameraWindowOptimization->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                          (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);
    }

    if(cameraModel == "distPinhole"){
        cv::Mat kc = (cv::Mat1d(1,5) << (dataType)(n["k1"]),(dataType)(n["k2"]),(dataType)(n["k3"]),(dataType)(n["k4"]),(dataType)(n["k5"]));
        camera = shared_ptr<Camera>{new CamPinholeDist((dataType)(n["fx"]), (dataType)(n["fy"]),
                                    (dataType)(n["cx"]), (dataType)(n["cy"]) ,kc,
                                    (int)(n["w"]),(int)(n["h"]),
                                    (dataType)(n["stereoBaseline"]),  (dataType)(n["associationThreshold"]))};
        seqSet.depthConstant = n["depthConstant"];
        camera->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        camera->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                    (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        camera->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                  (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);

        cameraTracking = shared_ptr<Camera>{new CamPinholeDist((dataType)(n["fx"]), (dataType)(n["fy"]),
                                                       (dataType)(n["cx"]), (dataType)(n["cy"]) ,kc,
                                                       (int)(n["w"]),(int)(n["h"]),
                                                       (dataType)(n["stereoBaseline"]),  (dataType)(n["associationThreshold"]))};
        cameraTracking->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        cameraTracking->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                    (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        cameraTracking->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                  (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);

        cameraWindowOptimization = shared_ptr<Camera>{new CamPinholeDist((dataType)(n["fx"]), (dataType)(n["fy"]),
                                                               (dataType)(n["cx"]), (dataType)(n["cy"]) ,kc,
                                                               (int)(n["w"]),(int)(n["h"]),
                                                               (dataType)(n["stereoBaseline"]),  (dataType)(n["associationThreshold"]))};
        cameraWindowOptimization->setCameraFrequence((dataType)n["RGB_hz"],(dataType)n["depth_hz"]);
        cameraWindowOptimization->set_photoDef2_param((dataType)n["maxPhotoDef2"],(dataType)n["minPhotoDef2"],
                                            (dataType)n["ctPhotoDef2"],(dataType)n["ccPhotoDef2"],(dataType)n["imgPhotoCov"]);
        cameraWindowOptimization->set_geoDef2_param((dataType)n["maxGeoDef2"],(dataType)n["minGeoDef2"],
                                          (dataType)n["ctGeoDef2"],(dataType)n["ccGeoDef2"],(dataType)n["imgGeoCov"]);
    }
}

void IDNav::SID_SLAM::processImage(Mat& rgbImg_, dataType& grayImgTs_,
                                   Mat& depthImg_, dataType& validDepth_,
                                   size_t& trFrame_id_,
                                   Mat& maskImg_){
    std::chrono::high_resolution_clock::time_point t1{},t2{};
    std::chrono::duration<float, std::milli> timeLastExecution{};
    int incT;
    double t_1;

    if(initializedSystem){
        t1 = std::chrono::high_resolution_clock::now();
        if(trackingFrontEnd->loadImage(rgbImg_,grayImgTs_,
                                       depthImg_, validDepth_,
                                       trFrame_id_,maskImg_)){
            if(trackingFrontEnd->relocalizationMode){
                cout <<LIGTH_RED_COUT << "SID-SLAM relocalizing ..." << RESET_COUT <<endl;
            }else{
                trackingFrontEnd->trackFrame();
                if(sysSet.visualization){
                    triggerMapVisualization();
                    triggerStatistics();
                }
            }

        }
#ifdef ONLINE_WINDOW_OPTIMIZATION
        t2 = std::chrono::high_resolution_clock::now();
        timeLastExecution = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
        incT = (int) (grayImgTs_ - t_1)*1000.0 - (timeLastExecution.count());
        if((incT > 0)&&(!rosActive)){
            usleep(incT);
        }
        t_1 = grayImgTs_;
#endif
        if(resetSystem){
            reset();
        }
    }
    else{
        initialize(rgbImg_,grayImgTs_,depthImg_,validDepth_, trFrame_id_,maskImg_);
        t_1 = grayImgTs_;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread Functions

void IDNav::sidSlamWindowOptimizationRun(IDNav::SID_SLAM* sidSlam_) {
    while(!sidSlam_->killSystem){
        if(sidSlam_->initializedSystem){
            sidSlam_->trackingFrontEnd->windowOptimization();
        }
        usleep(1000);
    }
}

void IDNav::mapVisualizerRun(std::shared_ptr<IDNav::MapVisualizer> mapVisualizer_){

    // Create Interactive View in window (Pangolin functions)
    pangolin::BindToContext(mapVisualizer_->mapWindowName);
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    const int UI_WIDTH = 180;
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -float(mapVisualizer_->mapWindow_w)/float(mapVisualizer_->mapWindow_h))
            .SetHandler(new pangolin::Handler3D(*mapVisualizer_->sCam));
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(UI_WIDTH));

    // Set buttons
    mapVisualizer_->setMenuButtons();

    // Save sequence map and video streams
    int numTrackedFrames{0};
    if(mapVisualizer_->saveSequence){
        mapVisualizer_->seqSet->removeFilesFromDirectory("savedSequence/map/", ".png");
        mapVisualizer_->seqSet->removeFilesFromDirectory("savedSequence/trackedFrame/", ".png");
    }

    // Map visualization loop
    while(!pangolin::ShouldQuit())
    {
        if(mapVisualizer_->newFrameTracked){

            // Clear screen and activate view to render into (Pangolin functions)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(*mapVisualizer_->sCam);
            glClearColor(0.0f,0.0f,0.0f,0.0f);

            // Check menu
            mapVisualizer_->checkMenu();

            // Swap frames and Process Events
            pangolin::FinishFrame();

            cvtColor(mapVisualizer_->trackedFrameImage,mapVisualizer_->trackedFrameImage, cv::COLOR_GRAY2RGB);
            mapVisualizer_->drawPoints(mapVisualizer_->trackedPoints,mapVisualizer_->trackedFeatures,mapVisualizer_->trackedFrameImage);

            cv::Mat img_;
            float imageRatio = (float(mapVisualizer_->trackedFrameImage.cols)/float(mapVisualizer_->trackedFrameImage.rows));
            int h = 600;//1.0*480;
            int w = imageRatio*h;
            resize(mapVisualizer_->trackedFrameImage, img_, Size(w,h));
            cv::imshow(mapVisualizer_->trackFrameWindowName,img_);
            cv::waitKey(1);
            if((mapVisualizer_->saveSequence)||(mapVisualizer_->screenshot)){
                string windowName = "savedSequence/trackedFrame/" + to_string(numTrackedFrames) + ".png";
                windowName = "/home/afontan/toDelete/trackedFrame_" + to_string(numTrackedFrames) + ".png";
                imwrite( windowName,mapVisualizer_->trackedFrameImage);
                //std::cout << " save tracked frame image" << std::endl;

                //windowName = "savedSequence/map/map_" + to_string(numTrackedFrames);
                windowName = "/home/afontan/toDelete/map_" + to_string(numTrackedFrames);
                d_cam.SaveOnRender(windowName);
                std::cout << " saved map" << std::endl;
                mapVisualizer_->saveSequence = false;
                mapVisualizer_->screenshot = false;

            }

            ++numTrackedFrames;
            mapVisualizer_->newFrameTracked = false;

        }
    }
    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void IDNav::statisticVisualizerRun(shared_ptr<IDNav::StatisticVisualizer> statisticVisualizer_){

    // Initialize Statisctics thread
    pangolin::WindowInterface& statisticWindow = pangolin::CreateWindowAndBind("statisticWindow",4000,400);
    statisticWindow.Move(1900,637);

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int graphShift = -1 - statisticVisualizer_->seqSet->numRGB/20;
    // Add Logs
    std::vector<std::string> labels;

    /*labels.emplace_back(std::string("NumPoints"));
    statisticVisualizer->addLog(labels,3000,0,statisticVisualizer->seqSet->numRGB,graphShift);*/

    labels.clear();
    labels.emplace_back("Information (bits)");
    labels.emplace_back("Information. Threshold (bits) ");
    labels.emplace_back("Inf. Max. (bits) ");
    //labels.emplace_back("Sig.Center");

    statisticVisualizer_->addLog(labels,57,45,statisticVisualizer_->seqSet->numRGB,graphShift);

    /*labels.clear();
    labels.emplace_back("30 us");
    labels.emplace_back("Tracking (us)");
    labels.emplace_back("Track Frame (us)");
    labels.emplace_back("add Keyframe (us)");
    labels.emplace_back("prep opt (us)");
    labels.emplace_back("optimize (us)");
    statisticVisualizer->addLog(labels,1.5*statisticVisualizer->seqSet->RGB_seconds*1000.0,0.0,statisticVisualizer->seqSet->numRGB,graphShift);*/

    /*labels.clear();
    labels.emplace_back("alpha");
    labels.emplace_back("beta");
    statisticVisualizer->addLog(labels,3.00,0.0,statisticVisualizer->seqSet->numRGB,graphShift);*/

    /*labels.clear();
    labels.emplace_back("inliersRatio");
    labels.emplace_back("inliersRatio_hgp");
    labels.emplace_back("inliersRatio_features");
    labels.emplace_back("minInliersRatio");
    labels.emplace_back("95%");
    statisticVisualizer->addLog(labels,1.1,0.45, statisticVisualizer->seqSet->numRGB,graphShift);*/

    //Add plotters to display
    pangolin::Display("multiEstatistic")
            .SetBounds(0.0, 1.0, 1.0, 0.0)
            .SetLayout(pangolin::LayoutEqual);

    for(size_t iGraphic{0}; iGraphic < statisticVisualizer_->logs.size(); ++iGraphic){
        pangolin::Display("multiEstatistic").AddDisplay(*statisticVisualizer_->plotters[iGraphic]);
    }

    statisticVisualizer_->statisticVisualizerInitialized = true;

    int numTrackedFrames{};
    bool save{false};
    //cleanDirectoryLogFiles("../experiments/statistics/", ".png");
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //cout << " cout 2"<< endl;
        // Log to data
        for(size_t iPlot{}; iPlot < statisticVisualizer_->numPlots; ++iPlot){
            if(statisticVisualizer_->thereIsNewData[iPlot]){
                statisticVisualizer_->thereIsNewData[iPlot] = false;
                statisticVisualizer_->logs[iPlot]->Log(statisticVisualizer_->data[iPlot]);
                if(not save){
                    string windowName = "../experiments/statistics/" + to_string(numTrackedFrames);
                    //pangolin::SaveWindowOnRender(windowName);
                    ++numTrackedFrames;
                    save = true;
                }
            }
        }
        save = false;


        pangolin::FinishFrame();

    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
