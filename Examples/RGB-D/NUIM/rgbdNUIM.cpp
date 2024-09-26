#include "../../../include/SID_SLAM.h"
#include <experimental/filesystem>
#include <memory>

namespace IDNav{
    void sidSlamTracking(IDNav::SID_SLAM* sidSlam_);

    void sidSlamWindowOptimizationRun(IDNav::SID_SLAM* idSlam);
    //void statisticVisualizerRun(IDNav::StatisticVisualizer* statisticVisualizer);
    //void mapVisualizerRun(IDNav::MapVisualizer* mapVisualizer);
    void getRGBAndDepthImage(Mat& rgbImg_, dataType& grayImgTs ,Mat& depthImg ,dataType& validDepth,Mat& maskImg,IDNav::SID_SLAM* idSlam, const size_t& id);
}

int main(int argc, char** argv) {
	if (argc == 1) {
		std::cout << "Usage: rgbdMT [path to dataset] [path to system settings]" << std::endl;
		return -1;
	}

	string sDirectory = std::string(argv[1]);
    chdir(sDirectory.c_str());

    // Read inputs
    std::cout << BOLDMAGENTA_COUT <<  "\nInitialize SID_SLAM in NUIM sequence ..." << RESET_COUT << std::endl;
    std::string systemSettings_yamlFile = std::string(argv[2]);
    std::string sequenceSettings_yamlFile_path = std::string(argv[1]) + "/sequenceSettings.yaml";
    std::string datasetFolder = "~/datasets/NUIM";

    int experimentID{0};
    if(argc > 3) experimentID = stoi(argv[3]);

    //Create IDSlam
    IDNav::SystemSettings sysSet{};
    sysSet.initialize(systemSettings_yamlFile);
    IDNav::SID_SLAM sidSlam(sequenceSettings_yamlFile_path, datasetFolder,sysSet);

    // Set log files
    std::string resultsFolder;
    if(argc > 4) resultsFolder = argv[4];
    else resultsFolder = "../../../results/RGB-D/NUIM/" + sidSlam.seqSet.sequenceName ;
    sidSlam.seqSet.setLogFiles(experimentID,10000,resultsFolder);
    sidSlam.trackingFrontEnd->information_Log = sidSlam.seqSet.information_Log;
    sidSlam.trackingFrontEnd->tracker->covCorr_Log = sidSlam.seqSet.covCorr_Log;

    // Load images
    sidSlam.seqSet.loadImages(3);
    sidSlam.seqSet.loadGroundtruth(0);
    sidSlam.loadGroundtruth(sidSlam.seqSet.groundtruth);

    std::cout << BOLDMAGENTA_COUT <<  "\nRun SID_SLAM in RGBD-NUIM sequence ..." << RESET_COUT << std::endl;

    std::thread sidSlamTrackingLoop = std::thread(IDNav::sidSlamTracking, &sidSlam);
#ifdef ONLINE_WINDOW_OPTIMIZATION
    std::thread sidSlamWindowOptimizationLoop = std::thread(IDNav::sidSlamWindowOptimizationRun,&sidSlam);
#endif
    if(sidSlam.sysSet.visualization){
        std::thread statisticVisualizerLoop;
        statisticVisualizerLoop = std::thread(IDNav::statisticVisualizerRun,sidSlam.visualizer->statisticVisualizer);
        std::thread mapVisualizerLoop;
        mapVisualizerLoop = std::thread(IDNav::mapVisualizerRun,sidSlam.visualizer->mapVisualizer);
        statisticVisualizerLoop.join();
        mapVisualizerLoop.join();
    }
    sidSlamTrackingLoop.join();
#ifdef ONLINE_WINDOW_OPTIMIZATION
    sidSlamWindowOptimizationLoop.join();
#endif
    return 0;
}

void IDNav::sidSlamWindowOptimizationRun(IDNav::SID_SLAM* idSlam) {
    while(true){
        if(idSlam->initializedSystem){
            idSlam->trackingFrontEnd->windowOptimization();
        }
        usleep(1000);
    }
}

void IDNav::sidSlamTracking(IDNav::SID_SLAM* idSlam_){
    Mat rgbImg,depthImg;
    dataType grayImgTs{};
    dataType validDepth{};
    Mat maskImg{};
    std::chrono::high_resolution_clock::time_point t1{},t2{};
    std::chrono::duration<float, std::milli> timeLastExecution{};
    int incT;
    double t_1;
    for (size_t trFrame_id = 0 ; trFrame_id < idSlam_->seqSet.numRGB; ++trFrame_id){
        getRGBAndDepthImage(rgbImg,grayImgTs,depthImg,validDepth,maskImg, idSlam_, trFrame_id);

        if(idSlam_->initializedSystem){
            t1 = std::chrono::high_resolution_clock::now();
            if(idSlam_->trackingFrontEnd->loadImage(rgbImg,grayImgTs,
                                                    depthImg, validDepth,
                                                    trFrame_id,maskImg)){
                idSlam_->trackingFrontEnd->trackFrame();
                if(idSlam_->sysSet.visualization){
                    idSlam_->triggerMapVisualization();
                    idSlam_->triggerStatistics();
                }
            }
#ifdef ONLINE_WINDOW_OPTIMIZATION
            t2 = std::chrono::high_resolution_clock::now();
            timeLastExecution = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
            incT = (int) (grayImgTs - t_1)*1000.0 - (timeLastExecution.count());
            if(incT > 0)
                usleep(incT);
            t_1 = grayImgTs;
#endif
        }
        else{
            idSlam_->initialize(rgbImg,grayImgTs,depthImg,validDepth, trFrame_id,maskImg);
            t_1 = grayImgTs;
        }

    }

    /*idSlam_->trackingFrontEnd->globalOptimizer->resetOptimization();
    idSlam_->trackingFrontEnd->globalOptimizer->prepareOptimization(idSlam_->keyframes,"iterativeOptimization");
    idSlam_->trackingFrontEnd->globalOptimizer->optimize(10);
    idSlam_->trackingFrontEnd->globalOptimizer->updateVariables();*/

    idSlam_->saveKeyframesTrajectoryTUM();

    std::cout << BOLDMAGENTA_COUT <<  "\nRun idnav profiling in RGBD-TUM sequence ..." << RESET_COUT << std::endl;
    idSlam_->trackingFrontEnd->trackingProfiler.showProfile();
    idSlam_->trackingFrontEnd->addKeyframeProfiler.showProfile();
    idSlam_->trackingFrontEnd->tracker->trackFrameProfiler.showProfile();
    idSlam_->trackingFrontEnd->windowOptimizer->prepareOptProfiler.showProfile();
    idSlam_->trackingFrontEnd->windowOptimizer->optimizeProfiler.showProfile();
    idSlam_->seqSet.closeLogFiles();
    terminate();
}

void IDNav::getRGBAndDepthImage(Mat& rgbImg_, dataType& grayImgTs ,Mat& depthImg ,dataType& validDepth, Mat& maskImg, IDNav::SID_SLAM* idSlam, const size_t& id){
    if(idSlam->seqSet.operationMode == "rgbd"){
        string grayImgAddress = idSlam->seqSet.RGB_list[id];
        rgbImg_ = cv::imread(grayImgAddress);
        grayImgTs = idSlam->seqSet.RGB_list_timestamp[id];

        // Find corresponding depth image and compute validDepth.
        string depthImgAddress = idSlam->seqSet.depthList[id];
        validDepth = 0.0;
        depthImg = cv::imread(depthImgAddress, cv::IMREAD_ANYDEPTH);
        if((abs(idSlam->seqSet.depthConstant-1.0) > 1e-5) || depthImg.type()!=DEPHT_MAT_TYPE)
            depthImg.convertTo(depthImg,DEPHT_MAT_TYPE,1.0/idSlam->seqSet.depthConstant);


    }
}
/*
void IDNav::statisticVisualizerRun(IDNav::StatisticVisualizer* statisticVisualizer){

    // Initialize Statisctics thread
    pangolin::WindowInterface& statisticWindow = pangolin::CreateWindowAndBind("statisticWindow",4000,400);
    statisticWindow.Move(1900,637);

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int graphShift = -1 - statisticVisualizer->seqSet->numRGB/20;
    // Add Logs
    std::vector<std::string> labels;

    /*labels.emplace_back(std::string("NumPoints"));
    statisticVisualizer->addLog(labels,3000,0,statisticVisualizer->seqSet->numRGB,graphShift);*

    labels.clear();
    labels.emplace_back("Information (bits)");
    labels.emplace_back("Information. Threshold (bits) ");
    labels.emplace_back("Inf. Max. (bits) ");
    //labels.emplace_back("Sig.Center");

    statisticVisualizer->addLog(labels,55,40,statisticVisualizer->seqSet->numRGB,graphShift);

    /*labels.clear();
    labels.emplace_back("30 us");
    labels.emplace_back("Tracking (us)");
    labels.emplace_back("Track Frame (us)");
    labels.emplace_back("add Keyframe (us)");
    labels.emplace_back("prep opt (us)");
    labels.emplace_back("optimize (us)");
    statisticVisualizer->addLog(labels,1.5*statisticVisualizer->seqSet->RGB_seconds*1000.0,0.0,statisticVisualizer->seqSet->numRGB,graphShift);*

    /*labels.clear();
    labels.emplace_back("alpha");
    labels.emplace_back("beta");
    statisticVisualizer->addLog(labels,3.00,0.0,statisticVisualizer->seqSet->numRGB,graphShift);*

    /*labels.clear();
    labels.emplace_back("inliersRatio");
    labels.emplace_back("inliersRatio_hgp");
    labels.emplace_back("inliersRatio_features");
    labels.emplace_back("minInliersRatio");
    labels.emplace_back("95%");
    statisticVisualizer->addLog(labels,1.1,0.45, statisticVisualizer->seqSet->numRGB,graphShift);*

    //Add plotters to display
    pangolin::Display("multiEstatistic")
            .SetBounds(0.0, 1.0, 1.0, 0.0)
            .SetLayout(pangolin::LayoutEqual);

    for(size_t iGraphic{0}; iGraphic < statisticVisualizer->logs.size(); ++iGraphic){
        pangolin::Display("multiEstatistic").AddDisplay(*statisticVisualizer->plotters[iGraphic]);
    }

    statisticVisualizer->statisticVisualizerInitialized = true;

    int numTrackedFrames{};
    bool save{false};
    //cleanDirectoryLogFiles("../experiments/statistics/", ".png");
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Log to data
        for(size_t iPlot{}; iPlot < statisticVisualizer->numPlots; ++iPlot){
            if(statisticVisualizer->thereIsNewData[iPlot]){
                statisticVisualizer->thereIsNewData[iPlot] = false;
                statisticVisualizer->logs[iPlot]->Log(statisticVisualizer->data[iPlot]);
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

void IDNav::mapVisualizerRun(IDNav::MapVisualizer* mapVisualizer){

    // fetch the context and bind it to this thread
    pangolin::BindToContext(mapVisualizer->mapWindowName);

    // restore the properties of the context
    glEnable(GL_DEPTH_TEST);  // 3D Mouse handler requires depth testing to be enabled
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    const int UI_WIDTH = 180;

    // Create Interactive View in window
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -float(mapVisualizer->mapWindow_w)/float(mapVisualizer->mapWindow_h))
            .SetHandler(new pangolin::Handler3D(*mapVisualizer->sCam));

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(UI_WIDTH));
    mapVisualizer->setMenuButtons();
    int numTrackedFrames{0};
    if(mapVisualizer->saveSequence){
        mapVisualizer->seqSet->removeFilesFromDirectory("savedSequence/map/", ".png");
        mapVisualizer->seqSet->removeFilesFromDirectory("savedSequence/trackedFrame/", ".png");
    }

    while( !pangolin::ShouldQuit())
    {
        if(mapVisualizer->newFrameTracked){

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(*mapVisualizer->sCam);
            glClearColor(0.0f,0.0f,0.0f,0.0f);
            //glClearColor(1.0f,1.0f,1.0f,0.0f);
            mapVisualizer->drawCamera(mapVisualizer->Twc,mapVisualizer->cameraColor);
            mapVisualizer->checkMenu();


            // Swap frames and Process Events
            pangolin::FinishFrame();

            cvtColor(mapVisualizer->trackedFrameImage,mapVisualizer->trackedFrameImage, cv::COLOR_GRAY2RGB);
            mapVisualizer->drawPoints(mapVisualizer->trackedPoints,mapVisualizer->trackedFeatures,mapVisualizer->trackedFrameImage);
            //mapVisualizer->drawPoints(mapVisualizer->mapPoints,mapVisualizer->trackedFrameImage);
            cv::Mat img_;
            float imageRatio = (float(mapVisualizer->trackedFrameImage.cols)/float(mapVisualizer->trackedFrameImage.rows));
            int h = 600;//1.0*480;
            int w = imageRatio*h;
            resize(mapVisualizer->trackedFrameImage, img_, Size(w,h));
            cv::imshow(mapVisualizer->trackFrameWindowName,img_);
            cv::waitKey(1);
            if((mapVisualizer->saveSequence)||(mapVisualizer->screenshot)){
                string windowName = "savedSequence/trackedFrame/" + to_string(numTrackedFrames) + ".png";
                windowName = "/home/afontan/toDelete/trackedFrame_" + to_string(numTrackedFrames) + ".png";
                imwrite( windowName,mapVisualizer->trackedFrameImage);
                //std::cout << " save tracked frame image" << std::endl;

                //windowName = "savedSequence/map/map_" + to_string(numTrackedFrames);
                windowName = "/home/afontan/toDelete/map_" + to_string(numTrackedFrames);
                d_cam.SaveOnRender(windowName);
                std::cout << " saved map" << std::endl;
                mapVisualizer->saveSequence = false;
                mapVisualizer->screenshot = false;

            }

            ++numTrackedFrames;
            mapVisualizer->newFrameTracked = false;

        }
    }
    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();

}
 */