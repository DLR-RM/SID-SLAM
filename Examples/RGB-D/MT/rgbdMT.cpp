#include "../../../include/SID_SLAM.h"
#include <experimental/filesystem>
#include <memory>

namespace IDNav{
    void getRGBAndDepthImage(Mat& rgbImg_, dataType& grayImgTs ,Mat& depthImg ,dataType& validDepth,Mat& maskImg,IDNav::SID_SLAM* idSlam, const size_t& id);
    void sidSlamTracking(IDNav::SID_SLAM* sidSlam_);
}

int main(int argc, char** argv) {
	if (argc == 1) {
		std::cout << "Usage: rgbdMT [path to dataset] [path to system settings]" << std::endl;
		return -1;
	}

    string sDirectory = std::string(argv[1]);
    chdir(sDirectory.c_str());

    // Read inputs
    std::cout << BOLDMAGENTA_COUT <<  "\nInitialize SID_SLAM in TUM sequence ..." << RESET_COUT << std::endl;
    std::string systemSettings_yamlFile = std::string(argv[2]);
    std::string sequenceSettings_yamlFile_path = std::string(argv[1]) + "/sequenceSettings.yaml";
    std::string datasetFolder = "~/datasets/ID";

    int experimentID{0};
    if(argc > 3) experimentID = stoi(argv[3]);

    //Create IDSlam
    IDNav::SystemSettings sysSet{};
    sysSet.initialize(systemSettings_yamlFile);
    IDNav::SID_SLAM sidSlam(sequenceSettings_yamlFile_path, datasetFolder,sysSet);

    // Set log files
    std::string resultsFolder;
    if(argc > 4) resultsFolder = argv[4];
    else resultsFolder = "../../../results/RGB-D/TUM/" + sidSlam.seqSet.sequenceName ;
    sidSlam.seqSet.setLogFiles(experimentID,10000,resultsFolder);
    sidSlam.trackingFrontEnd->information_Log = sidSlam.seqSet.information_Log;
    sidSlam.trackingFrontEnd->tracker->covCorr_Log = sidSlam.seqSet.covCorr_Log;

    // Load images
    sidSlam.seqSet.loadImages(3);
    sidSlam.seqSet.loadGroundtruth(0);
    sidSlam.loadGroundtruth(sidSlam.seqSet.groundtruth);

    std::cout << BOLDMAGENTA_COUT <<  "\nRun SID_SLAM in RGBD-TUM sequence ..." << RESET_COUT << std::endl;

    std::thread sidSlamTrackingLoop = std::thread(IDNav::sidSlamTracking,&sidSlam);
    std::thread statisticVisualizerLoop;
    std::thread mapVisualizerLoop;
#ifdef ONLINE_WINDOW_OPTIMIZATION
    std::thread sidSlamWindowOptimizationLoop = std::thread(IDNav::sidSlamWindowOptimizationRun,&sidSlam);
#endif

    if(sidSlam.sysSet.visualization){
        statisticVisualizerLoop = std::thread(IDNav::statisticVisualizerRun,sidSlam.visualizer->statisticVisualizer);
        mapVisualizerLoop = std::thread(IDNav::mapVisualizerRun,sidSlam.visualizer->mapVisualizer);
        statisticVisualizerLoop.join();
        mapVisualizerLoop.join();
    }

#ifdef ONLINE_WINDOW_OPTIMIZATION
    sidSlamWindowOptimizationLoop.join();
#endif
    sidSlamTrackingLoop.join();

    std::cout << BOLDMAGENTA_COUT <<  "\nRun idnav profiling in RGBD-TUM sequence ..." << RESET_COUT << std::endl;
    sidSlam.saveKeyframesTrajectoryTUM();
    sidSlam.trackingFrontEnd->trackingProfiler.showProfile();
    sidSlam.trackingFrontEnd->addKeyframeProfiler.showProfile();
    sidSlam.trackingFrontEnd->tracker->trackFrameProfiler.showProfile();
    sidSlam.trackingFrontEnd->windowOptimizer->prepareOptProfiler.showProfile();
    sidSlam.trackingFrontEnd->windowOptimizer->optimizeProfiler.showProfile();
    sidSlam.seqSet.closeLogFiles();

    return 0;
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

void IDNav::sidSlamTracking(IDNav::SID_SLAM* sidSlam_){
    Mat rgbImg,depthImg;
    IDNav::dataType grayImgTs{};
    IDNav::dataType validDepth{};
    Mat maskImg{};
    for (size_t trFrame_id = 0 ; trFrame_id < sidSlam_->seqSet.numRGB; ++trFrame_id){
        getRGBAndDepthImage(rgbImg,grayImgTs,depthImg,validDepth,maskImg, sidSlam_, trFrame_id);
        sidSlam_->processImage(rgbImg,grayImgTs,depthImg, validDepth,trFrame_id,maskImg);
    }

    /*idSlam_->trackingFrontEnd->globalOptimizer->resetOptimization();
   idSlam_->trackingFrontEnd->globalOptimizer->prepareOptimization(idSlam_->keyframes,"iterativeOptimization");
   idSlam_->trackingFrontEnd->globalOptimizer->optimize(10);
   idSlam_->trackingFrontEnd->globalOptimizer->updateVariables();*/
}