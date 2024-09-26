
#include "SystemSettings.h"

void IDNav::SystemSettings::initialize(const string& path_systemSettings_yamlFile){
    FileStorage fs_settings =  openFileStorage(path_systemSettings_yamlFile);
    cv::FileNode n_sysSet;
    n_sysSet = (fs_settings)["pointSelector"];
    if(not(n_sysSet.empty())){
        numPointsToSelectWithInformation = (int)(n_sysSet["numPointsToSelectWithInformation"]);
        numPointsToExtract = (int)(n_sysSet["numPointsToExtract"]);
        minPhotoGradient =  (float)(n_sysSet["minPhotoGradient"]);
    }

    n_sysSet = (fs_settings)["isKeyframe"];
    if(not(n_sysSet.empty())){
        maxInformationLoss = (dataType)(n_sysSet["maxInformationLoss"]);
    }

    n_sysSet = (fs_settings)["visibility"];
    if(not(n_sysSet.empty())){
        maxLambdaVisParam  = (double)(n_sysSet["maxLambdaVisParam"]);
        maxKeyframeDistance  = (double)(n_sysSet["maxKeyframeDistance"]);
        minVisiblePointsRatio = (double)(n_sysSet["minVisiblePointsRatio"]);
    }

    n_sysSet = (fs_settings)["tracking"];
    if(not(n_sysSet.empty())){
        tstudent2Probability  = (double)(n_sysSet["tstudent2Probability"]);
        minInliersRatio  = (float)(n_sysSet["minInliersRatio"]);
    }

    n_sysSet = (fs_settings)["windowOptimization"];
    if(not(n_sysSet.empty())){
        numKeyframesWindowOptimization  = (int)(n_sysSet["numKeyframesWindowOptimization"]);
        optimizeDephts  = (bool)(int)(n_sysSet["optimizeDephts"]);
        tstudent2ProbabilityWindow  = (double)(n_sysSet["tstudent2Probability"]);
    }

    n_sysSet = (fs_settings)["Features"];
    if(not(n_sysSet.empty())){
        minResponse    = (double)(n_sysSet["minResponse"]);
        numOctaves     = (int)(n_sysSet["numOctaves"]);
        numSublevels   = (int)(n_sysSet["numSublevels"]);
        numThreads  = (int)(n_sysSet["numThreads"]);
        matchingThreshold   = (double)(n_sysSet["matchingThreshold"]);
        matchingDistance    = (double)(n_sysSet["matchingDistance"]);
        featureBias  = (double)(n_sysSet["featureBias"]);
    }

    n_sysSet = (fs_settings)["Visualizer"];
    if(not(n_sysSet.empty())){
        visualization = (int)(n_sysSet["visualization"]);
        saveSequence  = (int)(n_sysSet["saveSequence"]);
    }

    fs_settings.release();
}
