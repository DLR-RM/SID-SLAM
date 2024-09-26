
#include "../../include/trackingFrontEnd.h"

void IDNav::TrackingFrontEnd::trackFrame(){
    try {

        initTracking();
        motionEstimator->getSpeedEstimate(speed, speedCovInv, trackedFrame->grayImgTs);

        pointSelector->trackFeatures(trackedFrame, keyframesForTracking, refKeyframe_hc, speed);

        if (pointSelector->trackFeaturesSuccesful) {
            tracker->trackFrame(trackedFrame, keyframesForTracking, refKeyframe_hc,
                                trackedFrame->pose, speed, speedCovInv);
        }

        if (pointSelector->trackFeaturesSuccesful &&
            tracker->isTrackingSuccessful() &&
            motionEstimator->kinematicModelHolds(trackedFrame->pose)//&&
                ) {

            trackingSucceed();
            pointSelector->retrackFeatures(trackedFrame, keyframesForTracking);
            getRelativePoseToReferenceKeyframe();
            updateTrackingInformation();

            if (needNewKeyframe()) {
                ++numKeyframesCreatedWithInformation;
                pointSelector->retrackFeatures(trackedFrame, keyframesLocalWindow);
                if(!addKeyframe(trackedFrame)){
                    throw TrackingFrontEnd::TrackingFrontEndFail{"[TrackingFrontEnd](addKeyframe)"};
                }
            }
        } else {
            emergencyTracking();
            return;
        }

        triggerLoops();

        selectReferenceKeyframe();
        getTrackingMap();
        getKeyframesForTracking();
        finishTracking();
    }
    catch (const TrackingFrontEnd::TrackingFrontEndFail &t) {
        static int numExceptions{0};
        ++numExceptions;
        std::cout << LIGTH_RED_COUT << "    [TrackingFrontEnd] exception (#"<< numExceptions<<"): "<<t.errorMessage() << RESET_COUT << endl;
        relocalizationMode = true;
    }
}

std::mutex addKeyframeMutex;
bool IDNav::TrackingFrontEnd::addKeyframe(typeFrame frameToAdd_){

    initAddKeyframe();
    //////////////////////////////////////////////////////////////////
    typeFrame newKeyframe = createKeyframeFromFrame(frameToAdd_,numKeyframesCreated);
    if(!pointSelector->extractPoints(newKeyframe, keyframesLocalWindow)){
        return false;
    }
    newKeyframe->cam = cameraWindowOptimization;
    //////////////////////////////////////////////////////////////////
    tracker->resetSettings();
    motionEstimator->reset(speed,speedCovInv);
    //addKeyframeToLcd(newKeyframe);
    //////////////////////////////////////////////////////////////////

    const std::lock_guard<std::mutex> lock(addKeyframeMutex);
    stopWindowOptimization = true;

    while((!windowOptimizationFinished)||(prepareWindowOptimizationInProgress)) usleep(1000);

    newKeyframe->pose.copyPoseFrom(frameToAdd_->pose);
    updateFeatureObservations(newKeyframe);
    addKeyframeToWindow(newKeyframe);
    addKeyframeToLcd(newKeyframe);
    finishAddKeyframe(newKeyframe);

    windowOptimizer->resetOptimization();

    stopWindowOptimization = false;
    newWindowToOptimize = true;
    usleep(1000);

#ifdef SEQUENTIAL_WINDOW_OPTIMIZATION
    windowOptimizer->prepareOptimization(keyframesToOptimize,"iterativeOptimization");
    windowOptimizer->optimize(10000);
    windowOptimizer->updateVariables();
    //windowOptimizer->updateGlobalObservations(obs_hgp,obs_features);
#endif
    return true;
}

void IDNav::TrackingFrontEnd::windowOptimization(){
    while(!newWindowToOptimize){
        usleep(1000);
    }
    newWindowToOptimize = false;

    /*while(stopWindowOptimization){
        usleep(1000);
    }*/

    prepareWindowOptimizationInProgress = true;
    windowOptimizer->prepareOptimization(keyframesToOptimize,"iterativeOptimization");
    prepareWindowOptimizationInProgress = false;
    usleep(1000);

    windowOptimizationFinished = false;
    if(!stopWindowOptimization){
        windowOptimizer->optimize(5,0.5);
    }
    while(!stopWindowOptimization){
        windowOptimizer->optimize(2,0.005);
    }
    windowOptimizer->updateVariables();
    windowOptimizer->updateGlobalObservations(obs_hgp,obs_features);

    setRelativePoseToReferenceKeyframe();

    windowOptimizationFinished = true;
    usleep(1000);

}

bool IDNav::TrackingFrontEnd::needNewKeyframe(){
    //if(numTrackedFrames > 20) return true;
    //return false;
    return !thereIsEnoughInformationForTracking();
}