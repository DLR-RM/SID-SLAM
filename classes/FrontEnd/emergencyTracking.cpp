
#include "../../include/trackingFrontEnd.h"

void IDNav::TrackingFrontEnd::emergencyTracking(){
    static int numEmergencyKeyframe{0};
    static int numEmergencyTracking{0};

    if (previousFrame != nullptr){ // Create a new keyframe from a previous tracked frame and track the current frame again
        ++numEmergencyKeyframe;
        ++numKeyframesCreatedBecauseTrackingLost;
#ifdef ACTIVE_FEATURES
        Mat grayImgCV32;
        previousFrame->grayImgG[0].convertTo(grayImgCV32,CV_32F,1.0/255.0,0);
        pointSelector->akazeEvolution->Create_Nonlinear_Scale_Space(grayImgCV32);
#endif
        motionEstimator->reset(speed,speedCovInv);
        pointSelector->trackFeatures(previousFrame, keyframesLocalWindow, refKeyframe_hc, speed);
        addKeyframe(previousFrame);
        getTrackingMap();
        getKeyframesForTracking();

        cout <<LIGTH_RED_COUT << "    [TrackingFrontend]: Create emergency keyframe! (#"<< numEmergencyKeyframe<<
            "): previous frame id = " << refKeyframe->frameId  << " , new keyframe id = " << refKeyframe->keyId  << RESET_COUT << endl;

#ifdef ACTIVE_FEATURES
        trackedFrame->grayImgG[0].convertTo(grayImgCV32,CV_32F,1.0/255.0,0);
        pointSelector->akazeEvolution->Create_Nonlinear_Scale_Space(grayImgCV32);
#endif
    }else{

        cout <<LIGTH_RED_COUT << "    [TrackingFrontend]: No frame available to create emergency keyframe!! " <<RESET_COUT << endl;
        if(tracker->emergencySettings){

            cout <<LIGTH_RED_COUT << "    [TrackingFrontend]: Tracking Lost :( " <<RESET_COUT << endl;
            terminate();
            addKeyframe(trackedFrame);
            //windowOptimizer->prepareOptimization(keyframesToOptimize,"iterativeOptimization");
            //windowOptimizer->optimize(10000);
            //windowOptimizer->updateVariables();
            getTrackingMap();
            getKeyframesForTracking();
            finishTracking();
            return;
        }
        tracker->setEmergencySettings();
    }

    ++numEmergencyTracking;
    cout <<LIGTH_RED_COUT << "    [TrackingFrontend]: Emergency Tracking! (#" <<numEmergencyTracking <<
    "): tracked frame id = " << trackedFrame->frameId << " , reference keyframe id = " << refKeyframe->keyId <<RESET_COUT << endl;

    trackFrame();
}