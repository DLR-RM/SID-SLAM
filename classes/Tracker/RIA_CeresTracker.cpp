//
// Created by font_al on 2/14/20.
//

#include "../../include/Tracker/Tracker.h"

void IDNav::Tracker::tracking_ceres(dataType parameter_tolerance_ , int numIt_){
    set_config_tracking_ceres(parameter_tolerance_,numIt_);

    Pose pose0(trackedFrame->pose);
    dataType poseGuess[6] = {speedPrior(0),speedPrior(1),speedPrior(2),
                             speedPrior(3),speedPrior(4),speedPrior(5)};

    for (iPyr = higherPyrLevel; iPyr >= lowerPyrLevel; iPyr -=1 ) {
        trackedFrame->cam->setResolution(iPyr);
        if(selectPointsforTracking(pose0,poseGuess)){
            ceresTracker->estimatePoseCoarse(hgpForTracking,featuresForTracking,trackedFrame, poseGuess);
        }
    }

    ceresTracker->updatePose(trackedFrame, poseGuess , pose0);

    trackedFrame->cam->setResolution(0);
    for (auto &[keyframe,hgp] : clustered_hgpForTracking ) {
        keyframe->uv_2_XYZ(hgp);
    }
}

void IDNav::Tracker::refine_tracking_ceres(dataType parameter_tolerance_, int numIt_){


    Pose pose0(trackedFrame->pose);
    dataType updatePose[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    set_config_refineTracking_ceres(parameter_tolerance_,numIt_);
    ceresTracker->estimatePose(hgpForTracking,featuresForTracking,trackedFrame,updatePose);
    ceresTracker->updatePose(trackedFrame, updatePose , pose0);

    dataType photoScalar{0.0};
    dataType photoBias{0.0};
    set_config_estimatePhotoParameters_ceres(parameter_tolerance_, numIt_);
    ceresTracker->estimatePhotoParameters(hgpForTracking,trackedFrame,photoScalar,photoBias);
    trackedFrame->set_photo_parameters(photoScalar,photoBias);
}

void IDNav::Tracker::set_config_tracking_ceres(const dataType& parameter_tolerance_ , const int& numIt_){
    higherPyrLevel = int((PYR_LEVELS_MAX-1)*(pow(numHgpAvailable,2)/(pow(numHgpAvailable + numFtAvailable,2)))) ;
    lowerPyrLevel = 0;
    //if(numHgpAvailable > numFtAvailable){
        //higherPyrLevel =  (trackedFrame->cam)->getNumPyrLevelsUsed() - 1;
    //}
    //ceresTracker->solverCoarsePoseOptions.parameter_tolerance = parameter_tolerance_; //1e-8;*/
    //ceresTracker->solverCoarsePoseOptions.max_num_iterations = numIt_;
}

void IDNav::Tracker::set_config_refineTracking_ceres(const dataType& parameter_tolerance, const int& numIt){
    //ceresTracker->solverRefinedPoseOptions.parameter_tolerance = 1e-9;parameter_tolerance; //1e-8;*/
    //ceresTracker->solverRefinedPoseOptions.max_num_iterations = numIt;
}

void IDNav::Tracker::set_config_estimatePhotoParameters_ceres(const dataType& parameter_tolerance_ , const int& numIt_){
    //ceresTracker->solverPhotoParametersOptions.parameter_tolerance = parameter_tolerance_; //1e-8;*/
    //ceresTracker->solverPhotoParametersOptions.max_num_iterations = numIt_;
}

bool IDNav::Tracker::selectPointsforTracking(const Pose& pose0_, dataType poseGuess_[6]){
    ceresTracker->updatePose(trackedFrame, poseGuess_ , pose0_);
    {
        hgpForTracking.clear();
        hgpForTrackingMap.clear();
        for (auto &[keyframe, hgp] : clustered_hgpForTracking) {
            keyframe->fromKeyframeProjectHgpTo(trackedFrame,hgp);
            for(typePt& pt: hgp) {
                if (trackedFrame->cam->isPointIn(pt)) {
                    hgpForTracking.push_back(pt);
                    hgpForTrackingMap.insert({pt,true});
                }
            }
        }

        featuresForTracking.clear();
        featuresForTrackingMap.clear();
        for (auto &[keyframe, features] : clustered_featuresForTracking) {
            for(typeFt& ft: features) {
                featuresForTracking.push_back(ft);
                featuresForTrackingMap.insert({ft, true});
            }
        }

    }
    trackedFrame->pose.copyPoseFrom(pose0_);

    return (hgpForTracking.size() + featuresForTracking.size()) > minNumPointsForTracking;
}
