//
// Created by afontan on 7/6/21.
//
#include "../../include/Tracker/CeresTracker.h"


void IDNav::CeresTracker::estimatePoseCoarse(vecPt& hgpForTracking_, vecFt& featuresForTracking_,
                                             typeFrame& trackedFrame_, dataType poseGuess_ [6]) const {

    Pose pose0{trackedFrame_->pose};

    ceres::Problem::Options problemOptions;
    auto *eval_function = new CeresTracker::evaluationCallbackPose(trackedFrame_, &pose0, poseGuess_);
    problemOptions.evaluation_callback = eval_function;
    ceres::Problem problemPose{problemOptions};

    for (typePt & pt: hgpForTracking_) {
        ceres::CostFunction *cost_function = new PhotometricReprojectionErrorCoarse(pt, trackedFrame_);
        problemPose.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentTh_patch), poseGuess_);
    }

    for (typeFt& ft: featuresForTracking_) {
        ceres::CostFunction *cost_function = new CeresTracker::GeometricReprojectionError(ft, trackedFrame_);
        problemPose.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentTh_reproj), poseGuess_);
    }

    ceres::Solver::Summary poseSummary{};
    ceres::Solve(solverCoarsePoseOptions, &problemPose, &poseSummary);
    trackedFrame_->pose.copyPoseFrom(pose0);

    delete eval_function;
}

void IDNav::CeresTracker::estimatePose(vecPt& hgpForTracking_, vecFt& featuresForTracking_, typeFrame& trackedFrame_,
                                       dataType updatePose_ [6]) const{

    Pose pose0{trackedFrame_->pose};

    ceres::Problem::Options problemOptions;
    auto *eval_function = new CeresTracker::evaluationCallbackPose(trackedFrame_, &pose0, updatePose_);
    problemOptions.evaluation_callback = eval_function;
    ceres::Problem problemPose{problemOptions};

    for (typePt & pt: hgpForTracking_) {
        ceres::CostFunction *cost_function = new PhotometricReprojectionError(pt, trackedFrame_);
        problemPose.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentTh_patch), updatePose_);
    }

    for (typeFt& ft: featuresForTracking_) {
        ceres::CostFunction *cost_function = new CeresTracker::GeometricReprojectionError(ft, trackedFrame_);
        problemPose.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentTh_reproj), updatePose_);
    }

    ceres::Solver::Summary poseSummary{};
    ceres::Solve(solverRefinedPoseOptions, &problemPose, &poseSummary);
    trackedFrame_->pose.copyPoseFrom(pose0);

    delete eval_function;

}

void IDNav::CeresTracker::estimatePhotoParameters(vecPt& hgpForTracking_, typeFrame& trackedFrame_, dataType& photoScalar_,dataType& photoBias_) const {

    dataType photoScalar0{photoScalar_};
    dataType photoBias0{photoBias_};

    ceres::Problem::Options problemOptions;
    auto *eval_function = new CeresTracker::evaluationCallbackPhotoParameters(trackedFrame_, &photoScalar_, &photoBias_);
    problemOptions.evaluation_callback = eval_function;
    ceres::Problem problemPhotoParameters{problemOptions};

    problemPhotoParameters.AddParameterBlock(&photoScalar_, 1);
    problemPhotoParameters.AddParameterBlock(&photoBias_, 1);

    for (typePt & pt: hgpForTracking_) {
        ceres::CostFunction *cost_function = new PhotometricParametersReprojectionError(pt, trackedFrame_);
        problemPhotoParameters.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentTh_patch_photoParameters), &photoScalar_, &photoBias_);
    }

    /*if(!hgpForTracking_.empty()){
        problemPhotoParameters.SetParameterLowerBound(&photoScalar_, 0, -0.04);
        problemPhotoParameters.SetParameterUpperBound(&photoScalar_, 0, 0.04);
        problemPhotoParameters.SetParameterLowerBound(&photoBias_, 0, -10.0);
        problemPhotoParameters.SetParameterUpperBound(&photoBias_, 0, 10.0);
    }*/

    ceres::Solver::Summary photoParametersSummary{};
    ceres::Solve(solverPhotoParametersOptions, &problemPhotoParameters, &photoParametersSummary);

    trackedFrame_->set_photo_parameters(photoScalar0,photoBias0);

    delete eval_function;

}