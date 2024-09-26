
#include "../../include/trackingFrontEnd.h"

bool IDNav::TrackingFrontEnd::updateTrackingInformation(){
#ifdef COUT_COMPLETE_PIPELINE
    cout << "    [TrackingFrontend] updateTrackingInformation()" << endl;
#endif
#ifdef COMPLETE_PROFILING
    updateTrackingInformationProfiler.begin();
#endif
    // Compute tracking hessians
    mat6 totalHessian{};
    tracker->computeTrackingHessian(totalHessian);
    trackingInformationBits = computeDiffEntropy_pose(totalHessian) ;

    if(trackingInformationBits > refKeyframe_hc->trackingInformationBitsMax){
        refKeyframe_hc->trackingInformationBitsMax = trackingInformationBits;
        refKeyframe->trackingInformationBitsMax = trackingInformationBits;
        updateTrackingInformationThreshold();
    }

    //*(information_Log) <<
    //std::setprecision(16) << trackingInformationBits << " " << refKeyframe->trackingInformationBitsThreshold << "\n";

#ifdef COMPLETE_PROFILING
    updateTrackingInformationProfiler.end();
#endif
    return true;
}

void IDNav::TrackingFrontEnd::updateTrackingInformationThreshold(){

    // Get the mean value of the information in the local window
    informationMean = 0.0;
    for(auto& [idkey, keyframe]: keyframesLocalWindow){
        informationMean += keyframe->trackingInformationBitsMax;
    }
    informationMean /= keyframesLocalWindow.size();

    // Update tracking thresholds
    for(auto& [idkey, keyframe]: keyframesLocalWindow){
        keyframe->trackingInformationBitsThreshold = getInformationThreshold(keyframe->trackingInformationBitsMax);
        (*keyframes)[idkey]->trackingInformationBitsThreshold = keyframe->trackingInformationBitsThreshold;
    }

}

bool IDNav::TrackingFrontEnd::thereIsEnoughInformationForTracking(){
    return (trackingInformationBits > refKeyframe_hc->trackingInformationBitsThreshold);
}

void IDNav::TrackingFrontEnd::setMaxInformationLoss( dataType& maxInformationLoss_) {
        maxInformationLoss = maxInformationLoss_;
}

void IDNav::TrackingFrontEnd::changeMaxInformationLoss(dataType& infSigmoid_max_){
    setMaxInformationLoss(infSigmoid_max_);
    updateTrackingInformationThreshold();
}


IDNav::dataType IDNav::TrackingFrontEnd::getInformationThreshold(dataType& inf_){
    return (inf_ - maxInformationLoss) - (inf_ - informationMean)*0.5;
}