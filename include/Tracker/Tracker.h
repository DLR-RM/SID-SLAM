//
// Created by font_al on 1/31/20.
//

#ifndef RIA_H
#define RIA_H
#include "CeresTracker.h"

namespace IDNav{
    class Tracker {

    public: // Public members

        Profiler trackFrameProfiler{"profiler" , "    " ,"Tracker","trackFrame",LIGTH_YEL_COUT};
        typeFrame refKeyframe{};
        bool emergencySettings{false};
        float inliersRatio{0.0};
        float inliersRatio_hgp{0.0};
        float inliersRatio_features{0.0};
        double photoErrorMean{};
        const double photoErrorMeanMax0{15.0};
        double photoErrorMeanMax{photoErrorMeanMax0};
        float minInliersRatio0{0.7};
        float minInliersRatio{minInliersRatio0};
        std::ofstream* covCorr_Log{};

    private: // Private members

        // Tracking state
        bool trackingSucceed{false};

        // Tracking Members
        typeFrame trackedFrame{};
        unordered_map<size_t,typeFrame> keyframes{};

        // Map members
        vecPt hgpForTracking{};
        unordered_map<typeFrame,vecPt> clustered_hgpForTracking{};
        unordered_map<typePt,bool> hgpForTrackingMap{};
        vecFt featuresForTracking{};
        unordered_map<typeFrame,vecFt> clustered_featuresForTracking{};
        unordered_map<typeFt,bool> featuresForTrackingMap{};
        size_t numHgpAvailable{0};
        size_t numFtAvailable{0};

        // Ceres Tracker
        unique_ptr<CeresTracker> ceresTracker{};
        const double tstudent2_p_init{97.5};
        double tstudent2_p_refine{95.0};
        int numDofPhoto{9};
        int numDofReproj{9};

        const size_t minNumPointsForTracking{24};

        // Covariance Estimation
        dataType onlinePhotoCorr{};
        dataType onlineReprojCorr{};

        // Outlier rejection
        size_t numHgpAvailable_refKey{0};
        size_t numFtAvailable_refKey{0};
        size_t numHgpInliers_refKey{0};
        size_t numFtInliers_refKey{0};

        // Input variables
        Pose poseInput;
        dataType phScalar_input{0};
        dataType phBias_input{0};

        // Scale pyramid info
        int higherPyrLevel{};
        int lowerPyrLevel{};
        int iPyr{};

        // Kinematic prior
        vec6 speedPrior{vec6::Zero()};
        mat6 covInvPrior{mat6::Zero()};

        // Emergency settings
        float minInliersRatioEmergency{0.5f*minInliersRatio0};

    public: // Public methods
        explicit Tracker(){
            ceresTracker = std::make_unique<CeresTracker>();
        }

        void trackFrame(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_,
                        const Pose& pose0_ ,
                        const vec6& speed_ , const mat6& speedCovInv_);
        bool isTrackingSuccessful();
        void computeTrackingHessian(mat6& totalHessian_);

        void set_tstudent2Probability(const dataType& tstudent2_p_, const int& numDofPhoto_, const int& numDofReproj_){
            numDofPhoto = numDofPhoto_;
            numDofReproj = numDofReproj_;
            tstudent2_p_refine = tstudent2_p_;
            ceresTracker->set_tstudent2(tstudent2_p_refine, numDofPhoto, numDofReproj);
        }

        void set_minInliersRatio(const float& minInliersRatio_){
            minInliersRatio0 = minInliersRatio_;
            minInliersRatio = minInliersRatio0;
            minInliersRatioEmergency = 0.5*minInliersRatio0;
        }
    private: // Private methods

        void setInitialState(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_,
                             const Pose& pose0_ ,
                             const vec6& speed_, const mat6& speedCovInv_ );
        void clusterPoints(unordered_map<size_t,typeFrame>& keyframes_);
        void initProjectionCovariances();
        void saveInputState();

        void tracking_ceres(dataType parameter_tolerance_ = 1e-3 , int numIt_  = 20);
        void set_config_tracking_ceres(const dataType& parameter_tolerance_ , const int& numIt_);
        bool selectPointsforTracking(const Pose& pose0_, dataType poseGuess_[6]);

        void setRefineState();
        void estimateCovariancesWithModel();
        void set_config_estimatePhotoParameters_ceres(const dataType& parameter_tolerance_, const int& numIt_);
        IDNav::Tracker* correctCovariancesFromResiduals();
        void estimateCovariancesFromResiduals(dataType& photoStdOnline_, dataType& reprojStdOnline_);
        void eraseOutliers();
        void refine_tracking_ceres(dataType parameter_tolerance_ = 1e-2 , int numIt_  = 15);
        void set_config_refineTracking_ceres(const dataType& parameter_tolerance, const int& numIt);

        void checkTrackingSucceed();
        void finishOptimization();
        void resetOptimization();

        void relocalizeFrame(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_,
                             const Pose& pose0_ ,
                             const vec6& speed_ , const mat6& speedCovInv_,
                             mat4& Tji);

    public: // Getters and Setters

        vecPt* get_hgpForTracking() {return &hgpForTracking;}
        int get_num_hgpForTracking() {return (int)hgpForTracking.size();}
        unordered_map<typePt,bool>* get_hgpForTrackingMap(){return &hgpForTrackingMap;};

        vecFt* get_featuresForTracking() {return &featuresForTracking;}
        int get_num_featuresForTracking() {return (int)featuresForTracking.size();}
        unordered_map<typeFt,bool>* get_featuresForTrackingMap(){return &featuresForTrackingMap;};

        dataType get_onlinePhotoCorr() const {return onlinePhotoCorr;};
        dataType get_onlineReprojCorr() const {return onlineReprojCorr;};

        void setEmergencySettings(){
            emergencySettings = true;
            minInliersRatio = minInliersRatioEmergency;
            photoErrorMeanMax = 1.5*photoErrorMeanMax0;
        }

        void resetSettings(){
            emergencySettings = false;
            minInliersRatio = minInliersRatio0;
        }

        // EXCEPTIONS
        class TrackerFail: public std::exception {
        private:
            std::string message;
        public:
            explicit TrackerFail(const char* errorMessage = "\nTracker Failed"): message(errorMessage){};
            std::string errorMessage() const {return message;};
        };


    };
}
#endif //ID_RIA_H
