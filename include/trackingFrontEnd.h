
#ifndef TRACKINGFRONTEND_H
#define TRACKINGFRONTEND_H

#include "Tracker/Tracker.h"
#include "MotionEstimator.h"
#include "PointSelector.h"
#include "Optimizer/Projection.h"
#include "PoseGraphOptimizer.h"
#include "Optimizer/OptimizerCeres.h"

namespace IDNav {

    class RelativePose{
    public:
        mat3 Rwc{};
        vec3 twc{};
        size_t refKeyId{};
        dataType timestamp{};
        vec3 trel{};
        mat3 Rrel{};
        bool isKeyframe{false};
        RelativePose(typeFrame& frame_, typeFrame& refKeyframe_ ){
            Rwc = frame_->pose.Rwc;
            twc = frame_->pose.twc;

            timestamp = frame_->grayImgTs;
            refKeyId = refKeyframe_->keyId;

            if(frame_->frameId == refKeyframe_->frameId){
                setKeyframe(refKeyframe_);
            }else{
                setRelPose(frame_, refKeyframe_);
            }
        }

        void getAbsolutePoseInTUMformat(vec3& twc_, mat3& Rwc_, typeFrame& refKeyframe_){
            Rwc_ = Rrel*(refKeyframe_->pose.Rwc);
            twc_ = Rrel*(refKeyframe_->pose.twc) + trel;

            //Rwc_ = Rwc;
            //twc_ = twc;
            //qwc = Eigen::Quaterniond(Rwc);
        }

        void setRelPose(typeFrame& frame_, typeFrame& refKeyframe_){
            Rrel = (frame_->pose.Rwc)*(refKeyframe_->pose.Rcw);
            trel = (frame_->pose.Rwc)*(refKeyframe_->pose.tcw) + (frame_->pose.twc);
        }

        void setKeyframe(typeFrame& refKeyframe_){
            trel = vec3::Zero();
            Rrel = mat3::Identity();
            isKeyframe = true;
            refKeyId = refKeyframe_->keyId;
        };
    };

    class TrackingFrontEnd {

    public: // Public members

        std::ofstream* information_Log{};
        int numKeyframesCreatedWithInformation{0};
        int numKeyframesCreatedBecauseTrackingLost{0};
        bool relocalizationMode{false};

        // Profilers
        IDNav::Profiler processImageProfiler{"profiler" , "" ,"TrackingFrontend","loadImage"};
        IDNav::Profiler trackingProfiler{"profiler" , "" ,"TrackingFrontend","trackFrame"};
#ifdef COMPLETE_PROFILING
        IDNav::Profiler selectReferenceKeyframeProfiler{"profiler" , "    " ,"TrackingFrontend","selectReferenceKeyframe"};
        IDNav::Profiler updateTrackingInformationProfiler{"profiler" , "    " ,"TrackingFrontend","updateTrackingInformation"};
        IDNav::Profiler trackingSucceedProfiler{"profiler" , "    " ,"TrackingFrontend","trackingSucceed"};
        IDNav::Profiler getMapProfiler{"profiler" , "    " ,"TrackingFrontend","getMap"};
        IDNav::Profiler getKeyframesForTrackingProfiler{"profiler" , "    " ,"TrackingFrontend","getKeyframesForTracking"};
#endif
        IDNav::Profiler addKeyframeProfiler{"profiler" , "    " ,"TrackingFrontend","addKeyframe"};

        // Cameras
        shared_ptr<Camera> cameraTracking{};
        shared_ptr<Camera> cameraWindowOptimization{};

        // Tracking members
        typeFrame trackedFrame{};
        typeFrame refKeyframe{};
        typeFrame refKeyframe_hc{nullptr};
        unordered_map<size_t,typeFrame> *keyframes{};
        unordered_map<size_t,typeFrame> keyframesLocalWindow{};
        unordered_map<size_t,typeFrame> keyframesForTracking{};
        unordered_map<size_t,typeFrame> keyframesToOptimize{};

        // Window optimization
        int numKeyframesWindowOptimization{10};
        int numKeyframesCreated{0};

        bool newWindowToOptimize{false};
        bool stopWindowOptimization{false};
        bool windowOptimizationFinished{true};
        bool prepareWindowOptimizationInProgress{false};

        // Information Members
        dataType trackingInformationBits{};
        vector<dataType> trackingInformationBitsVector{};

        // Information Sigmoid
        dataType informationMean{};
        const dataType minInformation{38.0};
        dataType maxInformationLoss{4.0};

        // Robust Image Alignment (Tracker)
        std::unique_ptr<IDNav::Tracker> tracker{};

        SequenceSettings* seqSet{};
        std::shared_ptr<IDNav::Frame> previousFrame{};
        map<size_t,shared_ptr<RelativePose>> trajectory{};
        size_t numTrackedFrames{0};
        mat3 Rrel_refKeyframe{mat3::Identity()};
        vec3 trel_refKeyframe{vec3::Zero()};
        mat3 Rrel_previousFrame_refKeyframe{mat3::Identity()};
        vec3 trel_previousFrame_refKeyframe{vec3::Zero()};

        std::unique_ptr<IDNav::PointSelector> pointSelector{};
        unique_ptr<PoseGraphOptimizer> poseGraphOptimizer{};
        std::unique_ptr<IDNav::OptimizerCeres> windowOptimizer{};
        std::unique_ptr<IDNav::OptimizerCeres> globalOptimizer{};
        std::unique_ptr<LCDetectorAKAZE> lcd{};
        //std::shared_ptr<LCDetectorAKAZE> lcd{};

        bool switchedKeyframe{false};
        bool createdNewKeyframe{false};
        bool triggeredLoop{false};

        std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>> obs_hgp{};// < hostKey, projKey >
        std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>> obs_features{};

        // Keyframe Covisibility
        dataType maxKeyframeDistance{1.0};
        dataType minVisiblePointsRatio{0.2};

        bool visualization{true};

    private: // Private members

        // Motion Estimator
        std::unique_ptr<IDNav::MotionEstimator> motionEstimator{};
        vec6 speed{};
        mat6 speedCovInv{};

    public: // Public methods

        bool loadImage(Mat& grayImg_, dataType& grayImgTs_,
                       Mat& depthImage_, dataType& validDepth_,
                       size_t& id_,
                       Mat& maskImg_);
        bool initFrontEnd();
        void trackFrame();
        void windowOptimization();

        static void get_N_covisibleKeyframes(unordered_map<size_t,typeFrame>& covKeyframes,
                                             typeFrame& refKeyframe_,
                                             unordered_map<size_t,typeFrame>& keyframes_,
                                             unordered_map<size_t,Covisibility>& keyframesCov_,
                                             const double maxDistance_,
                                             const double minVisiblePointsRatio_,
                                             int maxCovKeyframes);

    private: // Private methods

        // Tracking methods
        void initTracking();
        void saveFrameInTrajectory();
        void computeOpticalFlow();
        void trackingSucceed();
        void bufferFrame();
        void selectReferenceKeyframe();
        void getRelativePoseToReferenceKeyframe();
        bool needNewKeyframe();
        void emergencyTracking();
        void getTrackingMap();
        void getKeyframesForTracking();
        void finishTracking();

        static void get_N_covisibleKeyframes(unordered_map<size_t,typeFrame>& covKeyframes,
                                             typeFrame& refKeyframe_,
                                             typeFrame& trackedFrame_,
                                             unordered_map<size_t,typeFrame>& keyframes_,
                                             unordered_map<size_t,Covisibility>& keyframesCov_,
                                             const double maxDistance_,
                                             const double minVisiblePointsRatio_,
                                             int maxCovKeyframes);

        void get_N_covisibleKeyframes_toOptimize(unordered_map<size_t,typeFrame>& covKeyframes,
                                                 typeFrame& refKeyframe_,
                                                 const double maxDistance_,
                                                 const double minVisiblePointsRatio_,
                                                 int maxCovKeyframes);

        // Add keyframe
        bool addKeyframe(typeFrame frameToAdd_);
        void initAddKeyframe();
        void updateFeatureObservations(typeFrame& newKeyframe_);
        void addKeyframeToWindow(typeFrame& newKeyframe_);
        void setNewKeyframeAsReferenceKeyframe(typeFrame& newKeyframe_);
        void connectFrames(typeFrame& frame1_, typeFrame& frame2_);
        void connectFramesConditioned(typeFrame& frame1_, typeFrame& frame2_);
        void connectFrames(const typeFrame& frame1_, const typeFrame& frame2_, mat4& Tji_);
        void finishAddKeyframe(std::shared_ptr<IDNav::Frame>& newKeyframe);

        // Tracking Information
        bool updateTrackingInformation();
        void updateTrackingInformationThreshold();
        bool thereIsEnoughInformationForTracking();
        dataType getInformationThreshold(dataType& infMax_);

        // Window Optimization
        void setRelativePoseToReferenceKeyframe();

        // Relocalization
        void addKeyframeToLcd(typeFrame& newKeyframe);
        void triggerLoops();
        bool isLoopConsistent(LCD::Result& loopResult_);
        void addConsistentLoop(LCD::Result& consistentLoop_);
        void addInconsistentLoop(LCD::Result& consistentLoop_);
        void triggerLoopClosure(std::vector<LCD::Result>& inconsistentLoops_);
        void performRelocalization();
        dataType getLoopImageError(typeFrame& dKeyframe_, typeFrame& qKeyframe_, LCD::Result& loopResult_);

    public:

        explicit TrackingFrontEnd(unordered_map<size_t,typeFrame> *keyframes_,
                                  SequenceSettings* seqSet_,
                                  shared_ptr<Camera>& cameraTracking_,
                                  shared_ptr<Camera>& cameraWindowOptimization_
        ): keyframes(keyframes_),seqSet(seqSet_){

            tracker = std::make_unique<IDNav::Tracker>();
            motionEstimator = std::make_unique<IDNav::MotionEstimator>(seqSet->RGB_seconds);
            pointSelector = std::make_unique<IDNav::PointSelector>(cameraTracking_);
            poseGraphOptimizer = std::make_unique<PoseGraphOptimizer>();
            windowOptimizer = std::make_unique<IDNav::OptimizerCeres>(keyframes,"local");
            globalOptimizer = std::make_unique<IDNav::OptimizerCeres>(keyframes,"global");

            LCD::LCDOptions options;
            options.relative_desc_dist = 0.1;
            options.match_ratio = 0.01;
            options.visualize_matches = true;
            //options.keyframe_selection_strategy = LCD::KFStrategy::KF_RANGE_SEL;
            options.keyframe_selection_strategy = LCD::KFStrategy::KF_NOCOV_SEL;

            //options.n_keyframe_nomatch = n_buff;
            options.is_verbose = false;
            lcd = std::make_unique<LCDetectorAKAZE>(options);

            //lcd = std::make_unique<LCDetectorAKAZE>(LCD::KFStrategy::KF_NOCOV_SEL);
            //lcd = std::make_unique<LCDetectorAKAZE>(LCD::KFStrategy::KF_RANGE_SEL, 10);
            //lcd->visualizeMatch(true);
            //LCDetectorAKAZE::verbosity_level = LCD::Verbosity::WARN;
            //LCDetectorAKAZE::verbosity_level = LCD::Verbosity::INFO;
            LCDetectorAKAZE::verbosity_level = LCD::Verbosity::NONE;

            cameraWindowOptimization = cameraWindowOptimization_;
            cameraTracking = cameraTracking_;
        };

        void setMaxInformationLoss(dataType& infSigmoid_max_);
        void changeMaxInformationLoss(dataType& infSigmoid_max_);
        void setKeyframeCovisibility(const dataType& maxKeyframeDistance_ , const dataType& minVisiblePointsRatio_);
        void set_numKeyframesWindowOptimization(const int& numKeyframesWindowOptimization_);

        // EXCEPTIONS
        class TrackingFrontEndFail: public std::exception {
        private:
            std::string message;
        public:
            explicit TrackingFrontEndFail(const char* errorMessage = "\nTracking FrontEnd Failed"): message(errorMessage){};
            std::string errorMessage() const {return message;};
        };
    };
}

#endif //TRACKINGFRONTEND_H
