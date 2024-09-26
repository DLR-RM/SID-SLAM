//
// Created by font_al on 5/17/19.
//

#ifndef IDNAV_POINTSELECTOR_H
#define IDNAV_POINTSELECTOR_H

#include "Frame.h"
#include "../utils/informationFunctions.h"

//#include <iostream>
//#include "opencv2/core.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;

namespace IDNav {

    class PointSelector{

    public: // Public members

#ifdef ACTIVE_FEATURES
    std::shared_ptr<cv::AKAZEFeaturesV2> akazeEvolution;
#endif
        bool trackFeaturesSuccesful{true};
        dataType featureBias{0.0};
        int numPointsToSelectWithInformation{32};
        int numPointsToExtract{64};

#ifdef ACTIVE_FEATURES
#ifdef COMPLETE_PROFILING
        IDNav::Profiler extractKeypointsProfiler{"profiler" , "    " ,"PointSelector","extractKeypoints"};
        IDNav::Profiler trackFeaturesProfiler{"profiler" , "    " ,"PointSelector","trackFeatures"};
#endif
#endif

    private: // Private members

#ifdef ACTIVE_FEATURES
        // Features
        cv::AKAZEOptionsV2 akazeOptions{};
        unordered_map<typeFrame,vecFt>availableFeaturesClustered{};
        const dataType maxOpticalFlow{20.0};
        const dataType minOpticalFlow{10.0};
        dataType akazeTh0{1.0};
        dataType distanceTh0{0.9};
        dataType opticalFlow{maxOpticalFlow};
        const float succesfulRatioFeaturesTracked{0.5};
#endif


#ifdef COUT_COMPLETE_PIPELINE
        // Profilers
        Profiler extractPointsProfiler{"profiler" , "    |   " ,"PointSelector","extractPoints",GREEN_COUT};
        Profiler hgpExtractionProfiler{"profiler" , "    |   |   " ,"PointSelector","extract_hgp",GREEN_COUT};
        Profiler hgpAssociationProfiler{"profiler" , "    |   |   " ,"PointSelector","associate_hgpToFrame",GREEN_COUT};
        Profiler informativePointSelectionProfiler{"profiler" , "    |   |   ","PointSelector","selectInformativePoints",GREEN_COUT};
        #ifdef ACTIVE_FEATURES_TRACKING
        Profiler featureExtractionProfiler{"profiler" , "    |   |   " ,"PointSelector","extract_features",GREEN_COUT};
        Profiler featureAssociationProfiler{"profiler" , "    |   |   " ,"PointSelector","associate_featuresToFrame",GREEN_COUT};
        Profiler featureTrackingProfiler{"profiler" , "    " ,"PointSelector","trackFeatures",GREEN_COUT};
        #endif
#endif

    public: // Public methods
        void trackFeatures(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_, typeFrame& refKeyframe_, vec6& speedPrior_);
        void extractKeypoints(vecKeyPt& keypoints_);

    private: // Private methods
        void tune_akaze_threshold(const int& last_nkp_);
        void trackFeatures(dataType& windowSize_, dataType& akazeTh_, dataType& distanceTh_,
                           typeFrame& trackedFrame_);
    private:

        // Extract points with grid and kinect information
        float maxDepthKinect{MAX_DEPTH_VALUE};
        ImageGrid imageGrid{};
        float minPhotoGradient{15.0f};

        // Plane estimation
        ceres::Solver::Options solveOptionsPlaneEstimate{};
        ceres::Solver::Summary summaryPlaneEstimate{};

        // Informative point selection
        vecPt temp_hgp{};
        vecFt temp_features{};
        matX poseJacobiansMatrix_hgp{};
        matX poseJacobiansMatrix_u{};
        matX poseJacobiansMatrix_v{};
        matX informationContribution{};
        mat6 poseHessian{};
        mat6 poseHessianInverse{};
        matX similarity{};
        matX observability{};
        matX scores{};
        int bestPtRow{};
        dataType maxEntropyContribution{};
        dataType maxSimilarityContribution{};
        const dataType balanced_entropy_similarity{1.0};
        dataType trackingInformationBits;
        Eigen::NoChange_t colsEigen{};

        // This is just an auxiliary struct used in the function: extract_hgp_grid_kinect()
        struct ptTemp{
            int uRef,vRef;
            float depthRef;
            ptTemp(int uRef, int vRef, float depthRef, short G_ref):uRef(uRef),vRef(vRef),depthRef(depthRef){}
        };


        // Feature extractor
        dataType minResponse0{0.0001};
        size_t numOctaves0{1};
        size_t numSublevels0{1};
        const size_t maxNumSublevels{4};
        size_t numThreads0{1};


        dataType hgp_balance{0.5};
        dataType ft_balance{1.0-hgp_balance};


    public:

        explicit PointSelector(shared_ptr<Camera> cam_) : cam(cam_) {
            imageGrid.build(cam->get_w(0),cam->get_h(0));
#ifdef ACTIVE_FEATURES
            setAkazeEvolution();
#endif
            solveOptionsPlaneEstimate.linear_solver_type = ceres::DENSE_QR;
            solveOptionsPlaneEstimate.minimizer_type = ceres::TRUST_REGION;
            solveOptionsPlaneEstimate.minimizer_progress_to_stdout = false;
            solveOptionsPlaneEstimate.ceres::Solver::Options::update_state_every_iteration = true;
            solveOptionsPlaneEstimate.use_inner_iterations = false;
            solveOptionsPlaneEstimate.num_threads = 1;
            //solveOptionsPlaneEstimate.max_num_iterations = 1;
            solveOptionsPlaneEstimate.check_gradients = false;
        };

        bool extractPoints(typeFrame& frame_, unordered_map<size_t,typeFrame>& keyframes_);
        void setNumPointsToSelect(const int& numPointsToSelectWithInformation_, const int& numPointsToExtract_){
            numPointsToSelectWithInformation = numPointsToSelectWithInformation_;
            numPointsToExtract = numPointsToExtract_;
        }

        void setFeatureBias(const double& featureBias_){
            featureBias = featureBias_;
        }

        static bool thereIsDepthKinect(float& depthOutput_, const int& uRef_, const int&vRef_, const Mat& depthImg_, const float& maxDepthKinect_);

    private:

        void extract_features(vecFt& features_ , vecFt& allFeatures_,
                              const vecKeyPt& keypoints0_, const Mat& grayImg_, const Mat& depthImg_, Mat& maskImg_);
        void extract_features_grid_kinect(vecFt& features_, vecFt& allFeatures_,
                                          const vecKeyPt& keypoints0,
                                          const Mat& grayImg_, const Mat& depthImg_, Mat& maskImg_,
                                          const int numFeaturesToExtract_ = 64,
                                          const float& maxDepthKinect_ = MAX_DEPTH_VALUE);
        void filterFeaturesWithDepth(vecFt& allFeatures_ ,vecKeyPt& keypoints_,
                                     const Mat& maskImg_,
                                     const Mat& depthImg_,
                                     const float& maxDepthKinect_ = MAX_DEPTH_VALUE) const ;
        void computeDescriptors(vecKeyPt& keypoints_, Mat& allDescriptors_);
        void computeAndAssociateDescriptors(const vecFt& features_, vecKeyPt& keypoints_);
        void filterFeaturesWithResponse(vecFt& features, vecFt& allFeatures, Mat& maskImg_);
        void updateMask(vecFt& features_ , Mat& maskImg_);

        void extract_hgp(vecPt& hgp_, dataType& imgGradient_, const Mat& grayImg_, const Mat& depthImg_, const Mat& maskImg_); // 03/02/22
        static void extract_hgp_grid_kinect(vecPt& hgp_, dataType& imgGradient_, const Mat& grayImg_, const Mat& depthImg_, const Mat& maskImg_,
                                     const ImageGrid& imageGrid_,
                                     const int num_hgpToExtract_ = 64,
                                     const float& maxDepthKinect_ = MAX_DEPTH_VALUE,
                                     const float& minPhotoGradient_ = 15.0);
        static void gradFunction(cv::Mat &grad_x_, cv::Mat &grad_y_, float& gradientScale_, float& pixelBias_, const cv::Mat& grayImg_ , const int method_ = 0) ;
        static void filterAndAddPoints(vector<ptTemp>& pointsOut_, const vector<ptTemp>& pointsIn_, const int& numPoints_ );

        void selectInformativePoints(typeFrame& frame_);
        void computePoseRefJacobiansMatrix();
        void selectInitialPoints();
        void computeInformationContribution();
        void findBestPoint();
        void addPoint(const int& row);
        void updateSimilarity(const IDNav::typeIdNavPoint pt_);
        void removePoint(const int& row);

    public:
        //


        void retrackFeatures(typeFrame& trackedFrame_, unordered_map<size_t,typeFrame>& keyframes_);
        void reuse_observations(typeFrame& frame_ ,  unordered_map<size_t,typeFrame>& keyframes_, Mat& maskImg_);

        // EXCEPTIONS
        class PointSelectorFail: public std::exception {
        private:
            std::string message;
        public:
            explicit PointSelectorFail(const char* errorMessage = "\nPoint Selector Failed"): message(errorMessage){};
            std::string errorMessage() const {return message;};
        };

    private:

        //
        shared_ptr<Camera> cam{};
        typeFrame frame{};
        vecFrame keyframes{};


    public:
        void setMinPhotoGradient(const float& minPhotoGradient_){
            minPhotoGradient = minPhotoGradient_;
        }

        void setFeaturesMinResponse(const dataType& minResponse_){
#ifdef ACTIVE_FEATURES
                minResponse0 = minResponse_;
                akazeOptions.dthreshold = float(minResponse0);
                cout << "[PointSelector]: akaze dthreshold = " << akazeOptions.dthreshold << endl;
#endif
        }

        void setFeaturesNumOctaves(const size_t& numOctaves_){
#ifdef ACTIVE_FEATURES
            numOctaves0 = numOctaves_;
            akazeOptions.omax = int(numOctaves0);
            cout << "[PointSelector]: akaze #octaves = " << akazeOptions.omax << endl;
#endif

        }

        void setFeaturesNumSublevels(const size_t& numSublevels_){
#ifdef ACTIVE_FEATURES
            if(numSublevels_ > maxNumSublevels){
                numSublevels0 = maxNumSublevels;
                akazeOptions.nsublevels = int(maxNumSublevels);
            }else{
                numSublevels0 = numSublevels_;
                akazeOptions.nsublevels = int(numSublevels_);
            }
            cout << "[PointSelector]: akaze #sublevels = " << akazeOptions.nsublevels << endl;
#endif
        }

        void setFeaturesNumThreads(const size_t& numThreads_){
            numThreads0 = numThreads_;
            cv::setNumThreads(int(numThreads0));
            cout << "[PointSelector]: opencv #threads = " << numThreads0 << endl;
        }
        void setAkazeEvolution(){
#ifdef ACTIVE_FEATURES
            akazeOptions.descriptor_size = 0;
            akazeOptions.dthreshold = float(minResponse0); // response threshold
            akazeOptions.soffset = 1.6;
            akazeOptions.omax = int(numOctaves0);
            akazeOptions.nsublevels = int(numSublevels0);
            akazeOptions.img_height = int(cam->get_h(0));
            akazeOptions.img_width = int(cam->get_w(0));
            //akaze_options.descriptor_channels = 1;
            cv::setNumThreads(int(numThreads0));
            akazeEvolution.reset(new cv::AKAZEFeaturesV2(akazeOptions));
#endif
        }

        void setOpticalFlow(const dataType& opticalFlow_){
#ifdef ACTIVE_FEATURES
#ifdef COUT_COMPLETE_PIPELINE
            cout << "[PointSelector] setOpticalFlow("<< opticalFlow_<<") ---> [" << minOpticalFlow<<"-"<<maxOpticalFlow<<"]"<< endl;
#endif
            opticalFlow = opticalFlow_;

            if(opticalFlow_ < minOpticalFlow)
                opticalFlow = minOpticalFlow;

            if(opticalFlow_ > maxOpticalFlow)
                opticalFlow = maxOpticalFlow;

#endif
        }

        void setFeatureTrackingParameters(const dataType& akazeTh_, const dataType& distanceTh_){
#ifdef ACTIVE_FEATURES
            akazeTh0 = akazeTh_;
            distanceTh0 = distanceTh_;
#endif
        }
        };
}
#endif //IDNAV_POINTSELECTOR_H
