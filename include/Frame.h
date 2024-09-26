//
// Created by font_al on 10/5/18.
//

#ifndef FRAME_H
#define FRAME_H

#include "Camera.h"
#include "../utils/Profiler.h"


namespace IDNav {
    void setVisibilityParameters(dataType& maxLambdaVis_);

    struct Covisibility{
        mat4 Tji{mat4::Identity()};
        dataType inliersRatio{0.0};
        dataType distance{0.0};
        Covisibility(mat4 Tji, dataType inliersRatio):Tji(Tji), inliersRatio(inliersRatio){
            distance = Tji.block<3,1>(0,3).norm();
        };
    };

    class Frame {
    public: // Public members

    private: // Private members

    public: // Public methods
        bool loadImage(Mat& rgbImg_, Mat& grayImg_, dataType& grayImgTs_,
                       Mat& depthImage_, dataType& validDepth_,
                       size_t& id_,
                       Mat& maskImg_);

    private: // Private methods

    public:
        size_t keyId{};
        size_t frameId{};
        size_t refKeyId{};

        // Points
        vecPt hgp{};
        vecPt hgp0{};
        vecPt hgpInf{};

        vecFt features{};
        vecFt featuresForLC{};
        //vecFt trackedFeatures{};
        unordered_map<typeFt,KeyPoint> observations{};

        vecKeyPt keypoints{};
        Mat descriptorsObject{};
        vector<bool> isDescriptorComputed{};
        int numTrackedFeatures{0};

        // Pose transformations
        Pose pose{};

        // Camera
        shared_ptr<Camera> cam{};

        // Intensity Images
        vector<Mat> grayImgG{};
        Mat grayImgDist{};
        dataType grayImgTs{};

        // Depth images
        Mat depthImg{};
        dataType validDepth{2};

        // Mask images
        Mat maskImg{};

        // Photo parameters
        dataType phScalar{0.0};
        dataType phScalarExp{1.0};
        dataType phBias{0.0};
        dataType imgGradient{150.0};

        // Information parameters
        dataType trackingInformationBitsMax{0.0};
        dataType trackingInformationBitsThreshold{0.0};

    public:

        unordered_map<size_t,Covisibility>keyframes{};

        // Visualization
        cv::Mat Mat_gray_clone;

        // Constructor
        explicit Frame(std::shared_ptr<IDNav::Camera>& camera_) {
            cam = camera_;
        };
        Frame()  = default;

        explicit Frame(std::shared_ptr<IDNav::Frame>& frame_) {

            frameId = frame_->frameId;
            grayImgTs = frame_->grayImgTs;
            validDepth = frame_->validDepth;
            pose = frame_->pose;
            cam = frame_->cam;

            phScalar = frame_->phScalar;
            phScalarExp = frame_->phScalarExp;
            phBias = frame_->phBias;
            imgGradient = frame_->imgGradient;

            trackingInformationBitsMax = 0.0;//frame_->trackingInformationBitsMax; ????????????????????????
            trackingInformationBitsThreshold = 0.0;//frame_->trackingInformationBitsThreshold; ????????????????????????

            for (int i{0}; i < frame_->grayImgG.size(); ++i){
                grayImgG.push_back(frame_->grayImgG[i].clone());
            }

            grayImgDist = frame_->grayImgDist.clone(); // ????????????????????????
            depthImg = frame_->depthImg.clone(); // ????????????????????????
            maskImg = frame_->maskImg.clone();
            Mat_gray_clone = frame_->Mat_gray_clone.clone();  // ????????????????????????

            keypoints = frame_->keypoints;
            observations = frame_->observations;
            descriptorsObject = frame_->descriptorsObject.clone();
            isDescriptorComputed = frame_->isDescriptorComputed;
        };

        void setPose(const Pose& poseEst_){
            pose.copyPoseFrom(poseEst_);
        }

        // Projection functions
        void uv_2_XYZ(typeIdNavPoint pt);
        void uv_2_XYZ(vecPt& hgp_);
        void uv_2_XYZ(vecFt& features_);
        void XYZ_2_uv(typeIdNavPoint pt);
        void XYZ_2_uv(vecPt& hgp_);
        void XYZ_2_uv(vecFt& features_);

        void projectPointsTo(shared_ptr<Frame>& frame);
        void projectHgpTo(shared_ptr<Frame>& frame_);
        void fromKeyframeProjectHgpTo(shared_ptr<Frame>& frame_, vecPt& hgp_);
        void projectFeaturesTo(shared_ptr<Frame>& frame_);

        void getVisiblePointsIn(shared_ptr<Frame> frame_, vecPt& hgp_, vecFt& features_);
        void getGeometryVisiblePointsIn(shared_ptr<Frame> frame_, vecPt& hgp_, vecFt& features_);
        void getInformativeVisibleHgpIn(shared_ptr<Frame> frame_, vecPt& hgp_);
        void getVisibleHgpIn(shared_ptr<Frame> frame_, vecPt& hgp_);
        void getVisibleFeaturesIn(shared_ptr<Frame> frame_, vecFt& features_);
        dataType getCovisibilityRatio(shared_ptr<Frame> frame_);


        // Photometric Error and Jacobians
        void get_PhotometricError(vecX& e, typePt& pt, const int& iPyr);
        void computeInformativeGeoJacobian(typeFt& ft, row26& duv_dx);
        void compute_inf_ph_poseJacobian(typePt& pt, mat16& dI_dx);


        //  Set of functions for extracting points in an image
        bool planeEstimation(dataType& alpha_ , dataType& beta_ , const typePt& pt, ceres::Solver::Options& problemOptions, ceres::Solver::Summary& summary,
                             const dataType& fx,const dataType& fy,
                             const dataType& cx,const dataType& cy,
                             const dataType& w, const dataType& h);
        bool planeEstimation(dataType& alpha_ , dataType& beta_, typeFt& ft, ceres::Solver::Options& problemOptions, ceres::Solver::Summary& summary,
                             const dataType& fx,const dataType& fy,
                             const dataType& cx,const dataType& cy,
                             const dataType& w, const dataType& h);

        //Set of functions for interpolating intensities and gradients in the different resolutions.
        bool extract_I_and_G_pixel_BilinInt(dataType& I, dataType& Gu, dataType& Gv, dataType& u, dataType& v);
        bool extract_I_and_G_pixel_Kernel(dataType& I, dataType& Gu, dataType& Gv, dataType& u, dataType& v);
        bool extract_I_pixel_BilinInt(dataType& I,  const dataType& u, const dataType& v);
        bool extract_I_pixel_Kernel(dataType& I,  const dataType& u, const dataType& v);

        void extract_I_and_G_BilinInt(shared_ptr<Pt>& pt);
        void extract_I_and_G_Kernel(shared_ptr<Pt>& pt);

        void extract_I_reference();
        void extract_I_and_G_ref_BilinInt(shared_ptr<Pt>& pt);
        void extract_I_and_G_ref_Kernel(shared_ptr<Pt>& pt);
        void extract_I_ref_BilinInt(shared_ptr<Pt>& pt);
        void extract_I_ref_Kernel(shared_ptr<Pt>& pt);

        void associate_hgpToFrame(vecPt& hgp_ ,
                                  ceres::Solver::Options& solveOptionsPlaneEstimate_,
                                  ceres::Solver::Summary& summaryPlaneEstimate_){
            const dataType fx{cam->getFocalLengthX()};
            const dataType fy{cam->getFocalLengthY()};
            const dataType cx{cam->getCentrePointX()};
            const dataType cy{cam->getCentrePointY()};
            const dataType w = (dataType)cam->get_w(0);
            const dataType h = (dataType)cam->get_h(0);

            // Init Variables
            hgp = hgp_;
            double numPlaneEstimationAchieved{0};
            for(typePt& pt: hgp){
                // Init Pointers
                pt->refKeyframe = this;
                pt->idKey = keyId;
                pt->refPhScalarExp = &phScalarExp;
                pt->refPhBias = &phBias;
                pt->imgGradient = &imgGradient;
                pt->photoStd = cam->get_imgPhotoStd();
                pt->setProjectionPointers();

                // Plane estimation
                dataType alpha{0.0},beta{0.0};
                if(planeEstimation(alpha,beta, pt,
                                   solveOptionsPlaneEstimate_,summaryPlaneEstimate_,
                                   fx,fy,cx,cy,w,h)) ++numPlaneEstimationAchieved;

                // Undistort point
                cam->undistortPt( pt->uRef, pt->vRef,  pt->uRef, pt->vRef);
                cam->uv_2_xyn(pt);

                // Set plane estimation
                pt->setPlaneEstimation(alpha,beta,fx,fy);
            }

            // Extracts the reference intensities for all HGP related to the frame, for the whole mask and, at all pyramid levels.
            extract_I_reference();

            //  Save all available points
            hgp0 = hgp;

            // Warning in the case the plane estimation is failing
            double planeEstimationRate = numPlaneEstimationAchieved/hgp.size();
            if(planeEstimationRate < 0.6) cout << GREEN_COUT << "    |   |   [PointSelector]: "<< LIGTH_RED_COUT << "Hgp Plane Estimation Success rate = " <<
                                                numPlaneEstimationAchieved << "/"<< hgp.size()<< " = "<<planeEstimationRate << RESET_COUT << endl;
#ifdef COUT_COMPLETE_PIPELINE
            else cout << GREEN_COUT <<"                                "<< LIGTH_GREEN_COUT << "Hgp Plane Estimation Success rate = " <<
                      numPlaneEstimationAchieved << "/"<< hgp.size()<< " = "<<planeEstimationRate << RESET_COUT << endl;
#endif
        }

        void associate_featuresToFrame(vecFt& features_, vecFt& allFeatures_ , ceres::Solver::Options& options, ceres::Solver::Summary& summary){
            const dataType fx{cam->getFocalLengthX()};
            const dataType fy{cam->getFocalLengthY()};
            const dataType cx{cam->getCentrePointX()};
            const dataType cy{cam->getCentrePointY()};
            const dataType w = (dataType)cam->get_w(0);
            const dataType h = (dataType)cam->get_h(0);

            //features.clear();
            featuresForLC.clear();
#ifdef ACTIVE_FEATURES_TRACKING
            for(typeFt& ft:features_){
                features.push_back(ft);
            }
#endif
            for(typeFt& ft:allFeatures_){
                featuresForLC.push_back(ft);
            }

            double numPlaneEstimationAchieved{0};
            for(typeFt& ft: featuresForLC){
                ft->refKeyframe = this;
                ft->idKey = keyId;
                ft->setProjectionPointers();
            }
#ifdef ACTIVE_FEATURES_TRACKING
            for(typeFt& ft: features){
                ft->refKeyframe = this;
                ft->idKey = keyId;
                ft->setProjectionPointers();
                dataType alpha{0.0},beta{0.0};
                if(planeEstimation(alpha,beta,ft,options,summary,fx,fy,cx,cy,w,h)) ++numPlaneEstimationAchieved;
                cam->uv_2_xyn(ft);

                // Set plane estimation
                ft->setPlaneEstimation(alpha,beta,fx,fy);
            }
            double planeEstimationRate = numPlaneEstimationAchieved/features.size();
            if(planeEstimationRate < 0.6) cout << GREEN_COUT << "    |   |   [PointSelector]: "<< LIGTH_RED_COUT << "Feature Plane Estimation Success rate = " <<
                                        numPlaneEstimationAchieved << "/"<< features.size()<< " = "<<planeEstimationRate << RESET_COUT << endl;
#ifdef COUT_COMPLETE_PIPELINE
            else cout << GREEN_COUT << "                                "<< LIGTH_GREEN_COUT << "Feature Plane Estimation Success rate = " <<
                      numPlaneEstimationAchieved << "/"<< features.size()<< " = "<<planeEstimationRate << RESET_COUT << endl;
#endif
#endif
        }
        
        void set_photo_parameters(const dataType& phScalar_, const dataType& phBias_){
            phScalar = phScalar_;
            phScalarExp = exposureFunction(phScalar);//exp(-phScalar);
            phBias = phBias_;
        }

        //Checks the point satisfied geometric visual restrictions.
        bool isGeoVisible(typePt& pt, dataType& maxDepth_, dataType& minDepth_, dataType& minCos_){
            if(not cam->isPointIn(pt))
                return false;

            dataType depthRef = 1.0/pt->lambdaRef;
            dataType depth = 1.0/pt->lambda[0];

            if(depth > maxDepth_*depthRef)
                return false;
            if(depth < minDepth_*depthRef)
                return false;

            if(viewCos(pt) < minCos_)
                return false;

            return true;

        };

        bool isGeoVisible(typeFt& ft, dataType& maxDepth_ , dataType& minDepth_, dataType minCos_){

            if(not cam->isPointIn(ft))
                return false;

            dataType depthRef = 1.0/ft->lambdaRef;
            dataType depth = 1.0/ft->lambda[0];

            if(depth > maxDepth_*depthRef)
                return false;
            if(depth < minDepth_*depthRef)
                return false;

            if(viewCos(ft) < minCos_)
                return false;

            return true;

        };

        dataType distanceToFrame(shared_ptr<Frame> frame){
            return euclideanDistance(pose.twc, frame->pose.twc);
        }

        dataType viewCos(typePt& pt){
            vec3 Po,Pn;
            auto refKeyframe = static_cast< IDNav::Frame*>(pt->refKeyframe);

            Pn(0) = pt->X[0] - refKeyframe->pose.twc[0];
            Pn(1) = pt->Y[0] - refKeyframe->pose.twc[1];
            Pn(2) = pt->Z[0] - refKeyframe->pose.twc[2];
            Pn /= Pn.norm();

            Po(0) = pt->X[0] - pose.twc[0];
            Po(1) = pt->Y[0] - pose.twc[1];
            Po(2) = pt->Z[0] - pose.twc[2];
            Po /= Po.norm();
            return Po.dot(Pn);
        }

        dataType viewCos(typeFt& pt){
            vec3 Po,Pn;
            auto refKeyframe = static_cast< IDNav::Frame*>(pt->refKeyframe);

            Pn(0) = pt->X[0] - refKeyframe->pose.twc[0];
            Pn(1) = pt->Y[0] - refKeyframe->pose.twc[1];
            Pn(2) = pt->Z[0] - refKeyframe->pose.twc[2];
            Pn /= Pn.norm();

            Po(0) = pt->X[0] - pose.twc[0];
            Po(1) = pt->Y[0] - pose.twc[1];
            Po(2) = pt->Z[0] - pose.twc[2];
            Po /= Po.norm();
            return Po.dot(Pn);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        bool isOptimalForKeyframeCreation(const dataType& association_th_){
            return validDepth < association_th_;
        }

        // PLANE ESTIMATION
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        class DepthResidual : public ceres::SizedCostFunction<1,1,1> {
        public:

            DepthResidual(dataType& z0, dataType& xn0, dataType& yn0, dataType& z, dataType& xn, dataType& yn):
            z0(z0),xn0(xn0),yn0(yn0),z(z),xn(xn),yn(yn){}

            virtual ~DepthResidual(){}

            virtual bool Evaluate(dataType const *const *param,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                dataType ct0 = 1.0-param[0][0]*xn0-param[1][0]*yn0;
                dataType ct = 1.0-param[0][0]*xn-param[1][0]*yn;
                dataType zEst = z0*ct0/ct;
                residuals[0] = zEst - z;

                if(jacobians){
                    jacobians[0][0] = z0*(-xn0*ct + xn*ct0)/(ct*ct);
                    jacobians[1][0] = z0*(-yn0*ct + yn*ct0)/(ct*ct);
                }
                return true;
            }

            dataType z0{},xn0{},yn0{},z{},xn{},yn{};
        };


        // VISUALIZATION FUNCTIONS
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        void drawPointRef(std::shared_ptr<IDNav::Pt>& pt, cv::Scalar color = cv::Scalar(0, 255, 0),size_t size_draw_mask = 1){
            circle(Mat_gray_clone, cv::Point(int(pt->uRef), int(pt->vRef)), 2, color, -1);
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void show_image(const std::string& windowName = "Display Frame") {

            std::cout << "\nShowing frame...  ,id = " << frameId << " ,timestamp = " << grayImgTs << std::endl;
            namedWindow(windowName, cv::WINDOW_NORMAL);
            cv::resizeWindow(windowName, 1.5*640,1.5*480);
            imshow(windowName, Mat_gray_clone);    // Show our image inside it.
            cv::waitKey(0);
        };

        void convertImageToColor() {
            Mat_gray_clone = grayImgG[0].clone();
            cvtColor(Mat_gray_clone,Mat_gray_clone, cv::COLOR_GRAY2RGB);
        };

        void drawPoints(shared_ptr<Frame>& frame_){
            cout << "cocoguagua 1 "<< endl;
            Mat imgRef = Mat_gray_clone.clone();
            cout << "cocoguagua 2 "<< endl;
            cvtColor(imgRef,imgRef, cv::COLOR_GRAY2RGB);
            cout << "cocoguagua 3 "<< endl;
            Mat imgProj = frame_->Mat_gray_clone.clone();
            cout << "cocoguagua 4 "<< endl;
            cvtColor(imgProj,imgProj, cv::COLOR_GRAY2RGB);
            cout << "cocoguagua 5 "<< endl;
            vecPt hgp_aux;
            vecFt features_aux;
            getVisiblePointsIn(frame_,hgp_aux,features_aux);
            int index{1};
            for(typePt& pt: hgp_aux){
                cout << index << endl;
                circle(imgProj, cv::Point(int(pt->u[0]), int(pt->v[0])), 5, cv::Scalar(1,100,100), 2);
                circle(imgRef, cv::Point(int(pt->uRef), int(pt->vRef)), 5, cv::Scalar(100,100,1), 2);
                ++index;
            }

            int sizeRect = 10;
            for(typeFt& ft: features_aux){
                Point pt1 = Point(ft->u[0] - sizeRect, ft->v[0] - sizeRect);
                Point pt2 = Point(ft->u[0] + sizeRect, ft->v[0] + sizeRect);
                rectangle(imgProj, pt1, pt2, cv::Scalar(1,150,150), 2);

                pt1 = Point(ft->uRef - sizeRect, ft->vRef - sizeRect);
                pt2 = Point(ft->uRef + sizeRect, ft->vRef + sizeRect);
                rectangle(imgRef, pt1, pt2, cv::Scalar(150,150,1), 2);
            }

            imshow("ref Img", imgRef );
            imshow("proj Img", imgProj );

            waitKey(0);
        }

        void drawPoints(shared_ptr<Frame>& frame_, unordered_map<typePt,bool> visibleHgp_){
            cout << "cocoguagua 1 "<< endl;
            Mat imgRef = Mat_gray_clone.clone();
            cout << "cocoguagua 2 "<< endl;
            cvtColor(imgRef,imgRef, cv::COLOR_GRAY2RGB);
            cout << "cocoguagua 3 "<< endl;
            Mat imgProj = frame_->Mat_gray_clone.clone();
            cout << "cocoguagua 4 "<< endl;
            cvtColor(imgProj,imgProj, cv::COLOR_GRAY2RGB);
            cout << "cocoguagua 5 "<< endl;
            vecPt hgp_aux;
            vecFt features_aux;
            getVisiblePointsIn(frame_,hgp_aux,features_aux);
            int index{1};
            for(typePt& pt: hgp_aux){
                //cout << index << endl;
                circle(imgProj, cv::Point(int(pt->u[0]), int(pt->v[0])), 5, cv::Scalar(1,100,100), 2);
                circle(imgRef, cv::Point(int(pt->uRef), int(pt->vRef)), 5, cv::Scalar(100,100,1), 2);
                ++index;
            }

            index = 0;
            for(typePt& pt: hgp_aux){
                if(visibleHgp_.find(pt) == visibleHgp_.end()){
                    //cout << index << endl;
                    circle(imgProj, cv::Point(int(pt->u[0]), int(pt->v[0])), 5, cv::Scalar(1,200,200), 2);
                    circle(imgRef, cv::Point(int(pt->uRef), int(pt->vRef)), 5, cv::Scalar(200,200,1), 2);
                    ++index;
                    //for(int iPatch{}; iPatch < pt->ptSize; ++iPatch) {

                    cout <<"pt index = " << index << endl;
                    cout << "u = << " << pt->u[0] << " v = " << pt->v[0] << endl;
                    cout << "pt->photoStd0 = << " << pt->photoStd0 << endl;
                    cout << "pt->photoStd = << " << pt->photoStd << endl;
                    cout << "I  = ";
                    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch) {
                        cout  << pt->I_ref[iPatch][0] << " , " ;
                    }
                    cout << " " << endl;
                    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch) {
                        cout  << pt->I[iPatch] << " , " ;
                    }
                    cout << " " << endl;
                    //}

                    imshow("ref Img", imgRef );
                    imshow("proj Img", imgProj );

                    waitKey(0);

                }
                ++index;
            }

            int sizeRect = 10;
            for(typeFt& ft: features_aux){
                Point pt1 = Point(ft->u[0] - sizeRect, ft->v[0] - sizeRect);
                Point pt2 = Point(ft->u[0] + sizeRect, ft->v[0] + sizeRect);
                rectangle(imgProj, pt1, pt2, cv::Scalar(1,150,150), 2);

                pt1 = Point(ft->uRef - sizeRect, ft->vRef - sizeRect);
                pt2 = Point(ft->uRef + sizeRect, ft->vRef + sizeRect);
                rectangle(imgRef, pt1, pt2, cv::Scalar(150,150,1), 2);
            }

            imshow("ref Img", imgRef );
            imshow("proj Img", imgProj );

            waitKey(0);
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Mat drawFeatures(vector<cv::KeyPoint>& keypoints){
            Mat img_keypoints = grayImgG[0].clone();
            drawKeypoints(img_keypoints, keypoints, img_keypoints );
            return img_keypoints;
        }
        void showFeatures(vector<cv::KeyPoint>& keypoints){
            Mat img_keypoints = drawFeatures(keypoints);
            imshow("Features", img_keypoints );
            waitKey(0);
        }

    };
    typedef shared_ptr<Frame> typeFrame;
    typedef vector<shared_ptr<Frame>> vecFrame;

    inline typeFrame bufferFrame(typeFrame& frame_) {
        static int numKeyframesCreated{0};

        typeFrame keyframe{new Frame{}};
        keyframe->keyId = 0;//numKeyframesCreated ;

        keyframe->frameId = frame_->frameId;
        keyframe->grayImgTs = frame_->grayImgTs;
        keyframe->validDepth = frame_->validDepth;

        keyframe->pose = frame_->pose;
        keyframe->cam = frame_->cam;

        keyframe->phScalar = frame_->phScalar;
        keyframe->phScalarExp = frame_->phScalarExp;
        keyframe->phBias = frame_->phBias;
        keyframe->imgGradient = frame_->imgGradient;

        keyframe->trackingInformationBitsMax = 0.0;//frame_->trackingInformationBitsMax; ????????????????????????
        keyframe->trackingInformationBitsThreshold = 0.0;//frame_->trackingInformationBitsThreshold; ????????????????????????

        for (int i{0}; i < frame_->grayImgG.size(); ++i){
            keyframe->grayImgG.push_back(frame_->grayImgG[i].clone());
        }

        keyframe->grayImgDist = frame_->grayImgDist.clone(); // ????????????????????????
        keyframe->depthImg = frame_->depthImg.clone(); // ????????????????????????
        keyframe->maskImg = frame_->maskImg.clone();
        keyframe->Mat_gray_clone = frame_->Mat_gray_clone.clone();  // ????????????????????????

        // Points
        //keyframe->hgp = frame_->hgp;
        //keyframe->hgp0 = frame_->hgp0;
        //keyframe->hgpInf = frame_->hgpInf;


        //keyframe->features = frame_->features;
        keyframe->keypoints = frame_->keypoints;
        keyframe->observations = frame_->observations;
        keyframe->descriptorsObject = frame_->descriptorsObject.clone();
        keyframe->isDescriptorComputed = frame_->isDescriptorComputed;

        return keyframe;
    }

    inline typeFrame createKeyframeFromFrame(typeFrame& frame_, int& numKeyframesCreated_) {
        typeFrame keyframe{new Frame{}};
        keyframe->keyId = numKeyframesCreated_ ;
        ++numKeyframesCreated_;

        keyframe->frameId = frame_->frameId;
        keyframe->grayImgTs = frame_->grayImgTs;
        keyframe->validDepth = frame_->validDepth;

        keyframe->pose = frame_->pose;
        keyframe->cam = frame_->cam;

        keyframe->phScalar = frame_->phScalar;
        keyframe->phScalarExp = frame_->phScalarExp;
        keyframe->phBias = frame_->phBias;
        keyframe->imgGradient = frame_->imgGradient;

        keyframe->trackingInformationBitsMax = 0.0;//frame_->trackingInformationBitsMax; ????????????????????????
        keyframe->trackingInformationBitsThreshold = 0.0;//frame_->trackingInformationBitsThreshold; ????????????????????????

        for (int i{0}; i < frame_->grayImgG.size(); ++i){
            keyframe->grayImgG.push_back(frame_->grayImgG[i].clone());
        }
        frame_->grayImgG.clear();

        keyframe->grayImgDist = frame_->grayImgDist.clone(); // ????????????????????????
        frame_->grayImgDist.release(); // ????????????????????????

        keyframe->depthImg = frame_->depthImg.clone(); // ????????????????????????
        frame_->depthImg.release();

        keyframe->maskImg = frame_->maskImg.clone();
        frame_->maskImg.release();

        keyframe->Mat_gray_clone = frame_->Mat_gray_clone.clone();  // ????????????????????????
        //frame_->Mat_gray_clone.release();  // ????????????????????????

        keyframe->keyframes.clear();

        // Points
        keyframe->hgp = frame_->hgp;
        keyframe->hgp0 = frame_->hgp0;
        keyframe->hgpInf = frame_->hgpInf;
        frame_->hgp.clear();
        frame_->hgp0.clear();
        frame_->hgpInf.clear();

        keyframe->features = frame_->features;
        frame_->features.clear();

        keyframe->keypoints = frame_->keypoints;
        frame_->keypoints.clear();

        keyframe->observations = frame_->observations;
        frame_->observations.clear();

        keyframe->descriptorsObject = frame_->descriptorsObject.clone();
        frame_->descriptorsObject.release();

        keyframe->isDescriptorComputed = frame_->isDescriptorComputed;
        frame_->isDescriptorComputed.clear();

        frame_->featuresForLC.clear();

        return keyframe;
    }


    inline typeFrame hardCopyFrame(typeFrame& frame_) {
        std::mutex mMutex;
        unique_lock<mutex> lock(mMutex);

        typeFrame newFrame{new Frame{}};

        newFrame->keyId = frame_->keyId;
        newFrame->frameId = frame_->frameId;
        newFrame->pose = frame_->pose;
        newFrame->cam = frame_->cam;
        newFrame->trackingInformationBitsMax = frame_->trackingInformationBitsMax;
        newFrame->trackingInformationBitsThreshold = frame_->trackingInformationBitsThreshold;
        newFrame->keypoints = frame_->keypoints;
        newFrame->keyframes = frame_->keyframes;
        newFrame->grayImgTs = frame_->grayImgTs;
        newFrame->phScalar = frame_->phScalar;
        newFrame->phScalarExp = frame_->phScalarExp;
        newFrame->phBias = frame_->phBias;
        newFrame->imgGradient = frame_->imgGradient;

        newFrame->Mat_gray_clone = frame_->Mat_gray_clone.clone();


        unordered_map<typePt,bool> pointsTmp{};
        for(typePt& pt_: frame_->hgpInf){
            pointsTmp[pt_] = true;
        }
        newFrame->hgpInf.clear();
        for(typePt& pt_: frame_->hgp0){
            typePt pt = hardCopyPt(pt_);
            newFrame->cam->uv_2_xyn(pt);
            newFrame->pose.xynlambda_2_XYZ(pt);
            newFrame->hgp0.push_back(pt);
            if(pointsTmp.find(pt_) != pointsTmp.end()){
                newFrame->hgpInf.push_back(pt);
            }
        }

        newFrame->hgp = newFrame->hgpInf;
        newFrame->observations.clear();
        for(typeFt& ft_: frame_->features){
            typeFt ft = hardCopyFt(ft_);
            newFrame->cam->uv_2_xyn(ft);
            newFrame->pose.xynlambda_2_XYZ(ft);
            newFrame->features.push_back(ft);
        }

        return newFrame;

    }
}
#endif //FRAME_H
