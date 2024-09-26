#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Frame.h"

// This header contains classes for visualization:
//  - class StatisticVisualizer
//  - class MapVisualizer
//  - class Visualizer

namespace IDNav{

    struct Point3D{
        int u{}, v{};
        float X{},Y{}, Z{};
        float cov{1.0};
        bool reUsedFeature{false};
    };

    struct GraphEdge{
        float X1{},X2{};
        float Y1{},Y2{};
        float Z1{},Z2{};
    };

    class StatisticVisualizer {

    public:
        size_t numPlots{};
        bool statisticVisualizerInitialized = false;
        std::vector< pangolin::DataLog*> logs{};
        std::vector< pangolin::Plotter*> plotters{};
        std::vector< bool> thereIsNewData{};
        std::vector< std::vector< float>> data{};
        SequenceSettings* seqSet{

        };

        explicit StatisticVisualizer(size_t numPlots , SequenceSettings* seqSet):numPlots(numPlots),seqSet(seqSet){
            std::vector< float>data_{0.0f};
            for(size_t iPlot{}; iPlot < numPlots; ++iPlot){
                thereIsNewData.push_back(false);
                data.push_back(data_);
            }
        };

        void addLog(std::vector<std::string>& labels, float maxY = 1000, float minY = -1000, float maxX = 1000, float minX = -1000 ){
            logs.push_back(new pangolin::DataLog{});
            logs.back()->SetLabels(labels);
            plotters.push_back(new pangolin::Plotter{logs.back(),minX,maxX,minY,maxY});
            plotters.back()->SetBounds(0.0, 1.0, 0.0, 1.0);
            plotters.back()->Track("$i");
            plotters.back()->SetAspect(6.0);
        }
    };

    struct CustomType
    {
        CustomType()
                : value(0.0), name("") {}

        CustomType(double value, std::string name)
                :value(value), name(name) {}

        double value;
        std::string name;
    };

    inline std::ostream& operator<< (std::ostream& os, const CustomType& o){
        os << o.name << " = " << o.value;
        return os;
    }

    inline std::istream& operator>> (std::istream& is, CustomType& o){
        is >> o.value;
        is >> o.name;
        return is;
    }

    class MapVisualizer{
    public:
        const string mapWindowName {"mapWindow"};
        bool saveSequence{false};
        bool newFrameTracked{false};

        // Pangolin variables
        pangolin::OpenGlRenderState* sCam{};

    private:


    public:

        map<size_t,Pose>* groundtruth{};

        SequenceSettings* seqSet{};
        cv::Scalar color_hgp = cv::Scalar( 227, 137, 0);
        cv::Scalar color_features = cv::Scalar( 30, 99, 255);


        const int mapWindow_w{1200};
        const int mapWindow_h{600};
        float projM_fu{840};
        float projM_IDNav{840};
        float projM_u0{476};
        float projM_v0{300};
        float projM_znear{0.1};
        float projM_zfar{100};
        int image_w{};
        int image_h{};

        bool screenshot{false};
        bool drawHgp{true};
        bool drawFeatures{true};
        bool resetSystem{false};

        const string trackFrameWindowName {"ID: Current Frame"};



        cv::Mat trackedFrameImage;
        cv::Mat trackedFrameMask;

        std::vector<Point3D> trackedPoints{};
        std::vector<Point3D> trackedFeatures{};
        std::vector<Point3D> lostFeatures{};
        std::vector<GraphEdge> graphEdges{};
        std::vector<Point3D> trajectory{};

        pangolin::Var<bool>*  menuFollowCamera{};
        pangolin::Var<bool>*  menuShowTrajectory{};
        pangolin::Var<bool>*  menuShowPoints{};
        pangolin::Var<bool>*  menuShowKeyframes{};
        pangolin::Var<bool>*  menuShowOptimizationWindow{};
        pangolin::Var<bool>*  menuShowLocalWindow{};
        pangolin::Var<bool>*  menuShowTrackingWindow{};
        pangolin::Var<bool>*  menuShowGraph{};
        pangolin::Var<bool>*  menuShowHgp{};
        pangolin::Var<bool>*  menuShowFeatures{};
        pangolin::Var<bool>*  menuScreenShot{};
        pangolin::Var<bool>*  menuReset{};
        pangolin::Var<bool>*  menuResetSystem{};

        pangolin::Var<int>* menuNum_hgp{};
        pangolin::Var<int>* menuNum_features{};
        pangolin::Var<int>* menuNum_keyframes{};
        int num_hgp{}, num_features{}, num_keyframes{};

        pangolin::Var<dataType>* menu_maxInformationLoss{};
        pangolin::Var<dataType>* menu_maxKeyframeDistance{};
        pangolin::Var<dataType>* menu_minVisiblePointsRatio{};

        pangolin::OpenGlMatrix Twc, Twc_ref;
        mat3 Rwc,Rwc_ref;
        vec3 twc,twc_ref;
        std::vector<pangolin::OpenGlMatrix> Twc_trackingKeyframes;
        std::vector<pangolin::OpenGlMatrix> Twc_localMapKeyframes;
        std::vector<pangolin::OpenGlMatrix> Twc_keyframesToOptimize;

        std::vector<pangolin::OpenGlMatrix> Twc_keyframes;
        GLfloat cameraColor[4]{0.8f,0.2f,0.8f,1.0f};
        GLfloat optWindowColor[4]{0.8f,0.2f,0.8f,1.0f};
        GLfloat localWindowColor[4]{0.4f,0.3f,0.4f,1.0f};
        GLfloat trackWindowColor[4]{0.2f,0.8f,0.8f,1.0f};

        GLfloat refKeyColor[4]{0.2f,0.8f,0.2f,1.0f};
        GLfloat keyframeColor[4]{0.7f,0.1f,0.7f,0.7f};
        GLfloat edgeColor[4]{0.7f,0.8f,0.7f,0.7f};

        const float w = 0.08f ;//mCameraSize;
        const float h = w*0.75f;
        const float z = w*0.6f;

        float maxCov_hgp{1.0};
        float minCov_hgp{1.0};

        // float inliersRatio{1.0};
        //float inliersRatio_hgp{1.0};
        //float inliersRatio_features{1.0};
        //float minInliersRatio{0.5};

        dataType maxInformationLoss{5.0};
        dataType maxKeyframeDistance{1.0};
        dataType minVisiblePointsRatio{0.2};

        MapVisualizer(SequenceSettings* seqSet_): seqSet(seqSet_){
            Twc.SetIdentity();

            // Create OpenGL window in single line

            pangolin::WindowInterface& mapWindow = pangolin::CreateWindowAndBind(mapWindowName,mapWindow_w, mapWindow_h);

            mapWindow.Move(1700,0);

            cv::namedWindow(trackFrameWindowName);

            cv::moveWindow(trackFrameWindowName,3500,0);

            // enable depth
            glEnable(GL_DEPTH_TEST);

            // Define Projection and initial ModelView matrix
            pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(mapWindow_w,mapWindow_h,projM_fu,projM_IDNav,projM_u0,projM_v0,projM_znear,projM_zfar);
            sCam = new pangolin::OpenGlRenderState(proj, pangolin::ModelViewLookAt(0,-0.7,-3.8, 0,0,0,0.0,-1.0, 0.0));

            // unset the current context from the main thread
            pangolin::GetBoundWindow()->RemoveCurrent();

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Menu Buttons
        void checkMenu(){

            if(*menuResetSystem){
                *menuResetSystem = false;
                resetSystem = true;
                return;
            }

            if(*menuReset){
                trackedPoints.clear();
                trackedFeatures.clear();
                lostFeatures.clear();
                Twc_localMapKeyframes.clear();
                Twc_keyframesToOptimize.clear();
                Twc_trackingKeyframes.clear();
                graphEdges.clear();

                *menuFollowCamera = true;
                *menuShowTrajectory = false;
                *menuShowPoints = true;
                *menuShowKeyframes = false;
                *menuShowOptimizationWindow = false;
                *menuShowTrackingWindow = true;
                *menuShowLocalWindow = false;
                *menuShowGraph = false;
                *menuShowHgp = true;
                *menuShowFeatures = true;
                *menuScreenShot = false;
                *menuReset = false;
                return;
            }

            drawCamera(Twc, cameraColor);

            if(*menuFollowCamera)
                sCam->Follow(Twc);
            if(*menuShowTrajectory)
                drawTrajectory();
            if(*menuShowPoints)
                drawMapPoints();
            if(*menuShowTrackingWindow)
                drawTrackingWindow();
            if(*menuShowOptimizationWindow)
                drawOptimizationWindow();
            if(*menuShowLocalWindow)
                drawLocalWindow();
            if(*menuShowKeyframes)
                drawKeyframeCameras();
            if(*menuShowGraph)
                drawGraph();

            drawHgp = *menuShowHgp;
            drawFeatures = *menuShowFeatures;
            screenshot = *menuScreenShot;
            *menuScreenShot = false;

            *menuNum_hgp = num_hgp;
            *menuNum_features = num_features;
            *menuNum_keyframes = num_keyframes;

            maxInformationLoss = *menu_maxInformationLoss;
            maxKeyframeDistance = *menu_maxKeyframeDistance;
            minVisiblePointsRatio = *menu_minVisiblePointsRatio;

        }

        void setMenuButtons(){
            menuFollowCamera           = new pangolin::Var<bool>{createMenuButton("menu.Follow Camera",true)};
            menuShowTrajectory         = new pangolin::Var<bool>{createMenuButton("menu.Trajectory",true)};
            menuShowPoints             = new pangolin::Var<bool>{createMenuButton("menu.Points",true)};
            menuShowKeyframes          = new pangolin::Var<bool>{createMenuButton("menu.Keyframes",true)};
            menuShowOptimizationWindow = new pangolin::Var<bool>{createMenuButton("menu.Opt Window",true)};
            menuShowTrackingWindow     = new pangolin::Var<bool>{createMenuButton("menu.Track Window",true)};
            menuShowLocalWindow        = new pangolin::Var<bool>{createMenuButton("menu.Local Window",false)};
            menuShowGraph              = new pangolin::Var<bool>{createMenuButton("menu.Graph",true)};
            menuShowHgp                = new pangolin::Var<bool>{createMenuButton("menu.Hgp",true)};
            menuShowFeatures           = new pangolin::Var<bool>{createMenuButton("menu.Features",true)};
            menuNum_hgp                = new pangolin::Var<int>{createMenuVar("menu.num hgp",0,0,4000)};
            menuNum_features           = new pangolin::Var<int>{createMenuVar("menu.num features",0,0,4000)};
            menuNum_keyframes          = new pangolin::Var<int>{createMenuVar("menu.num keyframes",0,0,1000)};
            menu_maxInformationLoss    = new pangolin::Var<dataType>{createMenuVar("menu.max inf loss",maxInformationLoss,1.0,10.0)};
            menu_maxKeyframeDistance   = new pangolin::Var<dataType>{createMenuVar("menu.key dist max",maxKeyframeDistance,0.1,2.0)};
            menu_minVisiblePointsRatio = new pangolin::Var<dataType>{createMenuVar("menu.key vis min",minVisiblePointsRatio,0.0,1.0)};
            menuScreenShot             = new  pangolin::Var<bool>{createMenuButton("menu.Screenshot",false,false)};
            menuReset                  = new  pangolin::Var<bool>{createMenuButton("menu.Reset View",false,false)};
            menuResetSystem            = new  pangolin::Var<bool>{createMenuButton("menu.Reset System",false,false)};
        }

        static pangolin::Var<bool> createMenuButton(const std::string& name ,bool initialValue = true, bool fixValue = true){
            pangolin::Var<bool> menuVar(name,initialValue,fixValue);
            return menuVar;
        }

        static pangolin::Var<IDNav::CustomType> createMenuText(const std::string& name , std::string text, double value ){
            IDNav::CustomType initialValue(value,text);
            pangolin::Var<IDNav::CustomType> menuVar(name,initialValue);
            return menuVar;
        }

        template <class T>
        static pangolin::Var<T> createMenuVar(const std::string& name ,T initialValue, T minValue, T maxValue){
            pangolin::Var<T> menuVar(name,initialValue,minValue,maxValue);
            return menuVar;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void drawEdge(GraphEdge& edge, GLfloat color[4], dataType width);
        void drawCamera(pangolin::OpenGlMatrix& Twc_, GLfloat color[4], dataType scale = 1.5, dataType width = 10);
        void drawKeyframeCameras();
        void drawOptimizationWindow();
        void drawTrackingWindow();
        void drawLocalWindow();
        void drawGraph();
        void drawTrajectory();
        void drawMapPoints();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void setCurrentCameraPose(const typeFrame& frame){
            static int gtId{5};
            Rwc =  frame->pose.Rwc;
            twc << frame->pose.twc[0],frame->pose.twc[1],frame->pose.twc[2];
            set_openGlMatrix4(Twc, Rwc , twc);

            Point3D currentPose;
            currentPose.X = (float)twc[0];
            currentPose.Y = (float)twc[1];
            currentPose.Z = (float)twc[2];
            trajectory.push_back(currentPose);
        };

        void setRefCameraPose(const typeFrame& frame){
            Rwc_ref =  frame->pose.Rwc;
            twc_ref  << frame->pose.twc[0],frame->pose.twc[1],frame->pose.twc[2];
            set_openGlMatrix4(Twc_ref, Rwc_ref , twc_ref);
        };

        void setKeyframesCameraPose(unordered_map<size_t,typeFrame> * keyframesLocalMap_,
                                    unordered_map<size_t,typeFrame> * trackingWindowKeyframes_,
                                    unordered_map<size_t,typeFrame> * keyframesToOptimize_,
                                    unordered_map<size_t,typeFrame> * keyframes){

            Twc_localMapKeyframes.clear();
            {
                std::mutex mMutex;
                unique_lock<mutex> lock(mMutex);
                for(auto& [idKey,keyframe]: *keyframesLocalMap_){
                    pangolin::OpenGlMatrix Twc_;
                    setTwc(Twc_,keyframe);
                    Twc_localMapKeyframes.push_back(Twc_);
                }
            }

            Twc_trackingKeyframes.clear();
            for(const auto& [indexKey, keyframe]: *trackingWindowKeyframes_){
                pangolin::OpenGlMatrix Twc_;
                setTwc(Twc_,keyframe);
                Twc_trackingKeyframes.push_back(Twc_);
            }

            Twc_keyframesToOptimize.clear();
            for(const auto& [indexKey, keyframe]: *keyframesToOptimize_){
                pangolin::OpenGlMatrix Twc_;
                setTwc(Twc_,keyframe);
                Twc_keyframesToOptimize.push_back(Twc_);
            }

            Twc_keyframes.clear();
            for(const auto& [indexKey, keyframe]: *keyframes){
                pangolin::OpenGlMatrix Twc_;
                setTwc(Twc_,keyframe);
                Twc_keyframes.push_back(Twc_);
            }

            graphEdges.clear();
            for(auto& [idKey1,keyframe1]: *keyframes){
                GraphEdge edge{};
                edge.X1 = keyframe1->pose.twc[0];
                edge.Y1 = keyframe1->pose.twc[1];
                edge.Z1 = keyframe1->pose.twc[2];
                for(auto& [idKey2,cov]: keyframe1->keyframes) {
                    edge.X2 =  (*keyframes)[idKey2]->pose.twc[0];
                    edge.Y2 = (*keyframes)[idKey2]->pose.twc[1];
                    edge.Z2 = (*keyframes)[idKey2]->pose.twc[2];
                    graphEdges.push_back(edge);
                }
            }
        };

        void setTwc(pangolin::OpenGlMatrix& Twc_ , const std::shared_ptr<IDNav::Frame>& frame);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void drawPoints(std::vector<IDNav::Point3D>& hgp,std::vector<IDNav::Point3D>& features,
                        cv::Mat& image);

        /*void drawPoints(std::vector<IDNav::Point3D>& hgp,std::vector<IDNav::Point3D>& features,
                        cv::Mat& image){

            int recThickness = 3*image_w/640;
            int sizeRect = 10*image_w/640;

            int circleThickness = 2*image_w/640;
            int circleRadius = 8*image_w/640;

            int centerThickness = 2*image_w/640;
            int centerRadius = (2*image_w/640);

            cv::Scalar color_aux = color_hgp;
            float factor;
            if(drawHgp){
                for(IDNav::Point3D& pt: hgp) {
                    Point pt0 = Point(pt.u, pt.v);
                    color_aux = color_hgp;
                    factor = (maxCov_hgp-pt.cov)/(maxCov_hgp - minCov_hgp);
                    if(factor < 0.0f) factor = 0.0f;
                    color_aux *= 1.0;//factor;

                    circle(image,pt0,centerRadius, color_aux, centerThickness,0);
                    circle(image,pt0,circleRadius, color_aux, circleThickness,0);
                }
            }
            if(drawFeatures){
                for(IDNav::Point3D& pt: features) {
                    Point pt0 = Point(pt.u, pt.v);
                    //if(pt.reUsedFeature){
                        //circle(image, pt0, centerRadius, 1.5*color_features, centerThickness);
                        //Point pt1 = Point(pt.u - sizeRect, pt.v - sizeRect);
                        //Point pt2 = Point(pt.u + sizeRect, pt.v + sizeRect);
                        //rectangle(image, pt1, pt2, 1.5*color_features, recThickness);
                    //}else{
                    circle(image, pt0, centerRadius, color_features, centerThickness);
                    Point pt1 = Point(pt.u - sizeRect, pt.v - sizeRect);
                    Point pt2 = Point(pt.u + sizeRect, pt.v + sizeRect);
                    rectangle(image, pt1, pt2, color_features, recThickness);
                    //}
                }
            }
        }*/
    };

    class Visualizer {

    public:

        std::shared_ptr<IDNav::StatisticVisualizer> statisticVisualizer{};
        std::shared_ptr<IDNav::MapVisualizer> mapVisualizer{};
        SequenceSettings* seqSet{};

        explicit Visualizer(SequenceSettings* seqSet_): seqSet(seqSet_){
            statisticVisualizer = std::make_shared<StatisticVisualizer>(1,seqSet);
            mapVisualizer = std::make_shared<MapVisualizer>(seqSet_);
        };

    };
}

#endif //VISUALIZER_H
