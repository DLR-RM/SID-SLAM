//
// Created by font_al on 10/26/18.
//

#ifndef IDNAV_CAMERA_H
#define IDNAV_CAMERA_H

#include "Pose.h"

namespace IDNav {

    //
    struct GridElement {
        int   ui{}, vi{}, uf{}, vf{};
    };

    class ImageGrid {
    public:
        int w{},h{};
        int imageMargin{4};
        int u_min{imageMargin}, u_max{}, v_min{imageMargin}, v_max{};

        dataType numCells_u{20}, numCells_v{15}, numCells{numCells_u*numCells_v};
        float  sizeCellPixels_u{}, sizeCellPixels_v{};
        vector<shared_ptr<GridElement>> grid{};

        ImageGrid() = default;
        void build(const int& w, const int& h);
        void get_cell(int& uCell, int& vCell, const int& u, const int& v);
    };

    //
    class Camera {
    protected:

        // Pyramid Levels
        int numPyrLevelsMax{PYR_LEVELS_MAX};
        int numPyrLevelsUsed{PYR_LEVELS_MAX};
        int iPyr{};
        dataType pyrLevelScaleSlope{sqrt(2.0)};
        dataType scaleFactorG[PYR_LEVELS_MAX]{};
        dataType scaleFactor{1.0};

        // Image dimensions
        size_t w{}, h{}, imageMargin{4};
        size_t wMax{}, hMax{};
        size_t wG[PYR_LEVELS_MAX]{}, hG[PYR_LEVELS_MAX]{};
        dataType wGmax[PYR_LEVELS_MAX]{}, hGmax[PYR_LEVELS_MAX]{};
        size_t numPixelsG[PYR_LEVELS_MAX]{};

    public:
        bool finestPyrLevel{false};

        // Patch parameters
        dataType patch_u[PATCH_SIZE]{}, patch_v[PATCH_SIZE]{};
        int patchSurface{};
        size_t patchSize{PATCH_SIZE};
        dataType patchWeight[PATCH_SIZE]{};

        // Grid over the image
        ImageGrid imageGrid{};

        // Covariance model
    protected:
        CovCalibration photoCal{};
        CovCalibration geoCal{};
    public:

        Mat cameraMatrix;
        Mat cameraMatrixUndist;
        Mat distCoeffs;

    public:

        Camera() = default;
        virtual void show_calibration(){};
        virtual void setResolution(const int& pyrLevel_){};
        void selectPatch(size_t patchIdentifier);

        virtual void xyn_2_uv(vecPt& hgp){};
        virtual void xyn_2_uv(vecFt& features){};
        virtual void xyn_2_uv(typeIdNavPoint pt){};
        virtual void xyn_2_uv_iPyr(typeIdNavPoint pt, const int& iPyr_){};
        virtual void uv_2_xyn(vecPt& hgp){};
        virtual void uv_2_xyn(vecFt& features){};
        virtual void uv_2_xyn(typeIdNavPoint pt){};

        virtual void distortPt(dataType& uDist, dataType& vDist, const dataType uUndist, const  dataType vUndist){};
        virtual void undistortPt(dataType& uUndist, dataType& vUndist, const dataType uDist, const  dataType vDist){};

        virtual void dI_dxyzc(vec3& dI_dxyzc_, const typePt& pt, const size_t& iPatch){};
        virtual void duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typePt& pt, const int& iPatch){};
        virtual void duv_dxyzc(mat23& duv_dxyzc_, typeFt& ft){};
        virtual void duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typeFt& ft){};

        virtual void computeJPhotometric(dataType &J1, dataType &J2, dataType &J3, shared_ptr<Pt> pt, const size_t& iMask){};
        virtual void computeJPhotometricRef(vec6& Jphoto, typePt& pt){};

        virtual void computeJGeometricRef(vec6& Ju,vec6& Jv, typeFt& keypt){};

        virtual void computeJCam(vec3& du_dxyzc, vec3& dv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_){};
        virtual void computeJCam(mat23& duv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_){};

        virtual bool isPixelInside(const int& up, const int& vp){};

        virtual int getNumPyrLevelsUsed(){};
        virtual int getPyrLevel(){};
        virtual dataType getScaleFactor(){};
        virtual dataType getScaleFactor(const int& pyrLevel_){};
        virtual dataType getPyrLevelScaleSlope(){};
        virtual dataType getFocalLengthX(){};
        virtual dataType getFocalLengthY(){};
        virtual dataType getCentrePointX(){};
        virtual dataType getCentrePointY(){};
        virtual size_t get_w(){};
        virtual size_t get_h(){};
        virtual size_t get_w(const int& pyrLevel_){};
        virtual size_t get_h(const int& pyrLevel_){};
        virtual size_t get_wMax(){};
        virtual size_t get_hMax(){};
        virtual dataType getStereoBaseline(){};
        virtual dataType getAssociationThreshold(){};
        virtual dataType getInvDepthCov(){};

        virtual const CovCalibration* get_photoDef2_param(){};
        virtual void set_photoDef2_param(dataType maxPhotoDef2_ , dataType minPhotoDef2_ ,  dataType ctPhotoDef2_, dataType ccPhotoDef2_, dataType imgPhotoCov_){};
        virtual void changePhotoDeformationConstants(const double& ctPhotoDef2_){};
        virtual dataType get_imgPhotoCov(){};
        virtual dataType get_imgPhotoStd(){};

        virtual const CovCalibration* get_geoDef2_param(){};
        virtual void set_geoDef2_param(dataType  maxGeoDef2_ , dataType  minGeoDef2_ , dataType  ctGeoDef2_, dataType  ccGeoDef2_, dataType  imgGeoCov_){};
        virtual dataType get_imgGeoCov(){};
        virtual dataType get_imgGeoStd(){};

        virtual dataType get_RGB_frequence_secs(){};
        virtual dataType get_RGB_frequence_msecs(){};
        virtual dataType get_RGB_frequence_hz(){};
        virtual dataType get_depth_frequence_hz(){};

        virtual void setCameraFrequence(const dataType& RGB_hz_, const dataType& depth_hz_){};

        virtual void readAndUndistortImage(cv::Mat& image){};

        #ifdef SEMANTIC_SEGMENTATION
        virtual bool isPointIn (std::shared_ptr<IDNav::Pt>& pt,Mat& mask){};
        virtual bool xyn_2_uv_isPointIn(std::shared_ptr<IDNav::Pt>& pt,Mat& mask){};
        #else
        virtual bool isPointIn (typePt& pt){};
        virtual bool isPointIn (typeFt& pt){};
        virtual bool xyn_2_uv_isPointIn(typePt& pt){};
        virtual bool xyn_2_uv_isPointIn(typeFt& ft){};
#endif
    };

    class CamPinhole: public Camera{

    protected:

        // Pinhole parameters
        dataType fx{0.0},  fy{0.0},  cx{0.0},  cy{0.0};
        dataType fxi{0.0}, fyi{0.0}, cxi{0.0}, cyi{0.0};
        dataType fxG[PYR_LEVELS_MAX]{},  fyG[PYR_LEVELS_MAX]{},  cxG[PYR_LEVELS_MAX]{},  cyG[PYR_LEVELS_MAX]{};
        dataType fxiG[PYR_LEVELS_MAX]{}, fyiG[PYR_LEVELS_MAX]{}, cxiG[PYR_LEVELS_MAX]{}, cyiG[PYR_LEVELS_MAX]{};

        dataType stereoBaseline{};
        dataType association_th{0.51};
        dataType RGB_hz{30.0} , RGB_seconds{0.033}, depth_hz{30.0};

        dataType pixelCov = 0.5* 0.5;
        dataType invDepthCov{};

        void computePyrLevelSlope(size_t wValue, size_t hValue);
        void initImageDimensions(const size_t& wValue, const size_t& hValue);
        void initGlobalCalibration(const dataType& fxValue,const dataType& fyValue,const dataType& cxValue,const dataType& cyValue);

    public:

        CamPinhole(const dataType& fxValue, const dataType& fyValue, const dataType& cxValue, const dataType& cyValue, const size_t& wValue, const size_t& hValue,
                   const dataType& stereoBaseline_, const dataType& association_th_);
        CamPinhole() = default;
        void show_calibration() override;
        void setResolution(const int& pyrLevel_) override;

        void xyn_2_uv(vecPt& hgp) override;
        void xyn_2_uv(vecFt& features) override;
        void xyn_2_uv(typeIdNavPoint pt) override;
        void xyn_2_uv_iPyr(typeIdNavPoint pt, const int& iPyr_) override;
        void uv_2_xyn(vecPt& hgp) override;
        void uv_2_xyn(vecFt& features) override;
        void uv_2_xyn(typeIdNavPoint pt) override;

        void distortPt(dataType& uDist, dataType& vDist, const dataType uUndist, const  dataType vUndist) override;
        void undistortPt(dataType& uUndist, dataType& vUndist, const dataType uDist, const  dataType vDist)override;

        void dI_dxyzc(vec3& dI_dxyzc_, const typePt& pt, const size_t& iPatch) override;
        void duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typePt& pt, const int& iPatch) override;
        void duv_dxyzc(mat23& duv_dxyzc_, typeFt& ft) override;
        void duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typeFt& ft) override;

        void computeJPhotometric(dataType &J1, dataType &J2, dataType &J3, shared_ptr<Pt> pt, const size_t& iMask) override;
        void computeJPhotometricRef(vec6& Jphoto, typePt& pt) override;
        void computeJGeometricRef(vec6& Ju,vec6& Jv, typeFt& keypt) override;

        //void computeJCam(dataType Ju[3], dataType Jv[3], typeIdNavPoint pt, const size_t& iMask) override;
        void computeJCam(vec3& du_dxyzc, vec3& dv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_) override;
        void computeJCam(mat23& duv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_) override;
        //void computeJCam(vec3& Ju, vec3& Jv,typeFt& pt) override;

        bool isPixelInside(const int& up, const int& vp);

        void readAndUndistortImage(Mat& image) override;

        int getNumPyrLevelsUsed() override {return numPyrLevelsUsed;};
        int getPyrLevel() override {return iPyr;};
        dataType getScaleFactor() override {return scaleFactor;};
        dataType getScaleFactor(const int& pyrLevel_) override {return scaleFactorG[pyrLevel_];};
        dataType getPyrLevelScaleSlope()override {return pyrLevelScaleSlope;};
        dataType getFocalLengthX() override {return fx;};
        dataType getFocalLengthY() override {return fy;};
        dataType getCentrePointX() override {return cx;};
        dataType getCentrePointY() override {return cy;};
        size_t get_w() override {return w;};
        size_t get_h() override {return h;};
        size_t get_w(const int& pyrLevel_){return wG[pyrLevel_];};
        size_t get_h(const int& pyrLevel_){return hG[pyrLevel_];};
        size_t get_wMax() override {return wMax;};
        size_t get_hMax() override {return hMax;};
        dataType getStereoBaseline() override {return stereoBaseline;};
        dataType getAssociationThreshold() override {return association_th;};
        dataType getInvDepthCov() override;

        // Covariance Calibration
        const CovCalibration* get_photoDef2_param() override {return &photoCal;};
        const CovCalibration* get_geoDef2_param() override {return &geoCal;};
        void set_photoDef2_param(dataType maxPhotoDef2_ , dataType minPhotoDef2_ , dataType ctPhotoDef2_, dataType ccPhotoDef2_,dataType imgPhotoCov_)override{
            photoCal.setPhotoCalibration(maxPhotoDef2_,minPhotoDef2_,
                                            ctPhotoDef2_ ,ccPhotoDef2_,
                                            imgPhotoCov_);
        };
        void set_geoDef2_param(dataType  maxGeoDef2_ , dataType  minGeoDef2_ , dataType  ctGeoDef2_, dataType  ccGeoDef2_, dataType  imgGeoCov_)override{
            geoCal.setReprojCalibration(maxGeoDef2_,minGeoDef2_,
                                          ctGeoDef2_ ,ccGeoDef2_,
                                          imgGeoCov_);
        };
        void changePhotoDeformationConstants(const double& ctPhotoDef2_){
            photoCal.ctDef2 = ctPhotoDef2_;
            photoCal.ccDef2 = ctPhotoDef2_;
        }

        dataType get_imgPhotoCov()override{return photoCal.imgCov;};
        dataType get_imgGeoCov()override{return geoCal.imgCov;};
        dataType get_imgPhotoStd()override{return photoCal.imgStd;};
        dataType get_imgGeoStd()override{return geoCal.imgStd;};
        //

        dataType get_RGB_frequence_secs()override{return RGB_seconds;};
        dataType get_RGB_frequence_msecs()override{return RGB_seconds*1000.0;};
        dataType get_RGB_frequence_hz()override{return RGB_hz;};
        dataType get_depth_frequence_hz()override{return depth_hz;};

        void setCameraFrequence(const dataType& RGB_hz_, const dataType& depth_hz_) override {
            RGB_hz = RGB_hz_;
            RGB_seconds = 1.0/RGB_hz;
            depth_hz = depth_hz_;
        };

        #ifdef SEMANTIC_SEGMENTATION
        bool isPointIn(std::shared_ptr<IDNav::Pt>& pt, Mat& mask) override;
        bool xyn_2_uv_isPointIn(std::shared_ptr<IDNav::Pt>& pt, Mat& mask) override;
        #else
        bool isPointIn(typePt& pt) override;
        bool isPointIn(typeFt& ft) override;
        bool xyn_2_uv_isPointIn(typePt& pt) override;
        bool xyn_2_uv_isPointIn(typeFt& ft) override;

        #endif
    };

    class CamPinholeDist: public CamPinhole{
    public:
        Mat map1GrayImage{},map2GrayImage{};
        Mat map1DepthImage{},map2DepthImage{};

    public:

        CamPinholeDist(const dataType& fxValue, const dataType& fyValue, const dataType& cxValue, const dataType& cyValue,
                const Mat& distCoeffsValue, const size_t& wValue, const size_t& hValue,
                const dataType& stereoBaseline, const dataType& association_th_);
        CamPinholeDist() = default;

        void initGlobalCalibrationDist(const dataType& fxValue,const dataType& fyValue,const dataType& cxValue,const dataType& cyValue,const Mat& distCoeffsValue);
        void readAndUndistortImage(Mat& image) override;

    };
}
#endif //IDNAV_CAMERA_H
