//
// Created by font_al on 10/26/18.
//

#include "../../include/Camera.h"

IDNav::CamPinholeDist::CamPinholeDist(const dataType& fxValue, const dataType& fyValue, const dataType& cxValue, const dataType& cyValue, const Mat& distCoeffsValue,
        const size_t& wValue, const size_t& hValue, const dataType& stereoBaseline_,const dataType& association_th_){

    IDNav::Camera::selectPatch(PATCH_SIZE);
    computePyrLevelSlope(wValue,hValue);
    initImageDimensions(wValue,hValue);
    initGlobalCalibrationDist(fxValue, fyValue, cxValue, cyValue,distCoeffsValue);
    IDNav::CamPinhole::setResolution(0);

    stereoBaseline = stereoBaseline_;
    association_th = association_th_;
    invDepthCov = pow(1/(stereoBaseline*fxValue),2)*pixelCov;
    imageGrid.build(wValue,hValue);

 }
// Function to compute the calibration for all pyramid levels
void IDNav::CamPinholeDist::initGlobalCalibrationDist(const dataType& fxValue,const dataType& fyValue,const dataType& cxValue,const dataType& cyValue, const Mat& distCoeffsValue){

    cameraMatrix = (Mat1d(3, 3) << fxValue, 0, cxValue, 0, fyValue, cyValue, 0, 0, 1);
    Size imageSize = Size(wG[0],hG[0]);;
    cameraMatrixUndist = getOptimalNewCameraMatrix(cameraMatrix,distCoeffsValue,imageSize,0);

    distCoeffs = distCoeffsValue;
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix,distCoeffsValue,imageSize, cv::noArray(),cameraMatrixDist);

    Mat R_image{},R_depth{};
    initUndistortRectifyMap(cameraMatrix,distCoeffsValue,R_image,cameraMatrixUndist, imageSize, CV_8U, map1GrayImage, map2GrayImage);
    initUndistortRectifyMap(cameraMatrix,distCoeffsValue,R_depth,cameraMatrixUndist, imageSize, CV_32F, map1DepthImage, map2DepthImage);
    //fisheye::initUndistortRectifyMap(cameraMatrix,distCoeffsValue,cv::noArray(),cameraMatrixDist,imageSize,CV_16SC2,map1GrayImage,map2GrayImage);

    dataType fxValue_, fyValue_, cxValue_,cyValue_;
    fxValue_ = dataType(cameraMatrixUndist.at<double>(0,0));
    fyValue_ = dataType(cameraMatrixUndist.at<double>(1,1));
    cxValue_ = dataType(cameraMatrixUndist.at<double>(0,2));
    cyValue_ = dataType(cameraMatrixUndist.at<double>(1,2));

    IDNav::CamPinhole::initGlobalCalibration(fxValue_, fyValue_, cxValue_, cyValue_);;

}

void IDNav::CamPinholeDist::readAndUndistortImage(cv::Mat& image){
    //cv::Mat imageTemporal = cv::imread(imagePath, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::remap(imageTemporal, image, map1GrayImage, map2GrayImage, cv::INTER_LINEAR);

    cv::Mat imageTemporal = image.clone();
    cv::remap(imageTemporal, image, map1GrayImage, map2GrayImage, cv::INTER_LINEAR);
    //cv::fisheye::undistortImage(imageTemporal, image, cameraMatrix, distCoeffs, cameraMatrixDist);
}

/*
void IDNav::CamPinholeDist::computeJCam(DATA_TYPE Ju[3],DATA_TYPE Jv[3], IDNav::Pt pt, size_t i_mask, int i_pyr){
    DATA_TYPE coef,r2;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    DATA_TYPE dxd_dxn, dxd_dyn, dyd_dxn, dyd_dyn;

    r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
    coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);

    dcoef_dxn = 2*pt.xn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
    dcoef_dyn = 2*pt.yn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

    ddx_dxn = 2*kc[2]*pt.yn[i_mask]+kc[3]*(6*pt.xn[i_mask]);
    ddx_dyn = 2*kc[2]*pt.xn[i_mask]+kc[3]*(2*pt.yn[i_mask]);
    ddy_dxn = 2*kc[3]*pt.yn[i_mask]+kc[3]*(2*pt.xn[i_mask]);
    ddy_dyn = 2*kc[3]*pt.xn[i_mask]+kc[3]*(6*pt.yn[i_mask]);

    dxd_dxn = (dcoef_dxn*pt.xn[i_mask]+coef+ddx_dxn);
    dxd_dyn = (dcoef_dyn*pt.xn[i_mask]+ddx_dyn);                cv::Mat Mat_gray_template(333,253,cv::DataType<bool>::type);

    dyd_dxn = (dcoef_dxn*pt.yn[i_mask]+ddy_dxn);
    dyd_dyn = (dcoef_dyn*pt.yn[i_mask]+coef+ddy_dyn);

    Ju[0] = (fxG[i_pyr]*dxd_dxn)/pt.zc[i_mask];
    Ju[1] = (fxG[i_pyr]*dxd_dyn)/pt.zc[i_mask];
    Ju[2] = -Ju[0]*pt.xn[i_mask]-Ju[1]*pt.yn[i_mask];

    Jv[0] = (fyG[i_pyr]*dyd_dxn)/pt.zc[i_mask];
    Jv[1] = (fyG[i_pyr]*dyd_dyn)/pt.zc[i_mask];
    Jv[2] = -Jv[0]*pt.xn[i_mask]-Jv[1]*pt.yn[i_mask];

}

void IDNav::CamPinholeDist::computeJPhotometric(DATA_TYPE& J1,DATA_TYPE&J2,DATA_TYPE&J3, IDNav::Pt pt, size_t i_mask){
    DATA_TYPE coef,r2;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    DATA_TYPE dxd_dxn, dxd_dyn, dyd_dxn, dyd_dyn;

    r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
    coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);

    dcoef_dxn = 2*pt.xn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
    dcoef_dyn = 2*pt.yn[i_mask]*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

    ddx_dxn = 2*kc[2]*pt.yn[i_mask]+kc[3]*(6*pt.xn[i_mask]);
    ddx_dyn = 2*kc[2]*pt.xn[i_mask]+kc[3]*(2*pt.yn[i_mask]);
    ddy_dxn = 2*kc[3]*pt.yn[i_mask]+kc[3]*(2*pt.xn[i_mask]);
    ddy_dyn = 2*kc[3]*pt.xn[i_mask]+kc[3]*(6*pt.yn[i_mask]);

    dxd_dxn = (dcoef_dxn*pt.xn[i_mask]+coef+ddx_dxn);
    dxd_dyn = (dcoef_dyn*pt.xn[i_mask]+ddx_dyn);
    dyd_dxn = (dcoef_dxn*pt.yn[i_mask]+ddy_dxn);
    dyd_dyn = (dcoef_dyn*pt.yn[i_mask]+coef+ddy_dyn);

    J1 = ((pt.Gu[i_mask]*fx)*dxd_dxn+(pt.Gv[i_mask]*fy)*dyd_dxn)/(-pt.zc[i_mask]);
    J2 = ((pt.Gu[i_mask]*fx)*dxd_dyn+(pt.Gv[i_mask]*fy)*dyd_dyn)/(-pt.zc[i_mask]);
    J3 = -(J1*pt.xn[i_mask])-(J2*pt.yn[i_mask]);
}

void IDNav::CamPinholeDist::xyn_2_uv(IDNav::Pt& pt){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < PATCH_SIZE; ++i_mask) {

        r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn[i_mask]*pt.yn[i_mask]+kc[3]*(r2+2*pt.xn[i_mask]*pt.xn[i_mask]);
        dy = kc[2]*(r2+2*pt.yn[i_mask]*pt.yn[i_mask])+2*kc[3]*pt.xn[i_mask]*pt.yn[i_mask];

        xd = coef*pt.xn[i_mask] + dx;
        yd = coef*pt.yn[i_mask] + dy;

        pt.u[i_mask] = fx * xd + cx;
        pt.v[i_mask] = fy * yd + cy;

    }

}

bool IDNav::CamPinholeDist::xyn_2_uv_isPointIn(IDNav::Pt& pt, int pyrLevel){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < PATCH_SIZE; ++i_mask) {

        r2 = (pt.xn[i_mask])*(pt.xn[i_mask])+(pt.yn[i_mask])*(pt.yn[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn[i_mask]*pt.yn[i_mask]+kc[3]*(r2+2*pt.xn[i_mask]*pt.xn[i_mask]);
        dy = kc[2]*(r2+2*pt.yn[i_mask]*pt.yn[i_mask])+2*kc[3]*pt.xn[i_mask]*pt.yn[i_mask];

        xd = coef*pt.xn[i_mask] + dx;
        yd = coef*pt.yn[i_mask] + dy;

        pt.u[i_mask] = fxG[pyrLevel] * xd + cxG[pyrLevel];
        pt.v[i_mask] = fyG[pyrLevel] * yd + cyG[pyrLevel];

//        pt.up[i_mask] = int(std::round(pt.u[i_mask]));
     //   pt.vp[i_mask] = int(std::round(pt.v[i_mask]));
    }

    return isPointIn(pt);
}

void IDNav::CamPinholeDist::xyn_2_uv_ref(IDNav::Pt &pt){
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE xd{}, yd{};

    for (size_t i_mask = 0; i_mask < PATCH_SIZE; ++i_mask) {

        r2 = (pt.xn_ref[i_mask])*(pt.xn_ref[i_mask])+(pt.yn_ref[i_mask])*(pt.yn_ref[i_mask]);
        coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
        dx = 2*kc[2]*pt.xn_ref[i_mask]*pt.yn_ref[i_mask]+kc[3]*(r2+2*pt.xn_ref[i_mask]*pt.xn_ref[i_mask]);
        dy = kc[2]*(r2+2*pt.yn_ref[i_mask]*pt.yn_ref[i_mask])+2*kc[3]*pt.xn_ref[i_mask]*pt.yn_ref[i_mask];

        xd = coef*pt.xn_ref[i_mask] + dx;
        yd = coef*pt.yn_ref[i_mask] + dy;

        pt.u_ref[i_mask] = fxG[0] * xd + cxG[0];
        pt.v_ref[i_mask] = fyG[0] * yd + cyG[0];

//        pt.up_ref[i_mask] = int(std::round(pt.u_ref[i_mask]));
  //      pt.vp_ref[i_mask] = int(std::round(pt.v_ref[i_mask]));
    }
}

void IDNav::CamPinholeDist::uv_2_xyn(std::vector<Pt>* points){
    for (IDNav::Pt& pt: *points){
        CamPinholeDist::uv_2_xyn(pt);
    }
}

void IDNav::CamPinholeDist::uv_2_xyn(IDNav::Pt& pt){

    DATA_TYPE xn, yn, xdRef, ydRef, xd, yd;;
    DATA_TYPE coef,r2,dx,dy;
    DATA_TYPE dcoef_dxn, dcoef_dyn;
    DATA_TYPE ddx_dxn, ddx_dyn, ddy_dxn, ddy_dyn;
    Eigen::VectorXf e(2);
    Eigen::Matrix2f J;
    Eigen::Matrix2f A;
    Eigen::VectorXf b(2);
    Eigen::VectorXf delta(2);

    for (size_t i_mask = 0; i_mask < PATCH_SIZE; ++i_mask) {
        xdRef = fxi*pt.u_ref[i_mask] + cxi;
        ydRef = fyi*pt.v_ref[i_mask] + cyi;
        xn = xdRef;
        yn = ydRef;

        for(size_t iteration{}; iteration < 10; ++iteration){

            r2 = (xn*xn)+(yn*yn);
            coef = 1+kc[0]*r2+kc[1]*(r2*r2)+kc[4]*(r2*r2*r2);
            dx = 2*kc[2]*xn*yn+kc[3]*(r2+2*xn*xn);
            dy = kc[2]*(r2+2*yn*yn)+2*kc[3]*xn*yn;

            xd = coef*xn + dx;
            yd = coef*yn + dy;

            e(0) = xdRef-xd;
            e(1) = ydRef-yd;

            dcoef_dxn = 2*xn*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);
            dcoef_dyn = 2*yn*(kc[0]+2*kc[1]*r2+3*kc[4]*r2*r2);

            ddx_dxn = 2*kc[2]*yn+kc[3]*(6*xn);
            ddx_dyn = 2*kc[2]*xn+kc[3]*(2*yn);
            ddy_dxn = 2*kc[3]*yn+kc[3]*(2*xn);
            ddy_dyn = 2*kc[3]*xn+kc[3]*(6*yn);

            J(0,0) = -(dcoef_dxn*xn+coef+ddx_dxn);
            J(0,1) = -(dcoef_dyn*xn+ddx_dyn);
            J(1,0) = -(dcoef_dxn*yn+ddy_dxn);
            J(1,1) = -(dcoef_dyn*yn+coef+ddy_dyn);

            A = J.transpose()*J;
            b = -J.transpose()*e;

            delta = A.colPivHouseholderQr().solve(b); // QR decomposition
            xn += delta(0);
            yn += delta(1);

        }
        pt.xn_ref[i_mask] = xn;
        pt.yn_ref[i_mask] = yn;
    }
}
*/

