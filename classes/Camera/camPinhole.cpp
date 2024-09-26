//
// Created by font_al on 10/26/18.
//

#include "../../include/Camera.h"

// TESTED FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////
// TESTED FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////
// TESTED FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////
// TESTED FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////
// TESTED FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////

void IDNav::CamPinhole::dI_dxyzc(vec3& dI_dxyzc_, const typePt& pt,  const size_t& iPatch){
    dI_dxyzc_(0) = pt->Gu[iPatch]*fx*pt->lambda[iPatch];
    dI_dxyzc_(1) = pt->Gv[iPatch]*fy*pt->lambda[iPatch];
    dI_dxyzc_(2) = -(dI_dxyzc_(0)*pt->xn[iPatch] + dI_dxyzc_(1)*pt->yn[iPatch]);
}

void IDNav::CamPinhole::duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typePt& pt , const int& iPatch){
    dataType du_dx = fx*pt->lambda[iPatch];
    dataType dv_dy = fy*pt->lambda[iPatch];
    du_dxyzc << du_dx,  0.0  , -du_dx*pt->xn[iPatch];
    dv_dxyzc <<  0.0 , dv_dy , -dv_dy*pt->yn[iPatch];
}

void IDNav::CamPinhole::duv_dxyzc(mat23& duv_dxyzc_, typeFt& ft){
    dataType du_dx = fxG[0]*ft->lambda[0];
    dataType dv_dy = fyG[0]*ft->lambda[0];
    duv_dxyzc_ << du_dx,  0.0  , -du_dx*ft->xn[0],
            0.0 , dv_dy , -dv_dy*ft->yn[0];
}

void IDNav::CamPinhole::duv_dxyzc(vec3& du_dxyzc, vec3& dv_dxyzc, const typeFt& ft){
    dataType du_dx = fxG[0]*ft->lambda[0];
    dataType dv_dy = fyG[0]*ft->lambda[0];
    du_dxyzc << du_dx,  0.0  , -du_dx*ft->xn[0];
    dv_dxyzc <<  0.0 , dv_dy , -dv_dy*ft->yn[0];
}

// Camera Jacobians
// This function computes the jacobians (du_dxyzc , dv_dxyzc) of the image coordinates (u v) of a point with respect to
// its camera coordinates (x y z)_c.
void IDNav::CamPinhole::computeJCam(vec3& du_dxyzc, vec3& dv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_){
    dataType du_dx = fxG[iPyr_]*pt_->lambda[iPatch_];
    dataType dv_dy = fyG[iPyr_]*pt_->lambda[iPatch_];
    du_dxyzc << du_dx, 0.0  , -du_dx*pt_->xn[iPatch_];
    dv_dxyzc <<  0.0 , dv_dy, -dv_dy*pt_->yn[iPatch_];
}

void IDNav::CamPinhole::computeJCam(mat23& duv_dxyzc, typeIdNavPoint pt_, const size_t& iPatch_, const size_t& iPyr_){
    dataType du_dx = fxG[iPyr_]*pt_->lambda[iPatch_];
    dataType dv_dy = fyG[iPyr_]*pt_->lambda[iPatch_];
    duv_dxyzc << du_dx,  0.0 , -du_dx*pt_->xn[iPatch_],
                  0.0 , dv_dy, -dv_dy*pt_->yn[iPatch_];
}

// NO TESTED FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
// NO TESTED FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
// NO TESTED FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
// NO TESTED FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////
// NO TESTED FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////////


IDNav::dataType IDNav::CamPinhole::getInvDepthCov(){return invDepthCov;};

IDNav::CamPinhole::CamPinhole(const dataType& fxValue, const dataType& fyValue,
        const dataType& cxValue, const dataType& cyValue,
        const size_t& wValue, const size_t& hValue,
        const dataType& stereoBaseline_, const dataType& association_th_){

    Camera::selectPatch(PATCH_SIZE);
    computePyrLevelSlope(wValue,hValue);
    initImageDimensions(wValue,hValue);
    cameraMatrix = (Mat1d(3, 3) << fxValue, 0, cxValue, 0, fyValue, cyValue, 0, 0, 1);
    cameraMatrixUndist = cameraMatrix;
    distCoeffs = Mat::zeros(1,5,CV_64F);
    initGlobalCalibration(fxValue, fyValue, cxValue, cyValue);
    IDNav::CamPinhole::setResolution(0);

    stereoBaseline = stereoBaseline_;
    association_th = association_th_;
    invDepthCov = pow(1/(stereoBaseline*fxValue),2)*pixelCov;
    imageGrid.build(int(w),int(h));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function sets the resolution parameters for a certain pyramid level.
void IDNav::CamPinhole::setResolution(const int& pyrLevel_){
    iPyr = pyrLevel_;
    scaleFactor = scaleFactorG[pyrLevel_];
    fx = fxG[pyrLevel_]; fy = fyG[pyrLevel_];
    cx = cxG[pyrLevel_]; cy = cyG[pyrLevel_];
    fxi = fxiG[pyrLevel_]; fyi = fyiG[pyrLevel_];
    cxi = cxiG[pyrLevel_]; cyi = cyiG[pyrLevel_];
    w = wG[pyrLevel_]; h = hG[pyrLevel_];
    wMax = wGmax[pyrLevel_];  hMax = hGmax[pyrLevel_];
    imageGrid.u_max = wGmax[pyrLevel_]-imageMargin;
    imageGrid.v_max = hGmax[pyrLevel_]-imageMargin;
    finestPyrLevel = (iPyr == 0);
}

// This function computes a slope factor for the resolution pyramid.
void IDNav::CamPinhole::computePyrLevelSlope(size_t w_, size_t h_){
    dataType factor = pow(0.5,numPyrLevelsMax-1);//sqrt(5000.0/double(w_*h_));

    pyrLevelScaleSlope = pow(factor,-1.0/(numPyrLevelsMax-1));
}

// This function computes the scaling factor and resolution for every pyramid level.
void IDNav::CamPinhole::initImageDimensions(const size_t& wValue, const size_t& hValue){
    for(int pyrLevel{}; pyrLevel < numPyrLevelsMax; ++pyrLevel){
        scaleFactorG[pyrLevel] = pow(pyrLevelScaleSlope,-pyrLevel);
        wG[pyrLevel]  = round(wValue*scaleFactorG[pyrLevel]);
        hG[pyrLevel]  = round(hValue*scaleFactorG[pyrLevel]);
        wGmax[pyrLevel] = wG[pyrLevel] - imageMargin;
        hGmax[pyrLevel] = hG[pyrLevel] - imageMargin;
        numPixelsG[pyrLevel] = wG[pyrLevel]*hG[pyrLevel];
    }

    imageGrid.sizeCellPixels_u = float(wValue)/float(imageGrid.numCells_u);
    imageGrid.sizeCellPixels_v = float(hValue)/float(imageGrid.numCells_v);
    imageGrid.u_min = imageMargin;
    imageGrid.v_min = imageMargin;
}

// Function to compute the calibration for all pyramid levels
void IDNav::CamPinhole::initGlobalCalibration(const dataType& fxValue,const dataType& fyValue,const dataType& cxValue,const dataType& cyValue){

    for(size_t pyr = 0; pyr < numPyrLevelsMax; ++pyr){
        fxG[pyr] = fxValue*scaleFactorG[pyr];
        fyG[pyr] = fyValue*scaleFactorG[pyr];
        cxG[pyr] = cxValue*scaleFactorG[pyr];
        cyG[pyr] = cyValue*scaleFactorG[pyr];

        fxiG[pyr] = 1/fxG[pyr];
        fyiG[pyr] = 1/fyG[pyr];
        cxiG[pyr] = - cxG[pyr]/fxG[pyr];
        cyiG[pyr] = - cyG[pyr]/fyG[pyr];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IDNav::CamPinhole::xyn_2_uv(vecPt& hgp){
    for (typePt& pt: hgp)
        xyn_2_uv(pt);
}
void IDNav::CamPinhole::xyn_2_uv(vecFt& features){
    for (typeFt& ft: features)
        xyn_2_uv(ft);
}
// This function projects one point 'pt' from normalized coordinates (xn,yn,lambda) to image coordinates (uv).
void IDNav::CamPinhole::xyn_2_uv(typeIdNavPoint pt){
    for (int iPatch = 0; iPatch < pt->ptSize; ++iPatch) {
        pt->u[iPatch] = fx *  pt->xn[iPatch] + cx;
        pt->v[iPatch] = fy *  pt->yn[iPatch] + cy;
    }
}
void IDNav::CamPinhole::xyn_2_uv_iPyr(typeIdNavPoint pt, const int& iPyr_){
    for (int iPatch = 0; iPatch < pt->ptSize; ++iPatch) {
        pt->u[iPatch] = fxG[iPyr_] *  pt->xn[iPatch] + cxG[iPyr_];
        pt->v[iPatch] = fyG[iPyr_] *  pt->yn[iPatch] + cyG[iPyr_];
    }
}

void IDNav::CamPinhole::uv_2_xyn(vecPt& hgp){
    for (typePt& pt: hgp)
        uv_2_xyn(pt);
}
void IDNav::CamPinhole::uv_2_xyn(vecFt& features){
    for (typeFt& ft: features)
        uv_2_xyn(ft);
}

// This function project one point 'pt' from image coordinates uv to normalized coordinates xn, yn. 'iPyr' stands for the
// resolution level of the pyramid.
void IDNav::CamPinhole::uv_2_xyn(typeIdNavPoint pt){
    for (int iPatch = 0; iPatch < pt->ptSize; ++iPatch) {
        pt->xnRef[iPatch] = fxi*(pt->uRef*scaleFactor + patch_u[iPatch]) + cxi;
        pt->ynRef[iPatch] = fyi*(pt->vRef*scaleFactor + patch_v[iPatch]) + cyi;
    }
}

void IDNav::CamPinhole::distortPt(dataType& uDist, dataType& vDist, const dataType uUndist, const  dataType vUndist){
    Point2d ptUndist;
    ptUndist.x = uUndist;
    ptUndist.y = vUndist;
    vector<Point2d> ptsOut{ptUndist};
    vector<Point3d> ptsTemp{};
    Mat rtemp, ttemp;
    rtemp.create( 3, 1, CV_32F );
    rtemp.setTo( 0 );
    rtemp.copyTo( ttemp );
    undistortPoints( ptsOut, ptsOut, cameraMatrixUndist, noArray());
    convertPointsToHomogeneous( ptsOut, ptsTemp );
    projectPoints( ptsTemp, rtemp, ttemp, cameraMatrix, distCoeffs, ptsOut );
    uDist = ptsOut[0].x;
    vDist = ptsOut[0].y;
}

void IDNav::CamPinhole::undistortPt(dataType& uUndist, dataType& vUndist, const dataType uDist, const  dataType vDist){
    Mat point(1,2,CV_64F);
    point.at<double>(0,0) = uDist;
    point.at<double>(0,1) = vDist;
    undistortPoints (point, point, cameraMatrix, distCoeffs, Mat() , cameraMatrixUndist);
    uUndist = point.at<double>(0,0);
    vUndist = point.at<double>(0,1);
}

//This function computes the jacobian of the intensity I with respect to the camera coordinates of the point (x y z)_c.
void IDNav::CamPinhole::computeJPhotometric(dataType& dI_dx, dataType& dI_dy, dataType& dI_dz, shared_ptr<Pt> pt, const size_t& iMask){
    dI_dx = pt->Gu[iMask]*fx*pt->lambda[iMask];
    dI_dy = pt->Gv[iMask]*fy*pt->lambda[iMask];
    dI_dz = -(dI_dx*pt->xn[iMask] + dI_dy*pt->yn[iMask]);
}

void IDNav::CamPinhole::computeJPhotometricRef(vec6& dI_dpose, typePt& pt){
    vec3 dI_xynlambda{};
    get_dI_xynlambda(dI_xynlambda, pt->Gu_ref, pt->Gv_ref, fxG[0], fyG[0] ,pt->xnRef[0],pt->ynRef[0], pt->lambdaRef);
    get_dI_pose(dI_dpose, dI_xynlambda, pt->xnRef[0] , pt->ynRef[0] , pt->lambdaRef);

    /*dataType dI_dx = pt->Gu_ref*fx*pt->lambdaRef;
    dataType dI_dy = pt->Gv_ref*fy*pt->lambdaRef;
    dataType dI_dz = -(dI_dx*pt->xnRef[0] + dI_dy*pt->ynRef[0]);
    get_dI_pose(dI_dpose, dI_dx, dI_dy, dI_dz, pt->xnRef[0] , pt->ynRef[0] , pt->lambdaRef);
    dI_dpose << dI_dx, dI_dy, dI_dz,
               (-dI_dy + dI_dz * (pt->ynRef[0])) / pt->lambdaRef,
               ( dI_dx - dI_dz * (pt->xnRef[0])) / pt->lambdaRef,
               (-dI_dx * (pt->ynRef[0]) + dI_dy * (pt->xnRef[0])) / pt->lambdaRef;*/
}

void IDNav::CamPinhole::computeJGeometricRef(vec6& Ju,vec6& Jv, typeFt& keypt){
    vec3 Ju_, Jv_;
    Ju_(0) = fxG[0] * keypt->lambdaRef;
    Ju_(1) = 0;
    Ju_(2) = -Ju_(0)*keypt->xnRef[0];
    Jv_(0) = 0;
    Jv_(1) = fyG[0]*keypt->lambdaRef;
    Jv_(2) = -Jv_(1)*keypt->ynRef[0];

    Ju << Ju_(0), Ju_(1), Ju_(2),
            (-Ju_(1) + Ju_(2) * (keypt->ynRef[0])) / keypt->lambdaRef,
            (Ju_(0) - Ju_(2) * (keypt->xnRef[0])) / keypt->lambdaRef,
            (-Ju_(0) * (keypt->ynRef[0]) + Ju_(1) * (keypt->xnRef[0])) / keypt->lambdaRef;

    Jv << Jv_(0), Jv_(1), Jv_(2),
            (-Jv_(1) + Jv_(2) * (keypt->ynRef[0])) / keypt->lambdaRef,
            (Jv_(0) - Jv_(2) * (keypt->xnRef[0])) / keypt->lambdaRef,
            (-Jv_(0) * (keypt->ynRef[0]) + Jv_(1) * (keypt->xnRef[0])) / keypt->lambdaRef;

}


/*void IDNav::CamPinhole::computeJCam(dataType Ju[3], dataType Jv[3], typeIdNavPoint pt, const size_t& iMask){
    Ju[0] = fx*pt->lambda[iMask];
    Ju[1] = 0;
    Ju[2] = -Ju[0]*pt->xn[iMask];
    Jv[0] = 0;
    Jv[1] = fy*pt->lambda[iMask];
    Jv[2] = -Jv[1]*pt->yn[iMask];
}*/

/*void IDNav::CamPinhole::computeJCam(vec3& Ju, vec3& Jv,typeFt& pt){
    Ju[0] = fx*pt->lambda[0];
    Ju[1] = 0;
    Ju[2] = -Ju[0]*pt->xn[0];
    Jv[0] = 0;
    Jv[1] = fy*pt->lambda[0];
    Jv[2] = -Jv[1]*pt->yn[0];
}*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SEMANTIC_SEGMENTATION
// This function returns if a certain pixel is inside the limits of the image.
bool IDNav::CamPinhole::isPixelInside(const int& up, const int& vp){
    if (up < 0) return false;
    if (vp < 0) return false;
    return ((up < (wG[iPyr]-1))&&((vp< hG[iPyr]-1)));
}

bool IDNav::CamPinhole::xyn_2_uv_isPointIn(std::shared_ptr<IDNav::Pt>& pt, Mat& mask){
    for (size_t i_mask = 0; i_mask < 1; ++i_mask) {
        pt->u[i_mask] = fxG[iPyr] *  pt->xn[i_mask] + cxG[iPyr];
        pt->v[i_mask] = fyG[iPyr] *  pt->yn[i_mask] + cyG[iPyr];
    }
    #ifdef SEMANTIC_SEGMENTATION
    return isPointIn(pt,mask);
    #else
    return isPointIn(pt);
    #endif
}

// This function returns if a projected point is inside the limits of the image.
bool IDNav::CamPinhole::isPointIn(std::shared_ptr<IDNav::Pt>& pt, Mat& mask){
    if (pt->lambda[0] <  0) return false;
    if (pt->u[0] < patchSurface) return false;
    if (pt->v[0] < patchSurface) return false;
    if((pt->u[0] > (w-patchSurface))||((pt->v[0] > h-patchSurface))){return false;}
    if(int(mask.at<uchar>(int(pt->v[0]),int(pt->u[0]))) == 0){
        return true;
    }else{
        return false;
    }
}
#else
// This function returns if a certain pixel is inside the limits of the image.
bool IDNav::CamPinhole::isPixelInside(const int& up, const int& vp){
    if (up < 0) return false;
    if (vp < 0) return false;
    return ((up < (wG[iPyr]-1))&&((vp< hG[iPyr]-1)));
}

bool IDNav::CamPinhole::xyn_2_uv_isPointIn(std::shared_ptr<IDNav::Pt>& pt){
    for (size_t i_mask = 0; i_mask < 1; ++i_mask) {
        pt->u[i_mask] = fxG[iPyr] *  pt->xn[i_mask] + cxG[iPyr];
        pt->v[i_mask] = fyG[iPyr] *  pt->yn[i_mask] + cyG[iPyr];
    }
    return isPointIn(pt);
}

bool IDNav::CamPinhole::xyn_2_uv_isPointIn(typeFt& ft){
    ft->u[0] = fxG[0] *  ft->xn[0] + cxG[0];
    ft->v[0] = fyG[0] *  ft->yn[0] + cyG[0];
    return isPointIn(ft);
}

// This function returns if a projected point is inside the limits of the image.
bool IDNav::CamPinhole::isPointIn(typePt& pt){
    if (pt->lambda[0] <  0) return false;
    if (pt->u[0] < patchSurface) return false;
    if (pt->v[0] < patchSurface) return false;
    return ((pt->u[0] < (w-patchSurface))&&((pt->v[0]< h-patchSurface)));
}
bool IDNav::CamPinhole::isPointIn(typeFt& ft){
    if (ft->lambda[0] <  0) return false;
    if (ft->u[0] < patchSurface) return false;
    if (ft->v[0] < patchSurface) return false;
    return ((ft->u[0] < (w-patchSurface))&&((ft->v[0]< h-patchSurface)));
}
#endif


void IDNav::CamPinhole::show_calibration(){

    std::cout << BOLDBLACK_COUT << "Show CamPinhole calibration" << RESET_COUT<<std::endl;
    std::cout <<std::setw(10)<< "pyrLevel"<<std::setw(10)<< "fx" << std::setw(10)<< "fy" << std::setw(10)<< "cx"<< std::setw(10)<< "cy" <<
              std::setw(10)<< "wG" << std::setw(10)<< "hG" << std::endl;

    for(size_t i = 0; i < numPyrLevelsMax; ++i)
    {
        std::cout << std::setw(10) << i << std::setw(10) << fxG[i] <<  std::setw(10) << fyG[i] << std::setw(10) << cxG[i] << std::setw(10) << cyG[i]
                  << std::setw(10) << wG[i] << std::setw(10) << hG[i]<< std::endl;
    }
    cout << "RGB frequence    : " << RGB_hz   << " (Hz) "<<endl;
    cout << "depth frequence  : " << depth_hz << " (Hz) "<<endl;
    std::cout << BOLDBLACK_COUT << "Deformation model Parameters" << RESET_COUT<<std::endl;
    cout << "maxPhotoDef2     : " << photoCal.maxDef2      <<"     minPhotoDef2     : " << photoCal.minDef2 <<endl;
    cout << "ctPhotoDef2      : " << photoCal.ctDef2       <<"     ccPhotoDef2      : " << photoCal.ccDef2 << endl;
    cout << "imgPhotoCov      : " << photoCal.imgCov       <<"     imgPhotoStd      : " << photoCal.imgStd << endl;
    cout << "maxGeoDef2       : " << geoCal.maxDef2        <<"     minGeoDef2       : " << geoCal.minDef2 <<endl;
    cout << "ctGeoDef2        : " << geoCal.ctDef2         <<"     ccGeoDef2        : " << geoCal.ccDef2 << endl;
    cout << "imgGeoCov        : " << geoCal.imgCov         <<"     imgGeoStd        : " << geoCal.imgStd << endl;
    std::cout << BOLDBLACK_COUT << "Kinect sensor" << RESET_COUT<<std::endl;
}

void IDNav::CamPinhole::readAndUndistortImage(cv::Mat& image){}