//
// Created by font_al on 10/5/18.
//

#include "../../include/Frame.h"
#include "../../utils/informationFunctions.h"

IDNav::dataType IDNav::maxLambdaVis{1.2};
IDNav::dataType IDNav::minLambdaVis{1.0/1.2};
IDNav::dataType IDNav::minCosVis{0.5};

void IDNav::setVisibilityParameters(dataType& maxLambdaVis_){
    maxLambdaVis = maxLambdaVis_;
    minLambdaVis = 1.0/maxLambdaVis_;
}


void IDNav::Frame::get_PhotometricError(vecX& e, typePt& pt, const int& iPyr) {
    for(int iPatch{}; iPatch < cam->patchSize; ++iPatch){
        e(iPatch) = (*(pt->refPhScalarExp))*(pt->I_ref[iPatch][iPyr]-(*(pt->refPhBias))) - phScalarExp*(pt->I[iPatch]-phBias);
    }
}

void IDNav::Frame::compute_inf_ph_poseJacobian(typePt& pt, mat16& dI_dx) {

    //dataType photoConstant = 1.0;//phScalarExp/pt->photoStd;
    dataType photoConstant = cam->get_imgPhotoStd()/pt->photoStd0;

    vec3 du_dxyzc{}, dv_dxyzc{};
    cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

    mat26 duv_dx{};
    pt->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);

    mat12 dI_duv{};
    dataType Gu{0.0},Gv{0.0};
    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch){
        extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch],
                                                     pt->u[iPatch], pt->v[iPatch]);
        Gu += pt->Gu[iPatch];
        Gv += pt->Gv[iPatch];
    }
    Gu /= PATCH_SIZE;
    Gv /= PATCH_SIZE;
    dataType G = sqrt(pow(Gu,2) + pow(Gv,2));
    dI_duv << Gu, Gv;
    if(G > 0.0){
    //    dI_duv *= *(pt->imgGradient)/G;
        dI_duv /= G;
    }
    dI_dx = dI_duv * duv_dx;
    dI_dx *= photoConstant;
    //cout << dI_dx << endl;
    //cout << " " << endl;
    /*vec3 Jphoto;
    size_t iPatch = 0;
    cam->computeJPhotometric(Jphoto(0), Jphoto(1), Jphoto(2), pt, iPatch);

    J  << -Jphoto(0), -Jphoto(1), -Jphoto(2),
        -(-Jphoto(1) + Jphoto(2) * pt->yn[iPatch]) / pt->lambda[iPatch],
        -(Jphoto(0) - Jphoto(2) * pt->xn[iPatch]) / pt->lambda[iPatch],
        -(-Jphoto(0) * pt->yn[iPatch] + Jphoto(1) * pt->xn[iPatch]) / pt->lambda[iPatch];

    //J *= phScalarExp;
    dataType G = sqrt(pow(pt->Gu[iPatch], 2) + pow(pt->Gv[iPatch], 2));
    if (G > 0.0) {
        J /= G;
        J *= pt->G_mean;//pt->G_ref;
    }
    J /= (pt->photoStd);*/
}

void IDNav::Frame::computeInformativeGeoJacobian(typeFt& ft, row26& duv_dx_inf) {

    vec3 du_dxyzc{}, dv_dxyzc{};
    cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);
    mat26 duv_dx{};
    ft->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);
    //duv_dx_inf = ft->geoInf * duv_dx;
    duv_dx_inf =  ft->geoInf0 *duv_dx;

    /*vec3 Ju,  Jv;
    row26 Jtemp{row26::Zero()};
    cam->computeJCam(Ju, Jv,ft,0,0);
    Jtemp.row(0) << -Ju(0), -Ju(1), -Ju(2),
        -(-Ju(1) + Ju(2) * ft->yn[0]) / ft->lambda[0],
        -(Ju(0) - Ju(2) * ft->xn[0]) / ft->lambda[0],
        -(-Ju(0) * ft->yn[0] + Ju(1) * ft->xn[0]) / ft->lambda[0];
    Jtemp.row(1) << -Jv(0), -Jv(1), -Jv(2),
        -(-Jv(1) + Jv(2) * ft->yn[0]) / ft->lambda[0],
        -(Jv(0) - Jv(2) * ft->xn[0]) / ft->lambda[0],
        -(-Jv(0) * ft->yn[0] + Jv(1) * ft->xn[0]) / ft->lambda[0];

    J = ft->geoInf*Jtemp;*/
}

bool IDNav::Frame::planeEstimation(dataType& alpha_ , dataType& beta_ ,
                                   const typePt& pt_,
                                   ceres::Solver::Options& problemOptions_,
                                   ceres::Solver::Summary& summary_,
                                   const dataType& fx_,const dataType& fy_,
                                   const dataType& cx_,const dataType& cy_,
                                   const dataType& w_, const dataType& h_){
    // Function parameters
    const static int minNumObservations{4};
    const static double depthInc{0.05};

    static const double minDepthKinect{0.01};
    static const double maxDepthKinect{MAX_DEPTH_VALUE};

    // Init point projection
    double xn0 = (pt_->uRef - cx_)/fx_;
    double yn0 = (pt_->vRef - cy_)/fy_;
    double z0 = 1.0/pt_->lambdaRef;

#ifdef PLANE_ESTIMATION
    // Search for nearest depth values
    vector<double> z_vec{},xn_vec{},yn_vec{};
    int numObservations{0};
    {
        double xn,yn,z,u,v;
        for (int iPatch{0}; iPatch < PATCH_SIZE; ++iPatch){
            u  = pt_->uRef + cam-> patch_u[iPatch];
            v  = pt_->vRef + cam-> patch_v[iPatch];
            if((u >= 0)&&(v >= 0)&&(u < w_)&&(v < h_)){
                z = double(depthImg.at<DEPHT_VALUE_TYPE>(int(v),int(u)));
                if((not std::isinf(z))&&(not std::isnan(z))&&(z > minDepthKinect)&&(z < MAX_DEPTH_VALUE)){
                    xn = (u - cx_)/fx_;
                    yn = (v - cy_)/fy_;
                    z_vec.push_back(z);
                    xn_vec.push_back(xn);
                    yn_vec.push_back(yn);
                    ++numObservations;
                }
            }
        }
    }

    // Get consistent depth values for plane estimation
    vector<double> z_vec_tmp{},xn_vec_tmp{},yn_vec_tmp{};
    {
        if(numObservations >= minNumObservations){
            numObservations = 0;
            for(int i_z{0}; i_z < z_vec.size(); ++i_z){
                if(abs(z_vec[i_z]-z0) < depthInc){
                    z_vec_tmp.push_back(z_vec[i_z]);
                    xn_vec_tmp.push_back(xn_vec[i_z]);
                    yn_vec_tmp.push_back(yn_vec[i_z]);
                    ++numObservations;
                }
            }
        }
    }

    // Plane approximation
    {
        if(numObservations >= minNumObservations){
            ceres::Problem problem;
            ceres::CauchyLoss* cLoss = new ceres::CauchyLoss(0.01);
            for(int i_z{0}; i_z < z_vec_tmp.size(); ++i_z){
                ceres::CostFunction *cost_function = new DepthResidual(z0, xn0, yn0,
                                                                       z_vec_tmp[i_z],
                                                                       xn_vec_tmp[i_z],yn_vec_tmp[i_z]);
                problem.AddResidualBlock(cost_function, cLoss, &alpha_, &beta_);

            }
            ceres::Solve(problemOptions_, &problem, &summary_);
            if(summary_.termination_type != ceres::CONVERGENCE){
                alpha_ = -xn0;
                beta_  = -yn0;
                return false;
            }
            return true;
        }
        // If there is no plane estimation assumes the plane is perpendicular to the projection axis
        alpha_ = -xn0;
        beta_  = -yn0;
        return false;
    }
#else
    // If there is no plane estimation assumes the plane is perpendicular to the z-axis
    alpha_ = 0;
    beta_ = 0;
    return true;
#endif
}

bool IDNav::Frame::planeEstimation(dataType& alpha_ , dataType& beta_, typeFt& ft, ceres::Solver::Options& problemOptions, ceres::Solver::Summary& summary,
                                   const dataType& fx,const dataType& fy,
                                   const dataType& cx,const dataType& cy,
                                   const dataType& w, const dataType& h){

    dataType uRef,vRef;
    cam->distortPt(uRef,vRef,ft->uRef,ft->vRef);
    dataType xn0 = (uRef - cx)/fx;
    dataType yn0 = (vRef - cy)/fy;
    dataType z0 = 1.0/ft->lambdaRef;

#ifdef PLANE_ESTIMATION
    ceres::Problem problem;
    // Search for nearest depth values
    vector<double> z_vec{},xn_vec{},yn_vec{};
    int numObservations{0};
    {
        double xn,yn,z,u,v;
        for (int iPatch{0}; iPatch < PATCH_SIZE; ++iPatch){
            u  = uRef + cam-> patch_u[iPatch];
            v  = vRef + cam-> patch_v[iPatch];
            if((u >= 0)&&(v >= 0)&&(u < w)&&(v < h)){
                xn = (u - cx)/fx;
                yn = (v - cy)/fy;
                z = dataType(depthImg.at<DEPHT_VALUE_TYPE>(int(v),int(u)));
                if((not std::isinf(z))&&(not std::isnan(z))&&(z > 0.01)&&(z < MAX_DEPTH_VALUE)){
                    z_vec.push_back(z);
                    xn_vec.push_back(xn);
                    yn_vec.push_back(yn);
                    ++numObservations;
                }
            }
        }
    }

    // Get consistent depth values for plane estimation
    int minNumObservations{4};
    vector<double> z_vec_tmp{},xn_vec_tmp{},yn_vec_tmp{};
    {
        if(numObservations >= minNumObservations){
            numObservations = 0;
            for(int i_z{0}; i_z < z_vec.size(); ++i_z){
                if(abs(z_vec[i_z]-z0) < 0.05){
                    z_vec_tmp.push_back(z_vec[i_z]);
                    xn_vec_tmp.push_back(xn_vec[i_z]);
                    yn_vec_tmp.push_back(yn_vec[i_z]);
                    ++numObservations;
                }
            }
        }
    }

    // Plane approximation
    {
        ceres::CauchyLoss* cLoss = new ceres::CauchyLoss(0.01);
        if(numObservations >= minNumObservations){
            for(int i_z{0}; i_z < z_vec_tmp.size(); ++i_z){
                ceres::CostFunction *cost_function = new DepthResidual(z0, xn0, yn0,
                                                                       z_vec_tmp[i_z],
                                                                       xn_vec_tmp[i_z],yn_vec_tmp[i_z]);
                problem.AddResidualBlock(cost_function, cLoss, &alpha_, &beta_);
            }
            ceres::Solve(problemOptions, &problem, &summary);
            if(summary.termination_type != ceres::CONVERGENCE){
                alpha_ = -xn0;
                beta_  = -yn0;
                return false;
            }
        }else{
            alpha_ = -xn0;
            beta_  = -yn0;
            return false;
        }
    }

    return true;
#else
    alpha_ = 0; // alpha
    beta_ = 0; // beta
    return true;
#endif

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function extracts intensities and gradients of a certain pixel which belongs to the mask of a point.
bool IDNav::Frame::extract_I_and_G_pixel_BilinInt(dataType& I, dataType& Gu, dataType& Gv, dataType& u, dataType& v){
    int iPyr = cam->getPyrLevel();
    int up{int(u)}, vp{int(v)};
    if (cam->isPixelInside(up,vp)){
        dataType inc_u{u - up}, inc_v{v - vp};
        dataType I11 = grayImgG[iPyr].at<uchar>(vp, up);
        dataType I12 = grayImgG[iPyr].at<uchar>(vp, up + 1);
        dataType I21 = grayImgG[iPyr].at<uchar>(vp + 1, up);
        dataType I22 = grayImgG[iPyr].at<uchar>(vp + 1, up + 1);

        I = (I11 * (1 - inc_v) + I21 * inc_v) * (1 - inc_u) + (I12 * (1 - inc_v) + I22 * inc_v) * inc_u;
        Gu = (I22 - I21) * inc_v + (I12 - I11) * (1 - inc_v);
        Gv = (I22 - I12) * inc_u + (I21 - I11) * (1 - inc_u);
        return true;
    }
    else{
        //I = 0.0; Gu = 0.0; Gv = 0.0;
        Gu = 0.0; Gv = 0.0;
        return false;
    }
}

bool IDNav::Frame::extract_I_and_G_pixel_Kernel(dataType& I, dataType& Gu, dataType& Gv, dataType& u, dataType& v){
    int iPyr = cam->getPyrLevel();
    int up = round(u);
    int vp = round(v);
    if (cam->isPixelInside(up,vp)){
        I = grayImgG[iPyr].at<uchar>(vp, up);
        Gu = (dataType((dataType)grayImgG[iPyr].at<uchar>(vp-1, up+1) + 2.0*(dataType)grayImgG[iPyr].at<uchar>(vp, up+1) + (dataType)grayImgG[iPyr].at<uchar>(vp+1, up+1)) -
                dataType((dataType)grayImgG[iPyr].at<uchar>(vp-1, up-1) + 2.0*(dataType)grayImgG[iPyr].at<uchar>(vp, up-1) + (dataType)grayImgG[iPyr].at<uchar>(vp+1, up-1)))/8.0;

        Gv = (dataType((dataType) grayImgG[iPyr].at<uchar>(vp+1, up-1) + 2.0*(dataType)grayImgG[iPyr].at<uchar>(vp+1, up) + (dataType)grayImgG[iPyr].at<uchar>(vp+1, up+1))-
                dataType((dataType)grayImgG[iPyr].at<uchar>(vp-1, up-1) + 2.0*(dataType)grayImgG[iPyr].at<uchar>(vp-1, up) + (dataType)grayImgG[iPyr].at<uchar>(vp-1, up+1)))/8.0;
        return true;
    }
    else{
        I = 0.0;
        Gu = 0.0;
        Gv = 0.0;
        return false;
    }
}

//This function extracts the intensity for a certain pixel which belongs to the mask of a point.
bool IDNav::Frame::extract_I_pixel_BilinInt(dataType& I,  const dataType& u, const dataType& v){
    int iPyr = cam->getPyrLevel();
    int up{int(u)}, vp{int(v)};
    if (cam->isPixelInside(up,vp)){
        dataType inc_u{u - up}, inc_v{v - vp};
        dataType I11 = grayImgG[iPyr].at<uchar>(vp, up);
        dataType I12 = grayImgG[iPyr].at<uchar>(vp, up + 1);
        dataType I21 = grayImgG[iPyr].at<uchar>(vp + 1, up);
        dataType I22 = grayImgG[iPyr].at<uchar>(vp + 1, up + 1);

        I = (I11 * (1 - inc_v) + I21 * inc_v) * (1 - inc_u) + (I12 * (1 - inc_v) + I22 * inc_v) * inc_u;
        return true;
    }
    else{
        //I = 0.0;
        return false;
    }
}

bool IDNav::Frame::extract_I_pixel_Kernel(dataType& I,  const dataType& u, const dataType& v){
    int iPyr = cam->getPyrLevel();
    int up = round(u);
    int vp = round(v);
    if (cam->isPixelInside(up,vp)){
        I = (dataType)grayImgG[iPyr].at<uchar>(vp, up);
        return true;
    }{
        I = 0.0;
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function extracts intensities and gradients of the pixels which belongs to the mask of a point.
void IDNav::Frame::extract_I_and_G_BilinInt(typePt& pt){
    for(size_t iMask{}; iMask < cam->patchSize; ++iMask)
        extract_I_and_G_pixel_BilinInt(pt->I[iMask],pt->Gu[iMask],pt->Gv[iMask], pt->u[iMask], pt->v[iMask]);
}

void IDNav::Frame::extract_I_and_G_Kernel(shared_ptr<Pt>& pt){
    for(size_t iMask{}; iMask < cam->patchSize; ++iMask)
        extract_I_and_G_pixel_Kernel(pt->I[iMask],pt->Gu[iMask],pt->Gv[iMask], pt->u[iMask], pt->v[iMask]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function extracts the reference intensities for all HGP related to the frame, for the whole mask and, at all pyramid levels.
void IDNav::Frame::extract_I_reference() {
    cam->setResolution(0);
    for (shared_ptr<Pt> &pt: hgp)
        extract_I_ref_BilinInt(pt);
        //extract_I_ref_Kernel(pt);

    for (int iPyr{0}; iPyr < cam->getNumPyrLevelsUsed(); ++iPyr) {
        cam->setResolution(iPyr);
        for (shared_ptr<IDNav::Pt> &pt: hgp)
            extract_I_ref_BilinInt(pt);
            //extract_I_ref_Kernel(pt);
    }
    cam->setResolution(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function extracts intensities of the pixels which belongs to the mask of a point.
void IDNav::Frame::extract_I_ref_BilinInt(shared_ptr<Pt>& pt) {
    int iPyr = cam->getPyrLevel();
    dataType u0{pt->uRef * cam->getScaleFactor()},v0{pt->vRef * cam->getScaleFactor()};
    extract_I_pixel_BilinInt(pt->I_ref[0][iPyr],u0,v0);
    dataType uRef,vRef{};
    for (int iMask{1}; iMask < cam->patchSize; ++iMask) {
        uRef = u0 + cam->patch_u[iMask];
        vRef = v0 + cam->patch_v[iMask];
        extract_I_pixel_BilinInt(pt->I_ref[iMask][iPyr],uRef, vRef);
    }
}

void IDNav::Frame::extract_I_ref_Kernel(shared_ptr<Pt>& pt) {
    int iPyr = cam->getPyrLevel();
    dataType u0{pt->uRef * cam->getScaleFactor()},v0{pt->vRef * cam->getScaleFactor()};
    extract_I_pixel_Kernel(pt->I_ref[0][iPyr],u0,v0);
    dataType uRef,vRef{};
    for (int iMask{1}; iMask < cam->patchSize; ++iMask) {
        uRef = u0 + cam->patch_u[iMask];
        vRef = v0 + cam->patch_v[iMask];
        extract_I_pixel_Kernel(pt->I_ref[iMask][iPyr],uRef, vRef);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function extract the reference intensity and gradients for ONE point related to the frame,
// for the whole mask and, at ONE pyramid level.
void IDNav::Frame::extract_I_and_G_ref_BilinInt(shared_ptr<Pt>& pt) {
    int iPyr = cam->getPyrLevel();
    dataType u0{pt->uRef},v0{pt->vRef};
    dataType emptyGu,emptyGv;
    extract_I_and_G_pixel_BilinInt(pt->I_ref[0][iPyr],emptyGu,emptyGv,u0,v0);
    //pt->G_ref = sqrt(pow(pt->Gu_ref[0],2)+pow(pt->Gv_ref[0],2));
    //pt->dirRef = vecX::Zero(2);
    //pt->dirRef(0) = pt->Gu_ref[0]/pt->G_ref;
    //pt->dirRef(1) = pt->Gv_ref[0]/pt->G_ref;

    //pt->Gu_mask = pt->Gu_ref[0];
    //pt->Gv_mask = pt->Gv_ref[0];

    dataType uRef,vRef;
    for (int iMask{1}; iMask < PATCH_SIZE; ++iMask) {
        uRef  = u0 + cam->patch_u[iMask];
        vRef  = v0 + cam->patch_v[iMask];
        extract_I_and_G_pixel_BilinInt(pt->I_ref[iMask][iPyr],emptyGu,emptyGv,uRef,vRef);

        //pt->Gu_mask += pt->Gu_ref[iMask]*cam->patchWeight[iMask];
        //pt->Gv_mask += pt->Gv_ref[iMask]*cam->patchWeight[iMask];
    }
}

void IDNav::Frame::extract_I_and_G_ref_Kernel(shared_ptr<Pt>& pt) {
    int iPyr = cam->getPyrLevel();
    dataType u0{pt->uRef},v0{pt->vRef};
    extract_I_and_G_pixel_Kernel(pt->I_ref[0][iPyr],pt->Gu_ref,pt->Gv_ref,u0,v0);

    //pt->Gu_mask = pt->Gu_ref[0];
    //pt->Gv_mask = pt->Gv_ref[0];
    //dataType G_ref = sqrt(pow(pt->Gu_ref[0],2)+pow(pt->Gv_ref[0],2));
    //pt->dirRef = vecX::Zero(2);
    //pt->dirRef(0) = pt->Gu_ref[0]/G_ref;
    //pt->dirRef(1) = pt->Gv_ref[0]/G_ref;

    dataType uRef,vRef;
    dataType emptyGu,emptyGv;
    for (int iMask{1}; iMask < PATCH_SIZE; ++iMask) {
        uRef  = u0 + cam->patch_u[iMask];
        vRef  = v0 + cam->patch_v[iMask];
        extract_I_and_G_pixel_Kernel(pt->I_ref[iMask][iPyr],emptyGu,emptyGv,uRef,vRef);

        //pt->Gu_mask += pt->Gu_ref[iMask]*cam->patchWeight[iMask];
        //pt->Gv_mask += pt->Gv_ref[iMask]*cam->patchWeight[iMask];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IDNav::Frame::projectPointsTo(typeFrame& frame_){
    projectHgpTo(frame_);
    projectFeaturesTo(frame_);
}

void IDNav::Frame::fromKeyframeProjectHgpTo(typeFrame& frame_, vecPt& hgp_){
    uv_2_XYZ(hgp_);
    frame_->XYZ_2_uv(hgp_);
    //for(typePt& pt: hgp_){
    //    cam->uv_2_xyn(pt);
    //    pose.xynlambda_2_XYZ(pt);
    //    frame_->XYZ_2_uv(pt);
    //}
}

void IDNav::Frame::projectHgpTo(typeFrame& frame_){
    for(typePt& pt: hgp)
        frame_->XYZ_2_uv(pt);
}

void IDNav::Frame::projectFeaturesTo(typeFrame& frame_){
    for(typeFt& ft: features)
        frame_->XYZ_2_uv(ft);
}

void IDNav::Frame::getVisiblePointsIn(typeFrame frame_, vecPt& hgp_, vecFt& features_){
    vecFt features_aux{};
    getGeometryVisiblePointsIn(frame_, hgp_, features_aux);

    features_.clear();
    for(typeFt& ft: features_aux) {
        if(frame_->observations.find(ft) != frame_->observations.end()){
            features_.push_back(ft);
        }
    }
}

void IDNav::Frame::getGeometryVisiblePointsIn(typeFrame frame_, vecPt& hgp_, vecFt& features_){
    projectPointsTo(frame_);
    hgp_.clear();
    for(typePt& pt: hgp) {
        if (frame_->isGeoVisible(pt, maxLambdaVis, minLambdaVis, minCosVis)){
            hgp_.push_back(pt);
        }
    }
    features_.clear();
    for(typeFt& ft: features) {
        if (frame_->isGeoVisible(ft, maxLambdaVis, minLambdaVis, minCosVis)){
            features_.push_back(ft);
        }
    }
}

void IDNav::Frame::getVisibleHgpIn(typeFrame frame_, vecPt& hgp_){
    projectHgpTo(frame_);

    hgp_.clear();
    for(typePt& pt: hgp) {
        if (frame_->isGeoVisible(pt, maxLambdaVis, minLambdaVis, minCosVis)){
            hgp_.push_back(pt);
        }
    }
}

void IDNav::Frame::getVisibleFeaturesIn(typeFrame frame_, vecFt& features_){
    projectFeaturesTo(frame_);

    size_t geoVisibleFt = 0;
    size_t obsVisibleFt = 0;
    features_.clear();
    for(typeFt& ft: features) {
        ft->visible = false;
        if (frame_->isGeoVisible(ft, maxLambdaVis, minLambdaVis, minCosVis)){
            ++geoVisibleFt;
            if(frame_->observations.find(ft) != frame_->observations.end()){
                ++obsVisibleFt;
                ft->visible = true;
                features_.push_back(ft);
            }
        }
    }
}

IDNav::dataType IDNav::Frame::getCovisibilityRatio(typeFrame frame_){
    std::mutex mMutex;
    unique_lock<mutex> lock(mMutex);
    vecPt hgp_aux{};
    vecFt ft_aux{};
    getGeometryVisiblePointsIn(frame_,hgp_aux,ft_aux);
    return float(hgp_aux.size() + ft_aux.size())/float(hgp.size() + features.size());
}

void IDNav::Frame::XYZ_2_uv(typeIdNavPoint pt){
    pose.XYZ_2_xynlambda(pt);
    cam->xyn_2_uv(pt);
}
void IDNav::Frame::XYZ_2_uv(vecPt& hgp_){
    pose.XYZ_2_xynlambda(hgp_);
    cam->xyn_2_uv(hgp_);
}
void IDNav::Frame::XYZ_2_uv(vecFt& features_){
    pose.XYZ_2_xynlambda(features_);
    cam->xyn_2_uv(features_);
}

void IDNav::Frame::uv_2_XYZ(typeIdNavPoint pt){
    cam->uv_2_xyn(pt);
    pose.xynlambda_2_XYZ(pt);
}
void IDNav::Frame::uv_2_XYZ(vecPt& hgp_){
    cam->uv_2_xyn(hgp_);
    pose.xynlambda_2_XYZ(hgp_);
}
void IDNav::Frame::uv_2_XYZ(vecFt& features_){
    cam->uv_2_xyn(features_);
    pose.xynlambda_2_XYZ(features_);
}



