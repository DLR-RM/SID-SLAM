//
// Created by font_al on 1/31/20.
//

#include "../include/Pose.h"

/// Projection Functions ///////////////////////////////////////////////////////////////////////////////////////////////
void IDNav::Pose::xynlambda_2_XYZ(vecPt& hgp) {
    for(typePt& pt: hgp)
        xynlambda_2_XYZ(pt);
}
void IDNav::Pose::xynlambda_2_XYZ(vecFt& features) {
    for(typeFt& ft: features)
        xynlambda_2_XYZ(ft);
}
void IDNav::Pose::XYZ_2_xynlambda(vecPt& hgp) {
    for(typePt& pt: hgp)
        XYZ_2_xynlambda(pt);
}
void IDNav::Pose::XYZ_2_xynlambda(vecFt& features) {
    for(typeFt& ft: features)
        XYZ_2_xynlambda(ft);
}

void IDNav::Pose::xynlambda_2_XYZ(typeIdNavPoint pt) {
    dataType xc_ref{}, yc_ref{}, zc_ref{};
    for (int iPatch = 0; iPatch < pt->ptSize; ++iPatch) {
        //Depth estimation
        zc_ref = pt->planeParameters(2)/(1.0-pt->xnRef[iPatch]*pt->planeParameters(0)-pt->ynRef[iPatch]*pt->planeParameters(1));
        //Depth projection
        xc_ref = pt->xnRef[iPatch] * zc_ref;
        yc_ref = pt->ynRef[iPatch] * zc_ref;
        // 3D projection
        pt->X[iPatch] = xc_ref * Rwc(0,0) + yc_ref * Rwc(0,1) + zc_ref * Rwc(0,2) + twc(0);
        pt->Y[iPatch] = xc_ref * Rwc(1,0) + yc_ref * Rwc(1,1) + zc_ref * Rwc(1,2) + twc(1);
        pt->Z[iPatch] = xc_ref * Rwc(2,0) + yc_ref * Rwc(2,1) + zc_ref * Rwc(2,2) + twc(2);
    }
}

void IDNav::Pose::XYZ_2_xynlambda(typeIdNavPoint pt){
    dataType xc,yc;
    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch){
        xc = pt->X[iPatch]*Rcw(0,0) + pt->Y[iPatch]*Rcw(0,1) + pt->Z[iPatch]*Rcw(0,2) + tcw(0);
        yc = pt->X[iPatch]*Rcw(1,0) + pt->Y[iPatch]*Rcw(1,1) + pt->Z[iPatch]*Rcw(1,2) + tcw(1);
        pt->lambda[iPatch] = 1.0/(pt->X[iPatch]*Rcw(2,0) + pt->Y[iPatch]*Rcw(2,1) + pt->Z[iPatch]*Rcw(2,2) + tcw(2));
        //Coordinate normalization
        pt->xn[iPatch] = xc*pt->lambda[iPatch];
        pt->yn[iPatch] = yc*pt->lambda[iPatch];
    }
}

/// Update Pose ////////////////////////////////////////////////////////////////////////////////////////////////////////
void IDNav::Pose::set_T_cw(const vec3& tcw_, const mat3& Rcw_) {
    tcw = tcw_;
    Rcw = Rcw_;
    IDNav::inv_T(twc, Rwc,tcw ,Rcw);
}

void IDNav::Pose::set_T_cw(const mat4& Tcw) {
    tcw = Tcw.block<3,1>(0,3);
    Rcw = Tcw.block<3,3>(0,0);
    IDNav::inv_T(twc, Rwc,tcw ,Rcw);
}

void IDNav::Pose::update_T_cw_left(vec6& delta_T){
    vec3 v = delta_T.segment(0,3);
    vec3 w = delta_T.segment(3,3);
    vec3 delta_t;
    mat3 delta_R;
    exp_lie(delta_t,delta_R,v,w);
    update_T_cw_left(delta_t,delta_R);
}

void IDNav::Pose::set_T_wc(const vec3& twc_, const mat3& Rwc_){
    twc = twc_;
    Rwc = Rwc_;
    IDNav::inv_T(tcw, Rcw,twc ,Rwc);
}

void IDNav::Pose::set_T_wc(const mat4& Twc) {
    twc = Twc.block<3,1>(0,3);
    Rwc = Twc.block<3,3>(0,0);
    IDNav::inv_T(tcw, Rcw,twc ,Rwc);
}

void IDNav::Pose::copyPoseFrom(const Pose& pose_){
    tcw = pose_.tcw;
    Rcw = pose_.Rcw;
    twc = pose_.twc;
    Rwc = pose_.Rwc;
}

void IDNav::Pose::copyStampedPoseFrom(const Pose& pose_) {
    copyPoseFrom(pose_);
    ts = pose_.ts;
}

void IDNav::Pose::update_T_cw_right(const vec3& delta_t, const mat3& delta_R) {
    IDNav::update_T_right(tcw,Rcw, delta_t, delta_R);
    IDNav::inv_T(twc, Rwc,tcw ,Rcw);
}

void IDNav::Pose::update_T_cw_left(const vec3& delta_t, const mat3& delta_R) {
    IDNav::update_T_left(delta_t,delta_R, tcw, Rcw);
    IDNav::inv_T(twc, Rwc,tcw ,Rcw);
}

void IDNav::Pose::update_T_wc_left(const vec3& delta_t, const mat3& delta_R) {
    IDNav::update_T_left(delta_t,delta_R, twc, Rwc);
    IDNav::inv_T(tcw, Rcw,twc ,Rwc);
}

void IDNav::Pose::print(){
    cout << "Pose = " << endl;
    cout << "twc = "  << twc.transpose() << " " << endl;
    cout << "Rwc = " << endl;
    cout << Rwc <<  endl;

}

/// Relative Movement //////////////////////////////////////////////////////////////////////////////////////////////////
IDNav::vec6 IDNav::Pose::relativeMovement(const Pose& pose_) const{
    mat3  Rrel;
    vec3  trel;
    vec3 v{},w{};
    vec6 relMov{};
    IDNav::transformConcatenation(Rrel, trel,pose_.Rcw, pose_.tcw,Rwc,twc);
    IDNav::log_lie(v,w,trel, Rrel);

    relMov = vecX::Zero(6);
    relMov.head(3) = v;
    relMov.tail(3) = w;

    return relMov;
};

void IDNav::Pose::relativeMovement(vec3& trel, mat3& Rrel, const Pose& pose_) const{
    IDNav::transformConcatenation(Rrel, trel,pose_.Rcw, pose_.tcw,Rwc,twc);
};

IDNav::mat4  IDNav::Pose::relMovement(const Pose& pose_) const{
    mat3  Rrel{};
    vec3  trel{};
    vec3 v{},w{};
    IDNav::transformConcatenation(Rrel, trel, pose_.Rwc, pose_.twc, Rcw,tcw);

    mat4 T{mat4::Identity()};
    T.block<3,1>(0,3) = trel;
    T.block<3,3>(0,0) = Rrel;

    return T;
}
