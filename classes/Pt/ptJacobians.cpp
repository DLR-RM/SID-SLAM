
#include "../../include/Pt.h"

void IDNav::IdNavPoint::duv_dPoseRef(mat26& duv_dPoseRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const size_t& iPatch){
    duv_dPoseRef.row(0) << -du_dxyzcRef(0),-du_dxyzcRef(1),-du_dxyzcRef(2),
            -(-du_dxyzcRef(1) + du_dxyzcRef(2) * ynRef[iPatch]) / lambdaRef,
            -( du_dxyzcRef(0) - du_dxyzcRef(2) * xnRef[iPatch]) / lambdaRef,
            -(-du_dxyzcRef(0) * ynRef[iPatch] + du_dxyzcRef(1) * xnRef[iPatch]) / lambdaRef;

    duv_dPoseRef.row(1) << -dv_dxyzcRef(0),-dv_dxyzcRef(1),-dv_dxyzcRef(2),
            -(-dv_dxyzcRef(1) + dv_dxyzcRef(2) * ynRef[iPatch]) / lambdaRef,
            -( dv_dxyzcRef(0) - dv_dxyzcRef(2) * xnRef[iPatch]) / lambdaRef,
            -(-dv_dxyzcRef(0) * ynRef[iPatch] + dv_dxyzcRef(1) * xnRef[iPatch]) / lambdaRef;
}

void IDNav::IdNavPoint::duv_dxRef(mat26& duv_dxRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const int& iPatch){
    duv_dxRef.row(0) << -du_dxyzcRef(0),-du_dxyzcRef(1),-du_dxyzcRef(2),
            -(-du_dxyzcRef(1) + du_dxyzcRef(2) * ynRef[iPatch]) / lambdaRef,
            -( du_dxyzcRef(0) - du_dxyzcRef(2) * xnRef[iPatch]) / lambdaRef,
            -(-du_dxyzcRef(0) * ynRef[iPatch] + du_dxyzcRef(1) * xnRef[iPatch]) / lambdaRef;

    duv_dxRef.row(1) << -dv_dxyzcRef(0),-dv_dxyzcRef(1),-dv_dxyzcRef(2),
            -(-dv_dxyzcRef(1) + dv_dxyzcRef(2) * ynRef[iPatch]) / lambdaRef,
            -( dv_dxyzcRef(0) - dv_dxyzcRef(2) * xnRef[iPatch]) / lambdaRef,
            -(-dv_dxyzcRef(0) * ynRef[iPatch] + dv_dxyzcRef(1) * xnRef[iPatch]) / lambdaRef;
}

void IDNav::IdNavPoint::duv_dx(mat26& duv_dx , const vec3& du_dxyzc, const vec3& dv_dxyzc, const int& iPatch){
    duv_dx.row(0) << du_dxyzc(0), 0.0 ,du_dxyzc(2),
            (du_dxyzc(2) * yn[iPatch]) / lambda[iPatch],
            ( du_dxyzc(0) - du_dxyzc(2) * xn[iPatch]) / lambda[iPatch],
            (-du_dxyzc(0) * yn[iPatch]) / lambda[iPatch];

    duv_dx.row(1) << 0.0 , dv_dxyzc(1), dv_dxyzc(2),
            (-dv_dxyzc(1) + dv_dxyzc(2) * yn[iPatch]) / lambda[iPatch],
            (- dv_dxyzc(2) * xn[iPatch]) / lambda[iPatch],
            (dv_dxyzc(1) * xn[iPatch]) / lambda[iPatch];
}

void IDNav::IdNavPoint::duv_dlambdaRef(mat21& duv_dlambdaRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const size_t& iPatch){
    duv_dlambdaRef(0) = (du_dxyzcRef(0)*xnRef[iPatch] + du_dxyzcRef(1)*ynRef[iPatch] + du_dxyzcRef(2))*(-1.0/(lambdaRef*lambdaRef));
    duv_dlambdaRef(1) = (dv_dxyzcRef(0)*xnRef[iPatch] + dv_dxyzcRef(1)*ynRef[iPatch] + dv_dxyzcRef(2))*(-1.0/(lambdaRef*lambdaRef));
}
