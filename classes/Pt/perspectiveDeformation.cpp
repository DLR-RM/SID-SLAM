
#include "../../include/Pt.h"

void IDNav::Pt::estimatePerspectiveDeformation(dataType& def2_, mat2& F_,  const dataType& fx_, const dataType& fy_, const mat3& R_) const {
    mat23 J01{mat23::Zero()};
    J01 << fx_ * lambda[0], 0.0, -fx_ * lambda[0] * xn[0],
            0.0, fy_ * lambda[0], -fy_ * lambda[0] * yn[0];
    F_ = J01 * R_ * Jref;
    if ((F_(0, 0) < 0.0) || (F_(1, 1) < 0.0)) {
        def2_ = -1.0;
        return;
    }
    mat2 Cright = F_.transpose() * F_; // Right Cauchy-Green deformation tensor
    def2_ = dirRef.transpose() * Cright * dirRef;
}
IDNav::dataType IDNav::Pt::estimatePhotoStd(const dataType& fx_, const dataType& fy_, const mat3& R_, const CovCalibration* photoCal_) const{
    dataType def2{};
    mat2 F{};
    estimatePerspectiveDeformation(def2, F , fx_, fy_, R_);

    vec2 dir{F*dirRef};
    dataType normDir{dir.norm()};
    dir /= normDir;

    vec3 dI_dxyzcRef{},dI_dxyzc{};
    dI_dxyzc(0) = dir(0)*fx_*lambda[0];
    dI_dxyzc(1) = dir(1)*fy_*lambda[0];
    dI_dxyzc(2) = -(dI_dxyzc(0)*xn[0] + dI_dxyzc(1)*yn[0]);
    dI_dxyzcRef = dI_dxyzc.transpose()*R_;
    dataType dI_dlambdaRef = (dI_dxyzcRef(0)*xnRef[0] + dI_dxyzcRef(1)*ynRef[0] + dI_dxyzcRef(2))*(-1.0/(lambdaRef*lambdaRef));
    dataType lambdaCov = dI_dlambdaRef*dI_dlambdaRef*lambdaRefCov;

    if(def2 > photoCal_->maxDef2) def2 = photoCal_->maxDef2;
    if(def2 < photoCal_->minDef2) def2 = photoCal_->minDef2;
    dataType def2Cov;
    if(def2 < 1.0){
        def2Cov = photoCal_->ccDef2*((1.0/def2) - 1.0);
        //def2Cov = (1.0/def2) - 1.0;
    }else{
        def2Cov = photoCal_->ctDef2*(def2 - 1.0);
        //def2Cov = def2 - 1.0;
    }
    dataType geoCov =  def2Cov + photoCal_->ccDef2*lambdaCov;
    //dataType photoStdTmp= sqrt(  photoCal_->imgCov*(*imgGradient)*(*imgGradient) + G_ref*G_ref*geoCov);
    dataType photoStdTmp= photoCal_->imgStd*sqrt(  1.0 + def2Cov + lambdaCov);

    //cout << "photoCoc = "<< 1.0 + def2Cov + lambdaCov <<" , def2Cov = "<< 100*def2Cov/(1.0 + def2Cov + lambdaCov)<< " , lambdaCov = " << 100*lambdaCov/(1.0 + def2Cov + lambdaCov) << endl;
    return photoStdTmp;
}

bool IDNav::Ft:: estimateProjectionDeformation2D(mat2& Cleft, const dataType& fx, const dataType& fy, const mat3& R){
#ifdef PLANE_ESTIMATION
    mat2 J0{mat2::Zero(2,2)};
    J0 <<  fx,0.0,
            0.0,fy;
    matX J1{matX::Zero(2,3)};
    J1 << 1.0,0.0,-xn[0],
          0.0,1.0,-yn[0];
    J1 *= lambda[0];
    mat2 F = J0*J1*R*Jref;

#else
    mat2 F{mat2::Zero()}; //Perspective Deformation gradient tensor
    F <<           R(0,0) - xn[0]*R(2,0), (fx/fy)*(R(0,1) - xn[0]*R(2,1)),
          (fy/fx)*(R(1,0) - yn[0]*R(2,0)),         R(1,1) - yn[0]*R(2,1);

    F *= (lambda[0]/lambdaRef);
#endif
    if((F(0,0) < 0.0)||(F(1,1) < 0.0)){
        Cleft = 1000.0*mat2::Identity();
        return true;
    }
    Cleft = F*F.transpose(); // Left Cauchy-Green deformation tensor
    return true;
}

IDNav::mat2 IDNav::Ft::estimateGeoInf(const dataType& fx, const dataType& fy, const mat3& R, const CovCalibration* geoCal, const KeyPoint& matchedKeypt_){

    dataType sigma2MeanRef = pow(keyPtRef.size/minSize,2);
    dataType sigma2MeanProj = pow(matchedKeypt_.size/minSize,2);

    mat2 Cleft{mat2::Identity()};
    estimateProjectionDeformation2D(Cleft, fx,  fy, R);
    SelfAdjointEigenSolver<mat2> eigensolver(Cleft);

    dataType du_dx = fx*lambda[0];
    dataType dv_dy = fy*lambda[0];
    mat23 duv_dxyzc;
    duv_dxyzc << du_dx,  0.0 , -du_dx*xn[0],
                  0.0 , dv_dy, -dv_dy*yn[0];
    mat23 Ji = duv_dxyzc*R;
    vec2 lambdaCovVector{vec2::Zero()};
    for(int i{0}; i < 2; ++i){
        dataType J_lambda = (Ji(i,0)*xnRef[0] + Ji(i,1)*ynRef[0] + Ji(i,2))*(-1.0/(lambdaRef*lambdaRef));
        lambdaCovVector(i) = (1.0 + J_lambda*J_lambda)*lambdaRefCov;
    }

    mat2 def2Cov{mat2::Identity()};
    for(int i{0}; i < 2; ++i){
        dataType def2_i = eigensolver.eigenvalues()(i);

        if(def2_i > geoCal->maxDef2) {
            def2_i = geoCal->maxDef2;
        }
        if(def2_i < geoCal->minDef2) {
            def2_i = geoCal->minDef2;
        }

        dataType def2Cov_i;
        if(def2_i < 1.0){
            def2Cov_i = geoCal->ccDef2*((1.0/def2_i) - 1.0);
        }else{
            def2Cov_i = geoCal->ctDef2*(def2_i - 1.0);
        }
        def2Cov(i,i) = sigma2MeanRef*(def2Cov_i);
    }
    mat2 geoCov = sigma2MeanProj*geoCal->imgCov*mat2::Identity() + eigensolver.eigenvectors()*def2Cov*eigensolver.eigenvectors().transpose();
    geoCov(0,0) += lambdaCovVector(0);
    geoCov(1,1) += lambdaCovVector(1);

    mat2 geoInf_sqrt = geoCov.inverse().sqrt();
    return geoInf_sqrt;
}