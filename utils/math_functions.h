//
// Created by font_al on 10/9/18.
//

#ifndef IDNAV_MATH_FUNCTIONS_H
#define IDNAV_MATH_FUNCTIONS_H

#include "SystemSettings.h"
#include "SequenceSettings.h"

namespace IDNav {

    struct CovCalibration{
        dataType maxDef2{};
        dataType minDef2{};
        dataType ctDef2{};
        dataType ccDef2{};
        dataType imgCov{};
        dataType imgStd{};

        void setPhotoCalibration(dataType maxDef2_ , dataType minDef2_ , dataType ctDef2_, dataType ccDef2_, dataType imgCov_){
            maxDef2 = maxDef2_;
            minDef2 = minDef2_;
            ctDef2 = ctDef2_;
            ccDef2 = ccDef2_;
            imgCov = pow(2.0*PATCH_SIZE,2.0)*imgCov_;
            imgStd = sqrt(imgCov);
        }
        void setReprojCalibration(dataType maxDef2_ , dataType minDef2_ , dataType ctDef2_, dataType ccDef2_, dataType imgCov_){
            maxDef2 = maxDef2_;
            minDef2 = minDef2_;
            ctDef2 = ctDef2_;
            ccDef2 = ccDef2_;
            imgCov = imgCov_;
            imgStd = sqrt(imgCov);
        }
    };

    inline void get_dI_xynlambda(vec3& dI_xynlambda,
                            const dataType & Gu, const dataType& Gv,
                            const dataType & fx, const dataType &fy ,
                            const dataType & xn, const dataType &yn, const dataType &lambda){
        dI_xynlambda(0) = Gu*fx*lambda;
        dI_xynlambda(1) = Gv*fy*lambda;
        dI_xynlambda(2) = -(dI_xynlambda(0)*xn + dI_xynlambda(1)*yn);
    }

    inline void get_dI_pose(vec6& dI_dpose,
                            const vec3& dI_xynlambda,
                            const dataType &xn, const dataType &yn , const dataType &lambda){
        dI_dpose << dI_xynlambda(0), dI_xynlambda(1), dI_xynlambda(2),
                (-dI_xynlambda(1) + dI_xynlambda(2) * yn) / lambda,
                ( dI_xynlambda(0) - dI_xynlambda(2) * xn) / lambda,
                (-dI_xynlambda(0) * yn + dI_xynlambda(1) * xn) / lambda;
    }

    static dataType exposureFunction(const dataType& x, dataType A = ((256.0/255.0)-1.0), dataType  v = 2.0){
        ///if(abs(x) > 2.5)
            //return 1.0 + (x/abs(x))*A;
        //return (2.0*A)/(1.0 + exp(-v*x)) + (1.0 - A);
        return exp(-x);
    }

    static dataType dexposureFunction_dx(const dataType& x, dataType A = ((256.0/255.0)-1.0), dataType  v = 2.0){
        //if(abs(x) > 2.5)
            //return 0.0;
        //return (2.0*A*v*exp(-v*x))/pow(1.0 + exp(-v*x),2);
        return -exp(-x);
    }

    inline dataType chi2Value(const size_t& dof, const dataType& p ){
        if(dof == 1){
            if(p == 90.0)
                return 2.706;
            if(p == 95.0)
                return 3.841;
        }
        if(dof == 2){
            if(p == 50.0)
                return 1.3863;
            if(p == 55.0)
                return 1.5970;
            if(p == 60.0)
                return 1.8326;
            if(p == 65.0)
                return 2.0996;
            if(p == 70.0)
                return 2.408;
            if(p == 75.0)
                return 2.7726;
            if(p == 80.0)
                return  3.219;
            if(p == 85.0)
                return 3.794;
            if(p == 90.0)
                return 4.605;
            if(p == 95.0)
                return 5.991;
            if(p == 97.5)
                return 7.378;
            if(p == 99.9)
                return 9.210;
            if(p == 99.95)
                return 10.597;
        }

        if(dof == 9){
            if(p == 50.0)
                return 8.3428;
            if(p == 55.0)
                return 8.8632;
            if(p == 60.0)
                return 9.4136;
            if(p == 65.0)
                return 10.0060;
            if(p == 70.0)
                return 10.6564;
            if(p == 75.0)
                return 11.3888;
            if(p == 80.0)
                return 12.2421;
            if(p == 85.0)
                return 13.288;
            if(p == 90.0)
                return 14.684;
            if(p == 95.0)
                return 16.919;
            if(p == 97.5)
                return 19.023;
            if(p == 99.9)
                return 21.666;
            if(p == 99.95)
                return 23.589;
        }
        return 0.0;
    }

    inline dataType tstudent2(const int& dof, const dataType& p ){
        if(dof == 2){
            if(p == 50.0)
                return 0.6680;
            if(p == 60.0)
                return  1.1305;
            if(p == 70.0)
                return 1.9178;
            if(p == 80.0)
                return 3.5645;
            if(p == 85.0)
                return 5.1670;
            if(p == 90.0)
                return  8.4759;
            if(p == 95.0)
                return 18.4388;
            if(p == 97.5)
                return 38.3971;
        }
        if(dof == 9){
            if(p == 50.0)
                return 0.4938;
            if(p == 60.0)
                return  0.7796;
            if(p == 70.0)
                return 1.2102;
            if(p == 80.0)
                return 1.9168;
            if(p == 85.0)
                return 2.4737;
            if(p == 90.0)
                return 3.3579;
            if(p == 95.0)
                return 5.0877;
            if(p == 97.5)
                return 7.2070;
        }

        cout << LIGTH_RED_COUT << "tstudent2 (): value not found for dof = "<< dof << " , p = "<< p << "%"<< RESET_COUT << endl;
        cout << LIGTH_RED_COUT << "              returning value for dof = "<< 9   << " , p = "<< 95.0 << "%"<< RESET_COUT << endl;

        return 0.0;
    }


    inline dataType tstudent2FromMedian(const size_t& dof, const dataType& p , const dataType& median){
        dataType m;
        if(dof == 2){
            if(p == 50.0)
                m =  1.0;
            if(p == 60.0)
                m =  1.6950;
            if(p == 70.0)
                m =  2.8823;
            if(p == 80.0)
                m =  5.3358;
            if(p == 85.0)
                m =  7.7549;
            if(p == 90.0)
                m = 12.7428;
            if(p == 95.0)
                m =  27.7360;
            if(p == 97.5)
                m =  57.7575;
        }
        if(dof == 9){
            if(p == 50.0)
                m = 1.0;
            if(p == 60.0)
                m = 1.5801;
            if(p == 70.0)
                m = 2.4479;
            if(p == 80.0)
                m = 3.8721;
            if(p == 85.0)
                m = 5.0062;
            if(p == 90.0)
                m = 6.7977;
            if(p == 95.0)
                m = 10.3134;
            if(p == 97.5)
                m = 14.5655;
        }

        return m*median;
    }

    /// Basic Algebra///////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void inv_R(mat3& R_inv, const mat3& R){
        //R_inv = R.transpose();
        R_inv = R.inverse();
    }

    inline void inv_T(vec3& t_inv, mat3& R_inv, const vec3& t, const mat3& R){
        inv_R(R_inv,R);
        t_inv = -R_inv*t;
    }

    inline void transformConcatenation(mat3& R, vec3& t, const mat3& R1, const vec3& t1, const mat3& R2,const vec3& t2){
        // T = T1*T2
        // R = R1*R2
        // t = R1*t2+t1
        R = R1*R2;
        t = R1*t2 + t1;
    }

    inline void update_T_right(vec3& t, mat3& R, const vec3& delta_t, const mat3& delta_R) {
        mat3 R_temp = R;
        vec3 t_temp = t;
        transformConcatenation(R,t, R_temp, t_temp, delta_R, delta_t);
    }

    inline void update_T_left(const vec3& delta_t , const mat3& delta_R, vec3& t, mat3& R) {
        mat3 R_temp = R;
        vec3 t_temp = t;
        transformConcatenation(R,t, delta_R, delta_t , R_temp, t_temp);
    }

    inline void antisymmetricMatrix(mat3& antiMatrix , const vec3& v){
        antiMatrix << 0.0        , -v(2), v(1),
                       v(2),     0.0    ,-v(0),
                      -v(1),  v(0),    0.0;
    }
    inline void exp_lie(vec3& delta_t, mat3& delta_R, const vec3& v, const vec3& w) {

        /*dataType w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};

        dataType theta = sqrt(w0_2 + w1_2 + w2_2);
        dataType coef1{1.0}, coef2{0.5}, coef3{1.0/6.0};

        if (theta > 0.0001) {
            coef1 = (sin(theta)) / (theta);
            coef2 = (1.0 - cos(theta)) / (theta * theta);
            coef3 = (theta - sin(theta)) / (theta * theta * theta);
        }

        dataType term0, term1, term2;
        /////////////
        term0 = -(w2_2 + w1_2);
        delta_R(0,0) =  1.0 + coef2 * term0;
        delta_t(0) = (1.0 + coef3 * term0) * v[0];
        /////////////
        term0 = -(w2_2 + w0_2);
        delta_R(1,1) =  1.0 + coef2 * term0;
        delta_t(1) = (1.0 + coef3 * term0) * v[1];
        /////////////
        term0 = -(w1_2 + w0_2);
        delta_R(2,2) =  1.0 + coef2 * term0;
        delta_t(2) = (1.0 + coef3 * term0) * v[2];
        /////////////
        term0 = w[0] * w[1];
        term1 = coef1 * w[2];
        term2 = coef2 * term0;
        delta_R(0,1) = -term1 + term2;
        delta_R(1,0) =  term1 + term2;
        /////////////
        term1 = coef2 * w[2];
        term2 = coef3 * term0;
        delta_t(0) += (-term1 + term2) * v[1];
        delta_t(1) += ( term1 + term2) * v[0];
        /////////////
        term0 = w[0] * w[2];
        term1 = coef1 * w[1];
        term2 = coef2 * term0;
        delta_R(0,2) =  term1 + term2;
        delta_R(2,0) = -term1 + term2;
        /////////////
        term1 = coef2 * w[1];
        term2 = coef3 * term0;
        delta_t(0) +=  (term1 + term2) * v[2];
        delta_t(2) += (-term1 + term2) * v[0];
        /////////////
        term0 = w[1] * w[2];
        term1 = coef1 * w[0];
        term2 = coef2 * term0;
        delta_R(1,2) = -term1 + term2;
        delta_R(2,1) =  term1 + term2;
        /////////////
        term1 = coef2 * w[0];
        term2 = coef3 * term0;
        delta_t(1) += (-term1 + term2) * v[2];
        delta_t(2) += ( term1 + term2) * v[1];

        cout << "exp_lie" << endl;
        cout << "t" << endl;
        cout <<  delta_t.transpose() << endl;
        cout << "R "<< endl;
        cout <<  delta_R << endl;*/

        mat4 Texp{};
        Texp <<  0.0         ,-w(2) , w(1),v(0),
                w(2) ,     0.0    ,  -w(0),v(1),
                -w(1) , w(0) ,     0.0    ,v(2),
                0.0    ,     0.0    ,     0.0   ,    0.0;
        mat4 T = Texp.exp();
        delta_t =  T.block<3,1>(0,3);
        delta_R = T.block<3,3>(0,0);
        /*cout << "exp_lie" << endl;
        cout << "t" << endl;
        cout <<  delta_t.transpose() << endl;
        cout << "R "<< endl;
        cout <<  delta_R << endl;
        wf(".");*/
    }

    inline void log_lie(vec3& v, vec3& w, const vec3& t, const mat3& R) {

        /*dataType theta = acos((R[0]+R[4]+R[8]-1)/2);
        dataType factor = 0.5;

        if (abs(theta) > 0.00000000001) {
            factor = theta/(2*sin(theta));
        }

        w[0] = factor*(R[7]-R[5]);
        w[1] = factor*(R[2]-R[6]);
        w[2] = factor*(R[3]-R[1]);

        dataType w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};
        dataType term0, term1;

        factor = dataType(1/12);
        theta = sqrt(w0_2 + w1_2 + w2_2);
        if (abs(theta) > 0.00000000001) {
            factor = (1-((theta*cos(theta/2))/(2*sin(theta/2))))/(theta*theta);
        }

        /////////////
        v[0] = (1-factor*(w2_2+w1_2))*t[0];
        v[1] = (1-factor*(w2_2+w0_2))*t[1];
        v[2] = (1-factor*(w1_2+w0_2))*t[2];
        /////////////
        term0 = 0.5f*w[2];
        term1 = factor*(w[0]*w[1]);
        v[0] += ( term0+term1)*t[1];
        v[1] += (-term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[1];
        term1 = factor*(w[0]*w[2]);
        v[0] += (-term0+term1)*t[2];
        v[2] += ( term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[0];
        term1 = factor*(w[1]*w[2]);
        v[1] += ( term0+term1)*t[2];
        v[2] += (-term0+term1)*t[1];*/

        /*cout << "log lie" << endl;
        cout << v.transpose() << endl;
        cout << w.transpose() << endl;*/

        mat4 T{mat4::Identity()};
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        mat4 Texp = T.log();
        v = T.block<3,1>(0,3);
        w << T(2,1),T(0,2),T(1,0);

        /*cout << v.transpose() << endl;
        cout << w.transpose() << endl;
        wf(".");*/
    }

    inline dataType euclideanDistance(const vec3& v1, const vec3& v2) {
        return (v1-v2).norm();
    }

    inline void meanAndStd(dataType& mean_, dataType& std_, const matX& v){
        mean_ = v.mean();
        std_ = sqrt(((v.array() - mean_).square()).mean());
    }

    inline void medianAndStd(dataType& median_, dataType& std_, const matX& v){
        median_ = 0.0;
        float numValues = v.size();
        float ratio{0.0};
        matX v_abs = v.array().abs();
        dataType meanValue = v_abs.mean();
        dataType step = meanValue/40.0;

        while(ratio < 0.5){
            median_ += step;
            ratio = float((v_abs.array() < median_).cast<int>().sum())/numValues;
        }
        std_ = median_/0.67;
    }

    inline mat4_float dataType_to_mat4_float(vec3&t, mat3& R){
        mat4_float T = mat4_float::Identity();
        for(int iTras{}; iTras < 3; ++iTras){
            T(iTras,3) = (float)t(iTras);
        }
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                T(i,j) = (float)R(i,j);
            }
        }
        return T;
    }

    inline mat4 mat4_float_toDataType(mat4_float& T_){
        mat4 T = mat4::Identity();
        for(int iTras{}; iTras < 3; ++iTras){
            T(iTras,3) = (dataType)T_(iTras,3);
        }
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                T(i,j) = (dataType)T_(i,j);
            }
        }
        return T;
    }

/// Basic Algebra///////////////////////////////////////////////////////////////////////////////////////////////////////
    /*inline bool isVectorFinite(vecX& vector){
        for (int i{}; i < vector.size(); ++i){
            if( not std::isfinite(vector[i]))
                return false;
        }
        return true;
    }

    inline bool isMatrixrFinite(matX& matrix){
        for (int i{}; i < matrix.size(); ++i){
            if( not std::isfinite(matrix(i)))
                return false;
        }
        return true;
    }

    inline bool isMatrixFinite(matX& matrix){
        for (int i{}; i < matrix.size(); ++i){
            if ((not std::isfinite(matrix(i))||(std::isnan(matrix(i)))))
                    return false;
        }
        return true;
    }

    //This function transforms a quaternion q = [qw;qx;qy;qz] into its corresponding rotational matrix.
    inline void quat2rot(mat3_& R, dataType const q[4]){
        R[0] = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]; R[1] = 2*q[1]*q[2]-2*q[0]*q[3];                 R[2] = 2*q[1]*q[3]+2*q[0]*q[2];
        R[3] = 2*q[1]*q[2]+2*q[0]*q[3];                 R[4] = q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3]; R[5] = 2*q[2]*q[3]-2*q[0]*q[1];
        R[6] = 2*q[1]*q[3]-2*q[0]*q[2];                 R[7] = 2*q[2]*q[3]+2*q[0]*q[1];                 R[8] = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
    }

    inline void assign_Zero_Vector3(mat3_& result){
        result[0] = 0; result[1] = 0; result[2] = 0;
    }

    inline void showVector3(const vec3_ v) {
        std::cout << v[0]<<" "<< v[1] << " " << v[2] << std::endl;
    }

    inline void showMatrix3(const mat3_ M) {
        std::cout << "M = "<<  std::endl;

        std::cout << M[0]<<" "<< M[1] << " " << M[2] << std::endl;
        std::cout << M[3]<<" "<< M[4] << " " << M[5] << std::endl;
        std::cout << M[6]<<" "<< M[7] << " " << M[8] << std::endl;
    }

    inline void assign_Matrix3(mat3_& result, const mat3_ M){
        std::copy_n(M,9,result);
    }

    inline void assign_Vector3(vec3_& result, const vec3_ v){
        std::copy_n(v,3,result);}

    inline void transpose3(mat3_ Rt, mat3_ const R){
        dataType aux{};
        Rt[0] =  R[0]; Rt[4] =  R[4]; Rt[8] =  R[8];
        aux = R[1]; Rt[1] =  R[3];Rt[3] =  aux;
        aux = R[2]; Rt[2] =  R[6];Rt[6] =  aux;
        aux = R[5]; Rt[5] =  R[7];Rt[7] =  aux;
    }

    inline void Matrix3_x_vector3(vec3_& result, const mat3_ M, const vec3_ v){
        result[0] = M[0]*v[0]+M[1]*v[1]+M[2]*v[2];
        result[1] = M[3]*v[0]+M[4]*v[1]+M[5]*v[2];
        result[2] = M[6]*v[0]+M[7]*v[1]+M[8]*v[2];
    }

    inline void Vector3_x_Matrix3(vec3_& result, const vec3_ v, const mat3_ M ){
        result[0] = v[0]*M[0]+v[1]*M[3]+v[2]*M[6];
        result[1] = v[0]*M[1]+v[1]*M[4]+v[2]*M[7];
        result[2] = v[0]*M[2]+v[1]*M[5]+v[2]*M[8];
    };

    inline void Matrix3_x_Vector3(vec3_& result,const mat3_ M, const vec3_ v ){
        result[0] = M[0]*v[0]+M[1]*v[1]+M[2]*v[2];
        result[1] = M[3]*v[0]+M[4]*v[1]+M[5]*v[2];
        result[2] = M[6]*v[0]+M[7]*v[1]+M[8]*v[2];
    };

    inline void Matrix3_x_Matrix3(mat3_& result, const mat3_ M1, const mat3_ M2){
        result[0] = M1[0]*M2[0]+M1[1]*M2[3]+M1[2]*M2[6];
        result[1] = M1[0]*M2[1]+M1[1]*M2[4]+M1[2]*M2[7];
        result[2] = M1[0]*M2[2]+M1[1]*M2[5]+M1[2]*M2[8];

        result[3] = M1[3]*M2[0]+M1[4]*M2[3]+M1[5]*M2[6];
        result[4] = M1[3]*M2[1]+M1[4]*M2[4]+M1[5]*M2[7];
        result[5] = M1[3]*M2[2]+M1[4]*M2[5]+M1[5]*M2[8];

        result[6] = M1[6]*M2[0]+M1[7]*M2[3]+M1[8]*M2[6];
        result[7] = M1[6]*M2[1]+M1[7]*M2[4]+M1[8]*M2[7];
        result[8] = M1[6]*M2[2]+M1[7]*M2[5]+M1[8]*M2[8];
    }


    inline void rotate_translate_v(vec3_& result, const vec3_ v, const mat3_ R, const vec3_ t){

        result[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2] + t[0];
        result[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2] + t[1];
        result[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2] + t[2];

    };


    inline dataType dotProduct3(const vec3_ v1,const vec3_ v2) {
        return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
    }

    inline void angle3(dataType& ang1,dataType& ang2,dataType& ang3,const vec3_ v1,const vec3_ v2) {

        ang1 = ((v1[0]*v2[0]+v1[1]*v2[1])/(sqrt(v1[0]*v1[0]+v1[1]*v1[1])*sqrt(v2[0]*v2[0]+v2[1]*v2[1])));
        ang2 = ((v1[0]*v2[0]+v1[2]*v2[2])/(sqrt(v1[0]*v1[0]+v1[2]*v1[2])*sqrt(v2[0]*v2[0]+v2[2]*v2[2])));
        ang3 = ((v1[1]*v2[1]+v1[2]*v2[2])/(sqrt(v1[1]*v1[1]+v1[2]*v1[2])*sqrt(v2[1]*v2[1]+v2[2]*v2[2])));

    }


    inline dataType vector3Magnitude(const vec3_ v) {
        return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    }

    inline dataType vector3Magnitude(const dataType& v0, const dataType& v1 ,const dataType& v2) {
        return sqrt(v0*v0+v1*v1+v2*v2);
    }

    inline void eigen_to_vector3(vec3_& w, vec3 v){
        for(int i{}; i < 3; ++i)
            w[i] = v[i];
    }

    inline mat3 matrix3_to_eigen(const mat3_ W){
        mat3 V;
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                V(i,j) = W[j+3*i];
            }
        }
        return V;
    }

    inline void eigen_to_matrix3(mat3_& W, mat3 V){
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                W[j+3*i] = V(i,j);
            }
        }
    }

    inline vec3 vector3_to_eigen(const vec3_ w){
        vec3 v;
        for(int i{}; i < 3; ++i) {
            v[i] = w[i];
        }
        return v;
    }
*/
    //inline void exp_lie(vec3_& delta_t, mat3_& delta_R, const vec3& v, const vec3& w) {

        /*dataType w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};

        dataType theta = sqrt(w0_2 + w1_2 + w2_2);
        dataType coef1{1}, coef2{0.5}, coef3{1/6};

        if (theta > 0.0001) {
            coef1 = (sin(theta)) / (theta);
            coef2 = (1 - cos(theta)) / (theta * theta);
            coef3 = (theta - sin(theta)) / (theta * theta * theta);
        }

        dataType term0, term1, term2;
        /////////////
        term0 = -(w2_2 + w1_2);
        delta_R[0] =  1 + coef2 * term0;
        delta_t[0] = (1 + coef3 * term0) * v[0];
        /////////////
        term0 = -(w2_2 + w0_2);
        delta_R[4] =  1 + coef2 * term0;
        delta_t[1] = (1 + coef3 * term0) * v[1];
        /////////////
        term0 = -(w1_2 + w0_2);
        delta_R[8] =  1 + coef2 * term0;
        delta_t[2] = (1 + coef3 * term0) * v[2];
        /////////////
        term0 = w[0] * w[1];
        term1 = coef1 * w[2];
        term2 = coef2 * term0;
        delta_R[1] = -term1 + term2;
        delta_R[3] =  term1 + term2;
        /////////////
        term1 = coef2 * w[2];
        term2 = coef3 * term0;
        delta_t[0] += (-term1 + term2) * v[1];
        delta_t[1] += ( term1 + term2) * v[0];
        /////////////
        term0 = w[0] * w[2];
        term1 = coef1 * w[1];
        term2 = coef2 * term0;
        delta_R[2] =  term1 + term2;
        delta_R[6] = -term1 + term2;
        /////////////
        term1 = coef2 * w[1];
        term2 = coef3 * term0;
        delta_t[0] +=  (term1 + term2) * v[2];
        delta_t[2] += (-term1 + term2) * v[0];
        /////////////
        term0 = w[1] * w[2];
        term1 = coef1 * w[0];
        term2 = coef2 * term0;
        delta_R[5] = -term1 + term2;
        delta_R[7] =  term1 + term2;
        /////////////
        term1 = coef2 * w[0];
        term2 = coef3 * term0;
        delta_t[1] += (-term1 + term2) * v[2];
        delta_t[2] += ( term1 + term2) * v[1];*/
        //cout << "exp_lie" << endl;
        //cout <<  delta_t[0]<< " "<<  delta_t[1] << " " << delta_t[2] << endl;
        //cout << "R "<< endl;
        //showMatrix3(delta_R);

        /*mat4 Texp{};
        Texp <<  0.0         ,-w(2) , w(1),v(0),
                w(2) ,     0.0    ,  -w(0),v(1),
                -w(1) , w(0) ,     0.0   ,v(2),
                0.0    ,     0.0    ,     0.0   ,    0.0;
        mat4 T = Texp.exp();
        eigen_to_vector3(delta_t, T.block<3,1>(0,3));
        eigen_to_matrix3(delta_R, T.block<3,3>(0,0));*/
        //cout <<  delta_t[0]<< " "<<  delta_t[1] << " " << delta_t[2] << endl;
        //cout << "R "<< endl;
        //showMatrix3(delta_R);
        //wf(".");
    //}

    //inline void log_lie(vec3& v, vec3& w, const vec3_ t, const mat3_ R) {

        /*dataType theta = acos((R[0]+R[4]+R[8]-1)/2);
        dataType factor = 0.5;

        if (abs(theta) > 0.00000000001) {
            factor = theta/(2*sin(theta));
        }

        w[0] = factor*(R[7]-R[5]);
        w[1] = factor*(R[2]-R[6]);
        w[2] = factor*(R[3]-R[1]);

        dataType w0_2{w[0] * w[0]}, w1_2{w[1] * w[1]}, w2_2{w[2] * w[2]};
        dataType term0, term1;

        factor = dataType(1/12);
        theta = sqrt(w0_2 + w1_2 + w2_2);
        if (abs(theta) > 0.00000000001) {
            factor = (1-((theta*cos(theta/2))/(2*sin(theta/2))))/(theta*theta);
        }

        /////////////
        v[0] = (1-factor*(w2_2+w1_2))*t[0];
        v[1] = (1-factor*(w2_2+w0_2))*t[1];
        v[2] = (1-factor*(w1_2+w0_2))*t[2];
        /////////////
        term0 = 0.5f*w[2];
        term1 = factor*(w[0]*w[1]);
        v[0] += ( term0+term1)*t[1];
        v[1] += (-term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[1];
        term1 = factor*(w[0]*w[2]);
        v[0] += (-term0+term1)*t[2];
        v[2] += ( term0+term1)*t[0];
        /////////////
        term0 = 0.5f*w[0];
        term1 = factor*(w[1]*w[2]);
        v[1] += ( term0+term1)*t[2];
        v[2] += (-term0+term1)*t[1];*/

        /*cout << "log lie" << endl;
        cout << v.transpose() << endl;
        cout << w.transpose() << endl;*/

        /*mat4 T{mat4::Identity()};
        T.block<3,3>(0,0) = matrix3_to_eigen(R);
        T.block<3,1>(0,3) = vector3_to_eigen(t);
        mat4 Texp = T.log();
        v = T.block<3,1>(0,3);
        w << T(2,1),T(0,2),T(1,0);*/

        /*cout << v.transpose() << endl;
        cout << w.transpose() << endl;
        wf(".");*/
    //}



    /*inline void mat4_float_to_Rt(const mat4_float& T ,dataType t[3], dataType R[9]){
        for(int iTras{}; iTras < 3; ++iTras){
            t[iTras] = (dataType)T(iTras,3);
        }
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                R[3*i + j] = (dataType)T(i,j);
            }
        }
    }



    inline mat4 Rt_to_mat4(vec3_ t, mat3_ R){
        mat4 T = mat4::Identity();
        for(int iTras{}; iTras < 3; ++iTras){
            T(iTras,3) = t[iTras];
        }
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                T(i,j) = R[3*i + j];
            }
        }
        return T;
    }

    inline void mat4_to_Rt(const mat4& T ,vec3_ t, mat3_ R){
        for(int iTras{}; iTras < 3; ++iTras){
            t[iTras] = (dataType)T(iTras,3);
        }
        for(int i{}; i < 3; ++i){
            for(int j{}; j < 3; ++j){
                R[3*i + j] = (dataType)T(i,j);
            }
        }
    }
    */

    inline dataType vectorMean(vecX& v){
        return v.mean();
    }

    inline dataType vectorMean(vector<dataType>& v){
        dataType meanValue{0.0};
        for(dataType& vi: v){
            meanValue += vi;
        }
        return meanValue/v.size();
    }

    inline dataType vectorStd(vecX& v){
        return sqrt((v.array()-v.mean()).square().mean());
    }

    inline dataType vectorMedian(vector<dataType> v){
        vector<dataType> w = v;
        std::sort (w.begin(), w.end());
        return w[int(w.size()/2)];
    }

    inline dataType vectorMax(vector<dataType> v){
        std::vector<dataType>::iterator result;
        result = std::max_element(v.begin(), v.end());
        return v[std::distance(v.begin(), result)];
    }

    inline dataType vectorMin(vector<dataType> v){
        std::vector<dataType>::iterator result;
        result = std::min_element(v.begin(), v.end());
        return v[std::distance(v.begin(), result)];
    }

    inline vecX vectorToEigen(vector<dataType> v){
        int numValues = v.size();
        vecX eigenVector = matX::Zero(numValues,1);
        for (int i{}; i < numValues; ++i){
            eigenVector(i) = v[i];
        }
        return eigenVector;
    }

    inline dataType vectorStd(vector<dataType>& v){
        vecX v_ = vectorToEigen(v);
        return vectorStd(v_);
    }

    inline dataType correlationCoeff(vecX v1, vecX v2){
        int n = v1.size();
        return ((n*(v1.array()*v2.array()).sum())-(v1.sum()*v2.sum()))/((sqrt(n*v1.array().square().sum()-pow(v1.sum(),2)))*(sqrt(n*v2.array().square().sum()-pow(v2.sum(),2))));
    }

    inline void linearRegression(vector<double>& x_, vector<double>& y_ , matX& mb){

        matX e = vecX::Zero(y_.size());
        vecX y = vecX::Zero(y_.size());
        vecX x = vecX::Zero(x_.size());
        for(int i{}; i < x_.size() ; ++i){
            y[i] = y_[i];
            x[i] = x_[i];
        }

        matX J;
        matX A;
        matX b;
        matX delta;
        J = matX::Ones(y.size(),2);
        J.col(0) = x;
        A = J.transpose()*J;
        //cout<< "A = "<< A<< endl;

        for(int it{}; it < 5; ++it){
            //cout<< "it= "<< it<< endl;
            e =(mb(0)*x.array() + mb(1)) -  y.array();
            b = -J.transpose()*e;
            delta = A.colPivHouseholderQr().solve(b);
            mb += delta;
        }
        cout<< "m = "<< mb(0) << " b = " << mb(1) << endl;
        cout << x.transpose() << endl;
        cout << y.transpose() << endl;

        wf(".");
    }

    inline void set_openGlMatrix4(pangolin::OpenGlMatrix& T, const mat3& R , const vec3& t){
        T.SetIdentity();
        T(0,0) = R(0,0);
        T(0,1) = R(0,1);
        T(0,2) = R(0,2);
        T(0,3) = t(0);
        T(1,0) = R(1,0);
        T(1,1) = R(1,1);
        T(1,2) = R(1,2);
        T(1,3) = t(1);
        T(2,0) = R(2,0);
        T(2,1) = R(2,1);
        T(2,2) = R(2,2);
        T(2,3) = t(2);
    }

    inline dataType evalSpline( const dataType& t_ , const int& numCoeffs, const vecX& coeffs){
        dataType xp = 0.0;
        for(int iCoeff{0}; iCoeff < numCoeffs; ++ iCoeff)
            xp += coeffs(iCoeff) *  pow(t_,iCoeff);
        return xp;
    }

    inline dataType determinationCoefficient(vector<dataType>& x, vector<dataType>& t_ , int numCoeffs, vecX& coeffs, dataType& speedError){
        dataType SStot = 0.0;
        dataType xmean = vectorMean(x);
        for(dataType& xi: x)
            SStot += pow(xmean-xi,2);

        dataType SSres = 0.0;
        dataType fi;
        int index{0};

        for(dataType& xi: x){
            fi = evalSpline( t_[index] ,numCoeffs,coeffs);
            SSres += pow(xi - fi,2);
            ++index;
        }
        speedError = sqrt(SSres/x.size());
        return (1.0 - (SSres/SStot));
    }


    inline dataType integralSpline( const dataType& ti, const dataType& tf , const int& numCoeffs, const vecX& coeffs){
        matX J = matX::Ones(1, numCoeffs);
        for(int j{0}; j < numCoeffs; ++j)
            J(0,j) = (1.0/(j + 1.0))*pow(ti,j + 1);


        dataType xi = 0.0;
        for(int iCoeff{0}; iCoeff < numCoeffs; ++ iCoeff)
            xi += (1.0/(iCoeff + 1.0)) * coeffs(iCoeff) *  pow(ti, iCoeff + 1);

        dataType xf = 0.0;
        for(int iCoeff{0}; iCoeff < numCoeffs; ++ iCoeff)
            xf += (1.0/(iCoeff + 1.0)) * coeffs(iCoeff) *  pow(tf, iCoeff + 1);

        return (xf-xi);
    }
    }
#endif //IDNAV_MATH_FUNCTIONS_H
