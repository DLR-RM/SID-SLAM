//
// Created by font_al on 08/10/21.
//

#ifndef IDNAV_PT_H
#define IDNAV_PT_H

#include "../utils/math_functions.h"

namespace IDNav{

    // This is a base class to represent a point:
    //  - It has a reference keyframe --> refKeyframe
    //  - Points belong to a 3D plane and are projected accordingly.
    //  - Derived classes allow changing the number of pixels --> ptSize
    //  - Derived classes allow considering points as photometric patches or classical features.

    class IdNavPoint {
    public:

        const size_t ptSize;
        const int ptArea;
        void *refKeyframe{};
        size_t idKey{};
        bool visible{true};

        //Geometric members
        dataType uRef{}, vRef{};  // Image coordinates in the reference keyframe
        shared_ptr<dataType[]> xnRef{new dataType[1]{}}, ynRef{new dataType[1]{}}; // Normalized coordinates in the projection frame
        dataType lambdaRef{0.0};    // Inverse z of the central pixel in the reference keyframe
        dataType lambdaRefCov{1.0}; // Inverse z covariance in the reference keyframe
        shared_ptr<dataType[]> X{}, Y{}, Z{}; // World-cooordinate-frame coordinates
        shared_ptr<dataType[]> xn{}, yn{};    // Normalized coordinates in the projection frame
        shared_ptr<dataType[]> lambda{};      // Inverse z in the projection frame
        shared_ptr<dataType[]> u{}, v{};      // Image coordinates in the projection frame

        // Plane parameters
        vec3 planeParameters{vec3::Zero()};

        // Covariance members
        mat32 Jref{mat32::Zero()};

        explicit IdNavPoint(size_t ptSize, int ptArea) : ptSize(ptSize) , ptArea(ptArea){};

        void XYZ_to_xynlambda(vec3& tcw, mat3& Rcw){
            dataType xc,yc;
            for(int iPatch{}; iPatch < ptSize; ++iPatch){
                xc = X[iPatch]*Rcw(0,0) + Y[iPatch]*Rcw(0,1) + Z[iPatch]*Rcw(0,2) + tcw(0,0);
                yc = X[iPatch]*Rcw(1,0) + Y[iPatch]*Rcw(1,1) + Z[iPatch]*Rcw(1,2) + tcw(1,0);
                lambda[iPatch] = 1.0/(X[iPatch]*Rcw(2,0) + Y[iPatch]*Rcw(2,1) + Z[iPatch]*Rcw(2,2) + tcw(2,0));
                //Coordinate normalization
                xn[iPatch] = xc*lambda[iPatch];
                yn[iPatch] = yc*lambda[iPatch];
            }
        }

        void xynlambda_to_XYZ(vec3& twc, mat3& Rwc, dataType& lambdaRef_){
            dataType xc_ref{}, yc_ref{}, zc_ref{};
            dataType planeParam2 = (1.0/lambdaRef_)*(1.0-planeParameters(0)*xnRef[0]-planeParameters(1)*ynRef[0]);
            for (int iPatch = 0; iPatch < ptSize; ++iPatch) {
                //Depth estimation
                zc_ref = planeParam2/(1.0-xnRef[iPatch]*planeParameters(0)-ynRef[iPatch]*planeParameters(1));
                //Depth projection
                xc_ref = xnRef[iPatch] * zc_ref;
                yc_ref = ynRef[iPatch] * zc_ref;
                // 3D projection
                X[iPatch] = xc_ref * Rwc(0,0) + yc_ref * Rwc(0,1) + zc_ref * Rwc(0,2) + twc(0,0);
                Y[iPatch] = xc_ref * Rwc(1,0) + yc_ref * Rwc(1,1) + zc_ref * Rwc(1,2) + twc(1,0);
                Z[iPatch] = xc_ref * Rwc(2,0) + yc_ref * Rwc(2,1) + zc_ref * Rwc(2,2) + twc(2,0);
            }
        }

        void project(dataType& u_, dataType& v_, vec3& t_,  mat3& R_,
                     const dataType& fx_ , const dataType& fy_ , const dataType& cx_ ,const dataType& cy_) const {
            dataType xc_ref{}, yc_ref{}, zc_ref{};
            zc_ref = planeParameters(2)/(1.0-xnRef[0]*planeParameters(0)-ynRef[0]*planeParameters(1));
            xc_ref = xnRef[0] * zc_ref;
            yc_ref = ynRef[0] * zc_ref;
            dataType xc_tmp = xc_ref * R_(0,0) + yc_ref * R_(0,1) + zc_ref * R_(0,2) + t_(0,0);
            dataType yc_tmp = xc_ref * R_(1,0) + yc_ref * R_(1,1) + zc_ref * R_(1,2) + t_(1,0);
            dataType zc_tmp = xc_ref * R_(2,0) + yc_ref * R_(2,1) + zc_ref * R_(2,2) + t_(2,0);
            xc_tmp /= zc_tmp;
            yc_tmp /= zc_tmp;
            u_ = fx_ * xc_tmp + cx_;
            v_ = fy_ * yc_tmp + cy_;
        }

        void setLambdaRef(dataType lambdaRef_) {
            lambdaRef = lambdaRef_;
            lambdaRefCov = 1.0/100000.0;//100000.0;
            planeParameters(0) = 0.0;
            planeParameters(1) = 0.0;
            planeParameters(2) = 1.0/lambdaRef;
        }
        void updateLambdaRef(dataType& lambdaRef_, dataType& lambdaRefCov_){
            lambdaRef = lambdaRef_;
            Jref /= planeParameters(2);
            planeParameters(2) = (1.0/lambdaRef)*(1.0-planeParameters(0)*xnRef[0]-planeParameters(1)*ynRef[0]);
            Jref *= planeParameters(2);
            lambdaRefCov = lambdaRefCov_;
        }
        void setPlaneEstimation(const dataType& alpha, const dataType& beta,const dataType& fx, const dataType& fy){
            planeParameters(0) = alpha;
            planeParameters(1) = beta;
            planeParameters(2) = (1.0-alpha*xnRef[0]-beta*ynRef[0])/lambdaRef;
            setJref(fx,fy);
        }

        void setJref(const dataType& fx, const dataType& fy){
            dataType inv_fx{1.0/fx},inv_fy{1.0/fy};
            Jref << inv_fx*(1.0-planeParameters(1)*ynRef[0])  ,    inv_fy*planeParameters(1)*xnRef[0],
                    inv_fx*planeParameters(0)*ynRef[0]        , inv_fy*(1.0 - planeParameters(0)*xnRef[0]),
                    inv_fx*planeParameters(0)                 ,       inv_fy*planeParameters(1);
            double k = 1.0 - planeParameters(0)*xnRef[0] - planeParameters(1)*ynRef[0];
            Jref *= planeParameters(2)/(k*k);
        }

        virtual void setProjectionPointers();

        void duv_dPoseRef(mat26& duv_dPoseRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const size_t& iPatch);

        void duv_dxRef(mat26& duv_dPoseRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const int& iPatch);
        void duv_dx(mat26& duv_dx , const vec3& du_dxyzc, const vec3& dv_dxyzc, const int& iPatch);

        void duv_dlambdaRef(mat21& duv_dlambdaRef , const vec3& du_dxyzcRef, const vec3& dv_dxyzcRef, const size_t& iPatch);

    };

    typedef shared_ptr<IdNavPoint> typeIdNavPoint;
    typedef vector<shared_ptr<IdNavPoint>> vecIdnavPoint;

    // This class represents a photometric patch of size PATCH_SIZE.
    // It is projected into PYR_LEVELS_MAX levels of a resolution pyramid.
    class Pt : public IdNavPoint {
    public:

        //Photometric members
        dataType* refPhScalarExp{}; // Pointer to the exponential photometric parameter of the reference keyframe.
        dataType* refPhBias{};      // Pointer to the bias photometric parameter of the reference keyframe.
        dataType* imgGradient{}; // Pointer to the photometric noise parameter of the reference keyframe.
        dataType I_ref[PATCH_SIZE][PYR_LEVELS_MAX]{};       // Pixel intensity in the reference keyframe
        dataType G_ref{},Gu_ref{},Gv_ref{}; // Pixel gradients in the reference keyframe
        vecX dirRef{};
        shared_ptr<dataType[]> I{};       // Pixel intensity in the projection frame
        shared_ptr<dataType[]> Gu{},Gv{}; // Pixel gradients in the projection frame
        dataType photoStd{1.0};
        dataType photoStd0{1.0};

        Pt() : IdNavPoint(PATCH_SIZE , PATCH_SIZE) {};
        void setProjectionPointers() override;
        void estimatePerspectiveDeformation(dataType& def2_, mat2& F_, const dataType& fx_, const dataType& fy_, const mat3& R_) const;
        dataType estimatePhotoStd(const dataType& fx_, const dataType& fy_, const mat3& R_, const CovCalibration* photoCal_) const;
    };

    typedef shared_ptr<Pt>typePt;
    typedef vector<shared_ptr<Pt>> vecPt;

    inline typePt createPt(const dataType& uRef_, const dataType& vRef_, const dataType& depth_){
        typePt pt = make_shared<Pt>();
        pt->uRef = uRef_;
        pt->vRef = vRef_;
        pt->setLambdaRef(1.0/depth_) ;
        return pt;
    }

    inline typePt hardCopyPt(typePt& pt_){
        std::mutex mMutex;
        unique_lock<mutex> lock(mMutex);
        typePt ptInput = pt_;

        typePt pt = make_shared<Pt>();
        pt->refKeyframe = ptInput->refKeyframe;
        pt->idKey = pt_->idKey;
        pt->visible = ptInput->visible;
        pt->uRef = ptInput->uRef;
        pt->vRef = ptInput->vRef;
        pt->lambdaRef = ptInput->lambdaRef;
        pt->lambdaRefCov = ptInput->lambdaRefCov;
        pt->planeParameters = pt_->planeParameters;
        pt->Jref = pt_->Jref;
        pt->refPhScalarExp = ptInput->refPhScalarExp;
        pt->refPhBias = ptInput->refPhBias;
        pt->imgGradient = ptInput->imgGradient;
        pt->G_ref = ptInput->G_ref;
        pt->Gu_ref = ptInput->Gu_ref;
        pt->Gv_ref = ptInput->Gv_ref;
        pt->dirRef = ptInput->dirRef;
        pt->photoStd = ptInput->photoStd;
        pt->photoStd0 = ptInput->photoStd0;

        for(int iPyr{0}; iPyr < PYR_LEVELS_MAX; ++iPyr){
            for (int iMask{}; iMask < pt->ptSize; ++iMask) pt->I_ref[iMask][iPyr] = ptInput->I_ref[iMask][iPyr];
        }
        pt->setProjectionPointers();
        return pt;
    }

    // This class represents a feature-based point with a descritor of size DESCRIPTOR_SIZE
    class Ft : public IdNavPoint {
        public:
            size_t id{0};
            cv::KeyPoint keyPtRef{};
            cv::KeyPoint* matchedKeypt{};
            size_t matchedKeyPtIndex{};
            cv::Mat descriptor{Mat::zeros(1, DESCRIPTOR_SIZE, CV_8UC1)};
            mat2 geoInf{mat2::Identity()};
            mat2 geoInf0{mat2::Identity()};
            dataType minSize{4.8};
            dataType x0{},y0{};
            bool reUsedFeature{false};
            size_t numObservations{0};

            Ft() : IdNavPoint(1,15) {};
            mat2 estimateGeoInf(const dataType& fx, const dataType& fy, const mat3& R, const CovCalibration* geoCal, const KeyPoint& matchedKeypt_);
            bool estimateProjectionDeformation2D(mat2& Cleft, const dataType& fx, const dataType& fy, const mat3& R);
            void incrementNumObservations(){
                ++numObservations;
                if(numObservations > 100) numObservations  = 100;
            }
    };

    typedef shared_ptr<Ft>typeFt;
    typedef vector<shared_ptr<Ft>> vecFt;

    inline typeFt createFt(const dataType& uRef_, const dataType& vRef_, const dataType& depth_, const KeyPoint& keyPtRef_){
        typeFt ft = make_shared<Ft>();
        ft->uRef = uRef_;
        ft->vRef = vRef_;
        ft->setLambdaRef(1.0/depth_) ;
        ft->keyPtRef = keyPtRef_;
        return ft;
    }

    inline typeFt hardCopyFt(typeFt& ft_){
        std::mutex mMutex;
        unique_lock<mutex> lock(mMutex);

        typeFt ft = make_shared<Ft>();
        ft->id = ft_->id;
        ft->idKey = ft_->idKey;
        ft->reUsedFeature = ft_->reUsedFeature;
        ft->numObservations = ft_->numObservations;
        ft->refKeyframe = ft_->refKeyframe;
        ft->visible = ft_->visible;
        ft->uRef = ft_->uRef;
        ft->vRef = ft_->vRef;
        ft->lambdaRef = ft_->lambdaRef;
        ft->lambdaRefCov = ft_->lambdaRefCov;
        ft->planeParameters = ft_->planeParameters;
        ft->Jref = ft_->Jref;
        ft->keyPtRef = ft_->keyPtRef;
        ft->matchedKeypt = ft_->matchedKeypt;
        ft->matchedKeyPtIndex = ft_->matchedKeyPtIndex;
        ft->descriptor = ft_->descriptor.clone();
        ft->geoInf = ft_->geoInf;
        ft->geoInf0 = ft_->geoInf0;
        ft->minSize = ft_->minSize;
        ft->x0 = ft_->x0;
        ft->y0 = ft_->y0;

        ft->setProjectionPointers();
        return ft;
    }
}
#endif //IDNAV_PT_H
