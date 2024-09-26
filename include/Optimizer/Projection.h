
#ifndef IDNAV_PROJECTION_H
#define IDNAV_PROJECTION_H

#include "../Frame.h"
#include "../PointSelector.h"

namespace IDNav {

    class Projection_hgp {
    public:
        bool optRefPose{true};
        bool optProjPose{true};
        bool optDepth{false};

        typePt pt{};
        typeFrame refKey{};
        typeFrame projKey{};
        size_t refKeyId{};
        size_t projKeyId{};

        dataType photoStd{1.0};
        mat3 *Rrel;

        dataType dI_dlambdaRef{0.0};
        dataType lambdaRefObs{0.0};
        bool thereIsDepthObs{false};
    public:

        Projection_hgp(typePt &pt_, typeFrame refKey_, typeFrame projKey_, mat3 *Rrel_) :
            pt{pt_}, refKey{refKey_}, projKey{projKey_}, Rrel{Rrel_} {
            refKeyId = refKey->keyId;
            projKeyId = projKey->keyId;
        }

        void computeResidualCov() {
            dataType fx{projKey->cam->getFocalLengthX()};
            dataType fy{projKey->cam->getFocalLengthY()};
            const CovCalibration *photoCal = projKey->cam->get_photoDef2_param();
            photoStd = pt->estimatePhotoStd(fx, fy, *Rrel, photoCal);
        }

        void getDepthObservation(){
            float depthOutput;
            if(PointSelector::thereIsDepthKinect(depthOutput,round(pt->u[0]),round(pt->v[0]),projKey->depthImg,MAX_DEPTH_VALUE)){
                typePt pt_tmp = createPt(pt->u[0], pt->v[0], depthOutput);
                pt_tmp->setProjectionPointers();
                projKey->uv_2_XYZ(pt_tmp);
                refKey->XYZ_2_uv(pt_tmp);
                lambdaRefObs = pt_tmp->lambda[0];
                dataType lambdaRefStd = 3.0*sqrt(pt->lambdaRefCov);
                if(abs(lambdaRefObs - pt->lambdaRef) < lambdaRefStd){
                    thereIsDepthObs = true;
                }
            }

        }
    };

    typedef shared_ptr<Projection_hgp> typeProj_hgp;
    typedef vector<shared_ptr<Projection_hgp>> vecProj_hgp;

    class Projection_ft {
    public:

        bool optRefPose{true};
        bool optProjPose{true};
        bool optDepth{true};

        typePt pt{};
        typeFt ft{};
        typeFrame refKey{};
        typeFrame projKey{};
        size_t refKeyId{};
        size_t projKeyId{};
        KeyPoint obs{};
        mat2 geoInfStd{mat2::Identity()};
        mat3 *Rrel;

        dataType duv_dlambdaRef{0.0};
        dataType lambdaRefObs{0.0};
        bool thereIsDepthObs{false};
    public:

        Projection_ft(typeFt ft_,
                      typeFrame refKey_, typeFrame projKey_,
                      mat3 *Rrel_, KeyPoint &obs_) :
                ft{ft_},
                refKey{refKey_}, projKey{projKey_},
                Rrel{Rrel_}, obs{obs_} {
            refKeyId = refKey->keyId;
            projKeyId = projKey->keyId;
        };

        Projection_ft(typePt pt_,
                      typeFrame refKey_, typeFrame projKey_,
                      mat3 *Rrel_, KeyPoint &obs_) :
                pt{pt_},
                refKey{refKey_}, projKey{projKey_},
                Rrel{Rrel_}, obs{obs_} {
            refKeyId = refKey->keyId;
            projKeyId = projKey->keyId;
        };


        void computeResidualCov() {
            dataType fx{projKey->cam->getFocalLengthX()};
            dataType fy{projKey->cam->getFocalLengthY()};
            const CovCalibration *geoCal = projKey->cam->get_geoDef2_param();
            geoInfStd = ft->estimateGeoInf(fx, fy, *Rrel, geoCal, obs);
        }

        void getDepthObservation(){
            float depthOutput;
            if(PointSelector::thereIsDepthKinect(depthOutput,round(ft->u[0]),round(ft->v[0]),projKey->depthImg,MAX_DEPTH_VALUE)){
                typePt pt_tmp = createPt(ft->u[0], ft->v[0], depthOutput);
                pt_tmp->setProjectionPointers();
                projKey->uv_2_XYZ(pt_tmp);
                refKey->XYZ_2_uv(pt_tmp);
                lambdaRefObs = pt_tmp->lambda[0];
                dataType lambdaRefStd = 3.0*sqrt(ft->lambdaRefCov);
                if(abs(lambdaRefObs - ft->lambdaRef) < lambdaRefStd){
                    thereIsDepthObs = true;
                }
            }

        }
    };
    typedef shared_ptr<Projection_ft> typeProj_ft;
    typedef vector<shared_ptr<Projection_ft>> vecProj_ft;

}
#endif //IDNAV_PROJECTION_H
