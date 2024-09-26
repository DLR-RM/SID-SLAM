
#ifndef IDNAV_CERESTRACKER_H
#define IDNAV_CERESTRACKER_H

#include "../Frame.h"

namespace IDNav{
    class CeresTracker {

    public:
        void estimatePoseCoarse(vecPt& hgpForTracking_, vecFt& featuresForTracking_, typeFrame& trackedFrame_, dataType poseGuess_ [6]) const;
        void estimatePose(vecPt& hgpForTracking_, vecFt& featuresForTracking_, typeFrame& trackedFrame_, dataType updatePose_ [6]) const;
        void estimatePhotoParameters(vecPt& hgpForTracking_, typeFrame& trackedFrame_, dataType& photoScalar_,dataType& photoBias_) const ;

    public:

        // Optimization performance
        ceres::Solver::Options solverRefinedPoseOptions{};
        ceres::Solver::Options solverCoarsePoseOptions{};
        ceres::Solver::Options solverPhotoParametersOptions{};

        // Outlier rejection
        dataType tstudent2Th_patch{tstudent2(PATCH_SIZE,95.0)};
        dataType tstudentTh_patch{sqrt(tstudent2Th_patch)};
        dataType tstudent2Th_reproj{tstudent2(2,95.0)};
        dataType tstudentTh_reproj{sqrt(tstudent2Th_reproj)};
        dataType tstudent2Th_patch_photoParameters{tstudent2(PATCH_SIZE,70.0)};
        dataType tstudentTh_patch_photoParameters{sqrt(tstudent2Th_patch_photoParameters)};

        void set_tstudent2(const dataType& p_, const int& numDofPhoto_, const int& numDofReproj_){
            tstudent2Th_patch = tstudent2(numDofPhoto_,p_);
            tstudentTh_patch = sqrt(tstudent2Th_patch);
            tstudent2Th_reproj = tstudent2(numDofReproj_,p_);
            tstudentTh_reproj  = sqrt(tstudent2Th_reproj);
        }

    public:
        CeresTracker(){

            const int numThreads{1};
            solverRefinedPoseOptions.linear_solver_type = ceres::DENSE_QR;
            solverRefinedPoseOptions.minimizer_type = ceres::TRUST_REGION;
            solverRefinedPoseOptions.minimizer_progress_to_stdout = false;
            solverRefinedPoseOptions.ceres::Solver::Options::update_state_every_iteration = true;
            solverRefinedPoseOptions.use_inner_iterations = false;
            solverRefinedPoseOptions.num_threads = numThreads;
            solverRefinedPoseOptions.use_nonmonotonic_steps = true;

            solverCoarsePoseOptions.linear_solver_type = ceres::DENSE_QR;
            solverCoarsePoseOptions.minimizer_type = ceres::TRUST_REGION;
            solverCoarsePoseOptions.minimizer_progress_to_stdout = false;
            solverCoarsePoseOptions.ceres::Solver::Options::update_state_every_iteration = true;
            solverCoarsePoseOptions.use_inner_iterations = false;
            solverCoarsePoseOptions.num_threads = numThreads;
            solverCoarsePoseOptions.use_nonmonotonic_steps = true;

            solverPhotoParametersOptions.linear_solver_type = ceres::DENSE_QR;
            solverPhotoParametersOptions.minimizer_type = ceres::TRUST_REGION;
            solverPhotoParametersOptions.minimizer_progress_to_stdout = false;
            solverPhotoParametersOptions.ceres::Solver::Options::update_state_every_iteration = true;
            solverPhotoParametersOptions.use_inner_iterations = false;
            solverPhotoParametersOptions.num_threads = numThreads;
            solverPhotoParametersOptions.use_nonmonotonic_steps = true;

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Evaluation callbacks
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        class evaluationCallbackPose : public ceres::EvaluationCallback {
        public:
            explicit evaluationCallbackPose(typeFrame& trackedFrame, Pose* pose0,
                                            dataType* incPose):
                trackedFrame(trackedFrame), pose0(pose0),
                incPose(incPose){}
            virtual ~evaluationCallbackPose() {}
            virtual void PrepareForEvaluation(bool evaluate_jacobians,bool new_evaluation_point){
                if(new_evaluation_point)
                    updatePose(trackedFrame, incPose ,*pose0);
            };
            typeFrame trackedFrame;
            Pose* pose0;
            dataType* incPose;
        };

        class evaluationCallbackPhotoParameters : public ceres::EvaluationCallback {
        public:
            explicit evaluationCallbackPhotoParameters(typeFrame& trackedFrame, dataType* incPhotoScalar,  dataType* incPhotoBias ):
                    trackedFrame(trackedFrame), incPhotoScalar(incPhotoScalar), incPhotoBias(incPhotoBias){}
            virtual ~evaluationCallbackPhotoParameters() {}
            virtual void PrepareForEvaluation(bool evaluate_jacobians,bool new_evaluation_point){
                if(new_evaluation_point)
                    trackedFrame->set_photo_parameters(*incPhotoScalar, *incPhotoBias);
            };
            typeFrame trackedFrame;
            dataType* incPhotoScalar{};
            dataType* incPhotoBias{};
        };


        static void updatePose(const typeFrame& trackedFrame_, dataType* incPose, const Pose& pose0_){
            vec3 v_{incPose[0], incPose[1], incPose[2]},w_{incPose[3], incPose[4], incPose[5]};
            trackedFrame_->pose.copyPoseFrom(pose0_);
            vec3 delta_t{};
            mat3 delta_R{};
            exp_lie(delta_t, delta_R,v_,w_);
            trackedFrame_->pose.update_T_cw_left(delta_t,delta_R);
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Jaobians and Residuals
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        static void photoResidual(dataType* residuals_, const typeFrame& trackedFrame_,
                                         const typePt& pt_, const int& iPatch_){
            residuals_[iPatch_] = (
                    ((pt_->I[iPatch_] - trackedFrame_->phBias)*(trackedFrame_->phScalarExp)) -
                    ((pt_->I_ref[iPatch_][trackedFrame_->cam->getPyrLevel()] - *(pt_->refPhBias))*(*(pt_->refPhScalarExp)))
                                  )/pt_->photoStd;
        }

        static void photoJacobians(dataType **jacobians,
                                   mat16 dI_dx, dataType& photoConstant,
                                   const typeFrame& trackedFrame_,
                                   const typePt& pt_, const int& iPatch_){

            dI_dx *= photoConstant;
            for(int iDof{0}; iDof < 6; ++iDof){
                jacobians[0][iDof + 6 * iPatch_] = dI_dx(iDof);
            }
        }


        static void photoJacobians( dataType **jacobians, dataType& photoConstant,
                                    const typeFrame& trackedFrame_,
                                    const typePt& pt_, const int& iPatch_){

            jacobians[0][0 + 1 * iPatch_] = ((pt_->I[iPatch_] - trackedFrame_->phBias)/pt_->photoStd) * dexposureFunction_dx(trackedFrame_->phScalar);
            jacobians[1][0 + 1 * iPatch_] = -photoConstant;

        }

        static void poseJacobianGeometric(dataType **jacobians,
                                          const typeFrame& trackedFrame_, const typeFt& ft_){

            vec3 du_dxyzc{}, dv_dxyzc{};
            trackedFrame_->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft_);
            mat26 duv_dx{};
            ft_->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);
            mat26 duv_dx_inf = ft_->geoInf * duv_dx;

            for(int iDof{0}; iDof < 6; ++iDof){
                jacobians[0][iDof + 6 * 0] = duv_dx_inf(0,iDof);
                jacobians[0][iDof + 6 * 1] = duv_dx_inf(1,iDof);
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Photometric Reprojection error functions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        class PhotometricReprojectionError : public ceres::SizedCostFunction<PATCH_SIZE,6> {
        public:

            PhotometricReprojectionError(typePt& pt, typeFrame& trackedFrame): pt(pt), trackedFrame(trackedFrame) {}

            virtual ~PhotometricReprojectionError() {}

            virtual bool Evaluate(dataType const *const *pose,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Project point into the frame
                trackedFrame->XYZ_2_uv(pt);
                for(int iPatch{}; iPatch < pt->ptSize; ++iPatch) {
                    trackedFrame->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch],
                                                                 pt->u[iPatch], pt->v[iPatch]);
                    photoResidual(residuals, trackedFrame, pt, iPatch);
                }

                // Compute analytic jacobians
                if(jacobians){
                    vec3 du_dxyzc{}, dv_dxyzc{};
                    trackedFrame->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

                    mat26 duv_dx{};
                    pt->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);

                    mat12 dI_duv{};
                    mat16 dI_dx{};
                    dataType photoConstant = trackedFrame->phScalarExp/pt->photoStd;
                    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch){
                        dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                        dI_dx = dI_duv * duv_dx;
                        photoJacobians(jacobians, dI_dx, photoConstant, trackedFrame, pt, iPatch);
                    }
                }

                return true;
            }

            typePt pt;
            typeFrame trackedFrame;
        };

        class PhotometricReprojectionErrorCoarse : public ceres::SizedCostFunction<PATCH_SIZE, 6> {
        public:

            PhotometricReprojectionErrorCoarse(typePt& pt, typeFrame& trackedFrame): pt(pt), trackedFrame(trackedFrame) {}

            virtual ~PhotometricReprojectionErrorCoarse() {}

            virtual bool Evaluate(dataType const *const *pose,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Project point into the frame
                trackedFrame->XYZ_2_uv(pt);
                for(int iPatch{}; iPatch < pt->ptSize; ++iPatch) {
                    trackedFrame->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch],
                                                                 pt->u[iPatch], pt->v[iPatch]);
                    photoResidual(residuals, trackedFrame, pt, iPatch);
                }

                // Compute analytic jacobians
                if(jacobians){
                    vec3 du_dxyzc{}, dv_dxyzc{};
                    trackedFrame->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

                    mat26 duv_dx{};
                    pt->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);

                    mat12 dI_duv{};
                    mat16 dI_dx{};

                    dataType photoConstant = trackedFrame->phScalarExp/pt->photoStd;
                    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch){
                        dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                        dI_dx = dI_duv * duv_dx;
                        photoJacobians(jacobians, dI_dx, photoConstant, trackedFrame, pt, iPatch);
                    }
                }
                return true;
            }

            typePt pt;
            typeFrame trackedFrame;
        };

        class PhotometricParametersReprojectionError : public ceres::SizedCostFunction<PATCH_SIZE,1,1> {
        public:

            PhotometricParametersReprojectionError(typePt& pt, typeFrame& trackedFrame): pt(pt), trackedFrame(trackedFrame) {}

            virtual ~PhotometricParametersReprojectionError() {}

            virtual bool Evaluate(dataType const *const *pose,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                for(int iPatch{}; iPatch < pt->ptSize; ++iPatch)
                    photoResidual(residuals, trackedFrame, pt, iPatch);

                // Compute photometric residuals and analytic jacobians
                if(jacobians){
                    dataType photoConstant = trackedFrame->phScalarExp/pt->photoStd;
                    for(int iPatch{}; iPatch < pt->ptSize; ++iPatch)
                        photoJacobians(jacobians, photoConstant, trackedFrame, pt, iPatch);
                }
                return true;
            }

            typePt pt;
            typeFrame trackedFrame;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Geometric Reprojection error functions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        class GeometricReprojectionError : public ceres::SizedCostFunction<2,6> {

        public:

            GeometricReprojectionError(typeFt& ft, typeFrame& trackedFrame): ft(ft), trackedFrame(trackedFrame){}

        virtual ~GeometricReprojectionError() {}

        virtual bool Evaluate(dataType const *const *pose,
                              dataType *residuals,
                              dataType **jacobians) const {

            // Project point into the frame
            trackedFrame->pose.XYZ_2_xynlambda(ft);
            trackedFrame->cam->xyn_2_uv_iPyr(ft,0);

            // Compute photometric residuals and analytic jacobians
            dataType eu = (ft->u[0] - ft->matchedKeypt->pt.x);
            dataType ev = (ft->v[0] - ft->matchedKeypt->pt.y);
            residuals[0] = eu*ft->geoInf(0,0) + ev*ft->geoInf(0,1);
            residuals[1] = eu*ft->geoInf(1,0) + ev*ft->geoInf(1,1);

            if(jacobians){
                poseJacobianGeometric(jacobians, trackedFrame,ft);
            }

            return true;
        }

        typeFt ft;
        typeFrame trackedFrame;
    };
};
}


#endif //IDNAV_CERESTRACKER_H
