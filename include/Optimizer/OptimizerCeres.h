
#ifndef IDNAV_OPTIMIZERCERES_H
#define IDNAV_OPTIMIZERCERES_H


#include "Projection.h"

namespace IDNav {

    class OptimizerCeres {

    public: // Public members

        std::string  spaces = "                                                                                              ";
        IDNav::Profiler prepareOptProfiler{"profiler" ,
                                           spaces ,
                                           "OptimizerCeres","prepareOptimization", BLUE_COUT};
        IDNav::Profiler optimizeProfiler{"profiler" , spaces ,"OptimizerCeres","optimize",BLUE_COUT};

    private: // Private members

        //
        string BA_type{"local"};

        // Optimization flags
        bool optimizationPrepared{false};
        bool optimizationConverged{false};
        bool optimizationConverged_pose{false};
        bool optimizationConverged_depth_photo{false};
        bool optimizationConverged_depth_reproj{false};
        bool beginOptimization{false};

        // Operation mode
        const size_t minNumKeyframesForOptimization{4};
        string optimizationMode{};
        bool optimizeDepths{false};

        // Sliding window
        unordered_map<size_t, typeFrame> keyframesToOptimize{};
        unordered_map<size_t, typeFrame> windowKeyframes{};
        unordered_map<size_t, typeFrame>* keyframes{};
        map<std::pair<typeFrame, typeFrame>, mat3> Rrel{}; // < hostKey, projKey >

        // Projection Variables
        unordered_map<typePt, vecProj_hgp> hgpObservations{};
        vecProj_hgp hgpProjections{};
        vecProj_hgp hgpProjections_map{};
        size_t numHgpProjections{};
        size_t maxGeoNumHgpProjections{};
        size_t maxNumHgpProjections{};
        size_t numHgpMapProjections{};

        unordered_map<typeFt, vecProj_ft> featureObservations{};
        vecProj_ft ftProjections{};
        vecProj_ft ftProjections_map{};
        size_t numFtProjections{};
        size_t maxGeoNumFtProjections{};
        size_t maxNumFtProjections{};
        size_t numFtMapProjections{};

        size_t numProjections{};
        size_t maxGeoNumProjections{};
        size_t maxNumProjections{};
        size_t numMapProjections{};

        // Fixed keyframes
        unordered_map<size_t, bool> fixedKeyframes{};
        const size_t numFixedKeyframes{2};
        const size_t minNumMapProjections{1000};

        // Observations
        const size_t minNumObservations{1};
        size_t numHgpToOptimize{};
        size_t numFtToOptimize{};

        // Depth Variables
        unordered_map<typePt, dataType> deltaLambdaHgp{};
        unordered_map<typeFt, dataType> deltaLambdaFeatures{};

        // Pose Variables
        unordered_map<size_t, dataType*> deltaPoses{};
        unordered_map<size_t, vec3> tcw{};
        unordered_map<size_t, mat3> Rcw{};
        unordered_map<size_t, vec3> twc{};
        unordered_map<size_t, mat3> Rwc{};

        // Photo Variables
        unordered_map<size_t, dataType> deltaPhotoScalar{};
        unordered_map<size_t, dataType> deltaPhotoExpScalar{};
        unordered_map<size_t, dataType> deltaPhotoBias{};

        // Problem Options
        shared_ptr<ceres::Problem::Options> problemOptions_pose{};
        shared_ptr<ceres::Problem::Options> problemOptions_depth_photo{};
        shared_ptr<ceres::Problem::Options> problemOptions_depth_reproj{};

        // Optimization Problem
        shared_ptr<ceres::Problem> problem_pose{};
        shared_ptr<ceres::Problem> problem_depth_photo{};
        shared_ptr<ceres::Problem> problem_depth_reproj{};

        // Optimization Summary
        shared_ptr<ceres::Solver::Summary> summary_pose{};
        shared_ptr<ceres::Solver::Summary> summary_depth_photo{};
        shared_ptr<ceres::Solver::Summary> summary_depth_reproj{};

        // Solver Options
        const int numThreads{1};
        shared_ptr<ceres::Solver::Options> solverOptions_pose{};
        shared_ptr<ceres::Solver::Options> solverOptions_depth_photo{};
        shared_ptr<ceres::Solver::Options> solverOptions_depth_reproj{};

        // Depth Covariance
        ceres::Covariance::Options covarianceOptions{};
        shared_ptr<ceres::Covariance> covariance_photo{};
        shared_ptr<ceres::Covariance> covariance_reproj{};
        vector<pair<const double*, const double*> > covariance_blocks_photo{};
        vector<pair<const double*, const double*> > covariance_blocks_reproj{};

        // Robust cost function
        dataType tstudent2_p{90.0};
        dataType tstudent2ThPhoto{tstudent2(PATCH_SIZE,tstudent2_p)};
        dataType tstudent2ThReproj{tstudent2(2,tstudent2_p)};
        dataType tstudentPhoto{sqrt(tstudent2ThPhoto)};
        dataType tstudentReproj{sqrt(tstudent2ThReproj)};

#ifdef OPTIMIZER_DEBUG
        matXi covisibilityWindow_hgp{};
        matXi covisibilityComplete_hgp{};
        matXi covisibilityWindow_features{};
        matXi covisibilityComplete_features{};
        unordered_map<size_t, size_t> keyframeIndexes{};
#endif

    public: // Public methods

        explicit OptimizerCeres(unordered_map<size_t, typeFrame>*keyframes_, const string& BA_type_):
            keyframes(keyframes_), BA_type(BA_type_){
            setProblemOptions();
            set_tstudent2(tstudent2_p);
        }

        void resetOptimization();
        void prepareOptimization(unordered_map<size_t, typeFrame> &keyframesToOptimize_,
                                 const std::string &optimizationMode_);
        void optimize(int numIt_ = 1, double maxTime_ = 1.0);
        void updateVariables();
        void updateGlobalObservations(std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>& obs_hgp_,
                                      std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>& obs_features_);
        void geoBA(std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>& obs_hgp_,
                   std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>& obs_features_);

        void setOptimizeDepths(const bool& optimizeDepths_);
        void set_tstudent2_p(const dataType& tstudent2_p_);

    private: // Private methods

        void setProblemOptions();
        void set_tstudent2_photo(const dataType &p_);
        void set_tstudent2_reproj(const dataType &p_);
        void set_tstudent2(const dataType &p_);

        void setInitialState(unordered_map<size_t, typeFrame> &keyframesToOptimize_,
                             const std::string &optimizationMode_);
        void updateRelativeRotations();
        void setProjections();
        void setProjections_hgp();
        void setProjections_features();
        void setFixedKeyframes();
        void setObservations();
        void buildProblem();
        void addVariables();
        void addDepths();
        void addPoses();
        void addProblemOptions();
        void addEvaluationCallback();
        void addResiduals_poseOptimization();
        void addResiduals_depthOptimization();
        void addVariableConstraints();

        void optimizeDepthGeo();
        void optimizeDepthPhoto();

#ifdef OPTIMIZER_DEBUG
        void optimizerDebug();
        void showCorrelation(int& refKeyId_ , int& projKeyId_ );
        void showCorrelations();
#endif

        //// Optimization Callback /////////////////////////////////////////////////////////////////////////////////////
        class evaluationCallback : public ceres::EvaluationCallback {
        public:
            explicit evaluationCallback(unordered_map<size_t, typeFrame> *keyframesToOptimize,
                                        unordered_map<size_t, dataType *> *deltaPoses,
                                        unordered_map<size_t, vec3> *tcw,
                                        unordered_map<size_t, mat3> *Rcw,
                                        unordered_map<size_t, vec3> *twc,
                                        unordered_map<size_t, mat3> *Rwc,
                                        unordered_map<size_t, bool> *fixedKeyframesIndexes,
                                        map<std::pair<typeFrame, typeFrame>, mat3> *Rrel,
                                        unordered_map<typePt, vecProj_hgp> *hgpObservations,
                                        unordered_map<typeFt, vecProj_ft> *featureObservations,
                                        unordered_map<typePt, dataType> *deltaLambdaHgp,
                                        unordered_map<typeFt, dataType> *deltaLambdaFt,
                                        unordered_map<size_t, dataType>* deltaPhotoScalar,
                                        unordered_map<size_t, dataType>* deltaPhotoExpScalar,
                                        unordered_map<size_t, dataType>* deltaPhotoBias
            ) :
                    keyframesToOptimize(keyframesToOptimize),
                    deltaPoses(deltaPoses),
                    tcw(tcw), Rcw(Rcw),
                    twc(twc), Rwc(Rwc),
                    fixedKeyframesIndexes(fixedKeyframesIndexes),
                    Rrel(Rrel),
                    hgpObservations(hgpObservations), featureObservations(featureObservations),
                    deltaLambdaHgp(deltaLambdaHgp), deltaLambdaFt(deltaLambdaFt),
                    deltaPhotoScalar(deltaPhotoScalar), deltaPhotoExpScalar(deltaPhotoExpScalar), deltaPhotoBias(deltaPhotoBias){}

            virtual ~evaluationCallback() {}

            virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
                if (new_evaluation_point) {
                    vec3 v{}, w{}, delta_t{};
                    mat3 delta_R{};
                    for (auto&[keyId, keyframe]: (*keyframesToOptimize)) {
                        if (fixedKeyframesIndexes->find(keyId) != fixedKeyframesIndexes->end()) continue;
                        dataType* incPose= (*deltaPoses)[keyId];
                        v << incPose[0], incPose[1], incPose[2];
                        w << incPose[3], incPose[4], incPose[5];
                        exp_lie(delta_t, delta_R, v, w);
                        mat3 Rcw_updated = delta_R * keyframe->pose.Rcw;
                        vec3 tcw_updated = delta_R * keyframe->pose.tcw + delta_t;
                        (*Rcw)[keyId] = Rcw_updated;
                        (*tcw)[keyId] = tcw_updated;
                        inv_R((*Rwc)[keyId],Rcw_updated);
                        (*twc)[keyId] = -(*Rwc)[keyId] * tcw_updated;
                        //cout << " keyId = "<< keyId << " photoScalar_ = " << (*deltaPhotoScalar)[keyId] << endl;
                        //cout << "     v = "<< v.transpose()  << endl;
                        //cout << "     w = "<< w.transpose() <<  endl;

                    }

                    /*for(auto&[keyId, photoScalar_]: *deltaPhotoScalar){
                        //if(abs(photoScalar_) > 2.5){
                            //photoScalar_ = 2.5*photoScalar_/abs(photoScalar_);
                        //}

                        (*deltaPhotoExpScalar)[keyId] = sigmoid(photoScalar_);
                    }*/

                    mat3 R;
                    size_t hostIndex{};
                    size_t projIndex{};
                    for (auto &R_: (*Rrel)) {
                        hostIndex = R_.first.first->keyId;
                        projIndex = R_.first.second->keyId;
                        if (hostIndex != projIndex) {
                            R = (*Rcw)[projIndex] * (*Rwc)[hostIndex];
                            (*Rrel)[std::make_pair(R_.first.first, R_.first.second)] = R;
                            inv_R((*Rrel)[std::make_pair(R_.first.second, R_.first.first)],R);
                        } else {
                            (*Rrel)[std::make_pair(R_.first.first, R_.first.second)] = mat3::Identity();
                            (*Rrel)[std::make_pair(R_.first.second, R_.first.first)] = mat3::Identity();
                        }
                    }

                    dataType lambdaRef;
                    for (auto&[pt, proj]: *hgpObservations) {
                        lambdaRef = (*deltaLambdaHgp)[pt];
                        hostIndex = proj[0]->refKeyId;
                        pt->xynlambda_to_XYZ((*twc)[hostIndex], (*Rwc)[hostIndex], lambdaRef);
                    }
                    for (auto&[ft, proj]: *featureObservations) {
                        lambdaRef =  (*deltaLambdaFt)[ft];
                        hostIndex = proj[0]->refKeyId;
                        ft->xynlambda_to_XYZ((*twc)[hostIndex], (*Rwc)[hostIndex], lambdaRef);
                    }
                }
            };

            unordered_map<size_t, typeFrame> *keyframesToOptimize;
            unordered_map<size_t, dataType *> *deltaPoses;

            unordered_map<size_t, vec3> *tcw;
            unordered_map<size_t, mat3> *Rcw;
            unordered_map<size_t, vec3> *twc;
            unordered_map<size_t, mat3> *Rwc;
            map<std::pair<typeFrame, typeFrame>, mat3> *Rrel;
            unordered_map<size_t, bool> *fixedKeyframesIndexes{};
            unordered_map<typePt, vecProj_hgp> *hgpObservations;
            unordered_map<typeFt, vecProj_ft> *featureObservations;
            unordered_map<typePt, dataType> *deltaLambdaHgp;
            unordered_map<typeFt, dataType> *deltaLambdaFt;
            unordered_map<size_t, dataType>* deltaPhotoScalar{};
            unordered_map<size_t, dataType>* deltaPhotoExpScalar{};
            unordered_map<size_t, dataType>* deltaPhotoBias{};
        };

        //// Analytic Residuals and Jacobians //////////////////////////////////////////////////////////////////////////
        static void photoResidual_bilInt(dataType *residuals_,
                                         const typeProj_hgp &proj,
                                         dataType* phScalarExpRef, dataType* phBiasRef,
                                         dataType* phScalarExpProj, dataType* phBiasProj
                                         ) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;
            for (int iPatch{0}; iPatch < proj->pt->ptSize; ++iPatch) {
                projKey->extract_I_pixel_BilinInt(pt->I[iPatch],pt->u[iPatch], pt->v[iPatch]);

                residuals_[iPatch] = (
                        ((pt->I[iPatch] - *phBiasProj) * (*phScalarExpRef)) -
                        ((pt->I_ref[iPatch][0] - *phBiasRef) * (*phScalarExpProj))
                                      )/ proj->photoStd;
            }
        }

        static void set_dI_dLambdaRef(dataType **jacobians , const vec3& dI_dxyzcRef, const typePt& pt, const int& iPatch , const int& varIdx){
            jacobians[varIdx][iPatch] = (dI_dxyzcRef(0) * pt->xnRef[iPatch] + dI_dxyzcRef(1) * pt->ynRef[iPatch] + dI_dxyzcRef(2)) *
                                        (-1.0 / (pt->lambdaRef * pt->lambdaRef));
        }

        static void reprojResidual(dataType *residuals_, const typeProj_ft &proj) {
            typeFt ft = proj->ft;
            vec2 geoError{proj->obs.pt.x - ft->u[0], proj->obs.pt.y - ft->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);
        }

        static void set_duv_dPose(dataType **jacobians , const mat26& duv_dPose, const int& varIdx){
            jacobians[varIdx][0 * 6 + 0] = -duv_dPose(0,0);
            jacobians[varIdx][0 * 6 + 1] = -duv_dPose(0,1);
            jacobians[varIdx][0 * 6 + 2] = -duv_dPose(0,2);
            jacobians[varIdx][0 * 6 + 3] = -duv_dPose(0,3);
            jacobians[varIdx][0 * 6 + 4] = -duv_dPose(0,4);
            jacobians[varIdx][0 * 6 + 5] = -duv_dPose(0,5);

            jacobians[varIdx][1 * 6 + 0] = -duv_dPose(1,0);
            jacobians[varIdx][1 * 6 + 1] = -duv_dPose(1,1);
            jacobians[varIdx][1 * 6 + 2] = -duv_dPose(1,2);
            jacobians[varIdx][1 * 6 + 3] = -duv_dPose(1,3);
            jacobians[varIdx][1 * 6 + 4] = -duv_dPose(1,4);
            jacobians[varIdx][1 * 6 + 5] = -duv_dPose(1,5);
        }

        static void set_dI_dx(dataType **jacobians , const mat16& dI_dx, const int& initJacobian, const int& varIdx){
            jacobians[varIdx][initJacobian] = dI_dx(0,0);
            jacobians[varIdx][initJacobian + 1] = dI_dx(0,1);
            jacobians[varIdx][initJacobian + 2] = dI_dx(0,2);
            jacobians[varIdx][initJacobian + 3] = dI_dx(0,3);
            jacobians[varIdx][initJacobian + 4] = dI_dx(0,4);
            jacobians[varIdx][initJacobian + 5] = dI_dx(0,5);
        }

        static void set_duv_dlambdaRef(dataType **jacobians , const mat21& duv_dlambdaRef, const int& varIdx) {
            jacobians[varIdx][0] = -duv_dlambdaRef(0,0);
            jacobians[varIdx][1] = -duv_dlambdaRef(1,0);
        }

        //// Cost Functions Pose Optimization //////////////////////////////////////////////////////////////////////////
        static void photoResidualAndJacobians_poseOpt(dataType *residuals_, dataType **jacobians,
                                                      const typeProj_hgp &proj,
                                                      dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                                      dataType* phScalarExpProj, dataType* phScalarProj, dataType* phBiasProj) {
            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;

            int initJacobian;
            vec3 du_dxyzc{}, dv_dxyzc{};
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

            vec3 du_dxRef = du_dxyzc.transpose()*(*proj->Rrel);
            vec3 dv_dxRef = dv_dxyzc.transpose()*(*proj->Rrel);

            mat26 duv_dx{};
            pt->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);

            mat26 duv_dxRef{};
            pt->duv_dxRef(duv_dxRef,du_dxRef,dv_dxRef,0);

            mat12 dI_duv;
            mat16 dI_dx;
            mat16 dI_dxRef;
            dataType photoConstantProj = (*phScalarExpProj)/proj->photoStd;
            dataType photoConstantRef = (*phScalarExpRef)/proj->photoStd;
            for (int iPatch{0}; iPatch < proj->pt->ptSize; ++iPatch) {
                initJacobian = iPatch * 6;

                projKey->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch], pt->u[iPatch],
                                                        pt->v[iPatch]);

                residuals_[iPatch] = (
                        ((pt->I[iPatch] - *phBiasProj) * (*phScalarExpProj)) -
                        ((pt->I_ref[iPatch][0] - *phBiasRef) * (*phScalarExpRef))
                                     )/ proj->photoStd;

                dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                dI_duv *= photoConstantProj;
                dI_dx = dI_duv*duv_dx;
                dI_dxRef = dI_duv*duv_dxRef;

                set_dI_dx(jacobians , dI_dxRef, initJacobian ,0);
                set_dI_dx(jacobians , dI_dx, initJacobian, 1);

                //jacobians[2][iPatch] = -((pt->I_ref[iPatch][0] - *phBiasRef)/proj->photoStd) * dsigmoid_dx(*phScalarRef);
                //jacobians[3][iPatch] = photoConstantRef;
                //jacobians[4][iPatch] = ((pt->I[iPatch] - *phBiasProj)/proj->photoStd) * dsigmoid_dx(*phScalarProj);
                //jacobians[5][iPatch] = -photoConstantProj;

                /*cout << "dI_dxRef = "<<dI_dxRef<< endl;
                cout << "dI_dx = "<<dI_dx << endl;
                cout << "dIjacobians[2][iPatch]_dx = "<<jacobians[2][iPatch] << endl;
                cout << "dIjacobians[3][iPatch]_dx = "<<jacobians[3][iPatch] << endl;
                cout << "dIjacobians[4][iPatch]_dx = "<<jacobians[4][iPatch] << endl;
                cout << "dIjacobians[5][iPatch]_dx = "<<jacobians[5][iPatch] << endl;*/
            }
        }

        static void photoResidualAndJacobians_poseOpt_ref(dataType *residuals_, dataType **jacobians,
                                                          const typeProj_hgp &proj,
                                                          dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                                          dataType* phScalarExpProj, dataType* phScalarProj, dataType* phBiasProj) {
            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;

            int initJacobian;
            vec3 du_dxyzc{}, dv_dxyzc{};
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

            vec3 du_dxRef = du_dxyzc.transpose()*(*proj->Rrel);
            vec3 dv_dxRef = dv_dxyzc.transpose()*(*proj->Rrel);

            mat26 duv_dxRef{};
            pt->duv_dxRef(duv_dxRef,du_dxRef,dv_dxRef,0);

            mat12 dI_duv;
            mat16 dI_dxRef;
            dataType photoConstantProj = (*phScalarExpProj)/proj->photoStd;
            dataType photoConstantRef = (*phScalarExpRef)/proj->photoStd;
            for (int iPatch{0}; iPatch < proj->pt->ptSize; ++iPatch) {
                initJacobian = iPatch * 6;

                projKey->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch], pt->u[iPatch],
                                                        pt->v[iPatch]);

                residuals_[iPatch] = (
                        ((pt->I[iPatch] - *phBiasProj) * (*phScalarExpProj)) -
                        ((pt->I_ref[iPatch][0] - *phBiasRef) * (*phScalarExpRef))
                                     )/ proj->photoStd;

                dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                dI_duv *= photoConstantProj;
                dI_dxRef = dI_duv*duv_dxRef;

                set_dI_dx(jacobians , dI_dxRef, initJacobian ,0);
                //jacobians[1][iPatch] = -((pt->I_ref[iPatch][0] - *phBiasRef)/proj->photoStd) * dsigmoid_dx(*phScalarRef);
                //jacobians[2][iPatch] = photoConstantRef;
            }
        }

        static void photoResidualAndJacobians_poseOpt_proj(dataType *residuals_, dataType **jacobians,
                                                           const typeProj_hgp &proj,
                                                           dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                                           dataType* phScalarExpProj, dataType* phScalarProj, dataType* phBiasProj) {
            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;

            int initJacobian;
            vec3 du_dxyzc{}, dv_dxyzc{};
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

            mat26 duv_dx{};
            pt->duv_dx(duv_dx,du_dxyzc,dv_dxyzc,0);

            mat12 dI_duv;
            mat16 dI_dx;
            dataType photoConstantProj = (*phScalarExpProj)/proj->photoStd;
            for (int iPatch{0}; iPatch < proj->pt->ptSize; ++iPatch) {
                initJacobian = iPatch * 6;

                projKey->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch], pt->u[iPatch],
                                                        pt->v[iPatch]);

                residuals_[iPatch] = (
                        ((pt->I[iPatch] - *phBiasProj) * (*phScalarExpProj)) -
                        ((pt->I_ref[iPatch][0] - *phBiasRef) * (*phScalarExpRef))
                                     )/ proj->photoStd;

                dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                dI_duv *= photoConstantProj;
                dI_dx = dI_duv*duv_dx;

                set_dI_dx(jacobians , dI_dx, initJacobian, 0);
                //jacobians[1][iPatch] = ((pt->I[iPatch] - *phBiasProj)/proj->photoStd) * dsigmoid_dx(*phScalarProj);
                //jacobians[2][iPatch] = -photoConstantProj;
            }
        }

        class PhotoResidual_poseOpt : public ceres::SizedCostFunction<PATCH_SIZE, 6, 6> {
        public:

            PhotoResidual_poseOpt(typeProj_hgp &proj, vec3 *tcw, mat3 *Rcw, dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                                                            dataType* phScalarExpProj,dataType* phScalarProj, dataType* phBiasProj) :
                    proj(proj), tcw(tcw), Rcw(Rcw), phScalarExpRef(phScalarExpRef), phScalarRef(phScalarRef), phBiasRef(phBiasRef),
                                                    phScalarExpProj(phScalarExpProj), phScalarProj(phScalarProj), phBiasProj(phBiasProj) {}

            virtual ~PhotoResidual_poseOpt() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->pt->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->pt);
                if (jacobians)
                    photoResidualAndJacobians_poseOpt(residuals, jacobians, proj, phScalarExpRef, phScalarRef, phBiasRef, phScalarExpProj, phScalarProj, phBiasProj);
                else
                    photoResidual_bilInt(residuals, proj, phScalarExpRef, phBiasRef, phScalarExpProj, phBiasProj);

                return true;
            }

            typeProj_hgp proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
            dataType* phScalarExpRef{};
            dataType* phScalarRef{};
            dataType* phBiasRef{};
            dataType* phScalarExpProj{};
            dataType* phScalarProj{};
            dataType* phBiasProj{};
        };

        class PhotoResidual_poseOpt_ref : public ceres::SizedCostFunction<PATCH_SIZE, 6> {
        public:

            PhotoResidual_poseOpt_ref(typeProj_hgp &proj, vec3 *tcw, mat3 *Rcw, dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                      dataType* phScalarExpProj,dataType* phScalarProj, dataType* phBiasProj) :
                    proj(proj), tcw(tcw), Rcw(Rcw), phScalarExpRef(phScalarExpRef), phScalarRef(phScalarRef), phBiasRef(phBiasRef),
                    phScalarExpProj(phScalarExpProj), phScalarProj(phScalarProj), phBiasProj(phBiasProj) {}

            virtual ~PhotoResidual_poseOpt_ref() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->pt->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->pt);

                if (jacobians)
                    photoResidualAndJacobians_poseOpt_ref(residuals, jacobians, proj, phScalarExpRef, phScalarRef, phBiasRef, phScalarExpProj, phScalarProj, phBiasProj);
                else
                    photoResidual_bilInt(residuals, proj, phScalarExpRef, phBiasRef, phScalarExpProj, phBiasProj);

                return true;
            }

            typeProj_hgp proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
            dataType* phScalarExpRef{};
            dataType* phScalarRef{};
            dataType* phBiasRef{};
            dataType* phScalarExpProj{};
            dataType* phScalarProj{};
            dataType* phBiasProj{};
        };

        class PhotoResidual_poseOpt_proj : public ceres::SizedCostFunction<PATCH_SIZE, 6> {
        public:

            PhotoResidual_poseOpt_proj(typeProj_hgp &proj, vec3 *tcw, mat3 *Rcw, dataType* phScalarExpRef, dataType* phScalarRef, dataType* phBiasRef,
                                       dataType* phScalarExpProj,dataType* phScalarProj, dataType* phBiasProj) :
                    proj(proj), tcw(tcw), Rcw(Rcw), phScalarExpRef(phScalarExpRef), phScalarRef(phScalarRef), phBiasRef(phBiasRef),
                    phScalarExpProj(phScalarExpProj), phScalarProj(phScalarProj), phBiasProj(phBiasProj) {}

            virtual ~PhotoResidual_poseOpt_proj() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->pt->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->pt);

                if (jacobians)
                    photoResidualAndJacobians_poseOpt_proj(residuals, jacobians, proj, phScalarExpRef, phScalarRef, phBiasRef, phScalarExpProj, phScalarProj, phBiasProj);
                else
                    photoResidual_bilInt(residuals, proj, phScalarExpRef, phBiasRef, phScalarExpProj, phBiasProj);

                return true;
            }

            typeProj_hgp proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
            dataType* phScalarExpRef{};
            dataType* phScalarRef{};
            dataType* phBiasRef{};
            dataType* phScalarExpProj{};
            dataType* phScalarProj{};
            dataType* phBiasProj{};
        };

        static void reprojResidualAndJacobians_poseOpt(dataType *residuals_, dataType **jacobians, const typeProj_ft &proj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typeFt ft = proj->ft;
            vec2 geoError{proj->obs.pt.x - ft->u[0], proj->obs.pt.y - ft->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);

            vec3 du_dxyzc, dv_dxyzc;
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);

            mat26 duv_dPose0{mat26::Zero()};
            mat26 duv_dPoseRef0{mat26::Zero()};
            vec3 du_dxyzcRef = du_dxyzc.transpose() * (*(proj->Rrel));
            vec3 dv_dxyzcRef = dv_dxyzc.transpose() * (*(proj->Rrel));

            ft->duv_dx(duv_dPose0, du_dxyzc, dv_dxyzc, 0);
            ft->duv_dxRef(duv_dPoseRef0, du_dxyzcRef, dv_dxyzcRef, 0);

            mat26 duv_dPose = proj->geoInfStd * duv_dPose0;
            mat26 duv_dPoseRef = proj->geoInfStd * duv_dPoseRef0;

            set_duv_dPose(jacobians , duv_dPoseRef, 0);
            set_duv_dPose(jacobians , duv_dPose, 1);

        }

        static void reprojResidualAndJacobians_poseOpt_ref(dataType *residuals_, dataType **jacobians, const typeProj_ft &proj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typeFt ft = proj->ft;
            vec2 geoError{proj->obs.pt.x - ft->u[0], proj->obs.pt.y - ft->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);

            vec3 du_dxyzc, dv_dxyzc;
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);

            mat26 duv_dPose0{mat26::Zero()};
            mat26 duv_dPoseRef0{mat26::Zero()};
            vec3 du_dxyzcRef = du_dxyzc.transpose() * (*(proj->Rrel));
            vec3 dv_dxyzcRef = dv_dxyzc.transpose() * (*(proj->Rrel));

            ft->duv_dxRef(duv_dPoseRef0, du_dxyzcRef, dv_dxyzcRef, 0);
            mat26 duv_dPoseRef = proj->geoInfStd * duv_dPoseRef0;

            set_duv_dPose(jacobians , duv_dPoseRef, 0);

        }

        static void reprojResidualAndJacobians_poseOpt_proj(dataType *residuals_, dataType **jacobians, const typeProj_ft &proj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typeFt ft = proj->ft;
            vec2 geoError{proj->obs.pt.x - ft->u[0], proj->obs.pt.y - ft->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);

            vec3 du_dxyzc, dv_dxyzc;
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);

            mat26 duv_dPose0{mat26::Zero()};
            ft->duv_dx(duv_dPose0, du_dxyzc, dv_dxyzc, 0);
            mat26 duv_dPose = proj->geoInfStd * duv_dPose0;

            set_duv_dPose(jacobians , duv_dPose, 0);
        }

        class ReprojResidual_poseOpt : public ceres::SizedCostFunction<2, 6, 6> {
        public:

            ReprojResidual_poseOpt(typeProj_ft &proj, vec3 *tcw, mat3 *Rcw) :
                    proj(proj), tcw(tcw), Rcw(Rcw) {}

            virtual ~ReprojResidual_poseOpt() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->ft->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->ft);

                if (jacobians)
                    reprojResidualAndJacobians_poseOpt(residuals, jacobians, proj);
                else
                    reprojResidual(residuals, proj);

                return true;
            }

            typeProj_ft proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
        };

        class ReprojResidual_poseOpt_ref : public ceres::SizedCostFunction<2, 6> {
        public:

            ReprojResidual_poseOpt_ref(typeProj_ft &proj, vec3 *tcw, mat3 *Rcw) :
                    proj(proj), tcw(tcw), Rcw(Rcw) {}

            virtual ~ReprojResidual_poseOpt_ref() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->ft->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->ft);

                if (jacobians)
                    reprojResidualAndJacobians_poseOpt_ref(residuals, jacobians, proj);
                else
                    reprojResidual(residuals, proj);

                return true;
            }

            typeProj_ft proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
        };

        class ReprojResidual_poseOpt_proj : public ceres::SizedCostFunction<2, 6> {
        public:

            ReprojResidual_poseOpt_proj(typeProj_ft &proj, vec3 *tcw, mat3 *Rcw) :
                    proj(proj), tcw(tcw), Rcw(Rcw) {}

            virtual ~ReprojResidual_poseOpt_proj() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->ft->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->ft);

                if (jacobians)
                    reprojResidualAndJacobians_poseOpt_proj(residuals, jacobians, proj);
                else
                    reprojResidual(residuals, proj);

                return true;
            }

            typeProj_ft proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
        };

        //// Cost Functions Depth optimization  ////////////////////////////////////////////////////////////////////////
        static void photoResidualAndJacobians_depthOpt(dataType *residuals_, dataType **jacobians,
                                                       const typeProj_hgp &proj,
                                                       dataType const *const *param1,
                                                       dataType* phScalarExpRef, dataType* phBiasRef,
                                                       dataType* phScalarExpProj, dataType* phBiasProj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;
            dataType lambdaRefUpdated = param1[0][0];

            vec3 du_dxyzc{}, dv_dxyzc{};
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, pt, 0);

            vec3 du_xyzRef, dv_xyzRef;
            du_xyzRef = du_dxyzc.transpose()* (*proj->Rrel);
            dv_xyzRef = dv_dxyzc.transpose()* (*proj->Rrel);

            mat21 duv_lambdaRef;
            duv_lambdaRef(0,0)= (du_xyzRef(0) * pt->xnRef[0] + du_xyzRef(1) * pt->ynRef[0] + du_xyzRef(2)) *
                                 (-1.0 / (lambdaRefUpdated*lambdaRefUpdated));
            duv_lambdaRef(1,0) = (dv_xyzRef(0) * pt->xnRef[0] + dv_xyzRef(1) * pt->ynRef[0] + dv_xyzRef(2)) *
                                (-1.0 / (lambdaRefUpdated*lambdaRefUpdated));

            mat12 dI_duv;
            dataType dI_dlambdaRef;
            dataType photoConstant = (*phScalarExpProj)/proj->photoStd;
            proj->dI_dlambdaRef = 0.0;
            for (int iPatch{0}; iPatch < proj->pt->ptSize; ++iPatch) {
                projKey->extract_I_and_G_pixel_BilinInt(pt->I[iPatch], pt->Gu[iPatch], pt->Gv[iPatch], pt->u[iPatch],
                                                        pt->v[iPatch]);

                residuals_[iPatch] = (
                                             ((pt->I[iPatch] - *phBiasProj) * (*phScalarExpProj)) -
                                             ((pt->I_ref[iPatch][0] - *phBiasRef) * (*phScalarExpRef))
                                     )/ proj->photoStd;

                dI_duv << pt->Gu[iPatch], pt->Gv[iPatch];
                dI_duv *= photoConstant;
                dI_dlambdaRef = dI_duv*duv_lambdaRef;
                jacobians[0][iPatch] = dI_dlambdaRef;
                proj->dI_dlambdaRef += dI_dlambdaRef*dI_dlambdaRef;
            }
        }

        class PhotoResidual_depthOpt : public ceres::SizedCostFunction<PATCH_SIZE,1> {
        public:

            PhotoResidual_depthOpt(typeProj_hgp &proj, vec3 *tcw, mat3 *Rcw, dataType* phScalarExpRef, dataType* phBiasRef, dataType* phScalarExpProj, dataType* phBiasProj) :
                    proj(proj), tcw(tcw), Rcw(Rcw), phScalarExpRef(phScalarExpRef), phBiasRef(phBiasRef),phScalarExpProj(phScalarExpProj), phBiasProj(phBiasProj) {}

            virtual ~PhotoResidual_depthOpt() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->pt->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->pt);

                if (jacobians)
                    photoResidualAndJacobians_depthOpt(residuals, jacobians, proj, param1, phScalarExpRef, phBiasRef, phScalarExpProj, phBiasProj);
                else
                    photoResidual_bilInt(residuals, proj, phScalarExpRef, phBiasRef, phScalarExpProj, phBiasProj);

                return true;
            }

            typeProj_hgp proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
            dataType* phScalarExpRef{};
            dataType* phBiasRef{};
            dataType* phScalarExpProj{};
            dataType* phBiasProj{};
        };

        static void reprojResidualAndJacobians_depthOpt(dataType *residuals_, dataType **jacobians, const typeProj_ft &proj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typeFt ft = proj->ft;
            vec2 geoError{proj->obs.pt.x - ft->u[0], proj->obs.pt.y - ft->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);

            vec3 du_dxyzc, dv_dxyzc;
            projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);

            mat21 duv_dlambdaRef0{mat21::Zero()};
            vec3 du_dxyzcRef = du_dxyzc.transpose() * (*(proj->Rrel));
            vec3 dv_dxyzcRef = dv_dxyzc.transpose() * (*(proj->Rrel));

            ft->duv_dlambdaRef(duv_dlambdaRef0 , du_dxyzcRef, dv_dxyzcRef, 0);

            mat21 duv_dlambdaRef = proj->geoInfStd * duv_dlambdaRef0;
            proj->duv_dlambdaRef = (duv_dlambdaRef.array()*duv_dlambdaRef.array()).sum();

            set_duv_dlambdaRef(jacobians , duv_dlambdaRef, 0);
            /*cout << "du_dxyzc = " << du_dxyzc << endl;
            cout << "dv_dxyzc = " << dv_dxyzc << endl;
            cout << "du_dxyzcRef = " << du_dxyzcRef << endl;
            cout << "dv_dxyzcRef = " << dv_dxyzcRef << endl;
            cout << "duv_dlambdaRef0 = " << duv_dlambdaRef0.transpose() << endl;
            cout << "duv_dlambdaRef = " << duv_dlambdaRef << endl;*/
        }


        class ReprojResidual_depthOpt : public ceres::SizedCostFunction<2,1> {
        public:

            ReprojResidual_depthOpt(typeProj_ft &proj, vec3 *tcw, mat3 *Rcw) :
                    proj(proj), tcw(tcw), Rcw(Rcw) {}

            virtual ~ReprojResidual_depthOpt() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->ft->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->ft);

                if (jacobians)
                    reprojResidualAndJacobians_depthOpt(residuals, jacobians, proj);
                else
                    reprojResidual(residuals, proj);

                return true;
            }

            typeProj_ft proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        class evaluationCallback_geoBA : public ceres::EvaluationCallback {
        public:
            explicit evaluationCallback_geoBA(unordered_map<size_t, typeFrame> *keyframesToOptimize,
                                        unordered_map<size_t, dataType *> *deltaPoses,
                                        unordered_map<size_t, vec3> *tcw,
                                        unordered_map<size_t, mat3> *Rcw,
                                        unordered_map<size_t, vec3> *twc,
                                        unordered_map<size_t, mat3> *Rwc,
                                        unordered_map<size_t, bool> *fixedKeyframesIndexes,
                                        map<std::pair<typeFrame, typeFrame>, mat3> *Rrel_,
                                        std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>* obs_hgp_,
                                        std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>* obs_features_
            ) :
                    keyframesToOptimize(keyframesToOptimize),
                    deltaPoses(deltaPoses),
                    tcw(tcw), Rcw(Rcw),
                    twc(twc), Rwc(Rwc),
                    fixedKeyframesIndexes(fixedKeyframesIndexes),
                    Rrel(Rrel_),
                    obs_hgp(obs_hgp_), obs_features(obs_features_){}

            virtual ~evaluationCallback_geoBA() {}

            virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
                if (new_evaluation_point) {
                    vec3 v{}, w{}, delta_t{};
                    mat3 delta_R{};
                    size_t keyIndex{};
                    for (auto&[keyId, keyframe]: (*keyframesToOptimize)) {
                        keyIndex = keyId;//(*indexKeyframes)[keyId];
                        if (fixedKeyframesIndexes->find(keyIndex) != fixedKeyframesIndexes->end()) continue;
                        dataType* incPose= (*deltaPoses)[keyIndex];
                        v << incPose[0], incPose[1], incPose[2];
                        w << incPose[3], incPose[4], incPose[5];
                        exp_lie(delta_t, delta_R, v, w);
                        mat3 Rcw_updated = delta_R * keyframe->pose.Rcw;
                        vec3 tcw_updated = delta_R * keyframe->pose.tcw + delta_t;
                        (*Rcw)[keyIndex] = Rcw_updated;
                        (*tcw)[keyIndex] = tcw_updated;
                        inv_R((*Rwc)[keyIndex],Rcw_updated);
                        (*twc)[keyIndex] = -(*Rwc)[keyIndex] * tcw_updated;
                    }

                    mat3 R;
                    size_t hostIndex{};
                    size_t projIndex{};
                    for (auto &R_: (*Rrel)) {
                        hostIndex = R_.first.first->keyId;
                        projIndex = R_.first.second->keyId;
                        if (hostIndex != projIndex) {
                            R = (*Rcw)[projIndex] * (*Rwc)[hostIndex];
                            (*Rrel)[std::make_pair(R_.first.first, R_.first.second)] = R;
                            inv_R((*Rrel)[std::make_pair(R_.first.second, R_.first.first)],R);
                        } else {
                            (*Rrel)[std::make_pair(R_.first.first, R_.first.second)] = mat3::Identity();
                            (*Rrel)[std::make_pair(R_.first.second, R_.first.first)] = mat3::Identity();
                        }

                    }

                    for (auto&[pt, proj]: *obs_hgp) {
                        hostIndex = pt->idKey;
                        pt->xynlambda_to_XYZ((*twc)[hostIndex], (*Rwc)[hostIndex], pt->lambdaRef);
                    }
                    for (auto&[ft, proj]: *obs_features) {
                        hostIndex = ft->idKey;
                        ft->xynlambda_to_XYZ((*twc)[hostIndex], (*Rwc)[hostIndex], ft->lambdaRef);
                    }

                }
            };

            unordered_map<size_t, typeFrame> *keyframesToOptimize;
            unordered_map<size_t, dataType *> *deltaPoses;

            unordered_map<size_t, vec3> *tcw;
            unordered_map<size_t, mat3> *Rcw;
            unordered_map<size_t, vec3> *twc;
            unordered_map<size_t, mat3> *Rwc;
            unordered_map<size_t, bool> *fixedKeyframesIndexes{};
            map<std::pair<typeFrame, typeFrame>, mat3> *Rrel;
            std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>* obs_hgp;
            std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>* obs_features;
        };

        static void reprojResidual_geoBA(dataType *residuals_, const typeProj_ft &proj) {
            typePt pt = proj->pt;
            vec2 geoError{proj->obs.pt.x - pt->u[0], proj->obs.pt.y - pt->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);
        }

        static void reprojResidualAndJacobians_poseOpt_geoBA(dataType *residuals_, dataType **jacobians, const typeProj_ft &proj) {

            typeFrame projKey = proj->projKey;
            typeFrame refKey = proj->refKey;
            typePt pt = proj->pt;
            vec2 geoError{proj->obs.pt.x - pt->u[0], proj->obs.pt.y - pt->v[0]};
            geoError = proj->geoInfStd * geoError;
            residuals_[0] = geoError(0);
            residuals_[1] = geoError(1);

            vec3 du_dxyzc, dv_dxyzc;

            //projKey->cam->duv_dxyzc(du_dxyzc, dv_dxyzc, ft);
            dataType du_dx = projKey->cam->getFocalLengthX()*pt->lambda[0];
            dataType dv_dy = projKey->cam->getFocalLengthY()*pt->lambda[0];
            du_dxyzc << du_dx,  0.0  , -du_dx*pt->xn[0];
            dv_dxyzc <<  0.0 , dv_dy , -dv_dy*pt->yn[0];

            mat26 duv_dPose0{mat26::Zero()};
            mat26 duv_dPoseRef0{mat26::Zero()};
            vec3 du_dxyzcRef = du_dxyzc.transpose() * (*(proj->Rrel));
            vec3 dv_dxyzcRef = dv_dxyzc.transpose() * (*(proj->Rrel));

            pt->duv_dx(duv_dPose0, du_dxyzc, dv_dxyzc, 0);
            pt->duv_dxRef(duv_dPoseRef0, du_dxyzcRef, dv_dxyzcRef, 0);

            mat26 duv_dPose = proj->geoInfStd * duv_dPose0;
            mat26 duv_dPoseRef = proj->geoInfStd * duv_dPoseRef0;

            set_duv_dPose(jacobians , duv_dPoseRef, 0);
            set_duv_dPose(jacobians , duv_dPose, 1);

        }

        class ReprojResidual_poseOpt_geoBA : public ceres::SizedCostFunction<2, 6, 6> {
        public:

            ReprojResidual_poseOpt_geoBA(typeProj_ft &proj, vec3 *tcw, mat3 *Rcw) :
                    proj(proj), tcw(tcw), Rcw(Rcw) {}

            virtual ~ReprojResidual_poseOpt_geoBA() {}

            virtual bool Evaluate(dataType const *const *param1,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                // Compute photometric residuals and analytic jacobians
                proj->pt->XYZ_to_xynlambda(*tcw, *Rcw);
                proj->projKey->cam->xyn_2_uv(proj->pt);

                if (jacobians)
                    reprojResidualAndJacobians_poseOpt_geoBA(residuals, jacobians, proj);
                else
                    reprojResidual_geoBA(residuals, proj);

                return true;
            }

            typeProj_ft proj{};
            vec3 *tcw{};
            mat3 *Rcw{};
        };
    };
}
#endif //IDNAV_OPTIMIZERCERES_H
