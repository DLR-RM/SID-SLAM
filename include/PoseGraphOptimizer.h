//
// Created by afontan on 21/10/21.
//

#ifndef IDNAV_POSEGRAPHOPTIMIZER_H
#define IDNAV_POSEGRAPHOPTIMIZER_H

#include "Frame.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace IDNav {

    class graphEdge{
    public:
        size_t id_i;
        size_t id_j;
        mat4 pose_ji;
        mat4 pose_i;
        mat4 pose_j;
        mat6 poseCov{mat6::Identity()};
        graphEdge(size_t& id_i, size_t& id_j, mat4& pose_i, mat4& pose_j,  mat4& pose_ji, mat6& poseCov):
                id_i(id_i), id_j(id_j) ,pose_ji(pose_ji),pose_i(pose_i),pose_j(pose_j),poseCov(poseCov){};
    };

    class PoseGraphOptimizer {
    public:
        IDNav::Profiler buildProfiler{"profiler" , "" ,"PoseGraphOptimizer","build"};
        IDNav::Profiler solveProfiler{"profiler" , "" ,"PoseGraphOptimizer","solve"};

        void optimizePoseGraph(unordered_map<size_t, typeFrame> *keyframes);

        class evaluationCallbackPoseGraph : public ceres::EvaluationCallback {
        public:
            explicit evaluationCallbackPoseGraph(unordered_map<size_t, typeFrame> *keyframesToOptimize,
                                        unordered_map<size_t, dataType *> *deltaPoses,
                                        unordered_map<size_t, vec3> *tcw,
                                        unordered_map<size_t, mat3> *Rcw,
                                        unordered_map<size_t, vec3> *twc,
                                        unordered_map<size_t, mat3> *Rwc) :
                    keyframesToOptimize(keyframesToOptimize),
                    deltaPoses(deltaPoses),
                    tcw(tcw), Rcw(Rcw),
                    twc(twc), Rwc(Rwc){}

            virtual ~evaluationCallbackPoseGraph() {}

            virtual void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
                if (new_evaluation_point) {
                    vec3 v{}, w{}, delta_t{};
                    mat3 delta_R{};
                    for (auto&[keyId, keyframe]: (*keyframesToOptimize)) {
                        if (keyId == 0) continue;
                        dataType* incPose= (*deltaPoses)[keyId];
                        v << incPose[0], incPose[1], incPose[2];
                        w << incPose[3], incPose[4], incPose[5];
                        exp_lie(delta_t, delta_R, v, w);
                        mat3 Rwc_updated = delta_R * keyframe->pose.Rwc;
                        vec3 twc_updated = delta_R * keyframe->pose.twc + delta_t;
                        (*Rwc)[keyId] = Rwc_updated;
                        (*twc)[keyId] = twc_updated;
                        inv_R((*Rcw)[keyId],Rwc_updated);
                        (*tcw)[keyId] = -(*Rcw)[keyId] * twc_updated;
                    }
                }
            };

            unordered_map<size_t, typeFrame> *keyframesToOptimize;
            unordered_map<size_t, dataType *> *deltaPoses;
            unordered_map<size_t, vec3> *tcw;
            unordered_map<size_t, mat3> *Rcw;
            unordered_map<size_t, vec3> *twc;
            unordered_map<size_t, mat3> *Rwc;
        };

        class PoseGraphResidual : public ceres::SizedCostFunction<6,6,6> {
        public:

            PoseGraphResidual(mat3& R_ji_metric, vec3& t_ji_metric,
                              mat3* Ri, vec3* ti,
                              mat3* Rj, vec3* tj,
                              mat3* Ri_inv, vec3* ti_inv,
                              mat3* Rj_inv, vec3* tj_inv,
                              dataType& poseCov,
                              size_t id_i, size_t id_j):
                    R_ji_metric(R_ji_metric), t_ji_metric(t_ji_metric),
                    Ri(Ri), ti(ti),
                    Rj(Rj), tj(tj),
                    Ri_inv(Ri_inv), ti_inv(ti_inv),
                    Rj_inv(Rj_inv), tj_inv(tj_inv),
                    poseCov(poseCov),
                    id_i(id_i), id_j(id_j){

            }
            virtual ~PoseGraphResidual() {}

            virtual bool Evaluate(dataType const *const *param,
                                  dataType *residuals,
                                  dataType **jacobians) const {

                mat3 Rji = (*Rj)*(*Ri_inv);
                vec3 tji =  (*Rj)*(*ti_inv) + (*tj);

                // mat4 poseResidual = T_ji_metric * (*Ti) * (*Tj_inv);
                mat3 R_Residual1 = (*Ri)*(*Rj_inv);
                vec3 t_Residual1 = (*Ri)*(*tj_inv) + (*ti);
                mat3 R_Residual = R_ji_metric*R_Residual1;
                vec3 t_Residual = R_ji_metric*(t_Residual1) + t_ji_metric;

                vec3 v,w;
                log_lie(v,w,t_Residual,R_Residual);

                for(int iTras{0}; iTras < 3; ++iTras)
                    residuals[iTras] = v[iTras]*poseCov;

                for(int iRot{0}; iRot < 3; ++iRot)
                    residuals[iRot + 3] = w[iRot]*poseCov;

                if(jacobians){
                    mat3 vx{},wx{};
                    antisymmetricMatrix(vx,v);
                    antisymmetricMatrix(wx,w);

                    mat6 temp1{mat6::Zero()};
                    temp1.block<3,3>(0,0) = wx;
                    temp1.block<3,3>(0,3) = vx;
                    temp1.block<3,3>(3,3) = wx;

                    mat6 Ji{mat6::Zero()};
                    if(id_i > 0){
                        mat6 temp2{mat6::Zero()};
                        temp2.block<3,3>(0,0) = Rji;
                        mat3 antisymmetric_tji;
                        antisymmetricMatrix(antisymmetric_tji,tji);
                        temp2.block<3,3>(0,3) = antisymmetric_tji*Rji;
                        temp2.block<3,3>(3,3) = Rji;
                        Ji = (mat6::Identity() - 0.5*temp1)*temp2;
                    }
                    mat6 Jj{mat6::Zero()};
                    if(id_j > 0){
                        Jj = -(mat6::Identity() + 0.5*temp1);
                    }

                    for(int iRes{0}; iRes < 6 ; ++iRes){
                        for(int iDof{0}; iDof < 6 ; ++iDof) {
                            jacobians[0][iRes * 6 + iDof] = Ji(iRes,iDof)*poseCov;;
                        }
                    }
                    for(int iRes{0}; iRes < 6 ; ++iRes){
                        for(int iDof{0}; iDof < 6 ; ++iDof) {
                            jacobians[1][iRes * 6 + iDof] = Jj(iRes,iDof)*poseCov;;
                        }
                    }
                }

                return true;
            }
            mat3 R_ji_metric;
            vec3 t_ji_metric;
            mat3* Ri;
            vec3* ti;
            mat3* Rj;
            vec3* tj;
            mat3* Ri_inv;
            vec3* ti_inv;
            mat3* Rj_inv;
            vec3* tj_inv;
            dataType poseCov;
            size_t id_i;
            size_t id_j;
        };
    };
}

/*
namespace IDNav {

    class graphEdge{
    public:
        size_t index_i;
        size_t index_j;
        mat4 pose_ji;
        mat4 pose_i;
        mat4 pose_j;
        mat6 poseCov{mat6::Identity()};
        graphEdge(size_t& index_i, size_t& index_j, mat4& pose_i, mat4& pose_j,  mat4& pose_ji, mat6& poseCov):
                index_i(index_i), index_j(index_j) ,pose_ji(pose_ji),pose_i(pose_i),pose_j(pose_j),poseCov(poseCov){};
    };

    class PoseGraphOptimizer {

    public:
        void optimizePoseGraph(unordered_map<size_t, typeFrame> * keyframes, vector<graphEdge>& newEdges);

    };

    class evaluationCallbackPoseGraph : public ceres::EvaluationCallback {
    public:
        explicit evaluationCallbackPoseGraph(unordered_map<size_t, typeFrame>* keyframes,
                                             unordered_map<size_t,mat4>* T_init,
                                             unordered_map<size_t,mat4>* T_end,
                                             unordered_map<size_t,mat4>* Tinv_end,
                                             unordered_map<size_t , dataType*>* incPoses):
                keyframes(keyframes), T_init{T_init}, T_end{T_end}, Tinv_end{Tinv_end}, incPoses(incPoses){}
        virtual ~evaluationCallbackPoseGraph() {}
        virtual void PrepareForEvaluation(bool evaluate_jacobians,bool new_evaluation_point){
            if(new_evaluation_point){
                cout << " PrepareForEvaluation" <<endl;
                vec3 v,w{};
                dataType delta_t[3], delta_R[9];
                for(auto& [keyIndex,keyframe]: (*keyframes)){
                    cout << " PrepareForEvaluation 1" <<endl;
                    dataType* incPose = (*incPoses)[keyIndex];
                    cout << " PrepareForEvaluation 2" <<endl;
                    v << incPose[0],  incPose[1],  incPose[2];
                    cout << " PrepareForEvaluation 3" <<endl;
                    w << incPose[3],  incPose[4],  incPose[5];
                    cout << " PrepareForEvaluation 4" <<endl;
                    exp_lie(delta_t, delta_R,v,w);
                    mat4 exp_i = Rt_to_mat4(delta_t,delta_R);
                    (*T_end)[keyIndex] = exp_i * (*T_init)[keyIndex];
                    (*Tinv_end)[keyIndex] = (*T_end)[keyIndex].inverse();
                }
            }
        };
        unordered_map<size_t, typeFrame>* keyframes;
        unordered_map<size_t , mat4>* T_init;
        unordered_map<size_t , mat4>* T_end;
        unordered_map<size_t , mat4>* Tinv_end;
        unordered_map<size_t , dataType*>* incPoses;
    };

    class PoseGraphResidual : public ceres::SizedCostFunction<6,6,6> {
    public:

        PoseGraphResidual(mat4& T_ji_metric, mat4* Ti, mat4* Tj,  mat4* Ti_inv, mat4* Tj_inv, mat6* poseCov, size_t& index_i, size_t& index_j):
                T_ji_metric(T_ji_metric), Ti(Ti), Tj(Tj), Ti_inv(Ti_inv), Tj_inv(Tj_inv), poseCov(poseCov), index_i(index_i),index_j(index_j){}

        virtual ~PoseGraphResidual() {}

        virtual bool Evaluate(dataType const *const *param,
                              dataType *residuals,
                              dataType **jacobians) const {

            mat4 Tji = (*Tj)*(*Ti_inv);
            mat4 poseResidual = T_ji_metric * (*Ti) * (*Tj_inv);

            mat3_ R;
            vec3_ t;
            mat4_to_Rt(poseResidual,t,R);

            vec3 v,w;
            log_lie(v,w,t,R);

            for(int iTras{0}; iTras < 3; ++iTras)
                residuals[iTras] = v[iTras];

            for(int iRot{0}; iRot < 3; ++iRot)
                residuals[iRot + 3] = w[iRot];

            if(jacobians){
                mat3 vx = antisymmetricMatrix(v);
                mat3 wx = antisymmetricMatrix(w);

                mat6 temp1{mat6::Zero()};
                temp1.block<3,3>(0,0) = wx;
                temp1.block<3,3>(0,3) = vx;
                temp1.block<3,3>(3,3) = wx;

                mat6 Ji{mat6::Zero()};
                if(index_i > 0){
                    mat6 temp2{mat6::Zero()};
                    vec3 tji = Tji.block<3,1>(0,3);
                    temp2.block<3,3>(0,0) = Tji.block<3,3>(0,0);
                    temp2.block<3,3>(0,3) = antisymmetricMatrix(tji)*Tji.block<3,3>(0,0);
                    temp2.block<3,3>(3,3) = Tji.block<3,3>(0,0);
                    Ji = (mat6::Identity() - 0.5*temp1)*temp2;
                }
                mat6 Jj{mat6::Zero()};
                if(index_j > 0){
                    Jj = -(mat6::Identity() + 0.5*temp1);
                }

                for(int iRes{0}; iRes < 6 ; ++iRes){
                    for(int iDof{0}; iDof < 6 ; ++iDof) {
                        jacobians[0][iRes * 6 + iDof] = Ji(iRes,iDof);
                    }
                }
                for(int iRes{0}; iRes < 6 ; ++iRes){
                    for(int iDof{0}; iDof < 6 ; ++iDof) {
                        jacobians[1][iRes * 6 + iDof] = Jj(iRes,iDof);
                    }
                }
            }
            return true;
        }
        mat6* poseCov{};
        mat4 T_ji_metric{};
        mat4* Ti{};
        mat4* Tj{};
        mat4* Ti_inv{};
        mat4* Tj_inv{};
        size_t index_i;
        size_t index_j;
    };
}
 */
#endif //IDNAV_POSEGRAPHOPTIMIZER_H
