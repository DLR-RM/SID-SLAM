//
// Created by afontan on 21/10/21.
//

#include "../include/PoseGraphOptimizer.h"

void IDNav::PoseGraphOptimizer::optimizePoseGraph(unordered_map<size_t, typeFrame> * keyframes) {

    //buildProfiler.begin();

    ceres::Solver::Summary poseGraphSummary{};
    ceres::Problem::Options problemOptions;
    ceres::Solver::Options poseGraphOptions{};
    //poseGraphOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //poseGraphOptions.minimizer_type = ceres::TRUST_REGION;
    poseGraphOptions.minimizer_progress_to_stdout = false;
    poseGraphOptions.ceres::Solver::Options::update_state_every_iteration = true;
    poseGraphOptions.use_inner_iterations = false;
    poseGraphOptions.check_gradients = false;
    //poseGraphOptions.max_num_iterations = 3;

    //poseGraphOptions.max_num_iterations = 1;

    unordered_map<size_t, dataType*> deltaPoses{};
    unordered_map<size_t , mat3> Rcw{};
    unordered_map<size_t , mat3> Rwc{};
    unordered_map<size_t , vec3> tcw{};
    unordered_map<size_t , vec3> twc{};

    for(auto& [keyId,keyframe]: *keyframes){
        twc.insert({keyId,keyframe->pose.twc});
        Rwc.insert({keyId,keyframe->pose.Rwc});
        tcw.insert({keyId,keyframe->pose.tcw});
        Rcw.insert({keyId,keyframe->pose.Rcw});
        //if(keyId > 0){
            deltaPoses[keyId] = new dataType[6]{0.0,0.0,0.0,0.0,0.0,0.0};
        //}
    }

    auto *eval_function = new PoseGraphOptimizer::evaluationCallbackPoseGraph(
            keyframes,
            &deltaPoses,
            &tcw , &Rcw,
            &twc , &Rwc
    );

    problemOptions.evaluation_callback = eval_function;
    ceres::Problem problemPoseGraph{problemOptions};

    mat3 Rji{};
    vec3 tji{};
    dataType poseCov{1.0};
    //unordered_map<size_t,unordered_map<size_t,bool>> edges;
    for (auto& [idKey_i, keyframe_i]: *keyframes) {
        //edges[idKey_i].insert({idKey_i,true});
        for (auto& [idKey_j,cov]: keyframe_i->keyframes) {
            //if(edges[idKey_i].find(idKey_j) != edges[idKey_i].end()) continue;
            //edges[idKey_i].insert({idKey_j,true});
            Rji = cov.Tji.block<3,3>(0,0);
            tji = cov.Tji.block<3,1>(0,3);
            poseCov = cov.inliersRatio;

            /*if((idKey_i == database_id)&&(idKey_j == query_id)){
                poseCov = 0.0001;
            }
            if((idKey_j == database_id)&&(idKey_i == query_id)){
                poseCov = 0.0001;
            }*/
            ceres::CostFunction *cost_function = new PoseGraphResidual(Rji,tji,
                                                                       &Rwc[idKey_i], &twc[idKey_i],
                                                                       &Rwc[idKey_j], &twc[idKey_j],
                                                                       &Rcw[idKey_i], &tcw[idKey_i],
                                                                       &Rcw[idKey_j], &tcw[idKey_j],
                                                                       poseCov,
                                                                       idKey_i,idKey_j);
            problemPoseGraph.AddResidualBlock(cost_function, nullptr , deltaPoses[idKey_i], deltaPoses[idKey_j]);
        }
    }
    //buildProfiler.end();
    //solveProfiler.begin();
    ceres::Solve(poseGraphOptions, &problemPoseGraph, &poseGraphSummary);
    //solveProfiler.end();
    cout << poseGraphSummary.FullReport() << endl;
    //wf(".");

    vec3 v{}, w{}, delta_t{};
    mat3 delta_R{};
    for (auto&[keyId, keyframe]: (*keyframes)) {
        if (keyId == 0) continue;
        dataType* deltaPose= (deltaPoses)[keyId];
        v << deltaPose[0], deltaPose[1], deltaPose[2];
        w << deltaPose[3], deltaPose[4], deltaPose[5];
        cout << " keyId = "<< keyId << " ---- > " << v.transpose() << " " << w.transpose() << endl;
        exp_lie(delta_t, delta_R, v, w);
        mat3 Rwc_updated = delta_R * keyframe->pose.Rwc;
        vec3 twc_updated = delta_R * keyframe->pose.twc + delta_t;
        keyframe->pose.set_T_wc(twc_updated,Rwc_updated);
        keyframe->uv_2_XYZ(keyframe->hgp);
        delete [] deltaPose;
    }
}


/*
void IDNav::PoseGraphOptimizer::optimizePoseGraph(unordered_map<size_t, typeFrame> * keyframes, vector<graphEdge>& newEdges){


    mat4 Tji;
    mat6 poseCov{mat6::Identity()};
    size_t index_i,index_j;
    unordered_map<size_t , dataType*> poseUpdate{};
    unordered_map<size_t , mat4> T_init{};
    unordered_map<size_t , mat4> T_end{};
    unordered_map<size_t , mat4> Tinv_end{};
    cout << "LLega aqui 1"<< endl;
    mat4 Ti;
    for (auto& [index_i, keyframe_i]: *keyframes) {
        Ti = Rt_to_mat4(keyframe_i->pose.twc,keyframe_i->pose.Rwc);
        T_init.insert({index_i,Ti});
        T_end.insert({index_i,Ti});
        Tinv_end.insert({index_i,Ti.inverse()});
        poseUpdate[index_i] = new dataType[6]{0.0,0.0,0.0,0.0,0.0,0.0};
        //std::fill((poseUpdate[index_i])[0], (poseUpdate[index_i])[5] , 0);
    }
    cout << "LLega aqui 2"<< endl;
    auto *eval_function = new evaluationCallbackPoseGraph(keyframes, &T_init, &T_end, &Tinv_end, &poseUpdate);
    problemOptions.evaluation_callback = eval_function;
    cout << "LLega aqui 3"<< endl;
    ceres::Problem problemPoseGraph{problemOptions};
    cout << "LLega aqui 4"<< endl;

    cout << "LLega aqui 5"<< endl;
    for (auto& [indexKey_i, keyframe_i]: *keyframes) {
        index_i = indexKey_i;
        cout << "LLega aqui 6"<< endl;
        for (auto& [keyframe_j,cov]: keyframe_i->keyframes) {
            index_j = keyframe_j->keyIndex;
            Tji = T_init[index_j]*Tinv_end[index_i];
            cout << "LLega aqui 7"<< endl;

            ceres::CostFunction *cost_function = new PoseGraphResidual(Tji, &(T_end[index_i]), &(T_end[index_j]),
                                                                       &(Tinv_end[index_i]), &(Tinv_end[index_j]),
                                                                       &poseCov,index_i,index_j);
            //cout << index_i << " " << index_j << endl;
            cout << "LLega aqui 8"<< endl;

            problemPoseGraph.AddResidualBlock(cost_function, nullptr , poseUpdate[index_i], poseUpdate[index_j]);
            cout << "LLega aqui 9"<< endl;

        }
    }
    cout << "LLega aqui 10"<< endl;

    for(int i = 0; i < 2; ++i){
        for (graphEdge& edge: newEdges) {
            cout << "LLega aqui 11"<< endl;
            index_i = edge.index_i;
            index_j = edge.index_j;
            ceres::CostFunction *cost_function = new PoseGraphResidual(edge.pose_ji,&(T_end[index_i]), &(T_end[index_j]),
                                                                       &(Tinv_end[index_i]), &(Tinv_end[index_j]),
                                                                       &poseCov,index_i,index_j);
            problemPoseGraph.AddResidualBlock(cost_function, nullptr , poseUpdate[index_i] , poseUpdate[index_j]);
        }
    }
    cout << "LLega aqui 12"<< endl;

    //wf(".");
    ceres::Solve(poseGraphOptions, &problemPoseGraph, &poseGraphSummary);
    cout << "LLega aqui 13"<< endl;

    //wf(".");
    //cout << poseGraphSummary.FullReport() << endl;
    //cout << " Pose updates = "<< keyframes->size() << endl;
    //int i = 0;
    for(auto& [indexKey, keyframe]: *keyframes){
        //cout << "indexKey = " << indexKey<< " , " << i << " num links = " << keyframe->keyframes.size() << endl;
        //cout << poseUpdate[i][0]<< " " << poseUpdate[i][1]<< " " << poseUpdate[i][2]<< " " <<
        //        poseUpdate[i][3]<< " " << poseUpdate[i][4]<< " " << poseUpdate[i][5]<< " " <<endl;
        vec3 v_{poseUpdate[indexKey][0], poseUpdate[indexKey][1], poseUpdate[indexKey][2]},
             w_{poseUpdate[indexKey][3], poseUpdate[indexKey][4], poseUpdate[indexKey][5]};
        dataType delta_t[3], delta_R[9];
        exp_lie(delta_t, delta_R,v_,w_);
        //cout << v_.transpose()<< endl;
        //cout << w_.transpose()<< endl;

        //showVector3(delta_t);
        //showMatrix3(delta_R);
        keyframe->pose.update_T_wc_left(delta_t,delta_R);
        keyframe->uv_2_XYZ(keyframe->hgp);
        //++i;
    }
    //wf(".");
    //if(poseGraphSummary.IsSolutionUsable()){
    //for(int iDOF{0}; iDOF < 6; ++iDOF){
    //    poseGuess_[iDOF] = poseGuess0_[iDOF];
    //}
    //}

    //delete eval_function;
}*/