
#include "../../include/Optimizer/OptimizerCeres.h"

void IDNav::OptimizerCeres::geoBA(std::unordered_map< typePt,std::unordered_map<size_t,Point_<dataType>>>& obs_hgp_,
                                  std::unordered_map< typeFt,std::unordered_map<size_t,Point_<dataType>>>& obs_features_){

    ///////////////////////////////////////////////////////////////// Add poses
    fixedKeyframes.clear();
    fixedKeyframes.insert({(*keyframes)[0]->keyId,true});

    for(auto[index,deltaPose_i] : deltaPoses){
        delete [] deltaPose_i;
    }
    deltaPoses.clear();
    twc.clear();
    Rwc.clear();
    tcw.clear();
    Rcw.clear();

    for(auto& [keyId,keyframe]: *keyframes){
        twc.insert({keyId,keyframe->pose.twc});
        Rwc.insert({keyId,keyframe->pose.Rwc});
        tcw.insert({keyId,keyframe->pose.tcw});
        Rcw.insert({keyId,keyframe->pose.Rcw});
        if (fixedKeyframes.find(keyId) == fixedKeyframes.end()){
            deltaPoses[keyId] = new dataType[6]{0.0,0.0,0.0,0.0,0.0,0.0};
        }
    }
    /////////////////////////////////////////////////////////////////
    Rrel.clear();
    updateRelativeRotations();

    ///////////////////////////////////////////////////////////////// Add depths
    deltaLambdaHgp.clear();
    deltaLambdaFeatures.clear();
    for(auto& [pt,projections]: obs_hgp_){
        deltaLambdaHgp[pt] = pt->lambdaRef;
    }
    for(auto& [ft,projections]: obs_features_){
        deltaLambdaFeatures[ft] = ft->lambdaRef;
    }
    ///////////////////////////////////////////////////////////////// addProblemOptions
    ceres::Problem::Options problemOptions_geoBA{};
    auto *eval_function = new OptimizerCeres::evaluationCallback_geoBA(
            keyframes,
            &deltaPoses,
            &tcw , &Rcw,
            &twc , &Rwc,
            &fixedKeyframes,
            &Rrel,
            &obs_hgp_,
            &obs_features_
    );
    problemOptions_geoBA.evaluation_callback = eval_function;

    ceres::Problem problem_geoBA{problemOptions_geoBA};
    ceres::Solver::Summary summary_geoBA{};

    typeProj_ft proj{};
    typeFrame hostKey;
    typeFrame projKey;
    KeyPoint obs;

    for(auto& [pt,projections]: obs_hgp_){
        if(pt->idKey == 0) continue;
        hostKey =  (*keyframes)[pt->idKey];
        for(auto& [projKeyId,pt_]: projections){
            if(projKeyId == 0) continue;
            projKey = (*keyframes)[projKeyId];
            obs.pt = pt_;
            proj.reset( new IDNav::Projection_ft(
                    pt, hostKey ,projKey,
                    &(Rrel[std::make_pair(hostKey,projKey)]),
                    obs));
            ceres::CostFunction *cost_function = new ReprojResidual_poseOpt_geoBA(proj,
                                                                            &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            //problem_geoBA.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                           //deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
            problem_geoBA.AddResidualBlock(cost_function, nullptr,
                                           deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
        }
    }

    for(auto& [ft,projections]: obs_features_){
        if(ft->idKey == 0) continue;
        hostKey =  (*keyframes)[ft->idKey];
        for(auto& [projKeyId,pt_]: projections){
            if(projKeyId == 0) continue;
            projKey = (*keyframes)[projKeyId];
            obs.pt = pt_;
            proj.reset( new IDNav::Projection_ft(
                    ft, hostKey ,projKey,
                    &(Rrel[std::make_pair(hostKey,projKey)]),
                    obs));

            ceres::CostFunction *cost_function = new ReprojResidual_poseOpt(proj,
                                                                            &(tcw[proj->projKeyId]),&(Rcw[proj->projKeyId]));
            //problem_geoBA.AddResidualBlock(cost_function, new ceres::CauchyLoss(tstudentReproj),
                                           //deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
            problem_geoBA.AddResidualBlock(cost_function, nullptr,
                                           deltaPoses[proj->refKeyId], deltaPoses[proj->projKeyId]);
        }
    }

    ceres::Solver::Options solverOptions_geoBA{};
    solverOptions_geoBA.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solverOptions_geoBA.minimizer_type = ceres::TRUST_REGION;
    solverOptions_geoBA.minimizer_progress_to_stdout = false;
    solverOptions_geoBA.ceres::Solver::Options::update_state_every_iteration = true;
    solverOptions_geoBA.use_inner_iterations = false;
    solverOptions_geoBA.check_gradients = false;
    solverOptions_geoBA.num_threads = numThreads;
    ceres::Solve(solverOptions_geoBA, &problem_geoBA, &summary_geoBA);
    cout << summary_geoBA.FullReport()<< endl;
    //terminate();
}

void IDNav::OptimizerCeres::optimizeDepthGeo(){
    for (auto&[ft, lambdaRef]: deltaLambdaFeatures) {
        int numObs = 1;
        dataType lambdaRefCum = ft->lambdaRef;
        dataType lambdaRefCov;
        for (auto &obs: featureObservations[ft]) {
            if (obs->thereIsDepthObs) {
                ++numObs;
                lambdaRefCum += obs->lambdaRefObs;
            }
        }
        lambdaRefCum /= numObs;
        lambdaRefCov = 1.0 / (100000.0 * numObs);
        deltaLambdaFeatures[ft] = lambdaRefCum;
    }
    //cout << depthObservations_[0]->ft->lambdaRef << " = " << lambdaRef << endl;
    //depthObservations_[0]->ft->updateLambdaRef(lambdaRef,lambdaRefCov);
    //cout << " " << endl;
}

void IDNav::OptimizerCeres::optimizeDepthPhoto(){
    for (auto&[pt, lambdaRef]: deltaLambdaHgp) {
        int numObs = 1;
        dataType lambdaRefCum = pt->lambdaRef;
        dataType lambdaRefCov = pt->lambdaRef;
        for (auto &obs: hgpObservations[pt]) {
            if (obs->thereIsDepthObs) {
                ++numObs;
                lambdaRefCum += obs->lambdaRefObs;
            }
        }
        lambdaRefCum /= numObs;
        lambdaRefCov = 1.0 / (100000.0 * numObs);
        deltaLambdaHgp[pt] = lambdaRefCum;
    }
}