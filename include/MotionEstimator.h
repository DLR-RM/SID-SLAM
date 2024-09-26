//
// Created by font_al on 7/20/21.
//

#ifndef IDNAV_MOTIONESTIMATOR_H
#define IDNAV_MOTIONESTIMATOR_H

#include "Frame.h"

#define NUM_COEFFS 2
namespace IDNav {

    class MotionEstimator {

    private:
        dataType freq_seconds{0.1};
        vector<Pose> poses{};
        mat3 R{mat3::Identity()};
        vec3 t{vec3::Zero()};
        vec3 v{vec3::Zero()},w{vec3::Zero()};
    public:

        explicit MotionEstimator(const dataType& freq_seconds_):freq_seconds(freq_seconds_){}

        void getSpeedEstimate(vec6& speedEst , mat6& speedEstCovInv, const dataType& imgTs){
#ifdef COUT_COMPLETE_PIPELINE
            cout << CYAN_COUT << "    [MotionEstimator] getSpeedEstimate()"<< RESET_COUT <<endl;
#endif
            speedEst =  vec6::Zero();
            speedEstCovInv = mat6::Zero();
            if(poses.size() == 2){
                if((imgTs - poses[1].ts) > 0.1) return;
                R = poses[1].Rcw * poses[0].Rwc;
                t = poses[1].Rcw*poses[0].twc + poses[1].tcw;
                log_lie(v,w,t,R);
                /*speedEst.segment(0,3) = v;
                speedEst.segment(3,3) = w;
                speedEstCovInv(0,0) = 500.0;
                speedEstCovInv(1,1) = 500.0;
                speedEstCovInv(2,2) = 500.0;*/
            }
        }

        bool kinematicModelHolds(Pose& pose_){
            poses.push_back(pose_);
            if(poses.size() > 2){
                poses.erase(poses.begin());
                if((poses[1].ts - poses[0].ts) > 0.1){
                    poses.erase(poses.begin());
                }
            }

            bool kinematicModelSuccesful{true};
            if(!kinematicModelSuccesful){
                cout << RED_COUT << "    [MotionEstimator]: << Kinematic model failed " << RESET_COUT << endl;
            }
#ifdef COUT_PIPELINE2
            else{
                cout << GREEN_COUT << "    [MotionEstimator]: Kinematic model holds" << RESET_COUT << endl;
            }
#endif
            return kinematicModelSuccesful;
        }

        void reset(vec6& speed , mat6& speedCovInv){
            speed = vec6::Zero();
            speedCovInv = mat6::Zero();
            poses.clear();
#ifdef COUT_PIPELINE2
            cout << RED_COUT << "    [MotionEstimator] reset()" << RESET_COUT << endl;
#endif
        }
    };
}
#endif //IDNAV_MOTIONESTIMATOR_H
