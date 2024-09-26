//
// Created by font_al on 1/31/20.
//

#ifndef IDNAV_POSE_H
#define IDNAV_POSE_H

#include "Pt.h"

namespace IDNav{

    class Pose {
    public:
        Pose() = default;
        /*Pose(const Pose& pose_){
            this->copyPoseFrom(pose_);
            ts = pose_.ts;
        }*/

        dataType ts{0};

        vec3 tcw{vec3::Zero()};
        mat3 Rcw{mat3::Identity()};
        vec3 twc{vec3::Zero()};
        mat3 Rwc{mat3::Identity()};

    public:
        void xynlambda_2_XYZ(typeIdNavPoint pt);
        void xynlambda_2_XYZ(vecPt& hgp);
        void xynlambda_2_XYZ(vecFt& features);
        void XYZ_2_xynlambda(typeIdNavPoint pt);
        void XYZ_2_xynlambda(vecPt& hgp);
        void XYZ_2_xynlambda(vecFt& features);

    public:
        void set_T_cw(const vec3& tcw_, const mat3& Rcw_);
        void set_T_cw(const mat4& Tcw);
        void update_T_cw_left(vec6& delta_T);
        void set_T_wc(const vec3& twc_, const mat3& Rwc_);
        void set_T_wc(const mat4& Twc);
        void update_T_cw_left(const vec3& delta_t, const mat3& delta_R);
        void update_T_wc_left(const vec3& delta_t, const mat3& delta_R);
        void update_T_cw_right(const vec3& delta_t,const mat3& delta_R);

        void copyPoseFrom(const Pose& pose_);
        void copyStampedPoseFrom(const Pose& pose_);

        void print();
        IDNav::vec6 relativeMovement(const Pose& pose_) const;
        void relativeMovement(vec3&  trel_, mat3& Rrel, const Pose& pose_) const;
        IDNav::mat4 relMovement(const Pose& pose_) const;
    };
}



#endif //IDNAV_POSE_H
