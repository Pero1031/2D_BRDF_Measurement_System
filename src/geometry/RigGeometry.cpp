/*
* @file src/geometry/RigGeometry.cpp
* @brief World-space geometry of the rig.
*/

#include "brdf/geometry/RigGeometry.hpp"

#include <utility>

namespace brdf::geometry {

    RigGeometry::RigGeometry(RigSetup setup)
        : setup_(std::move(setup)), kinematics_(setup_.links) {
    }

    Isometry3 RigGeometry::leftTipPose(const Joints& left) const {
        Isometry3 base = Isometry3::Identity();
        base.translate(setup_.left_base_position);
        return base * kinematics_.tipPose(left);
    }

    Vec3 RigGeometry::lightPosition(const Joints& left) const {
        return leftTipPose(left) * setup_.light_offset;
    }

    Isometry3 RigGeometry::sampleHolderPose(const Joints& right) const {
        Isometry3 base = Isometry3::Identity();
        base.translate(setup_.right_base_position);

        Isometry3 tip = base * kinematics_.tipPose(right);
        tip.translate(Vec3(0.0, 0.0, setup_.sample_surface_offset));
        return tip;
    }

    SurfaceFrame RigGeometry::tipFrame(const Joints& right) const {
        const Isometry3 pose = sampleHolderPose(right);

        SurfaceFrame frame;
        frame.origin = pose.translation();
        frame.axes = pose.linear();
        return frame;
    }

    SurfaceFrame RigGeometry::paperFrame(const Joints& right) const {
        const Isometry3 pose = sampleHolderPose(right);
        const Mat3 r = pose.linear();

        SurfaceFrame frame;
        frame.origin = pose.translation();
        // 紙の X = 先端の +Y、紙の Y = 先端の -X、紙の Z = 先端の +Z。
        // (Y × -X = Z なので右手系のまま)
        frame.axes.col(0) = r.col(1);
        frame.axes.col(1) = -r.col(0);
        frame.axes.col(2) = r.col(2);
        return frame;
    }

} // namespace brdf::geometry
