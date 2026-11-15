/*
* @file src/geometry/RigGeometry.cpp
* @brief World-space geometry of the rig.
*/

#include "brdf/geometry/RigGeometry.hpp"

#include <algorithm>
#include <limits>
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

    namespace {
        std::vector<Segment> polyline(const std::array<Vec3, 6>& p, const Vec3& base,
                                      const Vec3& extra_end) {
            std::vector<Segment> segments;
            for (int i = 0; i + 1 < 6; ++i) {
                segments.push_back({base + p[i], base + p[i + 1]});
            }
            segments.push_back({base + p[5], extra_end});   // 先端 → 光源 / 試料面
            return segments;
        }
    }

    std::vector<Segment> RigGeometry::leftArmSegments(const Joints& left) const {
        return polyline(kinematics_.linkPoints(left), setup_.left_base_position, lightPosition(left));
    }

    std::vector<Segment> RigGeometry::rightArmSegments(const Joints& right) const {
        return polyline(kinematics_.linkPoints(right), setup_.right_base_position,
                        sampleHolderPose(right).translation());
    }

    Real RigGeometry::minArmDistance(const Joints& left, const Joints& right) const {
        const auto ls = leftArmSegments(left);
        const auto rs = rightArmSegments(right);

        Real best = std::numeric_limits<Real>::max();
        for (const Segment& a : ls) {
            for (const Segment& b : rs) {
                best = std::min(best, segmentDistance(a, b));
            }
        }
        return best;
    }

} // namespace brdf::geometry
