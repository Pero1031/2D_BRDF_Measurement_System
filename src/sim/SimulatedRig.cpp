/*
* @file src/sim/SimulatedRig.cpp
* @brief The simulated world shared by the mock arms and the mock imager.
*/

#include "brdf/sim/SimulatedRig.hpp"

#include <sstream>
#include <utility>

namespace brdf::sim {

    namespace {
        // 起動時の姿勢: 両アームとも腕を前へ水平に伸ばした姿勢。
        geometry::Joints homePose() { return geometry::jointsFromDegrees(0, 0, 90, 0, 0, 0); }
    }

    PinholeCameraSetup SimulatedRig::defaultCamera(const geometry::RigGeometry& geometry) {
        PinholeCameraSetup cam;
        cam.position = geometry.cameraPosition();
        // 旧システムの測定でよく使う姿勢のときの試料位置を、カメラの向く先にする。
        cam.target = geometry.sampleHolderPose(
            geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0)).translation();
        return cam;
    }

    SimulatedRig::SimulatedRig(geometry::RigSetup setup)
        : geometry_(std::move(setup)),
          camera_(defaultCamera(geometry_)),
          left_(homePose()),
          right_(homePose()) {}

    SimulatedRig::SimulatedRig(geometry::RigSetup setup, PinholeCameraSetup camera)
        : geometry_(std::move(setup)),
          camera_(camera),
          left_(homePose()),
          right_(homePose()) {}

    const geometry::Joints& SimulatedRig::joints(ArmSide side) const {
        return side == ArmSide::Left ? left_ : right_;
    }

    Status SimulatedRig::moveTo(ArmSide side, const geometry::Joints& target) {
        const auto& setup = geometry_.setup();

        const int bad = setup.joint_limits.firstViolation(target);
        if (bad >= 0) {
            std::ostringstream msg;
            msg << toString(side) << " arm: J" << (bad + 1) << " = "
                << toDegrees(target(bad)) << " deg is outside the joint limits ["
                << toDegrees(setup.joint_limits.lower(bad)) << ", "
                << toDegrees(setup.joint_limits.upper(bad)) << "] deg";
            return fail(ErrorCode::JointLimit, msg.str());
        }

        const geometry::Joints& left  = side == ArmSide::Left  ? target : left_;
        const geometry::Joints& right = side == ArmSide::Right ? target : right_;

        const Real distance = geometry_.minArmDistance(left, right);
        if (distance < 2.0 * setup.arm_radius) {
            std::ostringstream msg;
            msg << toString(side) << " arm: the target pose brings the arms to " << distance
                << " mm (limit " << 2.0 * setup.arm_radius << " mm)";
            return fail(ErrorCode::Collision, msg.str());
        }

        (side == ArmSide::Left ? left_ : right_) = target;
        return {};
    }

    Vec3 SimulatedRig::lightPosition() const {
        return geometry_.lightPosition(left_);
    }

    geometry::SurfaceFrame SimulatedRig::sampleFrame() const {
        return geometry_.paperFrame(right_);
    }

} // namespace brdf::sim
