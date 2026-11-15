/*
* @file src/sim/MockRobotArm.cpp
* @brief Simulated robot arm that moves inside a SimulatedRig.
*/

#include "brdf/sim/MockRobotArm.hpp"

#include <spdlog/spdlog.h>

#include <utility>

namespace brdf::sim {

    MockRobotArm::MockRobotArm(std::string name, std::shared_ptr<SimulatedRig> rig, ArmSide side)
        : name_(std::move(name)), rig_(std::move(rig)), side_(side) {}

    Status MockRobotArm::connect() {
        connected_ = true;
        spdlog::info("[{}] Connected (mock).", name_);
        return {};
    }

    void MockRobotArm::disconnect() {
        connected_ = false;
        spdlog::info("[{}] Disconnected.", name_);
    }

    Status MockRobotArm::moveJ(const geometry::Joints& target, Real speed) {
        if (!connected_) {
            return fail(ErrorCode::NotConnected, name_ + ": not connected");
        }
        if (!(speed > 0.0 && speed <= 1.0)) {
            return fail(ErrorCode::InvalidArgument, name_ + ": speed must be in (0, 1]");
        }

        Status result = rig_->moveTo(side_, target);
        if (result) {
            spdlog::debug("[{}] moveJ done (speed {:.2f})", name_, speed);
        } else {
            spdlog::warn("[{}] moveJ rejected: {}", name_, result.error().message);
        }
        return result;
    }

    Result<geometry::Joints> MockRobotArm::currentJoints() const {
        if (!connected_) {
            return fail(ErrorCode::NotConnected, name_ + ": not connected");
        }
        return rig_->joints(side_);
    }

    void MockRobotArm::stop() {
        spdlog::info("[{}] STOP.", name_);
    }

    void MockRobotArm::emergencyStop() {
        spdlog::critical("[{}] EMERGENCY STOP!", name_);
        connected_ = false;   // 再接続するまで動かせない
    }

} // namespace brdf::sim
