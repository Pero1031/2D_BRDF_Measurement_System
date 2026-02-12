/*
* @file src/Driver/Mock/MockRobotArm.cpp
* @brief Mock implementation of a robotic arm for testing purposes.
*/

#include <iostream>

#include "Driver/Mock/MockRobotArm.hpp"

namespace brdf::driver {

    MockRobotArm::MockRobotArm(const std::string& name)
        : name_(name) {
		// Initialize mock state with default values 0
        current_pose_ = { Vec3::Zero(), Vec3::Zero() };

		// six joints initialized to zero angles
        current_joints_ = { VecX::Zero(6) };
    }

    tl::expected<void, std::string> MockRobotArm::connect() {
        spdlog::info("[{}] Connecting...", name_);
        is_connected_ = true;
        spdlog::info("[{}] Connected successfully.", name_);
        return {};
    }

    void MockRobotArm::disconnect() {
        spdlog::info("[{}] Disconnecting...", name_);
        is_connected_ = false;
    }

    bool MockRobotArm::isConnected() const {
        return is_connected_;
    }

    tl::expected<void, std::string> MockRobotArm::moveJ(const JointAngles& target, double speed) {
        if (!is_connected_) {
            return tl::unexpected("Not connected");
        }

		// output target joint angles for logging
        spdlog::info("[{}] Moving J (Speed: {:.2f}). Target angles set.", name_, speed);

		current_joints_ = target; // update internal state
        return {};
    }

    tl::expected<void, std::string> MockRobotArm::moveL(const RobotPose& target, double speed) {
        if (!is_connected_) {
            return tl::unexpected("Not connected");
        }

        spdlog::info("[{}] Moving L (Speed: {:.2f}). Target pose: Pos({:.2f}, {:.2f}, {:.2f})",
            name_, speed, target.position.x(), target.position.y(), target.position.z());

		current_pose_ = target; // update internal state
        return {};
    }

    tl::expected<RobotPose, std::string> MockRobotArm::getCurrentPose() const {
        if (!is_connected_) {
            return tl::unexpected("Not connected");
        }
        return current_pose_;
    }

    tl::expected<JointAngles, std::string> MockRobotArm::getCurrentJoints() const {
        if (!is_connected_) {
            return tl::unexpected("Not connected");
        }
        return current_joints_;
    }

    void MockRobotArm::stop() {
        spdlog::info("[{}] STOP command received.", name_);
    }

    void MockRobotArm::emergencyStop() {
        spdlog::critical("[{}] EMERGENCY STOP triggered!", name_);
        is_connected_ = false; 
    }

} // namespace brdf::driver