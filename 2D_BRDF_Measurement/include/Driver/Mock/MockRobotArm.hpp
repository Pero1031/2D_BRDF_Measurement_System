/*
* @file include/Driver/Mock/MockRobotArm.hpp    
* @brief Mock implementation of a robotic arm for testing purposes.
*/

#pragma once

#include <spdlog/spdlog.h>
#include "../IRobotArm.hpp"

namespace brdf::driver {

    class MockRobotArm : public IRobotArm {
    public:
		// constructor/destructor
		explicit MockRobotArm(const std::string& name);  // use name to identify the mock arm
        ~MockRobotArm() override = default;

        tl::expected<void, std::string> connect() override;
        void disconnect() override;
        bool isConnected() const override;

        tl::expected<void, std::string> moveJ(const JointAngles& target, double speed = 1.0) override;
        tl::expected<void, std::string> moveL(const RobotPose& target, double speed = 1.0) override;

        tl::expected<RobotPose, std::string> getCurrentPose() const override;
        tl::expected<JointAngles, std::string> getCurrentJoints() const override;

        void stop() override;
        void emergencyStop() override;

    private:
		std::string name_;  // name identifier for logging
        bool is_connected_ = false;

		// Internal state to simulate current pose and joint angles
        RobotPose current_pose_;
        JointAngles current_joints_;
    };

} // namespace brdf::driver