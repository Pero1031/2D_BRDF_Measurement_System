/*
* @file include/Driver/IRobotArm.hpp
* @brief Abstract interface for controlling a robotic arm.
*/

#pragma once

#include <Eigen/Dense>
#include <tl/expected.hpp>  
#include <string>

#include "Math/Algebra.hpp"

namespace brdf::driver {

	// Robot pose structure
    struct RobotPose {
        Vec3 position;    // [mm]
        Vec3 orientation; // [rad]
    };

    struct JointAngles {
        VecX angles; // [rad]
    };

    class IRobotArm {
    public:
        virtual ~IRobotArm() = default;

		// Connection management
        virtual tl::expected<void, std::string> connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

		// Movement commands
        virtual tl::expected<void, std::string> moveJ(
            const JointAngles& target,
            double speed = 1.0
        ) = 0;

        virtual tl::expected<void, std::string> moveL(
            const RobotPose& target,
            double speed = 1.0
        ) = 0;

		// State queries
        virtual tl::expected<RobotPose, std::string> getCurrentPose() const = 0;
        virtual tl::expected<JointAngles, std::string> getCurrentJoints() const = 0;

		// Emergency controls
        virtual void stop() = 0;
        virtual void emergencyStop() = 0;
    };

} // namespace brdf::driver