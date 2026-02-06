#pragma once

#include <Eigen/Dense>
#include <tl/expected.hpp>  // ← これを使う
#include <string>

namespace brdf::driver {

    struct RobotPose {
        Eigen::Vector3d position;    // [mm]
        Eigen::Vector3d orientation; // [rad]
    };

    struct JointAngles {
        Eigen::VectorXd angles; // [rad]
    };

    class IRobotArm {
    public:
        virtual ~IRobotArm() = default;

        // tl::expected を使用
        virtual tl::expected<void, std::string> connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

        virtual tl::expected<void, std::string> moveJ(
            const JointAngles& target,
            double speed = 1.0
        ) = 0;

        virtual tl::expected<void, std::string> moveL(
            const RobotPose& target,
            double speed = 1.0
        ) = 0;

        virtual tl::expected<RobotPose, std::string> getCurrentPose() const = 0;
        virtual tl::expected<JointAngles, std::string> getCurrentJoints() const = 0;

        virtual void stop() = 0;
        virtual void emergencyStop() = 0;
    };

} // namespace brdf::driver