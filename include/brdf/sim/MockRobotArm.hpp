/*
* @file include/brdf/sim/MockRobotArm.hpp
* @brief Simulated robot arm that moves inside a SimulatedRig.
*/

#pragma once

#include <memory>
#include <string>

#include "brdf/driver/IRobotArm.hpp"
#include "brdf/sim/SimulatedRig.hpp"

namespace brdf::sim {

    /// シミュレーション用のロボットアーム。SimulatedRig の左右どちらかのアームを動かす。
    /// 可動域外の指令と、もう一方のアームとの干渉は、実機と同じようにエラーになる。
    class MockRobotArm : public driver::IRobotArm {
    public:
        /// name はログの識別用。
        MockRobotArm(std::string name, std::shared_ptr<SimulatedRig> rig, ArmSide side);

        Status connect() override;
        void disconnect() override;
        bool isConnected() const override { return connected_; }

        Status moveJ(const geometry::Joints& target, Real speed) override;
        Result<geometry::Joints> currentJoints() const override;

        void stop() override;
        void emergencyStop() override;

    private:
        std::string name_;
        std::shared_ptr<SimulatedRig> rig_;
        ArmSide side_;
        bool connected_ = false;
    };

} // namespace brdf::sim
