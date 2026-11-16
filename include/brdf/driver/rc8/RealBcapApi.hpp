/*
* @file include/brdf/driver/rc8/RealBcapApi.hpp
* @brief IBcapApi implemented with the DENSO b-CAP client library (the only file that touches the library is RealBcapApi.cpp).
*
* Built only when BRDF_WITH_BCAP is ON. The class holds no state: every handle is passed in by the caller (Rc8Arm).
*/

#pragma once

#include "brdf/driver/rc8/BcapApi.hpp"

namespace brdf::driver::rc8 {

    class RealBcapApi final : public IBcapApi {
    public:
        Result<int> open(const std::string& address, int timeout_ms, int retry) override;
        Status serviceStart(int fd) override;
        Result<std::uint32_t> controllerConnect(int fd, const std::string& provider, const std::string& machine) override;
        Result<BcapRobot> getRobot(int fd, std::uint32_t controller, const std::string& name) override;

        Status releaseRobot(const BcapRobot& robot) override;
        Status controllerDisconnect(int fd, std::uint32_t controller) override;
        Status serviceStop(int fd) override;
        Status close(int fd) override;
        Status setTimeout(int fd, int timeout_ms) override;

        Status takeArm(const BcapRobot& robot) override;
        Status giveArm(const BcapRobot& robot) override;
        Status setMotor(const BcapRobot& robot, bool on) override;
        Status setExtSpeed(const BcapRobot& robot, double percent) override;
        Status moveJoint(const BcapRobot& robot, const Degrees6& target) override;
        Status moveJointAsync(const BcapRobot& robot, const Degrees6& target) override;
        Result<Degrees6> currentJoint(const BcapRobot& robot) override;
        Status halt(const BcapRobot& robot) override;
    };

} // namespace brdf::driver::rc8
