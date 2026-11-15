/*
* @file include/brdf/app/Devices.hpp
* @brief The set of devices a measurement session talks to.
*/

#pragma once

#include <memory>

#include "brdf/app/SpotProjector.hpp"
#include "brdf/driver/IRobotArm.hpp"
#include "brdf/driver/ISpectralImager.hpp"

namespace brdf::app {

    /// 測定セッションが使う装置一式。実機でもシミュレーションでも、同じインターフェースで渡す。
    struct Devices {
        std::shared_ptr<driver::IRobotArm> left_arm;       ///< 光源側
        std::shared_ptr<driver::IRobotArm> right_arm;      ///< 試料側
        std::shared_ptr<driver::ISpectralImager> imager;
        std::shared_ptr<SpotProjector> projector;          ///< 試料面の点 → 画像上の位置
    };

} // namespace brdf::app
