/*
* @file include/brdf/app/DeviceFactory.hpp
* @brief Builds the devices for the mode chosen in setup.toml.
*/

#pragma once

#include "brdf/app/Devices.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// setup.toml の [devices] mode に応じて装置を組み立てる。
    ///  - "sim" : シミュレータ(SimulatedRig + モックのアーム 2 台 + 模擬分光放射計)
    ///  - "real": 実機。P6 で対応するまでは NotImplemented を返す。
    Result<Devices> makeDevices(const io::SetupConfig& setup);

} // namespace brdf::app
