/*
* @file include/brdf/app/DeviceFactory.hpp
* @brief Builds the devices for the mode chosen in setup.toml.
*/

#pragma once

#include <filesystem>
#include <memory>

#include "brdf/app/Devices.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/driver/topcon/TopconImager.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// setup.toml の [devices] mode に応じて装置を組み立てる。
    ///  - "sim" : シミュレータ(SimulatedRig + モックのアーム 2 台 + 模擬分光放射計)
    ///  - "real": 実機。P6 で対応するまでは NotImplemented を返す。
    ///
    /// projector は、[calibration] projector = "camera-model" のときだけ設定される。
    /// "board" のときは空で、校正結果(calibration/homographies.csv)を読み込んだ側が設定する。
    ///
    /// kind が White のとき、シミュレータは標準拡散反射板(完全拡散、反射率は [sim.white])を測る。
    Result<Devices> makeDevices(const io::SetupConfig& setup,
                                domain::MeasurementKind kind = domain::MeasurementKind::Sample);

    /// setup.toml の [imager] と [spot_xml] から、実機の分光放射計(TOPCON)の設定を作る。
    /// work_dir は、レシピとスポット定義の XML を一時的に書く場所。
    driver::topcon::TopconImagerConfig makeTopconConfig(const io::SetupConfig& setup, const std::filesystem::path& work_dir);

    /// 実機の分光放射計を作る(まだ接続しない)。TOPCON の SDK なしでビルドした版(BRDF_WITH_TOPCON=OFF)では
    /// NotImplemented を返す。
    Result<std::shared_ptr<driver::ISpectralImager>> makeTopconImager(const io::SetupConfig& setup,
                                                                      const std::filesystem::path& work_dir);

} // namespace brdf::app
