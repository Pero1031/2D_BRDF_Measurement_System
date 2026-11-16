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
#include "brdf/driver/rc8/Rc8Arm.hpp"
#include "brdf/driver/topcon/TopconImager.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// setup.toml の [devices] mode に応じて装置を組み立てる。
    ///  - "sim" : シミュレータ(SimulatedRig + モックのアーム 2 台 + 模擬分光放射計)
    ///  - "real": 実機(DENSO RC8 のアーム 2 台 + TOPCON の分光放射計)。アームと分光放射計のドライバが、それぞれ
    ///            -DBRDF_WITH_BCAP / -DBRDF_WITH_TOPCON を付けてビルドされていなければ NotImplemented を返す。
    ///            ここでは接続しない(接続は MeasurementSession が行う)。
    ///
    /// projector は、[calibration] projector = "camera-model" のときだけ設定される。
    /// "board" のときは空で、校正結果(calibration/homographies.csv)を読み込んだ側が設定する。
    ///
    /// kind が White のとき、シミュレータは標準拡散反射板(完全拡散、反射率は [sim.white])を測る。
    /// work_dir は、実機の分光放射計が一時ファイルを書く場所(real のとき必須)。cancel は、中止の旗(real のとき、アームに渡す)。
    Result<Devices> makeDevices(const io::SetupConfig& setup,
                                domain::MeasurementKind kind = domain::MeasurementKind::Sample,
                                const std::filesystem::path& work_dir = {}, std::shared_ptr<CancelToken> cancel = nullptr);

    /// setup.toml の [imager] と [spot_xml] から、実機の分光放射計(TOPCON)の設定を作る。
    /// work_dir は、レシピとスポット定義の XML を一時的に書く場所。
    driver::topcon::TopconImagerConfig makeTopconConfig(const io::SetupConfig& setup, const std::filesystem::path& work_dir);

    /// 実機の分光放射計を作る(まだ接続しない)。TOPCON の SDK なしでビルドした版(BRDF_WITH_TOPCON=OFF)では
    /// NotImplemented を返す。
    Result<std::shared_ptr<driver::ISpectralImager>> makeTopconImager(const io::SetupConfig& setup,
                                                                      const std::filesystem::path& work_dir);

    /// 実機のアームの左右。左 = 光源側(LightArm)、右 = 試料側(SampleArm)。
    enum class RealArm { Left, Right };

    /// setup.toml の [controllers](IP アドレスなど)と [arm.limits](可動域)から、実機のアーム 1 台の設定を作る。
    driver::rc8::Rc8ArmConfig makeRc8Config(const io::SetupConfig& setup, RealArm arm);

    /// 実機のアーム(DENSO RC8)を 1 台作る(まだ接続しない)。b-CAP の SDK なしでビルドした版(BRDF_WITH_BCAP=OFF)では
    /// NotImplemented を返す。
    /// cancel は、中止の要求を伝える旗(Ctrl+C など)。
    Result<std::shared_ptr<driver::IRobotArm>> makeRc8Arm(const io::SetupConfig& setup, RealArm arm,
                                                          std::shared_ptr<CancelToken> cancel = nullptr);

} // namespace brdf::app
