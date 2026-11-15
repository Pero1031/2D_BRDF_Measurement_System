/*
* @file include/brdf/io/SetupConfig.hpp
* @brief setup.toml: the physical layout of the rig and the device mode.
*/

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "brdf/core/Error.hpp"
#include "brdf/geometry/RigSetup.hpp"
#include "brdf/sim/MockSpectralImager.hpp"

namespace brdf::io {

    /// 使うデバイス。Sim = シミュレーション、Real = 実機(P6 で対応)。
    enum class DeviceMode { Sim, Real };

    inline const char* toString(DeviceMode mode) { return mode == DeviceMode::Sim ? "sim" : "real"; }

    /// シミュレーションのカメラ。
    struct SimCameraConfig {
        int image_width = 1376;
        int image_height = 1024;
        Real pixels_per_mm = 1.81;
        std::optional<Vec3> target;   ///< 省略すると、典型的な測定姿勢での試料位置を向く
    };

    /// シミュレーションで「測られる試料」の BRDF。
    struct SimSampleConfig {
        std::string model = "lambert";   ///< "lambert" または "phong"
        Real peak_nm = 550.0;            ///< 分光反射率の山の中心 [nm]
        Real width_nm = 60.0;            ///< 山の幅 [nm]
        Real peak = 0.8;                 ///< 山の高さ
        Real base = 0.05;                ///< 底上げ
        Real specular = 0.5;             ///< phong: 鏡面成分の強さ
        Real shininess = 50.0;           ///< phong: 鋭さ
    };

    /// setup.toml の内容。旧システムでヘッダに直書きしていた装置の定数をここに集める。
    struct SetupConfig {
        DeviceMode mode = DeviceMode::Sim;
        geometry::RigSetup rig;
        SimCameraConfig camera;
        sim::MockImagerOptions imager;
        SimSampleConfig sample;
    };

    /// TOML の文字列から読む。origin はエラーメッセージに出すファイル名。
    Result<SetupConfig> parseSetup(std::string_view text, std::string_view origin = "setup.toml");

    /// ファイルから読む。
    Result<SetupConfig> loadSetup(const std::filesystem::path& path);

    /// コメント付きの TOML に書き出す。既定値の SetupConfig を書けば、そのままひな型になる。
    std::string toToml(const SetupConfig& setup);

} // namespace brdf::io
