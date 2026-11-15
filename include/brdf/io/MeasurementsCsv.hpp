/*
* @file include/brdf/io/MeasurementsCsv.hpp
* @brief Reading back the measurements.csv that a measurement run wrote.
*/

#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::io {

    /// measurements.csv の 1 行 = 1 つの(ポーズ, 測定点)。
    struct MeasuredRow {
        std::string pose_id;
        int spot_id = 0;
        Vec2 sample_mm = Vec2::Zero();
        Vec2 pixel = Vec2::Zero();
        std::array<Real, 4> angles_deg{};   ///< θi, φi, θo, φo [deg](ファイルに書かれた値のまま)
        VecX spectrum;                      ///< 分光放射輝度(波長ごと)
    };

    struct MeasurementTable {
        VecX wavelengths;                   ///< 波長 [nm]
        std::vector<MeasuredRow> rows;
    };

    /// 測定が書き出した measurements.csv(app::measurementsCsv の出力)を読む。
    /// 列: pose,spot,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,wl_380,wl_381,...
    Result<MeasurementTable> parseMeasurementsCsv(std::string_view text, std::string_view origin = "measurements.csv");

    Result<MeasurementTable> loadMeasurementsCsv(const std::filesystem::path& path);

} // namespace brdf::io
