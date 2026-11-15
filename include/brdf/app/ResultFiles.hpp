/*
* @file include/brdf/app/ResultFiles.hpp
* @brief Text formats of the files a measurement run produces.
*/

#pragma once

#include <string>

#include "brdf/app/MeasurementSession.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/geometry/RigGeometry.hpp"

namespace brdf::app {

    /// measurements.csv: 1 行 = 1 つの(ポーズ, 測定点)。角度は度。
    ///   pose,spot,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,wl_380,wl_381,...
    /// 旧システムの「角度 CSV と分光 CSV を行の順序で突き合わせる」約束はなく、1 行に角度とスペクトルが並ぶ。
    std::string measurementsCsv(const SessionResult& result);

    /// angles.csv: 機器を使わず、レシピの全ポーズ × 全測定点の角度だけを出力する(旧システムの AngleInfo に相当)。
    ///   pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg
    std::string anglesCsv(const geometry::RigGeometry& geometry, const domain::Recipe& recipe);

    /// 実行の記録。測定結果と一緒に保存して、後から条件を追えるようにする。
    struct RunManifest {
        std::string tool_version;
        std::string timestamp;
        std::string kind;
        std::string device_mode;
        std::string recipe_name;
        std::string operator_name;
        int pose_count = 0;
        int spot_count = 0;
        int row_count = 0;
        Real wavelength_start = 0.0;
        Real wavelength_end = 0.0;
        Real wavelength_step = 0.0;
    };

    std::string toToml(const RunManifest& manifest);

} // namespace brdf::app
