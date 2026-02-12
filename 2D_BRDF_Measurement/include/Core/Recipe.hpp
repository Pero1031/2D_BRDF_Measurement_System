#pragma once

#include <string>
#include <vector>
#include <optional>

namespace brdf {

    // 1つの測定点
    struct MeasurementPoint {
        double theta_i; // 光源 天頂角 [deg]
        double phi_i;   // 光源 方位角 [deg]
        double theta_o; // センサー 天頂角 [deg]
        double phi_o;   // センサー 方位角 [deg]
    };

    // キャリブレーション設定
    struct CalibrationConfig {
        bool do_dark_current = true;
        bool do_white_reference = true;
        // 白色板測定時の固定角度（通常は 45/0 や 0/45）
        MeasurementPoint white_ref_angle = { 45.0, 0.0, 0.0, 0.0 };
    };

    // 全体のレシピ
    struct Recipe {
        struct Meta {
            std::string project_name;
            std::string user;
            std::string description;
        } meta;

        struct Config {
            std::string output_dir;
            double integration_time_ms;
            double arm_speed;
        } config;

        CalibrationConfig calibration;

        // 幾何設定（ロボットの逆運動学計算に必要）
        struct Geometry {
            double sample_height_offset = 0.0;
            // 必要ならアームのベース位置関係などもここに記述
        } geometry;

        std::vector<MeasurementPoint> points;
    };

} // namespace brdf::core