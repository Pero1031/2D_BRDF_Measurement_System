/*
* @file include/brdf/io/CalibrationFile.hpp
* @brief calibration/homographies.csv: one homography per pose.
*/

#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "brdf/calibration/Homography.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::io {

    /// 1 つのポーズの校正結果。
    struct PoseCalibration {
        std::string pose_id;
        calibration::Homography homography;   ///< ボード座標 (mm) → 画像 (pixel)
        Real rms_px = 0.0;                    ///< 再投影誤差(rms)
        Real max_px = 0.0;                    ///< 再投影誤差(最大)
        int circles = 0;                      ///< 使った円の数
    };

    /// calibration/homographies.csv:
    ///   pose,h11,h12,h13,h21,h22,h23,h31,h32,h33,rms_px,max_px,circles
    std::string toCsv(const std::vector<PoseCalibration>& calibrations);

    Result<std::vector<PoseCalibration>> parseCalibrationCsv(std::string_view text,
                                                              std::string_view origin = "homographies.csv");

    Result<std::vector<PoseCalibration>> loadCalibration(const std::filesystem::path& path);

} // namespace brdf::io
