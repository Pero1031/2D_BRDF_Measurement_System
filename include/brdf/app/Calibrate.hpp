/*
* @file include/brdf/app/Calibrate.hpp
* @brief Calibration of every pose of a recipe from images of the calibration board.
*/

#pragma once

#include <filesystem>
#include <iosfwd>
#include <vector>

#include "brdf/core/Error.hpp"
#include "brdf/io/CalibrationFile.hpp"

namespace brdf::app {

    /// レシピの全ポーズで、キャリブボードを撮影してホモグラフィを求める。旧システムの「キャリブボード測定 →
    /// 円の中心の検出 → ホモグラフィ」に相当する(旧システムでは手作業や別ツールが混ざっていた)。
    ///
    /// 手順: 各ポーズで左右アームを動かし、画像を 1 枚撮影し(calibration/images/<ポーズ>.png)、黒円の中心を検出し、
    /// 77 点や 105 点の全部を使って再投影誤差が最小のホモグラフィを求める。
    /// 結果は calibration/homographies.csv に保存する。
    ///
    /// 準備: ボード(試料面と同じ平面)を試料の代わりに取り付けておく。
    /// 検出に失敗した、または再投影誤差(rms)が setup.toml の max_rms_px を超えたポーズがあれば、
    /// そのポーズを列挙したエラーを返す(成功したポーズの結果は保存する)。
    ///
    /// 実機モードでは、動かす前の確認(OK の入力)を in から読み、アームの実際の位置から最初の姿勢までの道のりを検査し、
    /// アームの移動に失敗したら、そこで止める(次の姿勢へ進まない)。Ctrl+C でも、次の姿勢へ進まずに止まる。
    Result<std::vector<io::PoseCalibration>> calibrateProject(const std::filesystem::path& dir, std::istream& in);

    /// 確認の入力を標準入力から読む。
    Result<std::vector<io::PoseCalibration>> calibrateProject(const std::filesystem::path& dir);

} // namespace brdf::app
