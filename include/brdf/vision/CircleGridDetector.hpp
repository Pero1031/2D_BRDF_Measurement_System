/*
* @file include/brdf/vision/CircleGridDetector.hpp
* @brief Finds the black circles of the calibration board in a camera image.
*/

#pragma once

#include <vector>

#include "brdf/calibration/BoardLayout.hpp"
#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/core/Image.hpp"

namespace brdf::vision {

    struct DetectorOptions {
        Real min_area_px = 100.0;       ///< 円として受け入れる面積の下限 [pixel²]
        Real max_area_px = 5000.0;      ///< 同、上限
        Real min_circularity = 0.7;     ///< 円らしさ(1 が真円)の下限
    };

    /// 画像から黒円の中心を検出し、BoardLayout::circleCenters() と同じ並び(行優先)で返す。
    ///
    /// ボードは対称な格子なので、検出した点の並びには、回転(180°)や反転(行内・行の順を逆)の別解がある
    /// (旧システムの「上下が崩れるとナンバリングできない」問題)。次の 2 段で 1 つに決める。
    ///  1. 反転した並びは、ボードを裏から見たことになり、ホモグラフィの向きが逆になるので除く。
    ///  2. 残った 180° 違いの並びから、**ボードの X 軸が画像の右向きに写る方**を選ぶ。
    /// 前提は、ボードの回転が ±90° 以内であること。旧システムの実データ 13 枚(+35° 〜 -52°)と、
    /// 合成画像の ±80° で正しく選べた。
    ///
    /// 円が見つからない、個数が足りない、並べられない場合はエラー。
    Result<std::vector<Vec2>> detectCircleGrid(const GrayImage& image,
                                               const calibration::BoardLayout& layout,
                                               const DetectorOptions& options = {});

} // namespace brdf::vision
