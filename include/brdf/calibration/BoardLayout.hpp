/*
* @file include/brdf/calibration/BoardLayout.hpp
* @brief Geometry of the calibration board (a grid of black circles on paper).
*/

#pragma once

#include <vector>

#include "brdf/core/Algebra.hpp"

namespace brdf::calibration {

    /// キャリブボード: 白い紙の上に、黒い円が格子状に並んだもの。
    ///
    /// 座標系(ボード座標): 原点 = 紙の角、X = 紙の幅方向、Y = 紙の高さ方向、単位 mm。
    /// 円は「行優先」に並べる: 最初の円 (first_x, first_y) から X 方向に columns 個、
    /// 次の行へ、というように、座標が小さい方から大きい方へ進む。
    ///
    /// 旧システムとの対応: 旧 Paperparam.h は紙の右下を基準に XDEF/YDEF を取っていた。
    ///   first_x = PAPER_WIDTH  - XDEF - (CORNER_W - 1) × PITCH_W
    ///   first_y = PAPER_HEIGHT - YDEF - (CORNER_H - 1) × PITCH_H
    /// (旧 getVirtualCorner が作る点の並びと同じ)
    struct BoardLayout {
        int columns = 15;
        int rows = 7;
        Real pitch_x = 25.0;          ///< 円の間隔(X 方向)[mm]
        Real pitch_y = 25.0;          ///< 円の間隔(Y 方向)[mm]
        Real first_x = 75.0;          ///< 最初の円の中心 [mm]
        Real first_y = 101.0;
        Real paper_width = 450.0;     ///< 紙の大きさ [mm]
        Real paper_height = 302.0;
        Real circle_diameter = 16.0;  ///< 円の直径 [mm](シミュレータの描画用)

        int count() const { return columns * rows; }

        /// 円の中心(ボード座標)。行優先、座標が小さい方から。要素数 = columns × rows。
        std::vector<Vec2> circleCenters() const;

        /// 論文当時(2020 年)のボード: 11×7 = 77 個、間隔 30×40 mm、紙 467×317 mm。
        static BoardLayout legacy2020();

        /// 2023 年以降のボード: 15×7 = 105 個、間隔 25×25 mm、紙 450×302 mm。
        static BoardLayout legacy2023();
    };

} // namespace brdf::calibration
