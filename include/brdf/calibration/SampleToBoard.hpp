/*
* @file include/brdf/calibration/SampleToBoard.hpp
* @brief How the calibration board is mounted relative to the sample coordinates.
*/

#pragma once

#include <string_view>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::calibration {

    /// 試料座標(紙の座標系, mm)→ ボード座標(mm)の変換。ボードを試料面にどう取り付けたかを表す。
    ///
    ///   ボード座標 = axes · 試料座標 + origin
    ///
    /// ボード座標は、ボードを正面(カメラ側)から見て、u = 右、v = 下 とする。
    /// axes の行は、ボードの u 軸・v 軸が試料座標のどの軸に当たるかを表す(±x または ±y)。
    ///
    /// 試料座標系は右手系(X × Y = 法線)なので、カメラから見ると Y 軸が上を向くことがあり、そのときは
    /// v = -y になる(鏡像)。ボードの円は対称なので、画像だけではこの違いを判別できない。
    /// 取り付け方に応じて、setup.toml の board_u / board_v で指定する。
    struct SampleToBoard {
        Mat2 axes = Mat2::Identity();
        Vec2 origin = Vec2::Zero();   ///< 試料座標の原点が、ボード座標のどこにあるか [mm]

        Vec2 apply(const Vec2& sample) const { return axes * sample + origin; }

        /// ボード座標 → 試料座標(axes は ±1/0 の直交行列なので、転置が逆行列)
        Vec2 inverse(const Vec2& board) const { return axes.transpose() * (board - origin); }

        /// 軸の名前("+x" "-x" "+y" "-y")から作る。u と v が同じ軸(±を含む)ならエラー。
        static Result<SampleToBoard> make(std::string_view board_u, std::string_view board_v, const Vec2& origin);
    };

} // namespace brdf::calibration
