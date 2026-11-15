/*
* @file include/brdf/calibration/Homography.hpp
* @brief A 3x3 projective transform between the board plane and the camera image.
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::calibration {

    /// ホモグラフィ行列 H。ボード座標 (mm) → 画像上の位置 (pixel)。
    ///   [x', y', w]ᵀ = H · [u, v, 1]ᵀ,   pixel = (x'/w, y'/w)
    /// 試料面もボードと同じ平面にあるとみなし、試料面の点を画像上の位置に直すのに使う。
    struct Homography {
        Mat3 matrix = Mat3::Identity();

        /// 点 (u, v) を変換する。w がほぼ 0(無限遠)のときは NaN を返す。
        Vec2 apply(const Vec2& p) const;

        /// 逆変換(画像上の位置 → ボード座標)。特異なら失敗。
        Result<Homography> inverse() const;
    };

} // namespace brdf::calibration
