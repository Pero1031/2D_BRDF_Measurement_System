/*
* @file include/brdf/vision/HomographyFit.hpp
* @brief Least-squares fit of a homography to point correspondences.
*/

#pragma once

#include <vector>

#include "brdf/calibration/Homography.hpp"
#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::vision {

    /// ホモグラフィの当てはめ結果と、その精度。
    struct HomographyFit {
        calibration::Homography homography;
        Real rms_px = 0.0;     ///< 再投影誤差の二乗平均平方根 [pixel]
        Real max_px = 0.0;     ///< 再投影誤差の最大値 [pixel]
        int points = 0;        ///< 使った対応点の数
    };

    /// ボード座標の点 → 画像上の点、の対応から H を求める。全点を使い、再投影誤差の二乗和が最小になるようにする。
    /// (4 点だけでも計算できるが精度が低いので、旧システムと同じく多数点で求める)
    /// 点の数が合わない、4 点未満、点が一直線上に並ぶなどで求まらなければエラー。
    Result<HomographyFit> fitHomography(const std::vector<Vec2>& board_points,
                                        const std::vector<Vec2>& image_points);

} // namespace brdf::vision
