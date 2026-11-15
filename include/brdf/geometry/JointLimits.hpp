/*
* @file include/brdf/geometry/JointLimits.hpp
* @brief Joint range limits of a 6-axis arm.
*/

#pragma once

#include "brdf/geometry/Joints.hpp"

namespace brdf::geometry {

    /// 関節の可動域 [rad]。
    ///
    /// 既定値は DENSO VS シリーズの公称値に近い「仮の値」で、実機の仕様書で未確認。
    /// 実機を動かす前(P6)に、必ず実機の値で setup.toml に書くこと。
    struct JointLimits {
        Joints lower = jointsFromDegrees(-170.0,  -90.0,  10.0, -190.0, -120.0, -360.0);
        Joints upper = jointsFromDegrees( 170.0,  135.0, 167.0,  190.0,  120.0,  360.0);

        /// 可動域を外れている最初の関節の番号 (0〜5)。全部範囲内なら -1。
        int firstViolation(const Joints& q) const {
            for (int i = 0; i < 6; ++i) {
                if (q(i) < lower(i) || q(i) > upper(i)) return i;
            }
            return -1;
        }
    };

} // namespace brdf::geometry
