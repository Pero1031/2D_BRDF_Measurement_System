/*
* @file include/brdf/geometry/Joints.hpp
* @brief Joint angle vector of a 6-axis arm.
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/core/MathUtils.hpp"

namespace brdf::geometry {

    /// 6軸アームの関節角 J1..J6 [rad]。
    /// ライブラリ内部では常にラジアンで持つ。度が現れるのは入出力の境界(ポーズCSV、RC8、ログ表示)だけ。
    using Joints = Vec6;

    /// 度で与えられた関節角から Joints を作る。旧システムのポーズCSV(RJ1..RJ6 / LJ1..LJ6)は度。
    inline Joints jointsFromDegrees(Real j1, Real j2, Real j3, Real j4, Real j5, Real j6) {
        Joints q;
        q << toRadians(j1), toRadians(j2), toRadians(j3),
             toRadians(j4), toRadians(j5), toRadians(j6);
        return q;
    }

} // namespace brdf::geometry
