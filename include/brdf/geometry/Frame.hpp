/*
* @file include/brdf/geometry/Frame.hpp
* @brief A right-handed coordinate frame placed in the world (e.g. the sample surface).
*/

#pragma once

#include "brdf/core/Algebra.hpp"

namespace brdf::geometry {

    /// ワールド座標系の中に置かれた右手系の座標系。
    /// 角度 (θ, φ) は必ずこの座標系を基準に測る:
    ///   Z 軸 = 面の法線(θ=0 の向き)、X 軸 = φ=0 の向き、Y 軸 = φ=90° の向き。
    struct SurfaceFrame {
        Vec3 origin = Vec3::Zero();     ///< 座標系の原点(ワールド座標, mm)
        Mat3 axes = Mat3::Identity();   ///< 列ベクトルが X, Y, Z 軸のワールド座標での向き(単位ベクトル)

        Vec3 xAxis() const { return axes.col(0); }
        Vec3 yAxis() const { return axes.col(1); }
        Vec3 zAxis() const { return axes.col(2); }
    };

} // namespace brdf::geometry
