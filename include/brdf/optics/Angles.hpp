/*
* @file include/brdf/optics/Angles.hpp
* @brief Incidence / observation angle computation for a point on a surface.
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/geometry/Frame.hpp"

namespace brdf::optics {

    /// 面の座標系で見た方向。単位はラジアン。
    struct Direction {
        Real theta = 0.0;   ///< 天頂角: 面の法線(Z軸)からの角度。範囲 [0, π]
        Real phi   = 0.0;   ///< 方位角: X軸から Y軸向きに測る。範囲 [0, 2π)
    };

    /// ある点から見た光源の方向(入射)とカメラの方向(観察)。
    struct GeometryAngles {
        Direction incidence;     ///< (θi, φi)
        Direction observation;   ///< (θo, φo)
    };

    /// patch(ワールド座標)から target(ワールド座標)を見た方向を、frame の座標系で表す。
    ///
    /// 約束:
    ///  - θ は atan2 で求めるので、θ>90°(面の裏側)でも正しい値になる。
    ///  - 法線方向(θ=0)では φ は定義できないので、φ=0 を返す。
    ///  - patch と target が一致している場合は θ=0, φ=0 を返す。
    ///
    /// 旧システムの Calc::calcAngle と同じ結果になる(θ<90° の範囲)が、
    /// 法線入射で NaN になる問題はなくなっている。
    Direction directionTo(const geometry::SurfaceFrame& frame, const Vec3& patch, const Vec3& target);

    /// frame の原点から target を見た方向。
    Direction directionTo(const geometry::SurfaceFrame& frame, const Vec3& target);

    /// patch における入射角・観察角。
    GeometryAngles computeAngles(const geometry::SurfaceFrame& frame, const Vec3& patch,
                                 const Vec3& light_position, const Vec3& camera_position);

    /// frame の原点における入射角・観察角。
    GeometryAngles computeAngles(const geometry::SurfaceFrame& frame,
                                 const Vec3& light_position, const Vec3& camera_position);

} // namespace brdf::optics
