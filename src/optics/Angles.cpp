/*
* @file src/optics/Angles.cpp
* @brief Incidence / observation angle computation for a point on a surface.
*/

#include "brdf/optics/Angles.hpp"

#include <cmath>

#include "brdf/core/MathUtils.hpp"

namespace brdf::optics {

    Direction directionTo(const geometry::SurfaceFrame& frame, const Vec3& patch, const Vec3& target) {
        // ワールド座標のベクトルを、面の座標系の成分に直す(axes は正規直交なので転置が逆行列)。
        const Vec3 local = frame.axes.transpose() * (target - patch);

        Direction d;
        d.theta = std::atan2(std::hypot(local.x(), local.y()), local.z());

        if (local.x() != 0.0 || local.y() != 0.0) {
            d.phi = std::atan2(local.y(), local.x());
            if (d.phi < 0.0)      d.phi += TWO_PI;
            if (d.phi >= TWO_PI)  d.phi -= TWO_PI;   // -1e-18 のような値が丸めで 2π になるのを防ぐ
        }
        return d;
    }

    Direction directionTo(const geometry::SurfaceFrame& frame, const Vec3& target) {
        return directionTo(frame, frame.origin, target);
    }

    GeometryAngles computeAngles(const geometry::SurfaceFrame& frame, const Vec3& patch,
                                 const Vec3& light_position, const Vec3& camera_position) {
        GeometryAngles a;
        a.incidence   = directionTo(frame, patch, light_position);
        a.observation = directionTo(frame, patch, camera_position);
        return a;
    }

    GeometryAngles computeAngles(const geometry::SurfaceFrame& frame,
                                 const Vec3& light_position, const Vec3& camera_position) {
        return computeAngles(frame, frame.origin, light_position, camera_position);
    }

} // namespace brdf::optics
