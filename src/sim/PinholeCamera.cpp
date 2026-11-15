/*
* @file src/sim/PinholeCamera.cpp
* @brief Pinhole camera model used by the simulated spectral imager.
*/

#include "brdf/sim/PinholeCamera.hpp"

#include <limits>

namespace brdf::sim {

    PinholeCamera::PinholeCamera(PinholeCameraSetup setup) : setup_(setup) {
        const Vec3 to_target = setup_.target - setup_.position;
        forward_ = to_target.normalized();

        Vec3 up = Vec3::UnitZ();
        if (std::abs(forward_.dot(up)) > 0.999) up = Vec3::UnitX();   // 真上・真下を向く場合の逃げ

        right_ = forward_.cross(up).normalized();
        down_ = forward_.cross(right_);

        // 注視点で 1mm が pixels_per_mm [pixel] に写るように焦点距離 [pixel] を決める。
        focal_px_ = setup_.pixels_per_mm * to_target.norm();
    }

    Vec3 PinholeCamera::rayDirection(const Vec2& pixel) const {
        const Real cx = setup_.image_width * 0.5;
        const Real cy = setup_.image_height * 0.5;
        const Vec3 d = forward_
                     + ((pixel.x() - cx) / focal_px_) * right_
                     + ((pixel.y() - cy) / focal_px_) * down_;
        return d.normalized();
    }

    Vec2 PinholeCamera::project(const Vec3& world_point) const {
        const Vec3 d = world_point - setup_.position;
        const Real z = d.dot(forward_);
        if (z <= 0.0) {
            const Real nan = std::numeric_limits<Real>::quiet_NaN();
            return Vec2(nan, nan);
        }
        const Real cx = setup_.image_width * 0.5;
        const Real cy = setup_.image_height * 0.5;
        return Vec2(cx + focal_px_ * d.dot(right_) / z,
                    cy + focal_px_ * d.dot(down_) / z);
    }

    bool PinholeCamera::inImage(const Vec2& pixel) const {
        return pixel.x() >= 0.0 && pixel.x() < setup_.image_width
            && pixel.y() >= 0.0 && pixel.y() < setup_.image_height;
    }

} // namespace brdf::sim
