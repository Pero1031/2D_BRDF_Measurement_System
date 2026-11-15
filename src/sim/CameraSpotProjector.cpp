/*
* @file src/sim/CameraSpotProjector.cpp
* @brief SpotProjector backed by the simulated camera model.
*/

#include "brdf/sim/CameraSpotProjector.hpp"

#include <cmath>

namespace brdf::sim {

    Result<Vec2> CameraSpotProjector::pixelFor(const Vec3& world_point) const {
        const PinholeCamera& camera = rig_->camera();
        const Vec2 pixel = camera.project(world_point);

        if (std::isnan(pixel.x()) || std::isnan(pixel.y())) {
            return fail(ErrorCode::OutOfRange, "the point is behind the camera");
        }
        if (!camera.inImage(pixel)) {
            return fail(ErrorCode::OutOfRange, "the point is outside the camera image");
        }
        return pixel;
    }

} // namespace brdf::sim
