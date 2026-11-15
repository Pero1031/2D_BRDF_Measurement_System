/*
* @file src/calibration/HomographySpotProjector.cpp
* @brief SpotProjector that uses one homography per pose, found from calibration-board images.
*/

#include "brdf/calibration/HomographySpotProjector.hpp"

#include <cmath>

namespace brdf::calibration {

    Result<Vec2> HomographySpotProjector::pixelFor(const app::SpotQuery& query) const {
        const auto it = by_pose_.find(query.pose_id);
        if (it == by_pose_.end()) {
            return fail(ErrorCode::InvalidArgument,
                        "no calibration (homography) for pose \"" + query.pose_id + "\"; run `brdf calibrate` first");
        }

        const Vec2 pixel = it->second.apply(query.sample_mm + origin_);
        if (std::isnan(pixel.x()) || std::isnan(pixel.y())) {
            return fail(ErrorCode::OutOfRange, "the point cannot be projected (the homography is degenerate)");
        }
        if (pixel.x() < 0.0 || pixel.x() >= width_ || pixel.y() < 0.0 || pixel.y() >= height_) {
            return fail(ErrorCode::OutOfRange, "the point is outside the camera image");
        }
        return pixel;
    }

} // namespace brdf::calibration
