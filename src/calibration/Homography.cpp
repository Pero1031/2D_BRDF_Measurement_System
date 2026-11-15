/*
* @file src/calibration/Homography.cpp
* @brief A 3x3 projective transform between the board plane and the camera image.
*/

#include "brdf/calibration/Homography.hpp"

#include <cmath>
#include <limits>

namespace brdf::calibration {

    Vec2 Homography::apply(const Vec2& p) const {
        const Vec3 v = matrix * Vec3(p.x(), p.y(), 1.0);
        if (std::abs(v.z()) < 1e-12) {
            const Real nan = std::numeric_limits<Real>::quiet_NaN();
            return Vec2(nan, nan);
        }
        return Vec2(v.x() / v.z(), v.y() / v.z());
    }

    Result<Homography> Homography::inverse() const {
        const Real det = matrix.determinant();
        if (!std::isfinite(det) || std::abs(det) < 1e-18) {
            return fail(ErrorCode::InvalidArgument, "the homography is singular and cannot be inverted");
        }
        Homography inv;
        inv.matrix = matrix.inverse();
        return inv;
    }

} // namespace brdf::calibration
