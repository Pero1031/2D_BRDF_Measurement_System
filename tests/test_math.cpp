#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/core/MathUtils.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;

TEST_CASE("degrees and radians convert both ways", "[math]") {
    CHECK_THAT(toRadians(180.0), WithinAbs(PI, 1e-12));
    CHECK_THAT(toRadians(90.0), WithinAbs(HALF_PI, 1e-12));
    CHECK_THAT(toDegrees(PI), WithinAbs(180.0, 1e-12));
    CHECK_THAT(toDegrees(toRadians(37.25)), WithinAbs(37.25, 1e-12));
}

TEST_CASE("sphericalToCartesian uses a Z-up convention", "[math]") {
    const Vec3 up = sphericalToCartesian(0.0, 0.0);
    CHECK_THAT(up.z(), WithinAbs(1.0, 1e-12));

    const Vec3 x = sphericalToCartesian(HALF_PI, 0.0);
    CHECK_THAT(x.x(), WithinAbs(1.0, 1e-12));
    CHECK_THAT(x.z(), WithinAbs(0.0, 1e-12));

    const Vec3 y = sphericalToCartesian(HALF_PI, HALF_PI);
    CHECK_THAT(y.y(), WithinAbs(1.0, 1e-12));

    CHECK_THAT(sphericalToCartesian(0.7, 2.0).norm(), WithinAbs(1.0, 1e-12));
}
