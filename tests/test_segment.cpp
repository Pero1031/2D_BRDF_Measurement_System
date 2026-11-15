#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/geometry/Segment.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::geometry;

TEST_CASE("segmentDistance: parallel segments", "[segment]") {
    const Segment a{Vec3(0, 0, 0), Vec3(10, 0, 0)};
    const Segment b{Vec3(0, 5, 0), Vec3(10, 5, 0)};
    CHECK_THAT(segmentDistance(a, b), WithinAbs(5.0, 1e-12));

    // 長さがずれていても、重なる範囲があれば距離は同じ
    const Segment c{Vec3(5, 5, 0), Vec3(20, 5, 0)};
    CHECK_THAT(segmentDistance(a, c), WithinAbs(5.0, 1e-12));
}

TEST_CASE("segmentDistance: skew segments cross above each other", "[segment]") {
    const Segment a{Vec3(-5, 0, 0), Vec3(5, 0, 0)};
    const Segment b{Vec3(0, -5, 3), Vec3(0, 5, 3)};
    CHECK_THAT(segmentDistance(a, b), WithinAbs(3.0, 1e-12));
    CHECK_THAT(segmentDistance(b, a), WithinAbs(3.0, 1e-12));
}

TEST_CASE("segmentDistance: closest points are end points", "[segment]") {
    const Segment a{Vec3(0, 0, 0), Vec3(1, 0, 0)};
    const Segment b{Vec3(4, 4, 0), Vec3(9, 4, 0)};
    // a の端 (1,0,0) と b の端 (4,4,0): 距離 5
    CHECK_THAT(segmentDistance(a, b), WithinAbs(5.0, 1e-12));
}

TEST_CASE("segmentDistance: touching and degenerate segments", "[segment]") {
    const Segment a{Vec3(0, 0, 0), Vec3(10, 0, 0)};
    const Segment touching{Vec3(5, 0, 0), Vec3(5, 5, 0)};
    CHECK_THAT(segmentDistance(a, touching), WithinAbs(0.0, 1e-12));

    const Segment point{Vec3(3, 4, 0), Vec3(3, 4, 0)};
    CHECK_THAT(segmentDistance(a, point), WithinAbs(4.0, 1e-12));
    CHECK_THAT(segmentDistance(point, a), WithinAbs(4.0, 1e-12));
    CHECK_THAT(segmentDistance(point, point), WithinAbs(0.0, 1e-12));
}
