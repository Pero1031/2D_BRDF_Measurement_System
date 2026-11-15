#include <array>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/core/MathUtils.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/optics/Angles.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::optics;
using geometry::SurfaceFrame;

namespace {

    // 周期 2π を考慮した角度の差
    Real circularDiff(Real a, Real b) {
        Real d = std::fmod(a - b, TWO_PI);
        if (d > PI)  d -= TWO_PI;
        if (d < -PI) d += TWO_PI;
        return d;
    }

} // namespace

TEST_CASE("directionTo on an identity frame: quadrants and zenith", "[optics]") {
    const SurfaceFrame frame;   // 原点・単位行列
    const Real tol = 1e-12;

    SECTION("straight above: theta = 0, phi falls back to 0 (no NaN)") {
        const Direction d = directionTo(frame, Vec3(0, 0, 100));
        CHECK_THAT(d.theta, WithinAbs(0.0, tol));
        CHECK_THAT(d.phi, WithinAbs(0.0, tol));
    }

    SECTION("45 degrees toward +X, +Y, -X, -Y") {
        const Direction px = directionTo(frame, Vec3(100, 0, 100));
        CHECK_THAT(px.theta, WithinAbs(PI / 4, tol));
        CHECK_THAT(px.phi, WithinAbs(0.0, tol));

        const Direction py = directionTo(frame, Vec3(0, 100, 100));
        CHECK_THAT(py.theta, WithinAbs(PI / 4, tol));
        CHECK_THAT(py.phi, WithinAbs(HALF_PI, tol));

        const Direction nx = directionTo(frame, Vec3(-100, 0, 100));
        CHECK_THAT(nx.phi, WithinAbs(PI, tol));

        const Direction ny = directionTo(frame, Vec3(0, -100, 100));
        CHECK_THAT(ny.phi, WithinAbs(3 * HALF_PI, tol));
    }

    SECTION("grazing direction: theta = 90 degrees") {
        const Direction d = directionTo(frame, Vec3(50, 50, 0));
        CHECK_THAT(d.theta, WithinAbs(HALF_PI, tol));
        CHECK_THAT(d.phi, WithinAbs(PI / 4, tol));
    }

    SECTION("below the surface: theta > 90 degrees is still correct") {
        const Direction d = directionTo(frame, Vec3(100, 0, -100));
        CHECK_THAT(d.theta, WithinAbs(3 * PI / 4, tol));
        CHECK_THAT(d.phi, WithinAbs(0.0, tol));
    }

    SECTION("phi is always in [0, 2pi)") {
        for (int deg = 0; deg < 360; deg += 7) {
            const Real ph = toRadians(static_cast<Real>(deg));
            const Direction d = directionTo(frame, 100.0 * sphericalToCartesian(0.8, ph));
            CHECK(d.phi >= 0.0);
            CHECK(d.phi < TWO_PI);
            CHECK_THAT(circularDiff(d.phi, ph), WithinAbs(0.0, 1e-12));
        }
    }

    SECTION("target equal to the patch gives (0, 0)") {
        const Direction d = directionTo(frame, Vec3(5, 5, 5), Vec3(5, 5, 5));
        CHECK_THAT(d.theta, WithinAbs(0.0, tol));
        CHECK_THAT(d.phi, WithinAbs(0.0, tol));
    }
}

TEST_CASE("directionTo measures from the patch, not from the frame origin", "[optics]") {
    const SurfaceFrame frame;
    const Direction d = directionTo(frame, Vec3(10, 0, 0), Vec3(110, 0, 100));
    CHECK_THAT(d.theta, WithinAbs(PI / 4, 1e-12));
    CHECK_THAT(d.phi, WithinAbs(0.0, 1e-12));
}

TEST_CASE("directionTo recovers the direction in a rotated and translated frame", "[optics]") {
    const Eigen::AngleAxis<Real> rot(0.9, Vec3(1, 2, 3).normalized());

    SurfaceFrame frame;
    frame.origin = Vec3(120, -40, 300);
    frame.axes = rot.toRotationMatrix();

    const Real theta = 0.7;
    const Real phi = 2.0;
    const Vec3 target = frame.origin + 500.0 * (frame.axes * sphericalToCartesian(theta, phi));

    const Direction d = directionTo(frame, target);
    CHECK_THAT(d.theta, WithinAbs(theta, 1e-12));
    CHECK_THAT(d.phi, WithinAbs(phi, 1e-12));
}

TEST_CASE("computeAngles returns incidence and observation", "[optics]") {
    const SurfaceFrame frame;
    const GeometryAngles a = computeAngles(frame, Vec3(0, 100, 100), Vec3(-100, 0, 100));

    CHECK_THAT(a.incidence.theta, WithinAbs(PI / 4, 1e-12));
    CHECK_THAT(a.incidence.phi, WithinAbs(HALF_PI, 1e-12));
    CHECK_THAT(a.observation.theta, WithinAbs(PI / 4, 1e-12));
    CHECK_THAT(a.observation.phi, WithinAbs(PI, 1e-12));
}

TEST_CASE("paper frame azimuth is the tip frame azimuth minus 90 degrees", "[optics]") {
    // 旧システムの角度表は先端の座標系(tipFrame)、試料点ごとの角度は紙の座標系(paperFrame)で出していた。
    // 両者の関係は φ_paper = φ_tip - 90° (θ は同じ)。
    const geometry::RigGeometry rig;
    const geometry::Joints left = geometry::jointsFromDegrees(65, 0, 90, 0, 0, 0);

    for (const auto& deg : {std::array<Real, 6>{-60, -15, 75, -15, -15, -15},
                            std::array<Real, 6>{-45, 0, 90, 0, 0, 0},
                            std::array<Real, 6>{-35, 12, 100, 10, 8, 14}}) {
        const geometry::Joints right =
            geometry::jointsFromDegrees(deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);

        const GeometryAngles tip = computeAngles(rig.tipFrame(right),
            rig.lightPosition(left), rig.cameraPosition());
        const GeometryAngles paper = computeAngles(rig.paperFrame(right),
            rig.lightPosition(left), rig.cameraPosition());

        CHECK_THAT(paper.incidence.theta, WithinAbs(tip.incidence.theta, 1e-9));
        CHECK_THAT(paper.observation.theta, WithinAbs(tip.observation.theta, 1e-9));
        CHECK_THAT(circularDiff(paper.incidence.phi, tip.incidence.phi - HALF_PI), WithinAbs(0.0, 1e-9));
        CHECK_THAT(circularDiff(paper.observation.phi, tip.observation.phi - HALF_PI), WithinAbs(0.0, 1e-9));
    }
}
