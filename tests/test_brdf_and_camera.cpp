#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/core/MathUtils.hpp"
#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/PinholeCamera.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace brdf;
using namespace brdf::sim;

namespace {
    optics::GeometryAngles anglesDeg(Real ti, Real pi_, Real to, Real po) {
        optics::GeometryAngles a;
        a.incidence   = {toRadians(ti), toRadians(pi_)};
        a.observation = {toRadians(to), toRadians(po)};
        return a;
    }
}

TEST_CASE("spectral curves", "[brdf]") {
    CHECK_THAT(flatCurve(0.3)(123.0), WithinAbs(0.3, 1e-15));

    const auto g = gaussianCurve(550.0, 60.0, 0.8, 0.1);
    CHECK_THAT(g(550.0), WithinAbs(0.9, 1e-12));          // 山の頂上 = base + peak
    CHECK(g(380.0) < g(500.0));
    CHECK_THAT(g(550.0 + 60.0), WithinAbs(0.1 + 0.8 * std::exp(-1.0), 1e-12));
}

TEST_CASE("Lambert BRDF is rho / pi and ignores the angles", "[brdf]") {
    const LambertBrdf brdf(flatCurve(0.5));
    const Real expected = 0.5 / PI;

    CHECK_THAT(brdf.value(550.0, anglesDeg(0, 0, 0, 0)), WithinAbs(expected, 1e-12));
    CHECK_THAT(brdf.value(550.0, anglesDeg(60, 30, 45, 200)), WithinAbs(expected, 1e-12));
}

TEST_CASE("Phong BRDF peaks in the mirror direction", "[brdf]") {
    const Real kd = 0.2, ks = 0.5, n = 50.0;
    const PhongBrdf brdf(flatCurve(kd), ks, n);

    // 鏡面方向: 入射 (θ=30°, φ=0°) の反対側 (θ=30°, φ=180°) から観察
    const Real peak = brdf.value(550.0, anglesDeg(30, 0, 30, 180));
    CHECK_THAT(peak, WithinAbs(kd / PI + ks * (n + 2) / TWO_PI, 1e-12));

    // 鏡面方向からずれるほど小さくなり、十分離れると拡散成分だけになる
    const Real near = brdf.value(550.0, anglesDeg(30, 0, 40, 180));
    const Real far = brdf.value(550.0, anglesDeg(30, 0, 70, 0));
    CHECK(near < peak);
    CHECK(far < near);
    CHECK_THAT(far, WithinAbs(kd / PI, 1e-6));
}

TEST_CASE("PinholeCamera: the image centre looks at the target", "[camera]") {
    PinholeCameraSetup setup;
    setup.position = Vec3(1000, 700, 500);
    setup.target = Vec3(100, 800, 400);
    const PinholeCamera cam(setup);

    const Vec2 centre(setup.image_width * 0.5, setup.image_height * 0.5);
    const Vec3 expected = (setup.target - setup.position).normalized();
    CHECK_THAT((cam.rayDirection(centre) - expected).norm(), WithinAbs(0.0, 1e-12));

    const Vec2 p = cam.project(setup.target);
    CHECK_THAT(p.x(), WithinAbs(centre.x(), 1e-9));
    CHECK_THAT(p.y(), WithinAbs(centre.y(), 1e-9));
}

TEST_CASE("PinholeCamera: project is the inverse of rayDirection", "[camera]") {
    PinholeCameraSetup setup;
    setup.position = Vec3(1136, 728, 560);
    setup.target = Vec3(300, 700, 500);
    const PinholeCamera cam(setup);

    for (const Vec2& pixel : {Vec2(10, 20), Vec2(688, 512), Vec2(1300, 900), Vec2(5, 1000)}) {
        const Vec3 point = cam.position() + 800.0 * cam.rayDirection(pixel);
        const Vec2 back = cam.project(point);
        CHECK_THAT(back.x(), WithinAbs(pixel.x(), 1e-9));
        CHECK_THAT(back.y(), WithinAbs(pixel.y(), 1e-9));
    }
}

TEST_CASE("PinholeCamera: resolution at the target is pixels_per_mm", "[camera]") {
    PinholeCameraSetup setup;
    setup.position = Vec3(1000, 0, 500);
    setup.target = Vec3(0, 0, 500);
    setup.pixels_per_mm = 1.81;
    const PinholeCamera cam(setup);

    // 視線に垂直な向きに 10 mm ずれた点は、1.81 × 10 pixel だけずれて写る。
    // カメラは -X 方向を見ていて、上方向が +Z なので、画像の右は +Y。
    const Vec2 a = cam.project(setup.target);
    const Vec2 b = cam.project(setup.target + Vec3(0, 10, 0));
    CHECK_THAT(b.x() - a.x(), WithinAbs(18.1, 1e-9));
    CHECK_THAT(b.y() - a.y(), WithinAbs(0.0, 1e-9));

    // 上(+Z)へ 10 mm ずれた点は、画像では上(v が小さい方)に写る
    const Vec2 c = cam.project(setup.target + Vec3(0, 0, 10));
    CHECK_THAT(c.y() - a.y(), WithinAbs(-18.1, 1e-9));
}

TEST_CASE("PinholeCamera: points behind the camera have no image", "[camera]") {
    PinholeCameraSetup setup;
    setup.position = Vec3(1000, 0, 500);
    setup.target = Vec3(0, 0, 500);
    const PinholeCamera cam(setup);

    const Vec2 p = cam.project(Vec3(2000, 0, 500));
    CHECK(std::isnan(p.x()));
    CHECK_FALSE(cam.inImage(Vec2(-1, 10)));
    CHECK_FALSE(cam.inImage(Vec2(10, 5000)));
    CHECK(cam.inImage(Vec2(10, 10)));
}
