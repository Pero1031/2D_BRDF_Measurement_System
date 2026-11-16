#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/core/MathUtils.hpp"
#include "brdf/optics/Angles.hpp"
#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/MockSpectralImager.hpp"
#include "brdf/sim/SimulatedRig.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace brdf;
using namespace brdf::sim;
using geometry::jointsFromDegrees;

namespace {

    // 旧システムの測定で最初に使われるポーズ
    const geometry::Joints kRightPose = jointsFromDegrees(-45, 0, 90, 0, 0, 0);
    const geometry::Joints kLeftPose  = jointsFromDegrees(45, 0, 90, 0, 0, -30);

    struct World {
        std::shared_ptr<SimulatedRig> rig = std::make_shared<SimulatedRig>();
        MockRobotArm left{"left", rig, ArmSide::Left};
        MockRobotArm right{"right", rig, ArmSide::Right};
        std::shared_ptr<IBrdfModel> brdf;
        MockSpectralImager imager;

        explicit World(std::shared_ptr<IBrdfModel> model, MockImagerOptions options = {})
            : brdf(std::move(model)), imager(rig, brdf, options) {
            REQUIRE(left.connect().has_value());
            REQUIRE(right.connect().has_value());
            REQUIRE(imager.connect().has_value());
            REQUIRE(left.moveJ(kLeftPose, 1.0).has_value());
            REQUIRE(right.moveJ(kRightPose, 1.0).has_value());
        }

        // いまの幾何での、カメラ〜試料の距離
        Real distance() const {
            return (rig->sampleFrame().origin - rig->camera().position()).norm();
        }

        driver::CaptureRequest request(const std::vector<Vec2>& pixels, Real integration_ms = 100.0) const {
            driver::CaptureRequest req;
            req.measure_distance_mm = distance();
            req.integration_time_ms = integration_ms;
            int id = 1;
            for (const Vec2& p : pixels) req.spots.push_back({id++, p});
            return req;
        }
    };

    const Vec2 kCentre(688, 512);

    std::shared_ptr<IBrdfModel> lambert(Real albedo) {
        return std::make_shared<LambertBrdf>(flatCurve(albedo));
    }

    // 撮影結果の独立した期待値(Lambert、ノイズなし): ρ/π · I0 · cosθi / d²(校正済みの分光放射輝度)
    Real expectedLambert(const World& w, const Vec2& pixel, Real albedo, Real intensity) {
        const auto& cam = w.rig->camera();
        const auto frame = w.rig->sampleFrame();
        const Vec3 ray = cam.rayDirection(pixel);
        const Real t = frame.zAxis().dot(frame.origin - cam.position()) / frame.zAxis().dot(ray);
        const Vec3 patch = cam.position() + t * ray;

        const auto angles = optics::computeAngles(frame, patch, w.rig->lightPosition(), cam.position());
        const Real d = (w.rig->lightPosition() - patch).norm();
        return albedo / PI * intensity * std::max(0.0, std::cos(angles.incidence.theta)) / (d * d);
    }

} // namespace

TEST_CASE("MockSpectralImager rejects bad requests", "[sim][imager]") {
    World w(lambert(0.5));

    SECTION("not connected") {
        MockSpectralImager other(w.rig, w.brdf);
        const auto r = other.capture(w.request({kCentre}));
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::NotConnected);
    }
    SECTION("no spots") {
        const auto r = w.imager.capture(w.request({}));
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
    }
    SECTION("integration time must not be negative") {
        const auto r = w.imager.capture(w.request({kCentre}, -1.0));
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
    }
    SECTION("integration time 0 means automatic") {
        const auto r = w.imager.capture(w.request({kCentre}, 0.0));
        REQUIRE(r.has_value());
        CHECK(r->integration_time_ms > 0.0);
    }
    SECTION("invalid wavelength range") {
        auto req = w.request({kCentre});
        req.wavelengths.end = 300.0;
        const auto r = w.imager.capture(req);
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
    }
    SECTION("a spot outside the image") {
        const auto r = w.imager.capture(w.request({kCentre, Vec2(5000, 10)}));
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::OutOfRange);
    }
}

TEST_CASE("MockSpectralImager checks the focus distance", "[sim][imager]") {
    World w(lambert(0.5));

    auto req = w.request({kCentre});
    req.measure_distance_mm += 10.0;   // 許容差 20 mm の内側
    CHECK(w.imager.capture(req).has_value());

    req.measure_distance_mm += 100.0;
    const auto r = w.imager.capture(req);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::FocusError);
}

TEST_CASE("MockSpectralImager returns what was asked for", "[sim][imager]") {
    World w(lambert(0.5));

    const std::vector<Vec2> pixels = {kCentre, Vec2(640, 500), Vec2(740, 560)};
    const auto r = w.imager.capture(w.request(pixels));
    REQUIRE(r.has_value());

    REQUIRE(r->spots.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(r->spots[i].id == static_cast<int>(i) + 1);
        CHECK_THAT(r->spots[i].pixel.x(), WithinAbs(pixels[i].x(), 1e-12));
        CHECK(r->spots[i].values.size() == 401);
    }
    REQUIRE(r->wavelengths.size() == 401);
    CHECK_THAT(r->wavelengths(0), WithinAbs(380.0, 1e-12));
    CHECK_THAT(r->wavelengths(400), WithinAbs(780.0, 1e-12));
    CHECK_THAT(r->integration_time_ms, WithinAbs(100.0, 1e-12));
}

TEST_CASE("MockSpectralImager measures a Lambert surface as expected", "[sim][imager]") {
    MockImagerOptions options;
    options.light_intensity = 2.0e6;
    World w(lambert(0.5), options);

    const std::vector<Vec2> pixels = {kCentre, Vec2(600, 450), Vec2(780, 600)};
    const auto r = w.imager.capture(w.request(pixels, 100.0));
    REQUIRE(r.has_value());

    Real first = 0, last = 0;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        const Real expected = expectedLambert(w, pixels[i], 0.5, 2.0e6);
        INFO("spot " << i);
        CHECK(expected > 0.0);
        // 波長によらない一定の反射率なので、全波長で同じ値
        CHECK_THAT(r->spots[i].values(0), WithinRel(expected, 1e-9));
        CHECK_THAT(r->spots[i].values(400), WithinRel(expected, 1e-9));
        if (i == 0) first = r->spots[i].values(0);
        if (i == 2) last = r->spots[i].values(0);
    }
    // 位置が違えば距離と入射角が違うので、値も違う(2D測定で点ごとに条件が違うことの確認)
    CHECK(std::abs(first - last) > 1e-9 * first);
}

TEST_CASE("the radiance does not depend on the integration time", "[sim][imager]") {
    // 実機は校正済みの分光放射輝度を返すので、積分時間を変えても値は変わらない
    // (積分時間は、ノイズや飽和に効くだけ)。試料と白色板で積分時間が違っても、比が取れる。
    World w(lambert(0.5));

    const auto a = w.imager.capture(w.request({kCentre}, 100.0));
    const auto b = w.imager.capture(w.request({kCentre}, 250.0));
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    CHECK_THAT(b->spots[0].values(10), WithinRel(a->spots[0].values(10), 1e-12));
}

TEST_CASE("a coloured sample shows its spectral peak", "[sim][imager]") {
    World w(std::make_shared<LambertBrdf>(gaussianCurve(550.0, 60.0, 0.8, 0.05)));

    const auto r = w.imager.capture(w.request({kCentre}));
    REQUIRE(r.has_value());

    const VecX& v = r->spots[0].values;
    Eigen::Index peak = 0;
    v.maxCoeff(&peak);
    CHECK_THAT(r->wavelengths(peak), WithinAbs(550.0, 1e-9));
}

TEST_CASE("moving the arms changes what the imager sees", "[sim][imager]") {
    World w(lambert(0.5));

    const auto before = w.imager.capture(w.request({kCentre}));
    REQUIRE(before.has_value());

    // 光源アームを別の姿勢へ動かすと、同じ点でも入射条件が変わる
    REQUIRE(w.left.moveJ(jointsFromDegrees(60, 0, 90, 0, 0, 0), 1.0).has_value());
    const auto after = w.imager.capture(w.request({kCentre}));
    REQUIRE(after.has_value());

    CHECK(std::abs(before->spots[0].values(0) - after->spots[0].values(0)) > 1e-6 * before->spots[0].values(0));
}

TEST_CASE("noise is reproducible with a fixed seed and absent when disabled", "[sim][imager]") {
    SECTION("no noise: repeated captures are identical") {
        World w(lambert(0.5));
        const auto a = w.imager.capture(w.request({kCentre}));
        const auto b = w.imager.capture(w.request({kCentre}));
        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        CHECK((a->spots[0].values - b->spots[0].values).norm() == 0.0);
    }

    SECTION("with noise: same seed gives the same first capture, a repeat differs") {
        MockImagerOptions options;
        options.noise_relative = 0.05;
        options.seed = 7;
        World w1(lambert(0.5), options);
        World w2(lambert(0.5), options);

        const auto a = w1.imager.capture(w1.request({kCentre}));
        const auto b = w2.imager.capture(w2.request({kCentre}));
        const auto c = w1.imager.capture(w1.request({kCentre}));
        REQUIRE(a.has_value());
        REQUIRE(b.has_value());
        REQUIRE(c.has_value());

        CHECK((a->spots[0].values - b->spots[0].values).norm() == 0.0);
        CHECK((a->spots[0].values - c->spots[0].values).norm() > 0.0);

        // ノイズは信号に対して 5% 程度(全波長の平均で確認)
        World clean(lambert(0.5));
        const auto base = clean.imager.capture(clean.request({kCentre}));
        REQUIRE(base.has_value());
        const Real rel = ((a->spots[0].values - base->spots[0].values).array()
                          / base->spots[0].values.array()).matrix().norm() / std::sqrt(401.0);
        CHECK(rel > 0.03);
        CHECK(rel < 0.07);
    }
}
