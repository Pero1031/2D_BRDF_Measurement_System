#include <cmath>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Commands.hpp"
#include "brdf/app/DeviceFactory.hpp"
#include "brdf/app/MeasurementSession.hpp"
#include "brdf/core/MathUtils.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace brdf;
using namespace brdf::app;

namespace {

    struct Fixture {
        io::SetupConfig setup;
        domain::Recipe recipe = defaultRecipe();
        Devices devices;

        Fixture() {
            auto d = makeDevices(setup);
            REQUIRE(d.has_value());
            devices = *d;
        }

        MeasurementSession session() { return MeasurementSession(geometry::RigGeometry(setup.rig), devices, recipe); }
    };

} // namespace

TEST_CASE("makeDevices: real mode is not supported yet", "[session]") {
    io::SetupConfig setup;
    setup.mode = io::DeviceMode::Real;
    const auto d = makeDevices(setup);
    REQUIRE_FALSE(d.has_value());
#if defined(BRDF_WITH_BCAP) && defined(BRDF_WITH_TOPCON)
    CHECK(d.error().code == ErrorCode::InvalidArgument);   // 作業用のフォルダがない(tests/test_real_run.cpp に、詳しいテスト)
#else
    CHECK(d.error().code == ErrorCode::NotImplemented);
#endif
}

TEST_CASE("a simulated session measures every pose and every spot", "[session]") {
    Fixture f;
    auto session = f.session();

    int calls = 0;
    const auto result = session.run([&](int, int, const domain::PoseStep&) { ++calls; });
    REQUIRE(result.has_value());

    // 3 ポーズ × 9 測定点
    REQUIRE(result->rows.size() == 27);
    CHECK(result->wavelengths.size() == 401);
    CHECK(calls == 3 + 1);   // 各ポーズの開始 + 終了

    for (const SpotMeasurement& m : result->rows) {
        CHECK(m.spectrum.size() == 401);
        CHECK(m.spectrum.minCoeff() > 0.0);
    }

    // 並び順: ポーズ順、その中で測定点順
    CHECK(result->rows[0].pose_id == "1");
    CHECK(result->rows[0].spot_id == 1);
    CHECK(result->rows[8].spot_id == 9);
    CHECK(result->rows[9].pose_id == "2");
}

TEST_CASE("session angles equal the independent geometry computation", "[session]") {
    Fixture f;
    const auto result = f.session().run();
    REQUIRE(result.has_value());

    const geometry::RigGeometry geometry(f.setup.rig);
    std::size_t row = 0;
    for (const domain::PoseStep& pose : f.recipe.poses) {
        for (const PatchAngles& p : computePoseAngles(geometry, pose, f.recipe.spots)) {
            const SpotMeasurement& m = result->rows[row++];
            CHECK(m.spot_id == p.spot_id);
            CHECK_THAT(m.angles.incidence.theta, WithinAbs(p.angles.incidence.theta, 1e-12));
            CHECK_THAT(m.angles.observation.phi, WithinAbs(p.angles.observation.phi, 1e-12));
        }
    }
}

TEST_CASE("the angles differ from spot to spot (the point of a 2D measurement)", "[session]") {
    Fixture f;
    const auto result = f.session().run();
    REQUIRE(result.has_value());

    Real min_ti = 1e9, max_ti = -1e9, min_to = 1e9, max_to = -1e9;
    for (std::size_t i = 0; i < 9; ++i) {   // 最初のポーズの 9 点
        const auto& a = result->rows[i].angles;
        min_ti = std::min(min_ti, a.incidence.theta);
        max_ti = std::max(max_ti, a.incidence.theta);
        min_to = std::min(min_to, a.observation.theta);
        max_to = std::max(max_to, a.observation.theta);
    }
    CHECK(max_ti - min_ti > toRadians(1.0));
    CHECK(max_to - min_to > toRadians(1.0));
}

TEST_CASE("the measured spectra follow cos(theta_i) / d^2 on a Lambert sample", "[session]") {
    // シミュレーション全体(アーム → 角度 → 画像位置 → 撮影)が、物理と整合していることの確認。
    // Lambert 面の分光値は、同じポーズ内の点どうしで cosθi / d²(光源までの距離)に比例する。
    Fixture f;
    const auto result = f.session().run();
    REQUIRE(result.has_value());

    const geometry::RigGeometry geometry(f.setup.rig);
    const domain::PoseStep& pose = f.recipe.poses[0];
    const Vec3 light = geometry.lightPosition(pose.left);
    const auto patches = computePoseAngles(geometry, pose, f.recipe.spots);

    auto factor = [&](std::size_t i) {
        const Real d = (light - patches[i].world_point).norm();
        return std::cos(patches[i].angles.incidence.theta) / (d * d);
    };

    const Real base = result->rows[0].spectrum(170) / factor(0);   // 550 nm
    for (std::size_t i = 1; i < 9; ++i) {
        CHECK_THAT(result->rows[i].spectrum(170), WithinRel(base * factor(i), 1e-9));
    }
}

TEST_CASE("a spot outside the camera image stops the session with the pose and the spot", "[session]") {
    Fixture f;
    f.recipe.spots = {{1, Vec2(0, 0)}, {2, Vec2(5000, 0)}};
    const auto result = f.session().run();

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::OutOfRange);
    CHECK_THAT(result.error().message, ContainsSubstring("pose \"1\""));
    CHECK_THAT(result.error().message, ContainsSubstring("spot 2"));
}

TEST_CASE("a rejected move stops the session, and the devices are disconnected", "[session]") {
    Fixture f;
    f.recipe.poses.push_back({"bad", geometry::jointsFromDegrees(0, 0, 200, 0, 0, 0),
                              geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0)});
    const auto result = f.session().run();

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::JointLimit);
    CHECK_THAT(result.error().message, ContainsSubstring("pose \"bad\""));
    CHECK_THAT(result.error().message, ContainsSubstring("left arm"));

    // 失敗しても、必ず切断される
    CHECK_FALSE(f.devices.left_arm->isConnected());
    CHECK_FALSE(f.devices.right_arm->isConnected());
    CHECK_FALSE(f.devices.imager->isConnected());
}

TEST_CASE("an incomplete device set is rejected", "[session]") {
    Fixture f;
    f.devices.imager.reset();
    const auto result = f.session().run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::InvalidArgument);
}
