#include <algorithm>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Commands.hpp"
#include "brdf/app/Preflight.hpp"
#include "brdf/io/RecipeFile.hpp"

using Catch::Matchers::ContainsSubstring;
using namespace brdf;
using namespace brdf::app;
using geometry::jointsFromDegrees;

namespace {

    domain::Recipe oneSpotRecipe(const geometry::Joints& left, const geometry::Joints& right) {
        domain::Recipe r = defaultRecipe();
        r.spots = {{1, Vec2(0, 0)}};
        r.poses = {{"A", left, right}};
        return r;
    }

    bool hasIssue(const PreflightReport& report, ErrorCode code, Severity severity) {
        return std::any_of(report.issues.begin(), report.issues.end(),
                           [&](const Issue& i) { return i.code == code && i.severity == severity; });
    }

} // namespace

TEST_CASE("the default recipe passes the pre-flight check cleanly", "[preflight]") {
    const geometry::RigGeometry geometry;
    const PreflightReport report = preflight(geometry, defaultRecipe());

    CHECK(report.ok());
    CHECK(report.errorCount() == 0);
    CHECK(report.warningCount() == 0);
}

TEST_CASE("every pose of the previous system passes the joint limit and collision checks", "[preflight]") {
    // 旧システムで実際に使われた 52 ポーズ(実ファイル)。厳しすぎる設定で誤って弾いていないことの確認。
    const auto poses = io::loadLegacyPoseCsv(std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv");
    REQUIRE(poses.has_value());

    domain::Recipe r = defaultRecipe();
    r.poses = *poses;
    const PreflightReport report = preflight(geometry::RigGeometry{}, r);
    CHECK(report.errorCount() == 0);
}

TEST_CASE("a joint outside its limits is an error that names the pose and the joint", "[preflight]") {
    const geometry::RigGeometry geometry;
    const auto report = preflight(geometry, oneSpotRecipe(jointsFromDegrees(0, 0, 200, 0, 0, 0),
                                                          jointsFromDegrees(0, 0, 90, 0, 0, 0)));
    REQUIRE_FALSE(report.ok());
    REQUIRE(hasIssue(report, ErrorCode::JointLimit, Severity::Error));

    const Issue& i = report.issues.front();
    CHECK(i.pose_id == "A");
    CHECK(i.pose_index == 0);
    CHECK_THAT(i.message, ContainsSubstring("left arm"));
    CHECK_THAT(i.message, ContainsSubstring("J3"));
}

TEST_CASE("arms that come too close are a collision error", "[preflight]") {
    geometry::RigSetup setup;
    setup.right_base_position = Vec3(0, 60, 0);
    const geometry::RigGeometry geometry(setup);

    const auto home = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    const auto report = preflight(geometry, oneSpotRecipe(home, home));
    CHECK_FALSE(report.ok());
    CHECK(hasIssue(report, ErrorCode::Collision, Severity::Error));
}

TEST_CASE("a collision that happens only while the left arm moves first is detected", "[preflight]") {
    // 左アーム → 右アームの順に動かすので、「新しい左 + 前の右」の組み合わせも検査される。
    // 前のポーズでは右アームが左アーム側(-Y)へ伸びていて、新しいポーズでは遠ざかる(+Y)。
    // 新しい左アームが右アーム側へ振れると、到達時は離れているが、途中では近づく。
    const auto left_home = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    const auto left_toward = jointsFromDegrees(90, 0, 90, 0, 0, 0);     // +Y(右アーム側)へ振る
    const auto right_toward = jointsFromDegrees(-90, 0, 90, 0, 0, 0);   // 左アーム側へ伸ばす
    const auto right_away = jointsFromDegrees(90, 0, 90, 0, 0, 0);      // 左アームから遠ざかる側へ伸ばす

    const geometry::RigGeometry probe;
    const Real arrival = probe.minArmDistance(left_toward, right_away);
    const Real transit = probe.minArmDistance(left_toward, right_toward);
    REQUIRE(transit < arrival);

    // しきい値(2 × 半径)を、途中の距離と到達時の距離の間に置く。1 つ目のポーズも安全であること。
    geometry::RigSetup setup;
    setup.arm_radius = (arrival + transit) / 4.0;
    const geometry::RigGeometry geometry(setup);
    REQUIRE(geometry.minArmDistance(left_home, right_toward) > 2 * setup.arm_radius);

    domain::Recipe r = defaultRecipe();
    r.spots = {{1, Vec2(0, 0)}};
    r.poses = {{"first", left_home, right_toward},
               {"second", left_toward, right_away}};

    const auto report = preflight(geometry, r);
    REQUIRE(report.errorCount() == 1);
    const auto it = std::find_if(report.issues.begin(), report.issues.end(),
                                 [](const Issue& i) { return i.severity == Severity::Error; });
    REQUIRE(it != report.issues.end());
    CHECK(it->code == ErrorCode::Collision);
    CHECK(it->pose_id == "second");
    CHECK_THAT(it->message, ContainsSubstring("while moving"));
}

TEST_CASE("a light behind the sample surface is only a warning", "[preflight]") {
    // 旧システムの角度表で θ_in が 92° になる姿勢 (左 J1=65、右 -40,15,90,15,15,-10)
    const geometry::RigGeometry geometry;
    const auto report = preflight(geometry, oneSpotRecipe(jointsFromDegrees(65, 0, 90, 0, 0, 0),
                                                          jointsFromDegrees(-40, 15, 90, 15, 15, -10)));
    CHECK(report.ok());
    REQUIRE(hasIssue(report, ErrorCode::OutOfRange, Severity::Warning));
    CHECK_THAT(report.issues.front().message, ContainsSubstring("light is behind"));
}
