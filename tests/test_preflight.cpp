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

    /// 到達点はどちらも安全で、動く途中でだけ近づく例: 右アームを J1 = -90° で止めて、左アームの J1 を -90° → 90° に動かす。
    /// しきい値(2 × 半径)を、途中の距離と到達点の距離の間に置く。
    struct MidpathCase {
        geometry::Joints left_from = jointsFromDegrees(-90, 0, 90, 0, 0, 0);
        geometry::Joints left_to = jointsFromDegrees(90, 0, 90, 0, 0, 0);
        geometry::Joints right = jointsFromDegrees(-90, 0, 90, 0, 0, 0);
        geometry::RigSetup setup;
        Real at_start = 0, at_end = 0, in_middle = 0;

        MidpathCase() {
            const geometry::RigGeometry probe;
            at_start = probe.minArmDistance(left_from, right);
            at_end = probe.minArmDistance(left_to, right);
            in_middle = at_start;
            for (int k = 0; k <= 20; ++k) {
                in_middle = std::min(in_middle, probe.minArmDistance(left_from + (left_to - left_from) * (k / 20.0), right));
            }
            setup.arm_radius = (std::min(at_start, at_end) + in_middle) / 4.0;
        }
    };

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
    REQUIRE(report.errorCount() >= 1);
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

TEST_CASE("a collision in the middle of a move is detected even when both ends are safe", "[preflight]") {
    const MidpathCase c;
    REQUIRE(c.in_middle < std::min(c.at_start, c.at_end) - 5.0);   // 途中のほうが、はっきり近い

    domain::Recipe r = defaultRecipe();
    r.spots = {{1, Vec2(0, 0)}};
    r.poses = {{"first", c.left_from, c.right}, {"second", c.left_to, c.right}};

    const geometry::RigGeometry geometry(c.setup);
    // どちらの姿勢も、到達点としては安全
    REQUIRE(geometry.minArmDistance(c.left_from, c.right) > 2 * c.setup.arm_radius);
    REQUIRE(geometry.minArmDistance(c.left_to, c.right) > 2 * c.setup.arm_radius);

    const auto report = preflight(geometry, r);
    REQUIRE(report.errorCount() == 1);
    const auto found = std::find_if(report.issues.begin(), report.issues.end(),
                                    [](const Issue& x) { return x.severity == Severity::Error; });
    REQUIRE(found != report.issues.end());
    const Issue& i = *found;
    CHECK(i.code == ErrorCode::Collision);
    CHECK(i.pose_id == "second");
    CHECK_THAT(i.message, ContainsSubstring("while moving the left arm"));

    // 途中を調べる点がなければ(到達点だけなら)、見逃す
    PreflightOptions endpoints_only;
    endpoints_only.path_samples = 1;
    CHECK(preflight(geometry, r, endpoints_only).errorCount() == 0);
}

TEST_CASE("every pose of the previous system is inside the previous-system range", "[preflight]") {
    const auto poses = io::loadLegacyPoseCsv(std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv");
    REQUIRE(poses.has_value());
    domain::Recipe r = defaultRecipe();
    r.poses = *poses;

    PreflightOptions options;
    options.warn_outside_previous_range = true;
    options.previous_range_margin_deg = 0.0;   // 余裕なしでも、旧システムの姿勢は旧システムの範囲の中
    const auto report = preflight(geometry::RigGeometry{}, r, options);
    for (const Issue& i : report.issues) CHECK(i.message.find("previous system used") == std::string::npos);
}

TEST_CASE("a pose outside the previous-system range is a warning, only when asked", "[preflight]") {
    // 左アームの J2 = 70°(旧システムは -20〜40°)
    const auto left = jointsFromDegrees(65, 70, 90, 0, 0, 0);
    const auto right = jointsFromDegrees(-45, 0, 90, 0, 0, 0);
    const geometry::RigGeometry geometry;
    const domain::Recipe r = oneSpotRecipe(left, right);

    SECTION("not asked: no such warning") {
        const auto report = preflight(geometry, r);
        for (const Issue& i : report.issues) CHECK(i.message.find("previous system used") == std::string::npos);
    }
    SECTION("asked: a warning that names the arm, the joint and the previous range") {
        PreflightOptions options;
        options.warn_outside_previous_range = true;
        const auto report = preflight(geometry, r, options);
        const auto it = std::find_if(report.issues.begin(), report.issues.end(), [](const Issue& i) {
            return i.message.find("previous system used") != std::string::npos;
        });
        REQUIRE(it != report.issues.end());
        CHECK(it->severity == Severity::Warning);
        CHECK_THAT(it->message, ContainsSubstring("left arm: J2"));
        CHECK_THAT(it->message, ContainsSubstring("[-20, 40]"));
    }
    SECTION("within the margin: no warning") {
        PreflightOptions options;
        options.warn_outside_previous_range = true;
        options.previous_range_margin_deg = 35.0;   // 40 + 35 = 75° まで許す
        const auto report = preflight(geometry, r, options);
        for (const Issue& i : report.issues) CHECK(i.message.find("previous system used") == std::string::npos);
    }
}

TEST_CASE("the start path check starts from where the arms really are", "[preflight][start]") {
    const geometry::RigGeometry geometry;
    const domain::PoseStep first{"first", jointsFromDegrees(65, 0, 90, 0, 0, 0), jointsFromDegrees(-45, 0, 90, 0, 0, 0)};

    SECTION("arms already at the first pose: nothing to report") {
        const auto report = checkStartPath(geometry, first.left, first.right, first);
        CHECK(report.ok());
    }
    SECTION("a current position outside the joint limits") {
        const auto report = checkStartPath(geometry, jointsFromDegrees(65, 0, 10, 0, 0, 0), first.right, first);
        REQUIRE_FALSE(report.ok());
        const Issue& i = report.issues.front();
        CHECK(i.code == ErrorCode::JointLimit);
        CHECK_THAT(i.message, ContainsSubstring("left arm is at J3"));
        CHECK_THAT(i.message, ContainsSubstring("from the current position"));
    }
    SECTION("arms that are already too close") {
        geometry::RigSetup close;   // 右アームの土台を左アームのすぐそばに置く(上の干渉のテストと同じ)
        close.right_base_position = Vec3(0, 60, 0);
        const auto home = jointsFromDegrees(0, 0, 90, 0, 0, 0);
        const auto report = checkStartPath(geometry::RigGeometry(close), home, home, first);
        CHECK(hasIssue(report, ErrorCode::Collision, Severity::Error));
        const auto it = std::find_if(report.issues.begin(), report.issues.end(),
                                     [](const Issue& i) { return i.message.find("already") != std::string::npos; });
        CHECK(it != report.issues.end());
    }
}

TEST_CASE("the start path check also looks in the middle of the first move", "[preflight][start]") {
    const MidpathCase c;
    const geometry::RigGeometry geometry(c.setup);
    const domain::PoseStep first{"first", c.left_to, c.right};

    // 実際の位置は左 J1 = -90°。最初の姿勢は +90° → 到達点は安全でも、途中で近づく
    const auto report = checkStartPath(geometry, c.left_from, c.right, first);
    REQUIRE(report.errorCount() == 1);
    CHECK(report.issues.front().code == ErrorCode::Collision);
    CHECK_THAT(report.issues.front().message, ContainsSubstring("while moving the left arm to the first pose"));
}

TEST_CASE("moving one arm is checked against where the other arm really is", "[preflight][start]") {
    const MidpathCase c;
    const geometry::RigGeometry geometry(c.setup);

    SECTION("a move whose end is safe but whose middle is not") {
        const auto report = checkSingleArmMove(geometry, c.left_from, c.right, true, c.left_to);
        REQUIRE(report.errorCount() == 1);
        CHECK(report.issues.front().code == ErrorCode::Collision);
        CHECK_THAT(report.issues.front().message, ContainsSubstring("while the left arm moves to the target"));
    }
    SECTION("the same left move is fine when the right arm is elsewhere") {
        const auto far_right = jointsFromDegrees(90, 0, 90, 0, 0, 0);   // 左アームから遠ざかる側
        CHECK(checkSingleArmMove(geometry, c.left_from, far_right, true, c.left_to).ok());
    }
    SECTION("a target outside the limits") {
        const auto bad = jointsFromDegrees(-90, 0, 10, 0, 0, 0);
        const auto report = checkSingleArmMove(geometry, c.left_from, c.right, true, bad);
        REQUIRE_FALSE(report.ok());
        CHECK(report.issues.front().code == ErrorCode::JointLimit);
        CHECK_THAT(report.issues.front().message, ContainsSubstring("target of the left arm"));
    }
    SECTION("moving the right arm is checked the same way") {
        const auto report = checkSingleArmMove(geometry, c.left_to, c.right, false, c.right);   // 動かない移動: 安全
        CHECK(report.ok());
    }
}
