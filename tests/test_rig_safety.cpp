#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/geometry/JointLimits.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"
#include "brdf/geometry/RigGeometry.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::geometry;

TEST_CASE("JointLimits finds the first joint outside its range", "[safety]") {
    const JointLimits limits;

    CHECK(limits.firstViolation(jointsFromDegrees(0, 0, 90, 0, 0, 0)) == -1);
    CHECK(limits.firstViolation(jointsFromDegrees(161, 0, 90, 0, 0, 0)) == 0);
    CHECK(limits.firstViolation(jointsFromDegrees(0, 0, 10, 0, 0, 0)) == 2);       // J3 は 19° 以上
    CHECK(limits.firstViolation(jointsFromDegrees(0, 0, 90, 0, 130, 0)) == 4);
    CHECK(limits.firstViolation(jointsFromDegrees(200, 0, 90, 0, 130, 0)) == 0);   // 最初の違反を返す
}

TEST_CASE("default limits and links follow the DENSO VP-6242 documents", "[safety]") {
    // ソフトウェアリミット出荷時設定値 (J1 ±160, J2 ±120, J3 +19..+160, J4 ±160, J5 ±120, J6 ±360)
    const JointLimits limits;
    const Real expected_lower[6] = {-160, -120, 19, -160, -120, -360};
    const Real expected_upper[6] = {160, 120, 160, 160, 120, 360};
    for (int i = 0; i < 6; ++i) {
        CHECK_THAT(toDegrees(limits.lower(i)), WithinAbs(expected_lower[i], 1e-9));
        CHECK_THAT(toDegrees(limits.upper(i)), WithinAbs(expected_upper[i], 1e-9));
    }
    // J3=90° が前腕水平。外形図の余裕 71°(上向き)と 70°(下向き)と一致する
    CHECK_THAT(90.0 - toDegrees(limits.lower(2)), WithinAbs(71.0, 1e-9));
    CHECK_THAT(toDegrees(limits.upper(2)) - 90.0, WithinAbs(70.0, 1e-9));

    // 外形図の寸法: 土台 280、J2-J3 間 210 と 75、前腕 210、手首〜ツール取付面 70
    const ArmLinks links;
    CHECK(links.base_height == 280.0);
    CHECK(links.upper_arm == 210.0);
    CHECK(links.elbow_offset == 75.0);
    CHECK(links.forearm == 210.0);
    CHECK(links.wrist == 70.0);
}

TEST_CASE("linkPoints agrees with tipPose", "[safety]") {
    const Rc8Kinematics kin;
    const Joints q = jointsFromDegrees(-40, 15, 85, 10, -20, 30);

    const auto points = kin.linkPoints(q);
    const Isometry3 tip = kin.tipPose(q);

    CHECK_THAT((points[0] - Vec3::Zero()).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((points[1] - Vec3(0, 0, 280)).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((points[5] - tip.translation()).norm(), WithinAbs(0.0, 1e-9));

    // 各リンクの長さは寸法のとおり(土台 280、第1アーム 210、ジョイント 75、第2アーム 210、手首 70)
    CHECK_THAT((points[1] - points[0]).norm(), WithinAbs(280.0, 1e-9));
    CHECK_THAT((points[2] - points[1]).norm(), WithinAbs(210.0, 1e-9));
    CHECK_THAT((points[3] - points[2]).norm(), WithinAbs(75.0, 1e-9));
    CHECK_THAT((points[4] - points[3]).norm(), WithinAbs(210.0, 1e-9));
    CHECK_THAT((points[5] - points[4]).norm(), WithinAbs(70.0, 1e-9));
}

TEST_CASE("arm segments include the link from the tip to the light / the sample", "[safety]") {
    const RigGeometry rig;
    const Joints q = jointsFromDegrees(0, 0, 90, 0, 0, 0);

    const auto left = rig.leftArmSegments(q);
    CHECK(left.size() == 6);
    CHECK_THAT((left.back().b - rig.lightPosition(q)).norm(), WithinAbs(0.0, 1e-9));

    const auto right = rig.rightArmSegments(q);
    CHECK_THAT((right.back().b - rig.sampleHolderPose(q).translation()).norm(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(right.front().a.y(), WithinAbs(945.0, 1e-12));   // 右アームの土台位置
}

TEST_CASE("minArmDistance: far apart arms and arms placed on top of each other", "[safety]") {
    const Joints home = jointsFromDegrees(0, 0, 90, 0, 0, 0);

    SECTION("default layout: the closest point is the light, which sticks out toward the right arm") {
        const RigGeometry rig;
        // 光源は左アーム先端から +Y 方向(右アーム側)へ 108 mm 張り出していて、高さは先端より 30 mm 上。
        // 右アームの先端は y=945 なので、最短距離は √(837² + 30²) = 837.54 mm。
        CHECK_THAT(rig.minArmDistance(home, home), WithinAbs(std::sqrt(837.0 * 837.0 + 30.0 * 30.0), 1e-6));
    }

    SECTION("a right arm placed 60 mm away from the left arm") {
        RigSetup setup;
        setup.right_base_position = Vec3(0, 60, 0);
        const RigGeometry rig(setup);
        // 腕どうしが 60 mm、さらに光源が右アーム側へ張り出すので、もっと近づく
        CHECK(rig.minArmDistance(home, home) < 60.0);
        CHECK(rig.minArmDistance(home, home) < 2 * setup.arm_radius);
    }
}
