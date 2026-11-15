#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/SimulatedRig.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::sim;
using geometry::jointsFromDegrees;

namespace {

    struct PoseRow {
        geometry::Joints right;
        geometry::Joints left;
    };

    std::vector<PoseRow> readLegacyPoses() {
        std::ifstream ifs(std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv");
        REQUIRE(ifs.good());
        std::vector<PoseRow> rows;
        std::string line;
        std::getline(ifs, line);   // ヘッダ RJ1..RJ6,LJ1..LJ6
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string cell;
            std::vector<Real> v;
            while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
            REQUIRE(v.size() == 12);
            rows.push_back({jointsFromDegrees(v[0], v[1], v[2], v[3], v[4], v[5]),
                            jointsFromDegrees(v[6], v[7], v[8], v[9], v[10], v[11])});
        }
        return rows;
    }

} // namespace

TEST_CASE("MockRobotArm refuses to move before it is connected", "[sim][arm]") {
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm arm("left", rig, ArmSide::Left);

    CHECK_FALSE(arm.isConnected());
    const Status s = arm.moveJ(jointsFromDegrees(10, 0, 90, 0, 0, 0), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::NotConnected);
    CHECK_FALSE(arm.currentJoints().has_value());
}

TEST_CASE("MockRobotArm moves the arm in the rig", "[sim][arm]") {
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm left("left", rig, ArmSide::Left);
    MockRobotArm right("right", rig, ArmSide::Right);
    REQUIRE(left.connect().has_value());
    REQUIRE(right.connect().has_value());

    const auto target = jointsFromDegrees(65, 0, 90, 0, 0, 0);
    REQUIRE(left.moveJ(target, 1.0).has_value());

    REQUIRE(left.currentJoints().has_value());
    CHECK_THAT((*left.currentJoints() - target).norm(), WithinAbs(0.0, 1e-15));
    CHECK_THAT((rig->joints(ArmSide::Left) - target).norm(), WithinAbs(0.0, 1e-15));

    // もう一方のアームは動いていない
    CHECK_THAT((*right.currentJoints() - jointsFromDegrees(0, 0, 90, 0, 0, 0)).norm(), WithinAbs(0.0, 1e-15));

    // 光源の位置が、動いたアームに合わせて変わっている
    const auto expected = rig->geometry().lightPosition(target);
    CHECK_THAT((rig->lightPosition() - expected).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("MockRobotArm validates the speed", "[sim][arm]") {
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm arm("left", rig, ArmSide::Left);
    REQUIRE(arm.connect().has_value());

    const auto target = jointsFromDegrees(10, 0, 90, 0, 0, 0);
    for (const Real bad : {0.0, -0.5, 1.5}) {
        const Status s = arm.moveJ(target, bad);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
    }
    CHECK(arm.moveJ(target, 1.0).has_value());
}

TEST_CASE("a pose outside the joint limits is rejected and the state is kept", "[sim][arm]") {
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm arm("right", rig, ArmSide::Right);
    REQUIRE(arm.connect().has_value());

    const auto before = rig->joints(ArmSide::Right);
    const Status s = arm.moveJ(jointsFromDegrees(0, 0, 200, 0, 0, 0), 0.5);   // J3 の上限は 167°
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::JointLimit);
    CHECK_THAT((rig->joints(ArmSide::Right) - before).norm(), WithinAbs(0.0, 1e-15));
}

TEST_CASE("a pose that brings the arms together is rejected as a collision", "[sim][arm]") {
    geometry::RigSetup setup;
    setup.right_base_position = Vec3(0, 60, 0);   // 左アームのすぐ隣に右アームを置いた配置
    auto rig = std::make_shared<SimulatedRig>(setup);
    MockRobotArm left("left", rig, ArmSide::Left);
    REQUIRE(left.connect().has_value());

    const auto before = rig->joints(ArmSide::Left);
    const Status s = left.moveJ(jointsFromDegrees(0, 0, 90, 0, 0, 0), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::Collision);
    CHECK_THAT((rig->joints(ArmSide::Left) - before).norm(), WithinAbs(0.0, 1e-15));
}

TEST_CASE("emergency stop disconnects the arm", "[sim][arm]") {
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm arm("left", rig, ArmSide::Left);
    REQUIRE(arm.connect().has_value());

    arm.emergencyStop();
    CHECK_FALSE(arm.isConnected());
    const Status s = arm.moveJ(jointsFromDegrees(10, 0, 90, 0, 0, 0), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::NotConnected);

    CHECK(arm.connect().has_value());   // 再接続すれば動かせる
    CHECK(arm.moveJ(jointsFromDegrees(10, 0, 90, 0, 0, 0), 0.5).has_value());
}

TEST_CASE("every pose of the legacy measurement is reachable in the simulated rig", "[sim][arm]") {
    // 旧システムで実際に使われていたポーズ一覧(52 行)。仮の可動域・干渉の設定で、
    // これらが誤って拒否されないことの確認(拒否されるなら、仮の値が厳しすぎる)。
    auto rig = std::make_shared<SimulatedRig>();
    MockRobotArm left("left", rig, ArmSide::Left);
    MockRobotArm right("right", rig, ArmSide::Right);
    REQUIRE(left.connect().has_value());
    REQUIRE(right.connect().has_value());

    const auto rows = readLegacyPoses();
    REQUIRE(rows.size() == 52);

    for (std::size_t i = 0; i < rows.size(); ++i) {
        INFO("pose row " << i + 1);
        const Status l = left.moveJ(rows[i].left, 1.0);
        const Status r = right.moveJ(rows[i].right, 1.0);
        CHECK(l.has_value());
        CHECK(r.has_value());
    }
}
