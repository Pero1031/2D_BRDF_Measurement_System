#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "brdf/geometry/Joints.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"
#include "brdf/geometry/RigGeometry.hpp"

using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::geometry;

namespace {

    void checkVec(const Vec3& actual, const Vec3& expected, Real tol = 1e-9) {
        CHECK_THAT(actual.x(), WithinAbs(expected.x(), tol));
        CHECK_THAT(actual.y(), WithinAbs(expected.y(), tol));
        CHECK_THAT(actual.z(), WithinAbs(expected.z(), tol));
    }

} // namespace

TEST_CASE("jointsFromDegrees stores radians", "[kinematics]") {
    const Joints q = jointsFromDegrees(90, -45, 180, 0, 30, -90);
    CHECK_THAT(q(0), WithinAbs(HALF_PI, 1e-12));
    CHECK_THAT(q(1), WithinAbs(-PI / 4, 1e-12));
    CHECK_THAT(q(2), WithinAbs(PI, 1e-12));
    CHECK_THAT(q(4), WithinAbs(PI / 6, 1e-12));
    CHECK_THAT(q(5), WithinAbs(-HALF_PI, 1e-12));
}

TEST_CASE("RC8 forward kinematics: arm stretched forward (J3 = 90 deg)", "[kinematics]") {
    // J=(0,0,90,0,0,0) は「腕を水平に前(+X)へ伸ばした」姿勢。
    // 高さ = 280 + 210 + 75 = 565、前方 = 210 + 70 = 280。先端の Z 軸は世界の +X を向く。
    const Rc8Kinematics kin;
    const Isometry3 t = kin.tipPose(jointsFromDegrees(0, 0, 90, 0, 0, 0));

    checkVec(t.translation(), Vec3(280, 0, 565));
    checkVec(t.linear() * Vec3::UnitZ(), Vec3(1, 0, 0));   // 先端の Z 軸
    checkVec(t.linear() * Vec3::UnitX(), Vec3(0, 0, -1));  // 先端の X 軸は下向き
    checkVec(t.linear() * Vec3::UnitY(), Vec3(0, 1, 0));
}

TEST_CASE("RC8 forward kinematics: J1 swings the arm around the base axis", "[kinematics]") {
    const Rc8Kinematics kin;
    const Isometry3 t = kin.tipPose(jointsFromDegrees(90, 0, 90, 0, 0, 0));

    checkVec(t.translation(), Vec3(0, 280, 565));
    checkVec(t.linear() * Vec3::UnitZ(), Vec3(0, 1, 0));
}

TEST_CASE("RC8 forward kinematics: J6 rolls about the tip axis only", "[kinematics]") {
    const Rc8Kinematics kin;
    const Isometry3 a = kin.tipPose(jointsFromDegrees(-30, 10, 80, 5, -12, 0));
    const Isometry3 b = kin.tipPose(jointsFromDegrees(-30, 10, 80, 5, -12, 37));

    checkVec(a.translation(), b.translation());
    checkVec(a.linear() * Vec3::UnitZ(), b.linear() * Vec3::UnitZ());
    // 軸まわりに 37° 回っている
    const Real cosine = (a.linear() * Vec3::UnitX()).dot(b.linear() * Vec3::UnitX());
    CHECK_THAT(cosine, WithinAbs(std::cos(toRadians(37.0)), 1e-9));
}

TEST_CASE("RC8 forward kinematics returns a proper rotation", "[kinematics]") {
    const Rc8Kinematics kin;
    const Isometry3 t = kin.tipPose(jointsFromDegrees(-47, 13, 83, -9, 21, 140));
    const Mat3 r = t.linear();

    CHECK_THAT((r.transpose() * r - Mat3::Identity()).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(r.determinant(), WithinAbs(1.0, 1e-12));
}

TEST_CASE("RigGeometry places the arms, the light and the sample holder in the world", "[kinematics]") {
    const RigGeometry rig;   // 既定値 = 旧システムの装置定数
    const Joints stretched = jointsFromDegrees(0, 0, 90, 0, 0, 0);

    SECTION("left tip is at the world origin's arm") {
        checkVec(rig.leftTipPose(stretched).translation(), Vec3(280, 0, 565));
    }

    SECTION("light is offset in the left tip frame") {
        // 先端の X 軸は下向き(0,0,-1)、Y 軸は +Y、Z 軸は +X。
        // offset (-30, 108, 0) を世界座標に直すと (0, 108, +30)(X軸が下向きなので -30 は +30 の高さ)。
        // 先端 (280, 0, 565) に足して (280, 108, 595)。
        checkVec(rig.lightPosition(stretched), Vec3(280, 108, 595));
    }

    SECTION("sample holder is shifted by base position and by the acrylic+paper thickness along tip Z") {
        // 右アーム先端 (280,0,565) + 土台 (0,945,0) + 厚み 6mm × 先端Z(=+X)
        checkVec(rig.sampleHolderPose(stretched).translation(), Vec3(286, 945, 565));
    }
}

TEST_CASE("paperFrame is a right-handed frame built from the tip axes", "[kinematics]") {
    const RigGeometry rig;
    const Joints q = jointsFromDegrees(-45, 12, 85, 0, -8, 20);

    const SurfaceFrame tip = rig.tipFrame(q);
    const SurfaceFrame paper = rig.paperFrame(q);

    CHECK_THAT(paper.axes.determinant(), WithinAbs(1.0, 1e-12));
    checkVec(paper.origin, tip.origin);

    checkVec(paper.xAxis(), tip.yAxis());     // 紙の X = 先端の +Y
    checkVec(paper.yAxis(), -tip.xAxis());    // 紙の Y = 先端の -X
    checkVec(paper.zAxis(), tip.zAxis());     // 紙の Z = 先端の +Z (法線)
}
