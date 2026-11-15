#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/core/MathUtils.hpp"
#include "brdf/io/SetupConfig.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::io;

namespace {

    void checkVec(const Vec3& a, const Vec3& b) { CHECK_THAT((a - b).norm(), WithinAbs(0.0, 1e-9)); }

    void checkSame(const SetupConfig& a, const SetupConfig& b) {
        CHECK(a.mode == b.mode);
        checkVec(a.rig.left_base_position, b.rig.left_base_position);
        checkVec(a.rig.right_base_position, b.rig.right_base_position);
        checkVec(a.rig.light_offset, b.rig.light_offset);
        checkVec(a.rig.camera_position, b.rig.camera_position);
        CHECK_THAT(a.rig.sample_surface_offset, WithinAbs(b.rig.sample_surface_offset, 1e-9));
        CHECK_THAT(a.rig.arm_radius, WithinAbs(b.rig.arm_radius, 1e-9));
        CHECK_THAT(a.rig.links.base_height, WithinAbs(b.rig.links.base_height, 1e-9));
        CHECK_THAT(a.rig.links.upper_arm, WithinAbs(b.rig.links.upper_arm, 1e-9));
        CHECK_THAT(a.rig.links.elbow_offset, WithinAbs(b.rig.links.elbow_offset, 1e-9));
        CHECK_THAT(a.rig.links.forearm, WithinAbs(b.rig.links.forearm, 1e-9));
        CHECK_THAT(a.rig.links.wrist, WithinAbs(b.rig.links.wrist, 1e-9));
        CHECK_THAT(a.rig.links.elbow_fixed_angle, WithinAbs(b.rig.links.elbow_fixed_angle, 1e-9));
        CHECK_THAT((a.rig.joint_limits.lower - b.rig.joint_limits.lower).norm(), WithinAbs(0.0, 1e-9));
        CHECK_THAT((a.rig.joint_limits.upper - b.rig.joint_limits.upper).norm(), WithinAbs(0.0, 1e-9));
        CHECK(a.camera.image_width == b.camera.image_width);
        CHECK(a.camera.image_height == b.camera.image_height);
        CHECK_THAT(a.camera.pixels_per_mm, WithinAbs(b.camera.pixels_per_mm, 1e-9));
        REQUIRE(a.camera.target.has_value() == b.camera.target.has_value());
        if (a.camera.target) checkVec(*a.camera.target, *b.camera.target);
        CHECK_THAT(a.imager.light_intensity, WithinAbs(b.imager.light_intensity, 1e-6));
        CHECK_THAT(a.imager.noise_relative, WithinAbs(b.imager.noise_relative, 1e-12));
        CHECK(a.imager.seed == b.imager.seed);
        CHECK_THAT(a.imager.focus_tolerance_mm, WithinAbs(b.imager.focus_tolerance_mm, 1e-9));
        CHECK(a.sample.model == b.sample.model);
        CHECK_THAT(a.sample.peak_nm, WithinAbs(b.sample.peak_nm, 1e-9));
        CHECK_THAT(a.sample.width_nm, WithinAbs(b.sample.width_nm, 1e-9));
        CHECK_THAT(a.sample.specular, WithinAbs(b.sample.specular, 1e-9));
        CHECK_THAT(a.sample.shininess, WithinAbs(b.sample.shininess, 1e-9));
    }

    std::string errorOf(const std::string& toml) {
        const auto r = parseSetup(toml, "setup.toml");
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
        return r.error().message;
    }

} // namespace

TEST_CASE("the default setup survives a write/read round trip", "[config]") {
    const SetupConfig original;
    const auto parsed = parseSetup(toToml(original));
    REQUIRE(parsed.has_value());
    checkSame(*parsed, original);
}

TEST_CASE("a modified setup survives a write/read round trip", "[config]") {
    SetupConfig c;
    c.mode = DeviceMode::Real;
    c.rig.camera_position = Vec3(1272.5, 710.25, 554.0);
    c.rig.right_base_position = Vec3(10, 950, -5);
    c.rig.arm_radius = 55.5;
    c.rig.joint_limits.upper(2) = toRadians(150.0);
    c.camera.target = Vec3(100, 200, 300);
    c.camera.image_width = 640;
    c.imager.noise_relative = 0.02;
    c.imager.seed = 99;
    c.sample.model = "phong";
    c.sample.shininess = 120.0;

    const auto parsed = parseSetup(toToml(c));
    REQUIRE(parsed.has_value());
    checkSame(*parsed, c);
}

TEST_CASE("a partial setup keeps the defaults for everything it omits", "[config]") {
    const auto parsed = parseSetup("[rig]\ncamera_position = [1272, 710, 554]\n");
    REQUIRE(parsed.has_value());

    checkVec(parsed->rig.camera_position, Vec3(1272, 710, 554));
    checkVec(parsed->rig.right_base_position, Vec3(0, 945, 0));      // 既定値
    CHECK(parsed->mode == DeviceMode::Sim);
    CHECK_THAT(parsed->rig.links.base_height, WithinAbs(280.0, 1e-12));
}

TEST_CASE("an empty setup file is valid and equals the defaults", "[config]") {
    const auto parsed = parseSetup("");
    REQUIRE(parsed.has_value());
    checkSame(*parsed, SetupConfig{});
}

TEST_CASE("setup errors name the file, the section and the key", "[config]") {
    SECTION("TOML syntax error has the line number") {
        CHECK_THAT(errorOf("[rig]\ncamera_position = [1, 2\n"), ContainsSubstring("setup.toml:"));
    }
    SECTION("wrong type") {
        const std::string m = errorOf("[rig]\ncamera_position = \"far\"\n");
        CHECK_THAT(m, ContainsSubstring("[rig]"));
        CHECK_THAT(m, ContainsSubstring("camera_position"));
    }
    SECTION("array of the wrong length") {
        CHECK_THAT(errorOf("[rig]\nlight_offset = [1, 2]\n"), ContainsSubstring("array of 3 numbers"));
    }
    SECTION("a mistyped key is reported instead of silently ignored") {
        const std::string m = errorOf("[arm.links]\nbase_hight = 280\n");
        CHECK_THAT(m, ContainsSubstring("unknown key 'base_hight'"));
        CHECK_THAT(m, ContainsSubstring("[arm.links]"));
    }
    SECTION("unknown section") {
        CHECK_THAT(errorOf("[camera]\nx = 1\n"), ContainsSubstring("unknown key 'camera'"));
    }
    SECTION("bad device mode") {
        CHECK_THAT(errorOf("[devices]\nmode = \"hardware\"\n"), ContainsSubstring("\"sim\" or \"real\""));
    }
    SECTION("limits must be ordered") {
        const std::string m = errorOf("[arm.limits]\nlower_deg = [0, 0, 0, 0, 0, 0]\nupper_deg = [10, 10, 10, 10, 10, -5]\n");
        CHECK_THAT(m, ContainsSubstring("lower_deg[6]"));
    }
    SECTION("non-positive values") {
        CHECK_THAT(errorOf("[arm]\nradius_mm = -1\n"), ContainsSubstring("radius_mm"));
        CHECK_THAT(errorOf("[arm.links]\nwrist = 0\n"), ContainsSubstring("link length"));
        CHECK_THAT(errorOf("[sim.camera]\nimage_width = 0\n"), ContainsSubstring("image_width"));
    }
    SECTION("unknown sample model") {
        CHECK_THAT(errorOf("[sim.sample]\nmodel = \"mirror\"\n"), ContainsSubstring("lambert"));
    }
}

TEST_CASE("loadSetup reports a missing file", "[config]") {
    const auto r = loadSetup("this/file/does/not/exist/setup.toml");
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("cannot open"));
}
