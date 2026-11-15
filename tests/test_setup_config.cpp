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
        CHECK(a.imager.device == b.imager.device);
        CHECK(a.imager.binning == b.imager.binning);
        CHECK(a.imager.lens == b.imager.lens);
        CHECK(a.imager.width() == b.imager.width());
        CHECK(a.imager.height() == b.imager.height());
        CHECK_THAT(a.spot_xml.spot_size, WithinAbs(b.spot_xml.spot_size, 1e-12));
        CHECK_THAT(a.analysis.white_reflectance, WithinAbs(b.analysis.white_reflectance, 1e-12));
        CHECK_THAT(a.analysis.min_wavelength_nm, WithinAbs(b.analysis.min_wavelength_nm, 1e-12));
        CHECK_THAT(a.analysis.max_wavelength_nm, WithinAbs(b.analysis.max_wavelength_nm, 1e-12));
        CHECK(a.analysis.irradiance_correction == b.analysis.irradiance_correction);
        CHECK_THAT(a.sim.white_reflectance, WithinAbs(b.sim.white_reflectance, 1e-12));
        CHECK_THAT(a.sim.camera.pixels_per_mm, WithinAbs(b.sim.camera.pixels_per_mm, 1e-9));
        REQUIRE(a.sim.camera.target.has_value() == b.sim.camera.target.has_value());
        if (a.sim.camera.target) checkVec(*a.sim.camera.target, *b.sim.camera.target);
        CHECK_THAT(a.sim.imager.light_intensity, WithinAbs(b.sim.imager.light_intensity, 1e-6));
        CHECK_THAT(a.sim.imager.noise_relative, WithinAbs(b.sim.imager.noise_relative, 1e-12));
        CHECK(a.sim.imager.seed == b.sim.imager.seed);
        CHECK_THAT(a.sim.imager.focus_tolerance_mm, WithinAbs(b.sim.imager.focus_tolerance_mm, 1e-9));
        CHECK(a.calibration.projector == b.calibration.projector);
        CHECK_THAT(a.calibration.max_rms_px, WithinAbs(b.calibration.max_rms_px, 1e-12));
        CHECK(a.calibration.board.columns == b.calibration.board.columns);
        CHECK(a.calibration.board.rows == b.calibration.board.rows);
        CHECK_THAT(a.calibration.board.pitch_x, WithinAbs(b.calibration.board.pitch_x, 1e-12));
        CHECK_THAT(a.calibration.board.first_y, WithinAbs(b.calibration.board.first_y, 1e-12));
        CHECK_THAT(a.calibration.board.circle_diameter, WithinAbs(b.calibration.board.circle_diameter, 1e-12));
        CHECK_THAT((a.calibration.sampleOrigin() - b.calibration.sampleOrigin()).norm(), WithinAbs(0.0, 1e-12));
        CHECK(a.calibration.board_u == b.calibration.board_u);
        CHECK(a.calibration.board_v == b.calibration.board_v);
        CHECK(a.sim.sample.model == b.sim.sample.model);
        CHECK_THAT(a.sim.sample.peak_nm, WithinAbs(b.sim.sample.peak_nm, 1e-9));
        CHECK_THAT(a.sim.sample.width_nm, WithinAbs(b.sim.sample.width_nm, 1e-9));
        CHECK_THAT(a.sim.sample.specular, WithinAbs(b.sim.sample.specular, 1e-9));
        CHECK_THAT(a.sim.sample.shininess, WithinAbs(b.sim.sample.shininess, 1e-9));
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
    c.calibration.projector = "board";                 // 実機はボード方式が必須
    c.calibration.max_rms_px = 1.5;
    c.calibration.board = calibration::BoardLayout::legacy2020();
    c.calibration.sample_origin = Vec2(120.5, 80.25);
    c.calibration.board_u = "-y";
    c.calibration.board_v = "+x";
    c.rig.camera_position = Vec3(1272.5, 710.25, 554.0);
    c.rig.right_base_position = Vec3(10, 950, -5);
    c.rig.arm_radius = 55.5;
    c.rig.joint_limits.upper(2) = toRadians(150.0);
    c.sim.camera.target = Vec3(100, 200, 300);
    c.imager.device = "SR5000";
    c.imager.binning = "BINNING_TYPE2";
    c.imager.lens = "WIDE";
    c.imager.image_width = 640;
    c.spot_xml.spot_size = 3.0;
    c.analysis.white_reflectance = 0.97;
    c.analysis.min_wavelength_nm = 400.0;
    c.analysis.max_wavelength_nm = 760.0;
    c.analysis.irradiance_correction = true;
    c.sim.white_reflectance = 0.95;
    c.sim.imager.noise_relative = 0.02;
    c.sim.imager.seed = 99;
    c.sim.sample.model = "phong";
    c.sim.sample.shininess = 120.0;

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
        CHECK_THAT(errorOf("[imager]\nimage_width = 0\n"), ContainsSubstring("image_width"));
        CHECK_THAT(errorOf("[sim.camera]\npixels_per_mm = 0\n"), ContainsSubstring("pixels_per_mm"));
    }
    SECTION("calibration settings") {
        CHECK_THAT(errorOf("[calibration]\nprojector = \"magic\"\n"), ContainsSubstring("camera-model"));
        CHECK_THAT(errorOf("[devices]\nmode = \"real\"\n"), ContainsSubstring("only for the simulator"));   // 実機に camera-model は不可
        CHECK_THAT(errorOf("[calibration]\nmax_rms_px = 0\n"), ContainsSubstring("max_rms_px"));
        CHECK_THAT(errorOf("[calibration.board]\ncolumns = 2\n"), ContainsSubstring("at least 3"));
        CHECK_THAT(errorOf("[calibration.board]\ncircle_diameter = 30\n"), ContainsSubstring("smaller than the pitch"));
        CHECK_THAT(errorOf("[calibration.board]\ncolumns = 30\n"), ContainsSubstring("do not fit on the paper"));
        CHECK_THAT(errorOf("[calibration.board]\npitch_xx = 25\n"), ContainsSubstring("unknown key 'pitch_xx'"));
        CHECK_THAT(errorOf("[calibration.board]\nboard_u = \"up\"\n"), ContainsSubstring("board_u"));
        CHECK_THAT(errorOf("[calibration.board]\nboard_u = \"+x\"\nboard_v = \"-x\"\n"), ContainsSubstring("different axes"));
    }
    SECTION("analysis settings") {
        CHECK_THAT(errorOf("[analysis]\nwhite_reflectance = 0\n"), ContainsSubstring("white_reflectance"));
        CHECK_THAT(errorOf("[analysis]\nwhite_reflectance = 3\n"), ContainsSubstring("white_reflectance"));
        CHECK_THAT(errorOf("[analysis]\nmin_wavelength_nm = 700\nmax_wavelength_nm = 500\n"), ContainsSubstring("smaller than max_wavelength_nm"));
        CHECK_THAT(errorOf("[analysis]\nirradiance_correction = \"yes\"\n"), ContainsSubstring("true or false"));
        CHECK_THAT(errorOf("[analysis]\nwhite_reflectence = 0.9\n"), ContainsSubstring("unknown key 'white_reflectence'"));
        CHECK_THAT(errorOf("[sim.white]\nreflectance = 0\n"), ContainsSubstring("reflectance"));
    }
    SECTION("imager settings") {
        CHECK_THAT(errorOf("[imager]\ndevice = \"SR9000\"\n"), ContainsSubstring("SR5100"));
        CHECK_THAT(errorOf("[imager]\nbinning = \"2\"\n"), ContainsSubstring("BINNING_TYPE1"));
        CHECK_THAT(errorOf("[imager]\nlens = \"zoom\"\n"), ContainsSubstring("STANDARD"));
        CHECK_THAT(errorOf("[spot_xml]\nbinning = \"BINNING_TYPE1\"\n"), ContainsSubstring("unknown key 'binning'"));
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

TEST_CASE("the image size follows the imager model and the binning", "[config][imager]") {
    SECTION("the default is the newest instrument, SR-5100") {
        const SetupConfig c;
        CHECK(c.imager.device == "SR5100");
        CHECK(c.imager.width() == 2448);
        CHECK(c.imager.height() == 2048);
    }
    SECTION("SR-5000 and the binning types") {
        SetupConfig c;
        c.imager.device = "SR5000";
        CHECK(c.imager.width() == 1376);
        CHECK(c.imager.height() == 1024);
        c.imager.binning = "BINNING_TYPE2";
        CHECK(c.imager.width() == 688);
        c.imager.device = "SR5100";
        CHECK(c.imager.width() == 1224);
        CHECK(c.imager.height() == 1024);
        c.imager.binning = "BINNING_TYPE4";
        CHECK(c.imager.width() == 612);
    }
    SECTION("an explicit size wins, and the size is read from the file") {
        const auto parsed = parseSetup("[imager]\ndevice = \"SR5000\"\nimage_width = 1000\n");
        REQUIRE(parsed.has_value());
        CHECK(parsed->imager.width() == 1000);
        CHECK(parsed->imager.height() == 1024);   // 省略した方は機種から
    }
    SECTION("changing the device in the file changes the derived size (the template writes the size only as a comment)") {
        const auto parsed = parseSetup(toToml(SetupConfig{}) + "");
        REQUIRE(parsed.has_value());
        CHECK(parsed->imager.width() == 2448);

        std::string text = toToml(SetupConfig{});
        const auto pos = text.find("device = \"SR5100\"");
        REQUIRE(pos != std::string::npos);
        text.replace(pos, std::string("device = \"SR5100\"").size(), "device = \"SR5000\"");
        const auto changed = parseSetup(text);
        REQUIRE(changed.has_value());
        CHECK(changed->imager.width() == 1376);
        CHECK(changed->imager.height() == 1024);
    }
}
