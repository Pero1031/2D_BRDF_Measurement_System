#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Commands.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/io/RecipeFile.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::io;

namespace {

    const char* kMinimal = R"(
[[spots]]
x = 1.0
y = 2.0

[[poses]]
left  = [45, 0, 90, 0, 0, -30]
right = [-45, 0, 90, 0, 0, 0]
)";

    std::string errorOf(const std::string& toml) {
        const auto r = parseRecipe(toml, "recipe.toml");
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
        return r.error().message;
    }

} // namespace

TEST_CASE("a minimal recipe gets sensible defaults", "[recipe]") {
    const auto r = parseRecipe(kMinimal);
    REQUIRE(r.has_value());

    CHECK(r->kind == domain::MeasurementKind::Sample);
    CHECK_THAT(r->capture.integration_time_ms, WithinAbs(100.0, 1e-12));
    CHECK_THAT(r->capture.arm_speed, WithinAbs(0.5, 1e-12));
    CHECK(r->capture.wavelengths.count() == 401);

    REQUIRE(r->spots.size() == 1);
    CHECK(r->spots[0].id == 1);                 // 省略した ID は 1 からの連番
    CHECK_THAT(r->spots[0].sample_mm.y(), WithinAbs(2.0, 1e-12));

    REQUIRE(r->poses.size() == 1);
    CHECK(r->poses[0].id == "1");
    // 度で書かれた値がラジアンで保持される
    CHECK_THAT(r->poses[0].left(0), WithinAbs(toRadians(45.0), 1e-12));
    CHECK_THAT(r->poses[0].right(0), WithinAbs(toRadians(-45.0), 1e-12));
}

TEST_CASE("spot_grid expands to a regular grid centred on 'center'", "[recipe]") {
    const auto r = parseRecipe(R"(
[spot_grid]
center = [10.0, 20.0]
step = [10.0, 20.0]
count = [3, 2]

[[poses]]
left = [0, 0, 90, 0, 0, 0]
right = [0, 0, 90, 0, 0, 0]
)");
    REQUIRE(r.has_value());
    REQUIRE(r->spots.size() == 6);

    // x: 0, 10, 20 / y: 10, 30。行優先、ID は 1 から
    CHECK(r->spots[0].id == 1);
    CHECK_THAT(r->spots[0].sample_mm.x(), WithinAbs(0.0, 1e-12));
    CHECK_THAT(r->spots[0].sample_mm.y(), WithinAbs(10.0, 1e-12));
    CHECK_THAT(r->spots[2].sample_mm.x(), WithinAbs(20.0, 1e-12));
    CHECK_THAT(r->spots[3].sample_mm.y(), WithinAbs(30.0, 1e-12));
    CHECK(r->spots[5].id == 6);
}

TEST_CASE("spots and spot_grid can be combined; grid IDs continue after the listed ones", "[recipe]") {
    const auto r = parseRecipe(R"(
[[spots]]
id = 7
x = 0.0
y = 0.0

[spot_grid]
count = [2, 1]

[[poses]]
left = [0, 0, 90, 0, 0, 0]
right = [0, 0, 90, 0, 0, 0]
)");
    REQUIRE(r.has_value());
    REQUIRE(r->spots.size() == 3);
    CHECK(r->spots[0].id == 7);
    CHECK(r->spots[1].id == 8);
    CHECK(r->spots[2].id == 9);
}

TEST_CASE("the default recipe survives a write/read round trip", "[recipe]") {
    const domain::Recipe original = app::defaultRecipe();
    const auto parsed = parseRecipe(toToml(original));
    REQUIRE(parsed.has_value());

    CHECK(parsed->name == original.name);
    CHECK(parsed->kind == original.kind);
    CHECK(parsed->checklist == original.checklist);
    CHECK(parsed->description == original.description);

    REQUIRE(parsed->spots.size() == original.spots.size());
    for (std::size_t i = 0; i < original.spots.size(); ++i) {
        CHECK(parsed->spots[i].id == original.spots[i].id);
        CHECK_THAT((parsed->spots[i].sample_mm - original.spots[i].sample_mm).norm(), WithinAbs(0.0, 1e-9));
    }
    REQUIRE(parsed->poses.size() == original.poses.size());
    for (std::size_t i = 0; i < original.poses.size(); ++i) {
        CHECK(parsed->poses[i].id == original.poses[i].id);
        CHECK_THAT((parsed->poses[i].left - original.poses[i].left).norm(), WithinAbs(0.0, 1e-9));
        CHECK_THAT((parsed->poses[i].right - original.poses[i].right).norm(), WithinAbs(0.0, 1e-9));
    }
}

TEST_CASE("recipe errors name the problem", "[recipe]") {
    SECTION("no poses") {
        CHECK_THAT(errorOf("[[spots]]\nx = 0.0\ny = 0.0\n"), ContainsSubstring("no poses"));
    }
    SECTION("no spots") {
        CHECK_THAT(errorOf("[[poses]]\nleft = [0,0,90,0,0,0]\nright = [0,0,90,0,0,0]\n"), ContainsSubstring("no spots"));
    }
    SECTION("a pose without 'left' or 'right'") {
        const std::string head = "[[spots]]\nx = 0.0\ny = 0.0\n[[poses]]\n";
        CHECK_THAT(errorOf(head + "right = [0,0,90,0,0,0]\n"), ContainsSubstring("left"));
        CHECK_THAT(errorOf(head + "left = [0,0,90,0,0,0]\n"), ContainsSubstring("right"));
    }
    SECTION("a pose with the wrong number of joints") {
        const std::string m = errorOf(std::string(kMinimal) + "\n[[poses]]\nleft = [1,2,3,4,5]\nright = [0,0,90,0,0,0]\n");
        CHECK_THAT(m, ContainsSubstring("poses[2]"));
        CHECK_THAT(m, ContainsSubstring("6 numbers"));
    }
    SECTION("duplicate IDs") {
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[[spots]]\nid = 1\nx = 5.0\ny = 5.0\n"), ContainsSubstring("duplicate spot id 1"));
        CHECK_THAT(errorOf(std::string(kMinimal)
                           + "\n[[poses]]\nid = \"1\"\nleft = [0,0,90,0,0,0]\nright = [0,0,90,0,0,0]\n"),
                   ContainsSubstring("duplicate pose id"));
    }
    SECTION("capture settings") {
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[capture]\narm_speed = 0.0\n"), ContainsSubstring("arm_speed"));
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[capture]\narm_speed = 1.5\n"), ContainsSubstring("arm_speed"));
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[capture]\nintegration_time_ms = -1\n"), ContainsSubstring("integration_time_ms"));
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[capture]\nwavelength_end = 300\n"), ContainsSubstring("wavelength"));
    }
    SECTION("bad kind") {
        CHECK_THAT(errorOf(std::string("[recipe]\nkind = \"mirror\"\n") + kMinimal), ContainsSubstring("\"sample\", \"white\" or \"calib\""));
    }
    SECTION("a mistyped key") {
        CHECK_THAT(errorOf(std::string(kMinimal) + "\n[capture]\nintegratoin_time_ms = 50\n"),
                   ContainsSubstring("unknown key 'integratoin_time_ms'"));
    }
    SECTION("bad grid count") {
        CHECK_THAT(errorOf("[spot_grid]\ncount = [0, 3]\n[[poses]]\nleft=[0,0,90,0,0,0]\nright=[0,0,90,0,0,0]\n"),
                   ContainsSubstring("count"));
    }
}

TEST_CASE("legacy pose CSV: all 52 poses of the previous system are imported", "[recipe][legacy]") {
    const auto poses = loadLegacyPoseCsv(std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv");
    REQUIRE(poses.has_value());
    REQUIRE(poses->size() == 52);

    CHECK((*poses)[0].id == "1");
    CHECK((*poses)[51].id == "52");

    // 最初の行: RJ = (-45,0,90,0,0,0)、LJ = (45,0,90,0,0,-30)。RJ は右アーム(試料側)
    const auto& p = (*poses)[0];
    CHECK_THAT((p.right - geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0)).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((p.left - geometry::jointsFromDegrees(45, 0, 90, 0, 0, -30)).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("legacy pose CSV: format errors point to the line", "[recipe][legacy]") {
    SECTION("wrong header") {
        const auto r = parseLegacyPoseCsv("a,b,c\n1,2,3\n");
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("header"));
    }
    const std::string header = "RJ1,RJ2,RJ3,RJ4,RJ5,RJ6,LJ1,LJ2,LJ3,LJ4,LJ5,LJ6\n";
    SECTION("not a number") {
        const auto r = parseLegacyPoseCsv(header + "0,0,90,0,0,0,0,0,90,0,0,0\n0,0,x,0,0,0,0,0,90,0,0,0\n");
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring(":3:"));
    }
    SECTION("wrong number of columns") {
        const auto r = parseLegacyPoseCsv(header + "0,0,90\n");
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("expected 12 values"));
    }
    SECTION("header only") {
        CHECK_FALSE(parseLegacyPoseCsv(header).has_value());
    }
    SECTION("CRLF line endings and a BOM are accepted") {
        const auto r = parseLegacyPoseCsv("\xEF\xBB\xBFRJ1,RJ2,RJ3,RJ4,RJ5,RJ6,LJ1,LJ2,LJ3,LJ4,LJ5,LJ6\r\n"
                                          "-45,0,90,0,0,0,45,0,90,0,0,-30\r\n");
        REQUIRE(r.has_value());
        CHECK(r->size() == 1);
    }
}
