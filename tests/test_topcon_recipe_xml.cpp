#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/io/TopconRecipeXml.hpp"

using Catch::Matchers::ContainsSubstring;
using namespace brdf;
using namespace brdf::io;

namespace {

    std::string readFile(const std::filesystem::path& p) {
        std::ifstream ifs(p, std::ios::binary);
        std::stringstream ss;
        ss << ifs.rdbuf();
        return ss.str();
    }

    /// 改行コードの違い(git の自動変換)で落ちないように、CRLF を LF にそろえる。
    std::string lf(std::string s) {
        std::string out;
        for (const char c : s) {
            if (c != '\r') out += c;
        }
        return out;
    }

    /// 旧システムのレシピと同じ条件。
    TopconRecipeSettings legacySettings() {
        TopconRecipeSettings s;
        s.device = "SR5000";
        s.binning = "BINNING_TYPE1";
        s.lens = "STANDARD";
        s.measure_distance_mm = 1000;
        s.integration_time_ms = 0.0;       // AUTO
        s.wavelengths = {380.0, 780.0, 1.0};
        s.roi_width = 1376;
        s.roi_height = 1024;
        return s;
    }

} // namespace

TEST_CASE("the generated recipe is identical to the legacy recipe.xml for the same conditions", "[topcon][recipe]") {
    const std::string legacy = readFile(std::filesystem::path(BRDF_TEST_DATA_DIR) / "legacy_recipe.xml");
    REQUIRE_FALSE(legacy.empty());

    const std::string xml = toTopconRecipeXml(legacySettings());
    CHECK(lf(xml) == lf(legacy));
}

TEST_CASE("the recipe uses CRLF and tabs like the legacy file", "[topcon][recipe]") {
    const std::string xml = toTopconRecipeXml(legacySettings());
    CHECK_THAT(xml, ContainsSubstring("\r\n\t<BaseWhitePoint"));
    CHECK(xml.find("\n\n") == std::string::npos);
    CHECK(xml.substr(xml.size() - 2) == "\r\n");
}

TEST_CASE("the recipe carries the conditions the new system decides", "[topcon][recipe]") {
    TopconRecipeSettings s;
    s.device = "SR5100";
    s.binning = "BINNING_TYPE2";
    s.lens = "TELE";
    s.measure_distance_mm = 1234;
    s.wavelengths = {400.0, 700.0, 5.0};
    s.roi_width = 1224;
    s.roi_height = 1024;
    const std::string xml = toTopconRecipeXml(s);

    CHECK_THAT(xml, ContainsSubstring("<DeviceType type=\"SR5100\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<BinningType type=\"BINNING_TYPE2\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<LensType type=\"TELE\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<MeasureDistance mm=\"1234\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<MeasurementWaveLength min=\"400\" max=\"700\" step=\"5\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<MeasurementRange left=\"0\" top=\"0\" right=\"1224\" bottom=\"1024\"/>"));
    CHECK_THAT(xml, ContainsSubstring("<OptimizationArea left=\"0\" top=\"0\" right=\"1224\" bottom=\"1024\"/>"));
}

TEST_CASE("integration time: 0 is automatic, a positive value is manual", "[topcon][recipe]") {
    TopconRecipeSettings s = legacySettings();

    SECTION("automatic") {
        const std::string xml = toTopconRecipeXml(s);
        CHECK_THAT(xml, ContainsSubstring("integration_time_type=\"AUTO\" integration_time=\"100\""));
        CHECK(xml.find("MANUAL") == std::string::npos);
    }
    SECTION("manual, with a fraction") {
        s.integration_time_ms = 37.5;
        const std::string xml = toTopconRecipeXml(s);
        CHECK_THAT(xml, ContainsSubstring("integration_time_type=\"MANUAL\" integration_time=\"37.5\""));
        CHECK(xml.find("AUTO\" integration_time") == std::string::npos);
    }
    SECTION("manual, whole milliseconds") {
        s.integration_time_ms = 50.0;
        CHECK_THAT(toTopconRecipeXml(s), ContainsSubstring("integration_time_type=\"MANUAL\" integration_time=\"50\""));
    }
}

TEST_CASE("text in the attributes is escaped", "[topcon][recipe]") {
    TopconRecipeSettings s = legacySettings();
    s.comment = "a<b & \"c\"";
    const std::string xml = toTopconRecipeXml(s);
    CHECK_THAT(xml, ContainsSubstring("comment=\"a&lt;b &amp; &quot;c&quot;\""));
}
