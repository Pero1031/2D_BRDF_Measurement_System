#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Calibrate.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/io/SpotXml.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::io;
namespace fs = std::filesystem;

namespace {

    std::string readFile(const fs::path& p) {
        std::ifstream ifs(p, std::ios::binary);
        std::stringstream ss;
        ss << ifs.rdbuf();
        return ss.str();
    }

    std::string withoutCr(std::string s) {
        s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
        return s;
    }

    int count(const std::string& s, const std::string& what) {
        int n = 0;
        for (std::size_t p = s.find(what); p != std::string::npos; p = s.find(what, p + 1)) ++n;
        return n;
    }

    struct TempDir {
        fs::path path;
        explicit TempDir(const std::string& name) {
            path = fs::temp_directory_path() / ("brdf_test_" + name);
            fs::remove_all(path);
            fs::create_directories(path);
        }
        ~TempDir() { std::error_code ec; fs::remove_all(path, ec); }
    };

    void replaceInFile(const fs::path& p, const std::string& from, const std::string& to) {
        std::string t = readFile(p);
        const auto pos = t.find(from);
        REQUIRE(pos != std::string::npos);
        t.replace(pos, from.size(), to);
        std::ofstream(p, std::ios::binary) << t;
    }

} // namespace

TEST_CASE("spot XML: the previous system's file is read and written back identically", "[spotxml][legacy]") {
    // 旧システムが生成した 16 点のスポット定義ファイル。読み込んで、同じ設定で書き出すと、全く同じ内容になる。
    const std::string legacy = readFile(fs::path(BRDF_TEST_DATA_DIR) / "legacy_spot_1.xml");

    const auto spots = parseSpotXml(legacy);
    REQUIRE(spots.has_value());
    REQUIRE(spots->size() == 16);
    CHECK((*spots)[0].id == 1);
    CHECK_THAT((*spots)[0].pixel.x(), WithinAbs(486.0, 1e-12));
    CHECK_THAT((*spots)[0].pixel.y(), WithinAbs(485.0, 1e-12));
    CHECK_THAT((*spots)[1].pixel.x(), WithinAbs(509.0, 1e-12));

    SpotXmlSettings s;   // 旧ファイルのひな型の値
    s.roi_width = 1376;
    s.roi_height = 1024;
    s.binning = "BINNING_TYPE1";
    s.spot_size = 2.0;
    s.datetime = "2020/09/14 20:39:32.733";
    s.comment = "Random-Spot";

    const std::string written = toSpotXml(s, *spots);
    CHECK(withoutCr(written) == withoutCr(legacy));   // 改行コードの扱い(リポジトリでの正規化)を除いて、全く同じ
    CHECK(written.find("\r\n") != std::string::npos);  // 書き出しは旧ファイルと同じ CRLF
    CHECK(written.back() == '\n');
}

TEST_CASE("spot XML: structure, rounding and escaping", "[spotxml]") {
    std::vector<driver::SpotRequest> spots(3);
    spots[0] = {1, Vec2(100.4, 200.5)};
    spots[1] = {2, Vec2(100.6, 200.4)};
    spots[2] = {7, Vec2(0.0, 1023.0)};

    SpotXmlSettings s;
    s.comment = "a<b & \"c\"";
    const std::string xml = toSpotXml(s, spots);

    CHECK(count(xml, "<AreaData ") == 3);
    CHECK(count(xml, "</AreaData>") == 3);
    CHECK_THAT(xml, ContainsSubstring("id=\"7\""));
    // 最も近いピクセルへの丸め(旧システムは切り捨てだった)
    CHECK_THAT(xml, ContainsSubstring("<AreaCenter x=\"100\" y=\"201\"/>"));   // 100.4 → 100, 200.5 → 201
    CHECK_THAT(xml, ContainsSubstring("<AreaCenter x=\"101\" y=\"200\"/>"));   // 100.6 → 101, 200.4 → 200
    CHECK_THAT(xml, ContainsSubstring("<AreaSize width=\"2.00\" height=\"2.00\"/>"));
    CHECK_THAT(xml, ContainsSubstring("comment=\"a&lt;b &amp; &quot;c&quot;\""));

    // 書き出したものを読み直せる(座標は丸めた整数)
    const auto back = parseSpotXml(xml);
    REQUIRE(back.has_value());
    REQUIRE(back->size() == 3);
    CHECK_THAT((*back)[0].pixel.x(), WithinAbs(100.0, 1e-12));
    CHECK_THAT((*back)[1].pixel.x(), WithinAbs(101.0, 1e-12));
    CHECK((*back)[2].id == 7);
}

TEST_CASE("spot XML: an empty spot list and bad files", "[spotxml]") {
    const std::string xml = toSpotXml(SpotXmlSettings{}, {});
    CHECK(count(xml, "<AreaData ") == 0);
    CHECK(parseSpotXml(xml).value().empty());

    CHECK_THAT(parseSpotXml("<html></html>").error().message, ContainsSubstring("not a SpotDef"));
    CHECK_THAT(parseSpotXml("<SpotDef><AreaData id=\"1\"><AreaCenter x=\"1\" y=\"2\"/>").error().message,
               ContainsSubstring("not closed"));
    CHECK_THAT(parseSpotXml("<SpotDef><AreaData id=\"1\"></AreaData></SpotDef>").error().message,
               ContainsSubstring("no <AreaCenter>"));
}

TEST_CASE("spot XML: the datetime attribute", "[spotxml]") {
    CHECK(spotXmlDatetime("20261116_064647") == "2026/11/16 06:46:47.000");
    CHECK(spotXmlDatetime("not a timestamp") == "not a timestamp");
    CHECK(spotXmlDatetime("2026111_0646470") == "2026111_0646470");
}

TEST_CASE("brdf spots writes one definition file per pose from the calibration", "[spotxml][commands]") {
    TempDir tmp("spots");
    const fs::path dir = tmp.path / "p";
    REQUIRE(app::initProject(dir).has_value());

    SECTION("it needs the calibration-board projector") {
        const auto r = app::exportSpots(dir, "20260101_120000");
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("projector = \"board\""));
    }

    SECTION("with a calibration, every spot has a position inside the image") {
        replaceInFile(dir / "setup.toml", "projector = \"camera-model\"", "projector = \"board\"");
        REQUIRE(app::calibrateProject(dir).has_value());

        const auto r = app::exportSpots(dir, "20260101_120000");
        REQUIRE(r.has_value());
        CHECK(r->filename().string() == "20260101_120000_spots");

        for (const char* pose : {"1", "2", "3"}) {
            const fs::path f = *r / (std::string(pose) + "_spot.xml");
            REQUIRE(fs::exists(f));
            const std::string xml = readFile(f);
            CHECK_THAT(xml, ContainsSubstring("datetime=\"2026/01/01 12:00:00.000\""));
            CHECK_THAT(xml, ContainsSubstring("<MeasurementRoi x=\"0\" y=\"0\" width=\"1376\" height=\"1024\"/>"));

            const auto spots = parseSpotXml(xml);
            REQUIRE(spots.has_value());
            CHECK(spots->size() == 9);   // 3×3 の測定点
            for (const auto& s : *spots) {
                CHECK(s.pixel.x() >= 0.0);
                CHECK(s.pixel.x() < 1376.0);
                CHECK(s.pixel.y() >= 0.0);
                CHECK(s.pixel.y() < 1024.0);
            }
        }
    }

    SECTION("a spot that falls outside the image stops everything before any file is written") {
        replaceInFile(dir / "setup.toml", "projector = \"camera-model\"", "projector = \"board\"");
        REQUIRE(app::calibrateProject(dir).has_value());
        replaceInFile(dir / "recipe.toml", "x = -20.0\ny = -20.0", "x = 900.0\ny = -20.0");   // 画像の外へ出る点

        const auto r = app::exportSpots(dir, "20260101_120000");
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("spot 1"));
        CHECK_FALSE(fs::exists(dir / "result" / "20260101_120000_spots"));
    }
}
