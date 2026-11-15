#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Commands.hpp"
#include "brdf/app/ViewExport.hpp"

using Catch::Matchers::ContainsSubstring;
using namespace brdf;
using namespace brdf::app;
namespace fs = std::filesystem;

namespace {

    int count(const std::string& s, const std::string& what) {
        int n = 0;
        for (std::size_t p = s.find(what); p != std::string::npos; p = s.find(what, p + what.size())) ++n;
        return n;
    }

    LoadedProject sampleProject() {
        LoadedProject p;
        p.recipe = defaultRecipe();
        return p;
    }

} // namespace

TEST_CASE("the animation HTML is self-contained and has the data embedded", "[view]") {
    const auto html = buildViewHtml(sampleProject());
    REQUIRE(html.has_value());

    CHECK_THAT(*html, ContainsSubstring("<!DOCTYPE html>"));
    CHECK_THAT(*html, ContainsSubstring("const D = {\"name\":\"example\""));
    CHECK(html->find("/*__BRDF_DATA__*/") == std::string::npos);   // 差し込み済み

    // 外部のライブラリやネットワークを使わない(オフラインで開ける)
    CHECK(html->find("http://") == std::string::npos);
    CHECK(html->find("https://") == std::string::npos);
    CHECK(html->find("src=\"") == std::string::npos);
    CHECK(html->find("<link") == std::string::npos);
}

TEST_CASE("the frames follow the session's order: left arm, then right arm, then the measurement", "[view]") {
    ViewOptions opt;
    opt.steps_per_move = 5;
    const auto html = buildViewHtml(sampleProject(), opt);
    REQUIRE(html.has_value());

    // 既定のレシピ: 3 ポーズ。右アームは最初のポーズだけ動き(以降は同じ)、左アームは毎回動く。
    CHECK(count(*html, "\"s\":\"S\"") == 1);              // 初期位置
    CHECK(count(*html, "\"s\":\"M\"") == 3);              // 測定は 1 ポーズに 1 コマ
    CHECK(count(*html, "\"s\":\"L\"") == 3 * 5);          // 左: 3 回 × 5 コマ
    CHECK(count(*html, "\"s\":\"R\"") == 1 * 5);          // 右: 最初の 1 回だけ

    // 最初のポーズでは、左 → 右 の順
    const auto firstL = html->find("\"s\":\"L\"");
    const auto firstR = html->find("\"s\":\"R\"");
    const auto firstM = html->find("\"s\":\"M\"");
    CHECK(firstL < firstR);
    CHECK(firstR < firstM);
}

TEST_CASE("more steps give a smoother, bigger animation", "[view]") {
    ViewOptions a, b;
    a.steps_per_move = 3;
    b.steps_per_move = 30;
    const auto small = buildViewHtml(sampleProject(), a);
    const auto big = buildViewHtml(sampleProject(), b);
    REQUIRE(small.has_value());
    REQUIRE(big.has_value());
    CHECK(big->size() > small->size());
    CHECK(count(*big, "\"s\":\"L\"") == 3 * 30);
}

TEST_CASE("every frame has the data the viewer needs", "[view]") {
    const auto html = buildViewHtml(sampleProject());
    REQUIRE(html.has_value());
    const int frames = count(*html, "\"p\":") - 0;   // "p" は、フレームのポーズ番号(ポーズの定義には使わない)
    CHECK(frames == count(*html, "\"ld\":"));
    CHECK(frames == count(*html, "\"rd\":"));
    CHECK(frames == count(*html, "\"d\":"));
    CHECK(frames == count(*html, "\"a\":["));
    for (const char* key : {"\"clearance\"", "\"limits\"", "\"plate\"", "\"camera\"", "\"frustum\"", "\"spots\"", "\"poses\"", "\"issues\""}) {
        CHECK_THAT(*html, ContainsSubstring(key));
    }
}

TEST_CASE("names with HTML special characters cannot break out of the script", "[view]") {
    LoadedProject p = sampleProject();
    p.recipe.name = "</script><img src=x onerror=alert(1)>";
    p.recipe.poses[0].id = "a\"b";
    const auto html = buildViewHtml(p);
    REQUIRE(html.has_value());

    CHECK(html->find("</script><img") == std::string::npos);
    CHECK_THAT(*html, ContainsSubstring("\\u003c/script\\u003e"));
    CHECK_THAT(*html, ContainsSubstring("a\\\"b"));
    // ひな型の <script> は 1 組だけ
    CHECK(count(*html, "<script>") == 1);
    CHECK(count(*html, "</script>") == 1);
}

TEST_CASE("the pre-flight findings are in the data, and a dangerous movement between poses is pointed out", "[view]") {
    LoadedProject p = sampleProject();

    SECTION("a joint outside its limits") {
        p.recipe.poses[1].left(2) = 3.5;   // J3 = 3.5 rad(約 200°)は可動域外
        const auto html = buildViewHtml(p);
        REQUIRE(html.has_value());
        CHECK_THAT(*html, ContainsSubstring("outside the limits"));
    }
    SECTION("the arms come close while moving") {
        // 右アームを左アームのすぐ隣へ置く配置。動いている途中の距離も、ポーズ単位で指摘される
        p.setup.rig.right_base_position = Vec3(0, 60, 0);
        const auto html = buildViewHtml(p);
        REQUIRE(html.has_value());
        CHECK_THAT(*html, ContainsSubstring("動いている途中で、左右のアームが"));
        CHECK_THAT(*html, ContainsSubstring("come within"));   // 到達時の指摘(事前検査)も含まれる
    }
    SECTION("a clean recipe has no complaint about the path") {
        const auto html = buildViewHtml(p);
        REQUIRE(html.has_value());
        CHECK(html->find("動いている途中で") == std::string::npos);
    }
}

TEST_CASE("brdf view writes the file under result/", "[view]") {
    const fs::path dir = fs::temp_directory_path() / "brdf_test_view";
    fs::remove_all(dir);
    REQUIRE(initProject(dir).has_value());

    ViewOptions opt;
    opt.timestamp = "20260101_120000";
    const auto file = exportView(dir, opt);
    REQUIRE(file.has_value());
    CHECK(file->filename().string() == "view.html");
    CHECK(file->parent_path().filename().string() == "20260101_120000_view");

    std::ifstream ifs(*file, std::ios::binary);
    std::stringstream ss;
    ss << ifs.rdbuf();
    CHECK_THAT(ss.str(), ContainsSubstring("\"generated\":\"2026/01/01 12:00\""));

    std::error_code ec;
    fs::remove_all(dir, ec);
}
