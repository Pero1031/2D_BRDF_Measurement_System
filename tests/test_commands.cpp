#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/io/RecipeFile.hpp"
#include "brdf/io/SetupConfig.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::app;
namespace fs = std::filesystem;

namespace {

    // テストごとに作って、終わったら消す一時フォルダ
    struct TempDir {
        fs::path path;
        explicit TempDir(const std::string& name) {
            path = fs::temp_directory_path() / ("brdf_test_" + name);
            fs::remove_all(path);
            fs::create_directories(path);
        }
        ~TempDir() { std::error_code ec; fs::remove_all(path, ec); }
    };

    std::string readFile(const fs::path& p) {
        std::ifstream ifs(p, std::ios::binary);
        std::stringstream ss;
        ss << ifs.rdbuf();
        return ss.str();
    }

    void writeFile(const fs::path& p, const std::string& text) {
        std::ofstream ofs(p, std::ios::binary);
        ofs << text;
    }

    void replaceInFile(const fs::path& p, const std::string& from, const std::string& to) {
        std::string t = readFile(p);
        const auto pos = t.find(from);
        REQUIRE(pos != std::string::npos);
        t.replace(pos, from.size(), to);
        writeFile(p, t);
    }

    int countLines(const std::string& s) { return static_cast<int>(std::count(s.begin(), s.end(), '\n')); }

    std::size_t countColumns(const std::string& line) { return std::count(line.begin(), line.end(), ',') + 1; }

    std::vector<fs::path> resultFolders(const fs::path& project) {
        std::vector<fs::path> out;
        for (const auto& e : fs::directory_iterator(project / "result")) out.push_back(e.path());
        return out;
    }

    RunOptions yes(const std::string& timestamp = "20260101_120000") {
        RunOptions o;
        o.assume_yes = true;
        o.timestamp = timestamp;
        return o;
    }

} // namespace

TEST_CASE("init creates a project and refuses to overwrite it", "[commands]") {
    TempDir tmp("init");
    const fs::path dir = tmp.path / "proj";

    REQUIRE(initProject(dir).has_value());
    CHECK(fs::exists(dir / "setup.toml"));
    CHECK(fs::exists(dir / "recipe.toml"));
    CHECK(fs::is_directory(dir / "calibration"));
    CHECK(fs::is_directory(dir / "result"));

    // 作られたひな型は、そのまま読める
    CHECK(io::loadSetup(dir / "setup.toml").has_value());
    CHECK(io::loadRecipe(dir / "recipe.toml").has_value());

    // 2 回目は上書きしない(手で編集した内容を守る)
    writeFile(dir / "recipe.toml", "# edited by hand\n");
    const auto again = initProject(dir);
    REQUIRE_FALSE(again.has_value());
    CHECK_THAT(again.error().message, ContainsSubstring("already exists"));
    CHECK(readFile(dir / "recipe.toml") == "# edited by hand\n");
}

TEST_CASE("the template project config contains no absolute path", "[commands]") {
    TempDir tmp("noabs");
    REQUIRE(initProject(tmp.path / "p").has_value());

    for (const char* f : {"setup.toml", "recipe.toml"}) {
        const std::string text = readFile(tmp.path / "p" / f);
        CHECK(text.find(":\\") == std::string::npos);
        CHECK(text.find(":/") == std::string::npos);
    }
}

TEST_CASE("check passes on the template and reports errors for a bad recipe", "[commands]") {
    TempDir tmp("check");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    const auto good = checkProject(dir);
    REQUIRE(good.has_value());
    CHECK(good->ok());

    // J3 を可動域外にする
    replaceInFile(dir / "recipe.toml", "left = [45.0, 0.0, 90.0, 0.0, 0.0, -30.0]", "left = [45.0, 0.0, 200.0, 0.0, 0.0, -30.0]");
    const auto bad = checkProject(dir);
    REQUIRE(bad.has_value());
    CHECK_FALSE(bad->ok());
    CHECK(bad->errorCount() == 1);
    CHECK(bad->issues.front().code == ErrorCode::JointLimit);
    CHECK(bad->issues.front().pose_id == "1");
}

TEST_CASE("check reports a broken file with its name and line", "[commands]") {
    TempDir tmp("brokenfile");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    writeFile(dir / "setup.toml", "[rig\ncamera_position = 1\n");

    const auto r = checkProject(dir);
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("setup.toml:"));
}

TEST_CASE("run measures, saves the result and records how it was measured", "[commands]") {
    TempDir tmp("run");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    std::istringstream no_input;
    const auto run = runProject(dir, yes(), no_input);
    REQUIRE(run.has_value());

    CHECK(run->filename().string() == "20260101_120000_sample");
    for (const char* f : {"measurements.csv", "run_manifest.toml", "setup.toml", "recipe.toml"}) {
        CHECK(fs::exists(*run / f));
    }

    // measurements.csv: ヘッダ + 27 行(3 ポーズ × 9 点)、列は 10 + 401
    const std::string csv = readFile(*run / "measurements.csv");
    CHECK(countLines(csv) == 28);
    const std::string header = csv.substr(0, csv.find('\n'));
    CHECK_THAT(header, ContainsSubstring("pose,spot,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg"));
    CHECK_THAT(header, ContainsSubstring("wl_380,wl_381"));
    CHECK_THAT(header, ContainsSubstring("wl_780"));
    CHECK(countColumns(header) == 10 + 401);
    const std::size_t second_line = csv.find('\n') + 1;
    CHECK(countColumns(csv.substr(second_line, csv.find('\n', second_line) - second_line)) == 10 + 401);

    // マニフェスト
    const std::string manifest = readFile(*run / "run_manifest.toml");
    CHECK_THAT(manifest, ContainsSubstring("kind = \"sample\""));
    CHECK_THAT(manifest, ContainsSubstring("device_mode = \"sim\""));
    CHECK_THAT(manifest, ContainsSubstring("row_count = 27"));
    CHECK_THAT(manifest, ContainsSubstring("timestamp = \"20260101_120000\""));

    // 設定のコピーは、実行時の内容そのもの
    CHECK(readFile(*run / "setup.toml") == readFile(dir / "setup.toml"));
    CHECK(readFile(*run / "recipe.toml") == readFile(dir / "recipe.toml"));
}

TEST_CASE("run: the checklist must be confirmed with OK", "[commands]") {
    TempDir tmp("checklist");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    SECTION("anything but OK cancels, and nothing is saved") {
        std::istringstream in("yes\n");
        RunOptions o;
        o.timestamp = "20260101_120000";
        const auto r = runProject(dir, o, in);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("cancelled"));
        CHECK(resultFolders(dir).empty());
    }
    SECTION("OK runs the measurement") {
        std::istringstream in("OK\n");
        RunOptions o;
        o.timestamp = "20260101_120000";
        CHECK(runProject(dir, o, in).has_value());
    }
}

TEST_CASE("run: --kind overrides the recipe and sets the folder name", "[commands]") {
    TempDir tmp("kind");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    RunOptions o = yes();
    o.kind = domain::MeasurementKind::White;
    std::istringstream in;
    const auto r = runProject(dir, o, in);
    REQUIRE(r.has_value());
    CHECK(r->filename().string() == "20260101_120000_white");
    CHECK_THAT(readFile(*r / "run_manifest.toml"), ContainsSubstring("kind = \"white\""));
}

TEST_CASE("run never overwrites an earlier result", "[commands]") {
    TempDir tmp("nooverwrite");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    std::istringstream in;
    const auto a = runProject(dir, yes(), in);
    const auto b = runProject(dir, yes(), in);
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    CHECK(*a != *b);
    CHECK(b->filename().string() == "20260101_120000_sample_2");
    CHECK(resultFolders(dir).size() == 2);
}

TEST_CASE("run refuses before moving anything when the pre-flight check fails", "[commands]") {
    TempDir tmp("preflightfail");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    replaceInFile(dir / "recipe.toml", "right = [-45.0, 0.0, 90.0, 0.0, 0.0, 0.0]", "right = [-45.0, 0.0, 200.0, 0.0, 0.0, 0.0]");

    std::istringstream in;
    const auto r = runProject(dir, yes(), in);
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("pre-flight"));
    CHECK(resultFolders(dir).empty());   // 失敗した測定の空フォルダを残さない
}

TEST_CASE("run in real mode fails clearly and saves nothing", "[commands]") {
    TempDir tmp("real");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    replaceInFile(dir / "setup.toml", "mode = \"sim\"", "mode = \"real\"");

    std::istringstream in;
    const auto r = runProject(dir, yes(), in);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::NotImplemented);
    CHECK(resultFolders(dir).empty());
}

TEST_CASE("run uses the camera position written in setup.toml", "[commands]") {
    // カメラ位置は測定日ごとに変わる値。setup.toml の値が結果に反映されることの確認。
    TempDir tmp("camera");
    const fs::path a = tmp.path / "a";
    const fs::path b = tmp.path / "b";
    REQUIRE(initProject(a).has_value());
    REQUIRE(initProject(b).has_value());
    replaceInFile(b / "setup.toml", "camera_position = [1136.0, 728.0, 560.0]", "camera_position = [1272.0, 710.0, 554.0]");

    std::istringstream in;
    const auto ra = runProject(a, yes(), in);
    const auto rb = runProject(b, yes(), in);
    REQUIRE(ra.has_value());
    REQUIRE(rb.has_value());
    CHECK(readFile(*ra / "measurements.csv") != readFile(*rb / "measurements.csv"));
    CHECK_THAT(readFile(*rb / "setup.toml"), ContainsSubstring("[1272.0, 710.0, 554.0]"));
}

TEST_CASE("angles writes the per-spot angles without any device", "[commands]") {
    TempDir tmp("angles");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    const auto r = exportAngles(dir, "20260101_120000");
    REQUIRE(r.has_value());
    CHECK(r->filename().string() == "20260101_120000_angles");

    const std::string csv = readFile(*r / "angles.csv");
    CHECK(countLines(csv) == 28);
    CHECK_THAT(csv, ContainsSubstring("pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg"));

    // 1 行目の値は、独立に計算した角度と一致する
    const auto setup = io::loadSetup(dir / "setup.toml");
    const auto recipe = io::loadRecipe(dir / "recipe.toml");
    REQUIRE(setup.has_value());
    REQUIRE(recipe.has_value());
    const auto patches = computePoseAngles(geometry::RigGeometry(setup->rig), recipe->poses[0], recipe->spots);

    std::stringstream ss(csv);
    std::string line;
    std::getline(ss, line);   // ヘッダ
    std::getline(ss, line);
    std::stringstream row(line);
    std::vector<std::string> cells;
    for (std::string c; std::getline(row, c, ',');) cells.push_back(c);
    REQUIRE(cells.size() == 8);
    CHECK_THAT(std::stod(cells[4]), WithinAbs(toDegrees(patches[0].angles.incidence.theta), 1e-6));
    CHECK_THAT(std::stod(cells[7]), WithinAbs(toDegrees(patches[0].angles.observation.phi), 1e-6));
}

TEST_CASE("import-poses turns the previous system's pose CSV into a recipe", "[commands]") {
    TempDir tmp("import");
    const fs::path out = tmp.path / "recipe.toml";
    const std::string csv = std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv";

    REQUIRE(importPoses(csv, out, false).has_value());
    const auto recipe = io::loadRecipe(out);
    REQUIRE(recipe.has_value());
    CHECK(recipe->poses.size() == 52);
    CHECK(recipe->name == "legacy_poses_52");

    // 既存のファイルは --force なしでは上書きしない
    const auto again = importPoses(csv, out, false);
    REQUIRE_FALSE(again.has_value());
    CHECK_THAT(again.error().message, ContainsSubstring("--force"));
    CHECK(importPoses(csv, out, true).has_value());
}

TEST_CASE("a project made from the legacy poses passes check", "[commands]") {
    // 旧システムの 52 ポーズを取り込んだレシピが、そのまま検査を通り、全ポーズを模擬測定できる。
    TempDir tmp("legacyrun");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    REQUIRE(importPoses(std::string(BRDF_TEST_DATA_DIR) + "/legacy_poses_52.csv", dir / "recipe.toml", true).has_value());

    const auto check = checkProject(dir);
    REQUIRE(check.has_value());
    CHECK(check->errorCount() == 0);

    std::istringstream in;
    const auto run = runProject(dir, yes(), in);
    REQUIRE(run.has_value());
    CHECK(countLines(readFile(*run / "measurements.csv")) == 1 + 52 * 9);
}
