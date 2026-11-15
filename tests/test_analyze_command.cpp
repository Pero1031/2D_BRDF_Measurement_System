#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Analyze.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/sim/BrdfModel.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace brdf;
using namespace brdf::app;
namespace fs = std::filesystem;

namespace {

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

    struct Csv {
        std::vector<std::string> header;
        std::vector<std::vector<std::string>> rows;
    };

    Csv readCsv(const fs::path& p) {
        Csv c;
        std::ifstream ifs(p);
        REQUIRE(ifs.good());
        std::string line;
        bool first = true;
        while (std::getline(ifs, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            std::vector<std::string> cells;
            std::stringstream ss(line);
            for (std::string cell; std::getline(ss, cell, ',');) cells.push_back(cell);
            if (first) { c.header = cells; first = false; }
            else c.rows.push_back(cells);
        }
        return c;
    }

    // 試料の分光反射率(setup.toml の既定値: 山形 550 nm、幅 60、高さ 0.8、底上げ 0.05)
    Real sampleAlbedo(Real wavelength) { return sim::gaussianCurve(550.0, 60.0, 0.8, 0.05)(wavelength); }

    constexpr Real kWhite = 0.99;   // シミュレータの標準拡散反射板の反射率(既定値)

    RunOptions run(domain::MeasurementKind kind, const std::string& timestamp) {
        RunOptions o;
        o.assume_yes = true;
        o.kind = kind;
        o.timestamp = timestamp;
        return o;
    }

    // 試料と白色板を測って、2 つの結果フォルダを返す
    struct TwoRuns {
        fs::path sample, white;
    };

    TwoRuns measureBoth(const fs::path& dir) {
        std::istringstream in;
        const auto s = runProject(dir, run(domain::MeasurementKind::Sample, "20260101_120000"), in);
        const auto w = runProject(dir, run(domain::MeasurementKind::White, "20260101_130000"), in);
        REQUIRE(s.has_value());
        REQUIRE(w.has_value());
        return {*s, *w};
    }

    AnalyzeOptions analyzeOptions(const std::string& timestamp, std::optional<bool> correction = std::nullopt) {
        AnalyzeOptions o;
        o.timestamp = timestamp;
        o.irradiance_correction = correction;
        return o;
    }

    // 波長 λ の列の番号(ヘッダ "rf_550" など)
    std::size_t column(const Csv& c, const std::string& name) {
        for (std::size_t i = 0; i < c.header.size(); ++i) if (c.header[i] == name) return i;
        INFO("no column " << name);
        REQUIRE(false);
        return 0;
    }

} // namespace

TEST_CASE("a white run measures only the reference point of every pose", "[analyze]") {
    TempDir tmp("white_run");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    const auto runs = measureBoth(dir);
    CHECK(runs.white.filename().string() == "20260101_130000_white");

    const Csv white = readCsv(runs.white / "measurements.csv");
    REQUIRE(white.rows.size() == 3);                       // 3 ポーズ × 1 点
    for (const auto& r : white.rows) {
        CHECK(std::stod(r[2]) == 0.0);                     // x_mm
        CHECK(std::stod(r[3]) == 0.0);                     // y_mm
    }
    CHECK(readCsv(runs.sample / "measurements.csv").rows.size() == 27);
    CHECK_THAT(readFile(runs.white / "run_manifest.toml"), ContainsSubstring("kind = \"white\""));
    CHECK_THAT(readFile(runs.white / "run_manifest.toml"), ContainsSubstring("spot_count = 1"));
}

TEST_CASE("analysis: at the reference point the factor is exactly sample reflectance / white reflectance", "[analyze]") {
    TempDir tmp("analyze_ref");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    measureBoth(dir);

    const auto out = analyzeProject(dir, analyzeOptions("20260101_140000"));
    REQUIRE(out.has_value());
    CHECK(out->filename().string() == "20260101_140000_analysis");
    for (const char* f : {"radiance_factor.csv", "brdf.csv", "analysis_manifest.toml"}) CHECK(fs::exists(*out / f));

    const Csv rf = readCsv(*out / "radiance_factor.csv");
    REQUIRE(rf.rows.size() == 27);
    CHECK(rf.header.size() == 8 + 401);

    // 測定点 5 は、試料座標 (0, 0) = 反射板の基準点。全ポーズ・全波長で ρ_試料(λ) / ρ_白
    for (const auto& r : rf.rows) {
        if (r[1] != "5") continue;
        for (Real wl : {380.0, 450.0, 550.0, 700.0, 780.0}) {
            const Real expected = sampleAlbedo(wl) / kWhite;
            CHECK_THAT(std::stod(r[column(rf, "rf_" + std::format("{:g}", wl))]), WithinRel(expected, 1e-7));
        }
    }

    // BRDF = 放射輝度率 × ρ / π
    const Csv brdf = readCsv(*out / "brdf.csv");
    for (std::size_t i = 0; i < rf.rows.size(); ++i) {
        const Real a = std::stod(rf.rows[i][column(rf, "rf_550")]);
        const Real b = std::stod(brdf.rows[i][column(brdf, "brdf_550")]);
        CHECK_THAT(b, WithinRel(a * kWhite / PI, 1e-6));
    }

    const std::string manifest = readFile(*out / "analysis_manifest.toml");
    CHECK_THAT(manifest, ContainsSubstring("sample_run = \"20260101_120000_sample\""));
    CHECK_THAT(manifest, ContainsSubstring("white_run = \"20260101_130000_white\""));
    CHECK_THAT(manifest, ContainsSubstring("irradiance_correction = false"));
    CHECK_THAT(manifest, ContainsSubstring("rows = 27"));
}

TEST_CASE("analysis: the previous system's systematic error, and the irradiance correction that removes it", "[analyze]") {
    // 反射板はクロスヘアの 1 点だけを測るので、他の測定点は、照度の違い(cosθi / d²)の分だけ、反射率が
    // 大きく・小さく出る(旧システムの論文が述べている系統誤差)。照度の補正で、これが消える。
    TempDir tmp("analyze_corr");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    measureBoth(dir);

    const auto plain = analyzeProject(dir, analyzeOptions("20260101_140000", false));
    const auto corrected = analyzeProject(dir, analyzeOptions("20260101_150000", true));
    REQUIRE(plain.has_value());
    REQUIRE(corrected.has_value());

    const Csv a = readCsv(*plain / "radiance_factor.csv");
    const Csv b = readCsv(*corrected / "radiance_factor.csv");
    const std::size_t col = column(a, "rf_550");
    const Real truth = sampleAlbedo(550.0) / kWhite;

    Real worst_plain = 0.0, worst_corrected = 0.0;
    for (std::size_t i = 0; i < a.rows.size(); ++i) {
        worst_plain = std::max(worst_plain, std::abs(std::stod(a.rows[i][col]) / truth - 1.0));
        worst_corrected = std::max(worst_corrected, std::abs(std::stod(b.rows[i][col]) / truth - 1.0));
    }
    CHECK(worst_plain > 0.01);          // 補正なし: 点によって 1% 以上ずれる
    CHECK(worst_corrected < 1e-6);      // 補正あり: 全ての点で、真の ρ_試料 / ρ_白 に一致
    CHECK_THAT(readFile(*corrected / "analysis_manifest.toml"), ContainsSubstring("irradiance_correction = true"));
}

TEST_CASE("analysis recovers a glossy BRDF from the simulated measurement", "[analyze]") {
    // 往復テスト: シミュレータが「真の BRDF」(拡散 + 鏡面)を持つ試料を測り、反射板で割って BRDF を復元する。
    // 復元した値が、真の BRDF モデルと一致する。
    TempDir tmp("analyze_phong");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    replaceInFile(dir / "setup.toml", "model = \"lambert\"", "model = \"phong\"");
    replaceInFile(dir / "setup.toml", "irradiance_correction = false", "irradiance_correction = true");
    measureBoth(dir);

    const auto out = analyzeProject(dir, analyzeOptions("20260101_140000"));
    REQUIRE(out.has_value());

    const sim::PhongBrdf truth(sim::gaussianCurve(550.0, 60.0, 0.8, 0.05), 0.5, 50.0);   // 既定値
    const Csv brdf = readCsv(*out / "brdf.csv");
    REQUIRE(brdf.rows.size() == 27);

    for (const auto& r : brdf.rows) {
        optics::GeometryAngles angles;
        angles.incidence   = {toRadians(std::stod(r[4])), toRadians(std::stod(r[5]))};
        angles.observation = {toRadians(std::stod(r[6])), toRadians(std::stod(r[7]))};
        for (Real wl : {450.0, 550.0, 650.0}) {
            const Real recovered = std::stod(r[column(brdf, "brdf_" + std::format("{:g}", wl))]);
            CHECK_THAT(recovered, WithinRel(truth.value(wl, angles), 1e-4));
        }
    }
}

TEST_CASE("analysis does not depend on the integration time of the two runs", "[analyze]") {
    TempDir tmp("analyze_integration");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    std::istringstream in;
    REQUIRE(runProject(dir, run(domain::MeasurementKind::Sample, "20260101_120000"), in).has_value());
    replaceInFile(dir / "recipe.toml", "integration_time_ms = 100.0", "integration_time_ms = 37.0");   // 白色板だけ別の積分時間
    REQUIRE(runProject(dir, run(domain::MeasurementKind::White, "20260101_130000"), in).has_value());

    const auto out = analyzeProject(dir, analyzeOptions("20260101_140000"));
    REQUIRE(out.has_value());
    const Csv rf = readCsv(*out / "radiance_factor.csv");
    for (const auto& r : rf.rows) {
        if (r[1] == "5") CHECK_THAT(std::stod(r[column(rf, "rf_550")]), WithinRel(sampleAlbedo(550.0) / kWhite, 1e-7));
    }
}

TEST_CASE("analysis: the wavelength range comes from setup.toml", "[analyze]") {
    TempDir tmp("analyze_range");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    replaceInFile(dir / "setup.toml", "min_wavelength_nm = 380.0", "min_wavelength_nm = 400.0");
    replaceInFile(dir / "setup.toml", "max_wavelength_nm = 780.0", "max_wavelength_nm = 700.0");
    measureBoth(dir);

    const auto out = analyzeProject(dir, analyzeOptions("20260101_140000"));
    REQUIRE(out.has_value());
    const Csv rf = readCsv(*out / "radiance_factor.csv");
    CHECK(rf.header.size() == 8 + 301);
    CHECK(rf.header[8] == "rf_400");
    CHECK(rf.header.back() == "rf_700");
}

TEST_CASE("analysis picks the latest runs, or the ones named", "[analyze]") {
    TempDir tmp("analyze_pick");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());
    measureBoth(dir);

    std::istringstream in;
    REQUIRE(runProject(dir, run(domain::MeasurementKind::Sample, "20260102_090000"), in).has_value());   // より新しい試料

    SECTION("by default: the latest of each kind") {
        const auto out = analyzeProject(dir, analyzeOptions("20260103_000000"));
        REQUIRE(out.has_value());
        CHECK_THAT(readFile(*out / "analysis_manifest.toml"), ContainsSubstring("sample_run = \"20260102_090000_sample\""));
        CHECK_THAT(readFile(*out / "analysis_manifest.toml"), ContainsSubstring("white_run = \"20260101_130000_white\""));
    }
    SECTION("named runs") {
        AnalyzeOptions o = analyzeOptions("20260103_000000");
        o.sample_run = "20260101_120000_sample";
        o.white_run = "20260101_130000_white";
        const auto out = analyzeProject(dir, o);
        REQUIRE(out.has_value());
        CHECK_THAT(readFile(*out / "analysis_manifest.toml"), ContainsSubstring("sample_run = \"20260101_120000_sample\""));
    }
    SECTION("a folder that does not exist") {
        AnalyzeOptions o = analyzeOptions("20260103_000000");
        o.sample_run = "no_such_run";
        CHECK_THAT(analyzeProject(dir, o).error().message, ContainsSubstring("does not exist"));
    }
}

TEST_CASE("analysis refuses runs that cannot be compared", "[analyze]") {
    TempDir tmp("analyze_refuse");
    const fs::path dir = tmp.path / "p";
    REQUIRE(initProject(dir).has_value());

    SECTION("no white plate measurement yet") {
        std::istringstream in;
        REQUIRE(runProject(dir, run(domain::MeasurementKind::Sample, "20260101_120000"), in).has_value());
        const auto r = analyzeProject(dir, analyzeOptions("20260101_140000"));
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("--kind white"));
    }

    const auto runs = measureBoth(dir);

    SECTION("the sample and white folders are swapped") {
        AnalyzeOptions o = analyzeOptions("20260101_140000");
        o.sample_run = runs.white.filename().string();
        const auto r = analyzeProject(dir, o);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("is not a \"sample\" measurement"));
    }
    SECTION("the white plate was measured at a different pose") {
        replaceInFile(runs.white / "recipe.toml", "left = [45.0, 0.0, 90.0, 0.0, 0.0, -30.0]", "left = [45.0, 0.0, 90.0, 0.0, 0.0, -29.0]");
        const auto r = analyzeProject(dir, analyzeOptions("20260101_140000"));
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("pose \"1\""));
        CHECK_THAT(r.error().message, ContainsSubstring("not measured under the same conditions"));
    }
    SECTION("the camera was moved between the two measurements") {
        replaceInFile(runs.white / "setup.toml", "camera_position = [1136.0, 728.0, 560.0]", "camera_position = [1140.0, 728.0, 560.0]");
        const auto r = analyzeProject(dir, analyzeOptions("20260101_140000"));
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("camera_position"));
    }
    SECTION("a result folder without a manifest") {
        std::error_code ec;
        fs::remove(runs.white / "run_manifest.toml", ec);
        const auto r = analyzeProject(dir, analyzeOptions("20260101_140000"));
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("run_manifest.toml"));
    }
}
