#include <cmath>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/analysis/RadianceFactor.hpp"
#include "brdf/app/MeasurementSession.hpp"
#include "brdf/app/ResultFiles.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/io/MeasurementsCsv.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using namespace brdf;
using namespace brdf::analysis;

namespace {

    io::MeasuredRow row(const std::string& pose, int spot, std::initializer_list<Real> spectrum) {
        io::MeasuredRow r;
        r.pose_id = pose;
        r.spot_id = spot;
        r.sample_mm = Vec2(spot * 10.0, 0.0);
        r.angles_deg = {30.0, 10.0, 40.0, 190.0};
        r.spectrum.resize(static_cast<Eigen::Index>(spectrum.size()));
        Eigen::Index i = 0;
        for (Real v : spectrum) r.spectrum(i++) = v;
        return r;
    }

    io::MeasurementTable table(std::initializer_list<Real> wavelengths, std::vector<io::MeasuredRow> rows) {
        io::MeasurementTable t;
        t.wavelengths.resize(static_cast<Eigen::Index>(wavelengths.size()));
        Eigen::Index i = 0;
        for (Real w : wavelengths) t.wavelengths(i++) = w;
        t.rows = std::move(rows);
        return t;
    }

    AnalysisOptions options099() {
        AnalysisOptions o;
        o.white_reflectance = 0.99;
        return o;
    }

} // namespace

TEST_CASE("radiance factor = sample / white, and BRDF = factor x rho / pi", "[analysis]") {
    const auto sample = table({400, 500, 600}, {row("A", 1, {3, 3, 6}), row("A", 2, {6, 12, 12})});
    const auto white = table({400, 500, 600}, {row("A", 1, {3, 6, 12})});

    const auto r = computeRadianceFactor(sample, white, options099());
    REQUIRE(r.has_value());
    REQUIRE(r->rows.size() == 2);
    CHECK(r->invalid_values == 0);

    CHECK_THAT(r->rows[0].radiance_factor(0), WithinAbs(1.0, 1e-12));
    CHECK_THAT(r->rows[0].radiance_factor(1), WithinAbs(0.5, 1e-12));
    CHECK_THAT(r->rows[0].radiance_factor(2), WithinAbs(0.5, 1e-12));
    CHECK_THAT(r->rows[1].radiance_factor(0), WithinAbs(2.0, 1e-12));

    CHECK_THAT(r->rows[0].brdf(1), WithinRel(0.5 * 0.99 / PI, 1e-12));

    // 角度・座標・ID は、そのまま引き継がれる
    CHECK(r->rows[1].spot_id == 2);
    CHECK_THAT(r->rows[1].sample_mm.x(), WithinAbs(20.0, 1e-12));
    CHECK_THAT(r->rows[1].angles_deg[3], WithinAbs(190.0, 1e-12));
}

TEST_CASE("the white plate value of a pose is the mean of its rows", "[analysis]") {
    const auto sample = table({400, 500}, {row("A", 1, {3, 3})});
    const auto white = table({400, 500}, {row("A", 1, {2, 4}), row("A", 2, {4, 8})});   // 平均 (3, 6)
    const auto r = computeRadianceFactor(sample, white, options099());
    REQUIRE(r.has_value());
    CHECK_THAT(r->rows[0].radiance_factor(0), WithinAbs(1.0, 1e-12));
    CHECK_THAT(r->rows[0].radiance_factor(1), WithinAbs(0.5, 1e-12));
}

TEST_CASE("each pose is divided by its own white plate value", "[analysis]") {
    const auto sample = table({500}, {row("A", 1, {10}), row("B", 1, {10})});
    const auto white = table({500}, {row("A", 1, {5}), row("B", 1, {20})});
    const auto r = computeRadianceFactor(sample, white, options099());
    REQUIRE(r.has_value());
    CHECK_THAT(r->rows[0].radiance_factor(0), WithinAbs(2.0, 1e-12));
    CHECK_THAT(r->rows[1].radiance_factor(0), WithinAbs(0.5, 1e-12));
}

TEST_CASE("the wavelength range selects the output columns", "[analysis]") {
    const auto sample = table({380, 450, 550, 650, 770}, {row("A", 1, {1, 2, 3, 4, 5})});
    const auto white = table({380, 450, 550, 650, 770}, {row("A", 1, {1, 1, 1, 1, 1})});

    AnalysisOptions o = options099();
    o.min_wavelength_nm = 400.0;
    o.max_wavelength_nm = 700.0;
    const auto r = computeRadianceFactor(sample, white, o);
    REQUIRE(r.has_value());
    REQUIRE(r->wavelengths.size() == 3);
    CHECK_THAT(r->wavelengths(0), WithinAbs(450.0, 1e-12));
    CHECK_THAT(r->wavelengths(2), WithinAbs(650.0, 1e-12));
    CHECK_THAT(r->rows[0].radiance_factor(0), WithinAbs(2.0, 1e-12));

    o.min_wavelength_nm = 800.0;
    o.max_wavelength_nm = 900.0;
    CHECK_THAT(computeRadianceFactor(sample, white, o).error().message, ContainsSubstring("no measured wavelength"));
}

TEST_CASE("analysis refuses inconsistent measurements", "[analysis]") {
    const auto sample = table({400, 500}, {row("A", 1, {1, 1}), row("B", 1, {1, 1})});

    SECTION("a pose without a white plate measurement") {
        const auto white = table({400, 500}, {row("A", 1, {1, 1})});
        const auto r = computeRadianceFactor(sample, white, options099());
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("\"B\""));
    }
    SECTION("different wavelengths") {
        const auto white = table({400, 510}, {row("A", 1, {1, 1}), row("B", 1, {1, 1})});
        CHECK_THAT(computeRadianceFactor(sample, white, options099()).error().message, ContainsSubstring("different wavelengths"));
        const auto shorter = table({400}, {row("A", 1, {1}), row("B", 1, {1})});
        CHECK_FALSE(computeRadianceFactor(sample, shorter, options099()).has_value());
    }
    SECTION("bad options") {
        const auto white = table({400, 500}, {row("A", 1, {1, 1}), row("B", 1, {1, 1})});
        AnalysisOptions o;
        o.white_reflectance = 0.0;
        CHECK_FALSE(computeRadianceFactor(sample, white, o).has_value());
        o = AnalysisOptions{};
        o.min_wavelength_nm = 700;
        o.max_wavelength_nm = 400;
        CHECK_FALSE(computeRadianceFactor(sample, white, o).has_value());
    }
}

TEST_CASE("a white plate value of zero gives nan and is counted", "[analysis]") {
    const auto sample = table({400, 500}, {row("A", 1, {1, 1})});
    const auto white = table({400, 500}, {row("A", 1, {0, 2})});
    const auto r = computeRadianceFactor(sample, white, options099());
    REQUIRE(r.has_value());
    CHECK(std::isnan(r->rows[0].radiance_factor(0)));
    CHECK(std::isnan(r->rows[0].brdf(0)));
    CHECK_THAT(r->rows[0].radiance_factor(1), WithinAbs(0.5, 1e-12));
    CHECK(r->invalid_values == 1);

    const std::string csv = toCsv(*r, Quantity::RadianceFactor);
    CHECK_THAT(csv, ContainsSubstring("nan"));
}

TEST_CASE("the irradiance correction divides the factor by E(P) / E(reference)", "[analysis]") {
    const auto sample = table({400, 500}, {row("A", 1, {4, 6}), row("A", 2, {4, 6})});
    const auto white = table({400, 500}, {row("A", 1, {2, 3})});

    SECTION("a ratio of 2 halves the factor (the point is lit twice as much)") {
        const auto r = computeRadianceFactor(sample, white, options099(),
            [](const std::string&, const Vec2&) -> Result<Real> { return 2.0; });
        REQUIRE(r.has_value());
        CHECK_THAT(r->rows[0].radiance_factor(0), WithinAbs(1.0, 1e-12));   // 4/2/2
    }
    SECTION("the function receives the pose and the point") {
        std::vector<std::pair<std::string, Real>> seen;
        const auto r = computeRadianceFactor(sample, white, options099(),
            [&](const std::string& pose, const Vec2& p) -> Result<Real> { seen.emplace_back(pose, p.x()); return 1.0; });
        REQUIRE(r.has_value());
        REQUIRE(seen.size() == 2);
        CHECK(seen[0].first == "A");
        CHECK_THAT(seen[1].second, WithinAbs(20.0, 1e-12));
    }
    SECTION("an error from the function stops the analysis") {
        const auto r = computeRadianceFactor(sample, white, options099(),
            [](const std::string&, const Vec2&) -> Result<Real> { return fail(ErrorCode::OutOfRange, "no light"); });
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::OutOfRange);
    }
    SECTION("a point the light does not reach (ratio 0) gives nan") {
        const auto r = computeRadianceFactor(sample, white, options099(),
            [](const std::string&, const Vec2&) -> Result<Real> { return 0.0; });
        REQUIRE(r.has_value());
        CHECK(r->invalid_values == 4);
    }
}

TEST_CASE("the output CSV has the angles and one column per wavelength", "[analysis]") {
    const auto sample = table({450, 550}, {row("A", 1, {3, 6})});
    const auto white = table({450, 550}, {row("A", 1, {3, 3})});
    const auto r = computeRadianceFactor(sample, white, options099());
    REQUIRE(r.has_value());

    const std::string rf = toCsv(*r, Quantity::RadianceFactor);
    CHECK_THAT(rf, ContainsSubstring("pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,rf_450,rf_550\n"));
    CHECK_THAT(rf, ContainsSubstring("A,1,10,0,30,10,40,190,1,2\n"));

    const std::string brdf = toCsv(*r, Quantity::Brdf);
    CHECK_THAT(brdf, ContainsSubstring(",brdf_450,brdf_550\n"));
}

TEST_CASE("measurements.csv can be read back", "[analysis][csv]") {
    // 測定が書いたファイルを、そのまま読み戻せる
    app::SessionResult result;
    result.wavelengths.resize(3);
    result.wavelengths << 380.0, 381.5, 400.0;
    for (int k = 0; k < 2; ++k) {
        app::SpotMeasurement m;
        m.pose_id = k == 0 ? "1" : "left, \"B\"";   // カンマや引用符を含むポーズ名
        m.spot_id = 7 + k;
        m.sample_mm = Vec2(-20.5, 3.25);
        m.pixel = Vec2(1200.25, 800.5);
        m.angles.incidence = {toRadians(30.0), toRadians(200.0)};
        m.angles.observation = {toRadians(45.0), toRadians(10.0)};
        m.spectrum.resize(3);
        m.spectrum << 1.5 + k, 2.25, 1e-7;
        result.rows.push_back(m);
    }

    const auto back = io::parseMeasurementsCsv(app::measurementsCsv(result));
    REQUIRE(back.has_value());
    REQUIRE(back->wavelengths.size() == 3);
    CHECK_THAT(back->wavelengths(1), WithinAbs(381.5, 1e-9));
    REQUIRE(back->rows.size() == 2);
    CHECK(back->rows[1].pose_id == "left, \"B\"");
    CHECK(back->rows[1].spot_id == 8);
    CHECK_THAT(back->rows[0].sample_mm.x(), WithinAbs(-20.5, 1e-9));
    CHECK_THAT(back->rows[0].pixel.y(), WithinAbs(800.5, 1e-9));
    CHECK_THAT(back->rows[0].angles_deg[0], WithinAbs(30.0, 1e-6));
    CHECK_THAT(back->rows[0].angles_deg[1], WithinAbs(200.0, 1e-6));
    CHECK_THAT(back->rows[0].spectrum(2), WithinRel(1e-7, 1e-8));
    CHECK_THAT(back->rows[1].spectrum(0), WithinAbs(2.5, 1e-9));
}

TEST_CASE("measurements.csv errors point to the problem", "[analysis][csv]") {
    const std::string head = "pose,spot,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,wl_400,wl_500\n";
    CHECK_THAT(io::parseMeasurementsCsv("").error().message, ContainsSubstring("empty"));
    CHECK_THAT(io::parseMeasurementsCsv("a,b\n").error().message, ContainsSubstring("no wavelength columns"));
    CHECK_THAT(io::parseMeasurementsCsv("pose,spotX,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,wl_400\n")
                   .error().message, ContainsSubstring("column 2"));
    CHECK_THAT(io::parseMeasurementsCsv(head).error().message, ContainsSubstring("no data rows"));
    CHECK_THAT(io::parseMeasurementsCsv(head + "1,1,0,0,0,0,0,0,0,0,1\n").error().message, ContainsSubstring("expected 12 columns"));
    CHECK_THAT(io::parseMeasurementsCsv(head + "1,1,0,0,0,0,0,0,0,0,1,x\n").error().message, ContainsSubstring(":2:"));
    CHECK_THAT(io::loadMeasurementsCsv("no/such/file.csv").error().message, ContainsSubstring("cannot open"));
}
