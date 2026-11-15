#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/calibration/BoardLayout.hpp"
#include "brdf/calibration/Homography.hpp"
#include "brdf/calibration/HomographySpotProjector.hpp"
#include "brdf/calibration/SampleToBoard.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/vision/CircleGridDetector.hpp"
#include "brdf/vision/HomographyFit.hpp"
#include "brdf/vision/ImageIO.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::calibration;
namespace fs = std::filesystem;

namespace {

    // 既知のホモグラフィ: ボード(mm)を、回転・拡大・遠近をつけて画像(pixel)へ
    Homography makeHomography(Real rotation_deg, Real scale = 2.0, Vec2 shift = Vec2(300, 200)) {
        const Real a = toRadians(rotation_deg);
        Homography h;
        h.matrix << scale * std::cos(a), -scale * 0.8 * std::sin(a), shift.x(),
                    scale * std::sin(a),  scale * 0.8 * std::cos(a), shift.y(),
                    -2.0e-4,               1.5e-4,                    1.0;
        return h;
    }

    // ボードを画像として描く(逆写像で画素ごとに、紙・黒円・背景を決める)
    GrayImage renderBoard(const BoardLayout& layout, const Homography& h, int width, int height) {
        const auto inv = h.inverse();
        REQUIRE(inv.has_value());
        const auto centers = layout.circleCenters();
        const Real r = layout.circle_diameter * 0.5;

        GrayImage img(width, height, 40);   // 暗い背景
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const Vec2 b = inv->apply(Vec2(x + 0.5, y + 0.5));
                if (b.x() < 0 || b.y() < 0 || b.x() > layout.paper_width || b.y() > layout.paper_height) continue;
                std::uint8_t v = 210;   // 紙
                // 円までの距離(近い格子点だけ調べる)
                const int i = static_cast<int>(std::lround((b.x() - layout.first_x) / layout.pitch_x));
                const int j = static_cast<int>(std::lround((b.y() - layout.first_y) / layout.pitch_y));
                if (i >= 0 && i < layout.columns && j >= 0 && j < layout.rows) {
                    if ((b - centers[static_cast<std::size_t>(j) * layout.columns + i]).norm() < r) v = 20;
                }
                img.at(x, y) = v;
            }
        }
        return img;
    }

    std::vector<Vec2> readPoints(const fs::path& path) {
        std::ifstream ifs(path);
        REQUIRE(ifs.good());
        std::vector<Vec2> out;
        std::string line;
        std::getline(ifs, line);   // ヘッダ
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string a, b;
            std::getline(ss, a, ',');
            std::getline(ss, b, ',');
            out.emplace_back(std::stod(a), std::stod(b));
        }
        return out;
    }

    Homography readMatrix(const fs::path& path) {
        std::ifstream ifs(path);
        REQUIRE(ifs.good());
        Homography h;
        std::string line;
        for (int r = 0; r < 3; ++r) {
            std::getline(ifs, line);
            std::stringstream ss(line);
            std::string c;
            for (int k = 0; k < 3; ++k) {
                std::getline(ss, c, ',');
                h.matrix(r, k) = std::stod(c);
            }
        }
        return h;
    }

    // 紙の 4 隅を 2 つのホモグラフィで写したときの、位置の差の平均 [pixel]
    Real cornerDifference(const Homography& a, const Homography& b, const BoardLayout& l) {
        const Vec2 corners[] = {{0, 0}, {l.paper_width, 0}, {l.paper_width, l.paper_height}, {0, l.paper_height}};
        Real sum = 0;
        for (const Vec2& c : corners) sum += (a.apply(c) - b.apply(c)).norm();
        return sum / 4.0;
    }

    const fs::path kFixtures = fs::path(BRDF_TEST_DATA_DIR) / "legacy_calib_2020";

} // namespace

TEST_CASE("BoardLayout: the legacy boards", "[calibration]") {
    SECTION("2020: 11x7 = 77 circles") {
        const BoardLayout b = BoardLayout::legacy2020();
        const auto c = b.circleCenters();
        REQUIRE(c.size() == 77);
        CHECK(b.count() == 77);
        // 旧 getVirtualCorner の最初と最後の点
        CHECK_THAT(c.front().x(), WithinAbs(112.0, 1e-12));
        CHECK_THAT(c.front().y(), WithinAbs(42.0, 1e-12));
        CHECK_THAT(c.back().x(), WithinAbs(412.0, 1e-12));
        CHECK_THAT(c.back().y(), WithinAbs(282.0, 1e-12));
        // 行優先: 2 番目は X 方向に 30 mm、12 番目は次の行(Y 方向に 40 mm)
        CHECK_THAT((c[1] - c[0]).x(), WithinAbs(30.0, 1e-12));
        CHECK_THAT((c[11] - c[0]).y(), WithinAbs(40.0, 1e-12));
    }
    SECTION("2023: 15x7 = 105 circles") {
        const BoardLayout b = BoardLayout::legacy2023();
        const auto c = b.circleCenters();
        REQUIRE(c.size() == 105);
        CHECK_THAT(c.front().x(), WithinAbs(75.0, 1e-12));
        CHECK_THAT(c.front().y(), WithinAbs(101.0, 1e-12));
        CHECK_THAT(c.back().x(), WithinAbs(425.0, 1e-12));
        CHECK_THAT(c.back().y(), WithinAbs(251.0, 1e-12));
    }
}

TEST_CASE("SampleToBoard: how the board is mounted", "[calibration]") {
    SECTION("identity: u = +x, v = +y") {
        const auto m = SampleToBoard::make("+x", "+y", Vec2(100, 50));
        REQUIRE(m.has_value());
        CHECK_THAT((m->apply(Vec2(10, 20)) - Vec2(110, 70)).norm(), WithinAbs(0.0, 1e-12));
    }
    SECTION("the simulator's view: sample Y points up in the image, so v = -y (a mirror)") {
        const auto m = SampleToBoard::make("+x", "-y", Vec2(225, 151));
        REQUIRE(m.has_value());
        CHECK_THAT((m->apply(Vec2(10, 20)) - Vec2(235, 131)).norm(), WithinAbs(0.0, 1e-12));
    }
    SECTION("axes swapped, as in the previous system's code (x and y were exchanged)") {
        const auto m = SampleToBoard::make("+y", "+x", Vec2(0, 0));
        REQUIRE(m.has_value());
        CHECK_THAT((m->apply(Vec2(10, 20)) - Vec2(20, 10)).norm(), WithinAbs(0.0, 1e-12));
    }
    SECTION("inverse undoes apply for every valid mounting") {
        for (const char* u : {"+x", "-x", "+y", "-y"}) {
            for (const char* v : {"+x", "-x", "+y", "-y"}) {
                const auto m = SampleToBoard::make(u, v, Vec2(30, -7));
                if (!m) continue;   // 同じ軸の組み合わせは作れない
                const Vec2 s(12.5, -3.25);
                CHECK_THAT((m->inverse(m->apply(s)) - s).norm(), WithinAbs(0.0, 1e-12));
            }
        }
    }
    SECTION("invalid axes") {
        CHECK_THAT(SampleToBoard::make("up", "+y", Vec2::Zero()).error().message, ContainsSubstring("board_u"));
        CHECK_THAT(SampleToBoard::make("+x", "down", Vec2::Zero()).error().message, ContainsSubstring("board_v"));
        CHECK_THAT(SampleToBoard::make("+x", "-x", Vec2::Zero()).error().message, ContainsSubstring("different axes"));
    }
}

TEST_CASE("Homography: apply and inverse", "[calibration]") {
    const Homography h = makeHomography(25.0);
    const auto inv = h.inverse();
    REQUIRE(inv.has_value());

    for (const Vec2& p : {Vec2(0, 0), Vec2(100, 50), Vec2(450, 302)}) {
        const Vec2 back = inv->apply(h.apply(p));
        CHECK_THAT((back - p).norm(), WithinAbs(0.0, 1e-9));
    }

    Homography singular;
    singular.matrix.setZero();
    CHECK_FALSE(singular.inverse().has_value());
}

TEST_CASE("fitHomography recovers a known homography", "[calibration][vision]") {
    const BoardLayout layout = BoardLayout::legacy2020();
    const Homography truth = makeHomography(-30.0);

    std::vector<Vec2> board = layout.circleCenters(), image;
    for (const Vec2& p : board) image.push_back(truth.apply(p));

    SECTION("exact points") {
        const auto fit = vision::fitHomography(board, image);
        REQUIRE(fit.has_value());
        CHECK(fit->points == 77);
        CHECK(fit->rms_px < 1e-3);
        CHECK(fit->max_px < 1e-3);
        CHECK(cornerDifference(fit->homography, truth, layout) < 1e-3);
    }
    SECTION("points with noise: the error reflects the noise level") {
        std::vector<Vec2> noisy = image;
        for (std::size_t i = 0; i < noisy.size(); ++i) {
            noisy[i] += 0.5 * Vec2(std::sin(i * 12.9898) , std::cos(i * 78.233));   // 再現できる擬似ノイズ(±0.5 px)
        }
        const auto fit = vision::fitHomography(board, noisy);
        REQUIRE(fit.has_value());
        CHECK(fit->rms_px > 0.05);
        CHECK(fit->rms_px < 0.6);
        CHECK(cornerDifference(fit->homography, truth, layout) < 1.0);
    }
}

TEST_CASE("fitHomography rejects bad input", "[calibration][vision]") {
    const std::vector<Vec2> three = {{0, 0}, {1, 0}, {0, 1}};
    CHECK_THAT(vision::fitHomography(three, three).error().message, ContainsSubstring("at least 4"));

    const std::vector<Vec2> four = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};
    CHECK_THAT(vision::fitHomography(four, three).error().message, ContainsSubstring("different lengths"));

    const std::vector<Vec2> line = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}};
    CHECK_FALSE(vision::fitHomography(line, line).has_value());   // 一直線上の点
}

TEST_CASE("HomographySpotProjector: board coordinates, per-pose homographies and the image bounds", "[calibration]") {
    // 単純な H: ボード座標 (u, v) → 画像 (2u, 3v)
    Homography h;
    h.matrix << 2, 0, 0,
                0, 3, 0,
                0, 0, 1;

    SampleToBoard mapping;
    mapping.origin = Vec2(100, 50);                            // 試料座標の原点は、ボード座標 (100, 50)
    HomographySpotProjector proj(mapping, 1376, 1024);
    proj.setHomography("A", h);
    CHECK(proj.has("A"));
    CHECK_FALSE(proj.has("B"));
    CHECK(proj.size() == 1);

    SECTION("sample coordinates are shifted by the origin before the homography") {
        const auto p = proj.pixelFor({"A", Vec2(10, 20), Vec3::Zero()});
        REQUIRE(p.has_value());
        CHECK_THAT(p->x(), WithinAbs(2 * 110.0, 1e-9));
        CHECK_THAT(p->y(), WithinAbs(3 * 70.0, 1e-9));
    }
    SECTION("a pose without calibration is an error that tells what to do") {
        const auto p = proj.pixelFor({"B", Vec2(0, 0), Vec3::Zero()});
        REQUIRE_FALSE(p.has_value());
        CHECK_THAT(p.error().message, ContainsSubstring("brdf calibrate"));
    }
    SECTION("points outside the image are rejected") {
        const auto p = proj.pixelFor({"A", Vec2(1000, 0), Vec3::Zero()});   // x = 2 × 1100 = 2200 > 1376
        REQUIRE_FALSE(p.has_value());
        CHECK(p.error().code == ErrorCode::OutOfRange);
    }
}

TEST_CASE("legacy 2020 data: homographies fitted from the detected circles match the saved ones", "[calibration][legacy]") {
    // 旧システムの実データ(13 ポーズ)。検出済みの 77 点から求めた H が、旧システムが保存した H と一致する。
    const BoardLayout layout = BoardLayout::legacy2020();
    const auto board = layout.circleCenters();

    for (int n = 1; n <= 13; ++n) {
        INFO("pose " << n);
        const auto centers = readPoints(kFixtures / ("centers_" + std::to_string(n) + ".csv"));
        REQUIRE(centers.size() == 77);

        const auto fit = vision::fitHomography(board, centers);
        REQUIRE(fit.has_value());
        CHECK(fit->rms_px < 2.0);

        const Homography saved = readMatrix(kFixtures / ("H_saved_" + std::to_string(n) + ".csv"));
        CHECK(cornerDifference(fit->homography, saved, layout) < 0.5);
    }
}

TEST_CASE("circle grid detection on rendered boards", "[calibration][vision]") {
    const BoardLayout layout = BoardLayout::legacy2023();
    constexpr int W = 1376, H = 1024;

    // ボードの回転が ±90° 以内なら、向きを正しく選べる(旧システムの実データでは +35° 〜 -52°)
    for (const Real angle : {0.0, 35.0, -50.0, 80.0, -80.0}) {
        INFO("rotation " << angle << " deg");

        // 紙の中心が画像の中心に来るよう平行移動して、ボード全体が画像に収まるようにする
        Homography truth = makeHomography(angle, 1.5, Vec2(0, 0));
        const Vec2 centre = truth.apply(Vec2(layout.paper_width * 0.5, layout.paper_height * 0.5));
        Mat3 move = Mat3::Identity();
        move(0, 2) = W * 0.5 - centre.x();
        move(1, 2) = H * 0.5 - centre.y();
        truth.matrix = move * truth.matrix;

        const GrayImage image = renderBoard(layout, truth, W, H);

        const auto found = vision::detectCircleGrid(image, layout);
        REQUIRE(found.has_value());
        REQUIRE(found->size() == 105);

        const auto board = layout.circleCenters();
        Real worst = 0;
        for (std::size_t i = 0; i < board.size(); ++i) {
            worst = std::max(worst, (truth.apply(board[i]) - (*found)[i]).norm());
        }
        CHECK(worst < 1.0);   // 検出した並びと中心が、真の位置と 1 pixel 以内で一致
    }
}

TEST_CASE("circle grid detection fails clearly when there is no board", "[calibration][vision]") {
    const GrayImage blank(640, 480, 100);
    const auto r = vision::detectCircleGrid(blank, BoardLayout::legacy2020());
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("circle grid was not found"));

    CHECK_FALSE(vision::detectCircleGrid(GrayImage{}, BoardLayout::legacy2020()).has_value());
}

TEST_CASE("PNG save and load round trip", "[vision]") {
    GrayImage img(37, 23);
    for (int y = 0; y < img.height; ++y) {
        for (int x = 0; x < img.width; ++x) img.at(x, y) = static_cast<std::uint8_t>((x * 7 + y * 13) % 256);
    }
    const fs::path path = fs::temp_directory_path() / "brdf_test_roundtrip.png";
    REQUIRE(vision::saveGrayImage(path, img).has_value());

    const auto back = vision::loadGrayImage(path);
    REQUIRE(back.has_value());
    CHECK(back->width == 37);
    CHECK(back->height == 23);
    CHECK(back->pixels == img.pixels);

    std::error_code ec;
    fs::remove(path, ec);
    CHECK_FALSE(vision::loadGrayImage(path).has_value());   // 無いファイル
}

// 旧システムの画像そのもの(各 2.8 MB)での検証。通常のテストには含まれない。
//   set BRDF_LEGACY_CALIB_DIR=<旧システムの AgInkBeta フォルダ>
//   brdf_tests "[.full]"
TEST_CASE("legacy 2020 images: detection and fitting reproduce the saved homographies", "[.full][calibration][legacy]") {
    char* dir = nullptr;
#ifdef _MSC_VER
    size_t len = 0;
    _dupenv_s(&dir, &len, "BRDF_LEGACY_CALIB_DIR");
#else
    dir = std::getenv("BRDF_LEGACY_CALIB_DIR");
#endif
    REQUIRE(dir != nullptr);
    const fs::path base(dir);
#ifdef _MSC_VER
    std::free(dir);
#endif

    const BoardLayout layout = BoardLayout::legacy2020();
    const auto board = layout.circleCenters();
    for (int n = 1; n <= 13; ++n) {
        INFO("image " << n);
        const auto image = vision::loadGrayImage(base / "CalibBoard" / (std::to_string(n) + ".png"));
        REQUIRE(image.has_value());
        const auto found = vision::detectCircleGrid(*image, layout);
        REQUIRE(found.has_value());
        const auto fit = vision::fitHomography(board, *found);
        REQUIRE(fit.has_value());

        const Homography saved = readMatrix(base / "Homography" / (std::to_string(n) + ".csv"));
        CHECK(cornerDifference(fit->homography, saved, layout) < 1.0);   // 紙の四隅で 1 pixel 以内
    }
}
