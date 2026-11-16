#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Calibrate.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/DeviceFactory.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/calibration/HomographySpotProjector.hpp"
#include "brdf/io/CalibrationFile.hpp"
#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/MockSpectralImager.hpp"
#include "brdf/vision/CircleGridDetector.hpp"
#include "brdf/vision/ImageIO.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
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

    // テストを速くするため、画像を半分にしたプロジェクトを作る。画素数が 1/4 になる分、円が小さくなって検出が
    // 不安定にならないよう、ボードの円を少し大きく(直径 20 mm)する。
    fs::path makeSmallProject(const fs::path& base) {
        const fs::path dir = base / "p";
        REQUIRE(initProject(dir).has_value());
        replaceInFile(dir / "setup.toml", "# image_width = 2448", "image_width = 688");
        replaceInFile(dir / "setup.toml", "# image_height = 2048", "image_height = 512");
        replaceInFile(dir / "setup.toml", "pixels_per_mm = 3.22", "pixels_per_mm = 0.9");
        replaceInFile(dir / "setup.toml", "circle_diameter = 16.0", "circle_diameter = 20.0");
        return dir;
    }

    RunOptions yes() {
        RunOptions o;
        o.assume_yes = true;
        o.timestamp = "20260101_120000";
        return o;
    }

} // namespace

TEST_CASE("calibration CSV survives a write/read round trip exactly", "[calibrate]") {
    std::vector<io::PoseCalibration> in(2);
    in[0].pose_id = "1";
    in[0].homography.matrix << 0.892256123456789, -0.50582, 359.848, 0.629126, 1.63033, 111.405, -0.000404793, 0.000214448, 1.0;
    in[0].rms_px = 0.412;
    in[0].max_px = 1.2;
    in[0].circles = 77;
    in[1].pose_id = "left, \"A\"";   // カンマや引用符を含む名前も扱える
    in[1].homography.matrix = Mat3::Identity();
    in[1].circles = 105;

    const auto out = io::parseCalibrationCsv(io::toCsv(in));
    REQUIRE(out.has_value());
    REQUIRE(out->size() == 2);
    CHECK((*out)[0].pose_id == "1");
    CHECK((*out)[1].pose_id == "left, \"A\"");
    CHECK((*out)[0].homography.matrix == in[0].homography.matrix);   // 17 桁で保存するので完全に一致
    CHECK((*out)[0].circles == 77);
    CHECK_THAT((*out)[0].rms_px, WithinAbs(0.412, 1e-6));
}

TEST_CASE("calibration CSV errors point to the line", "[calibrate]") {
    const std::string header = "pose,h11,h12,h13,h21,h22,h23,h31,h32,h33,rms_px,max_px,circles\n";
    CHECK_THAT(io::parseCalibrationCsv("a,b\n").error().message, ContainsSubstring("header"));
    CHECK_THAT(io::parseCalibrationCsv(header + "1,2,3\n").error().message, ContainsSubstring("expected 13 columns"));
    CHECK_THAT(io::parseCalibrationCsv(header + "1,1,0,0,0,1,0,0,0,1,x,0,5\n").error().message, ContainsSubstring(":2:"));
    CHECK_THAT(io::parseCalibrationCsv(header + "1,1,0,0,0,1,0,0,0,1,0,0,5\n1,1,0,0,0,1,0,0,0,1,0,0,5\n").error().message,
               ContainsSubstring("duplicate pose"));
    CHECK_THAT(io::loadCalibration("no/such/file.csv").error().message, ContainsSubstring("brdf calibrate"));
}

TEST_CASE("the simulated imager draws the calibration board where the camera model says", "[calibrate][sim][slow]") {
    TempDir tmp("board_image");
    const fs::path dir = makeSmallProject(tmp.path);
    const auto project = loadProject(dir);
    REQUIRE(project.has_value());

    auto devices = makeDevices(project->setup);
    REQUIRE(devices.has_value());
    REQUIRE(devices->left_arm->connect().has_value());
    REQUIRE(devices->right_arm->connect().has_value());
    REQUIRE(devices->imager->connect().has_value());

    const auto& pose = project->recipe.poses[0];
    REQUIRE(devices->left_arm->moveJ(pose.left, 1.0).has_value());
    REQUIRE(devices->right_arm->moveJ(pose.right, 1.0).has_value());

    const geometry::RigGeometry geometry(project->setup.rig);
    driver::ImageRequest req;
    req.measure_distance_mm = measureDistance(geometry, pose);
    const auto image = devices->imager->captureImage(req);
    REQUIRE(image.has_value());
    CHECK(image->width == 688);
    CHECK(image->height == 512);

    // 検出できる(画像に 105 個の円が写っている)
    const auto found = vision::detectCircleGrid(*image, project->setup.calibration.board);
    REQUIRE(found.has_value());
    CHECK(found->size() == 105);

    // 紙の上の点は明るく、円の中心は暗い
    const auto centre = (*found)[52];
    CHECK(image->at(static_cast<int>(centre.x()), static_cast<int>(centre.y())) < 100);

    SECTION("errors") {
        driver::ImageRequest wrong = req;
        wrong.measure_distance_mm += 500.0;
        CHECK(devices->imager->captureImage(wrong).error().code == ErrorCode::FocusError);

        devices->imager->disconnect();
        CHECK(devices->imager->captureImage(req).error().code == ErrorCode::NotConnected);
    }
}

TEST_CASE("calibrate finds an accurate homography for every pose", "[calibrate][slow]") {
    TempDir tmp("calibrate");
    const fs::path dir = makeSmallProject(tmp.path);

    const auto result = calibrateProject(dir);
    REQUIRE(result.has_value());
    REQUIRE(result->size() == 3);

    const ProjectLayout layout(dir);
    CHECK(fs::exists(layout.calibrationFile()));
    CHECK(fs::exists(layout.calibrationImagesDir() / "1.png"));
    CHECK(fs::exists(layout.calibrationImagesDir() / "3.png"));

    for (const auto& c : *result) {
        INFO("pose " << c.pose_id);
        CHECK(c.circles == 105);
        CHECK(c.rms_px < 0.5);    // 合成画像なので、検出の誤差だけ
    }

    // 保存された CSV を読み直せる
    const auto loaded = io::loadCalibration(layout.calibrationFile());
    REQUIRE(loaded.has_value());
    CHECK(loaded->size() == 3);
}

TEST_CASE("the homography agrees with the true camera model for points on the sample", "[calibrate][slow]") {
    // シミュレータのカメラモデルは「真の答え」。校正(画像からの検出 + ホモグラフィ)がそれに近いことの確認。
    TempDir tmp("agree");
    const fs::path dir = makeSmallProject(tmp.path);
    REQUIRE(calibrateProject(dir).has_value());

    const auto project = loadProject(dir);
    REQUIRE(project.has_value());
    const ProjectLayout layout(dir);
    const auto cals = io::loadCalibration(layout.calibrationFile());
    REQUIRE(cals.has_value());

    calibration::HomographySpotProjector board(project->setup.calibration.mapping().value(), 688, 512);
    for (const auto& c : *cals) board.setHomography(c.pose_id, c.homography);

    io::SetupConfig setup = project->setup;
    auto devices = makeDevices(setup);
    REQUIRE(devices.has_value());

    // makeDevices が作った projector(camera-model)を、校正と同じポーズで比べる
    REQUIRE(devices->left_arm->connect().has_value());
    REQUIRE(devices->right_arm->connect().has_value());
    const geometry::RigGeometry geometry(setup.rig);

    Real worst = 0.0;
    for (const auto& pose : project->recipe.poses) {
        REQUIRE(devices->left_arm->moveJ(pose.left, 1.0).has_value());
        REQUIRE(devices->right_arm->moveJ(pose.right, 1.0).has_value());

        for (const Vec2 sample : {Vec2(0, 0), Vec2(30, 20), Vec2(-40, 25), Vec2(50, -35), Vec2(-60, -40)}) {
            const Vec3 world = geometry.paperFrame(pose.right).origin
                             + sample.x() * geometry.paperFrame(pose.right).xAxis()
                             + sample.y() * geometry.paperFrame(pose.right).yAxis();
            const auto a = devices->projector->pixelFor({pose.id, sample, world});
            const auto b = board.pixelFor({pose.id, sample, world});
            REQUIRE(a.has_value());
            REQUIRE(b.has_value());
            worst = std::max(worst, (*a - *b).norm());
        }
    }
    // 半分の解像度(0.9 pixel/mm)の画像で、真のカメラとの差が 1 pixel 以内。
    // 差の主な原因は、斜めから見た円の重心と、真の中心のわずかなずれ(解像度に比例する)。
    CHECK(worst < 1.0);
}

TEST_CASE("run with the board projector gives the same measurement as the camera model", "[calibrate][slow]") {
    TempDir tmp("run_board");
    const fs::path dir = makeSmallProject(tmp.path);

    std::istringstream in;
    const auto model_run = runProject(dir, yes(), in);
    REQUIRE(model_run.has_value());

    REQUIRE(calibrateProject(dir).has_value());
    replaceInFile(dir / "setup.toml", "projector = \"camera-model\"", "projector = \"board\"");
    RunOptions o = yes();
    o.timestamp = "20260101_130000";
    const auto board_run = runProject(dir, o, in);
    REQUIRE(board_run.has_value());

    // どちらの CSV も、行数・列数が同じ。角度は完全に同じ(幾何だけで決まる)。スペクトルは、画像位置の
    // 差(1 pixel 以内)による、わずかな違いだけ。
    auto parse = [](const std::string& csv) {
        std::vector<std::vector<std::string>> rows;
        std::stringstream ss(csv);
        std::string line;
        while (std::getline(ss, line)) {
            std::vector<std::string> cells;
            std::stringstream ls(line);
            for (std::string c; std::getline(ls, c, ',');) cells.push_back(c);
            rows.push_back(cells);
        }
        return rows;
    };
    const auto a = parse(readFile(*model_run / "measurements.csv"));
    const auto b = parse(readFile(*board_run / "measurements.csv"));
    REQUIRE(a.size() == b.size());
    REQUIRE(a.size() == 28);

    Real worst_pixel = 0.0, worst_spectrum = 0.0;
    for (std::size_t r = 1; r < a.size(); ++r) {
        for (std::size_t c : {6u, 7u, 8u, 9u}) CHECK(a[r][c] == b[r][c]);   // 角度
        worst_pixel = std::max(worst_pixel, std::hypot(std::stod(a[r][4]) - std::stod(b[r][4]),
                                                       std::stod(a[r][5]) - std::stod(b[r][5])));
        const Real sa = std::stod(a[r][10 + 170]);   // 550 nm
        const Real sb = std::stod(b[r][10 + 170]);
        worst_spectrum = std::max(worst_spectrum, std::abs(sa - sb) / sa);
    }
    CHECK(worst_pixel < 1.0);
    CHECK(worst_spectrum < 0.02);   // 2% 以内
}

TEST_CASE("board mode without a calibration refuses to run and tells what to do", "[calibrate][slow]") {
    TempDir tmp("nocalib");
    const fs::path dir = makeSmallProject(tmp.path);
    replaceInFile(dir / "setup.toml", "projector = \"camera-model\"", "projector = \"board\"");

    SECTION("run") {
        std::istringstream in;
        const auto r = runProject(dir, yes(), in);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("brdf calibrate"));
        CHECK_FALSE(fs::exists(dir / "result" / "20260101_120000_sample"));
    }
    SECTION("check reports it as an error") {
        const auto r = checkProject(dir);
        REQUIRE(r.has_value());
        CHECK_FALSE(r->ok());
    }
    SECTION("a calibration that misses a pose is reported by name") {
        REQUIRE(calibrateProject(dir).has_value());
        // 3 つのうち 1 つのポーズの行を消す
        std::string csv = readFile(ProjectLayout(dir).calibrationFile());
        const auto first = csv.find('\n') + 1;
        const auto second = csv.find('\n', first) + 1;
        csv.erase(first, second - first);   // ポーズ "1" の行
        writeFile(ProjectLayout(dir).calibrationFile(), csv);

        std::istringstream in;
        const auto r = runProject(dir, yes(), in);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("\"1\""));
    }
}

TEST_CASE("calibrate reports the poses where the board cannot be used", "[calibrate][slow]") {
    TempDir tmp("calib_fail");
    const fs::path dir = makeSmallProject(tmp.path);

    SECTION("the camera does not see the board") {
        // カメラの向きを、ボードのない方へ大きくずらす
        replaceInFile(dir / "setup.toml", "# target = [0.0, 0.0, 0.0]", "target = [1136.0, 3000.0, 560.0]");
        const auto r = calibrateProject(dir);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("calibration failed for 3 pose(s)"));
    }
    SECTION("the reprojection error limit is exceeded") {
        replaceInFile(dir / "setup.toml", "max_rms_px = 3.0", "max_rms_px = 0.000001");
        const auto r = calibrateProject(dir);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("reprojection error is too large"));
    }
    SECTION("a pose outside the joint limits stops before anything moves") {
        replaceInFile(dir / "recipe.toml", "left = [45.0, 0.0, 90.0, 0.0, 0.0, -30.0]", "left = [45.0, 0.0, 200.0, 0.0, 0.0, -30.0]");
        const auto r = calibrateProject(dir);
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("pre-flight"));
    }
}
