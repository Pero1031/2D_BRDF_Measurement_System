#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/DeviceFactory.hpp"
#include "brdf/driver/topcon/TopconImager.hpp"
#include "brdf/io/SpotXml.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::driver;
using namespace brdf::driver::topcon;
namespace fs = std::filesystem;

namespace {

    std::string readFile(const fs::path& p) {
        std::ifstream ifs(p, std::ios::binary);
        std::stringstream ss;
        ss << ifs.rdbuf();
        return ss.str();
    }

    /// SDK の代わりをするフェイク。呼ばれた順を記録し、指定した呼び出しで失敗させ、作った/解放した数を数える。
    class FakeApi final : public ITopconApi {
    public:
        // --- 振る舞いの設定 ---
        std::string fail_at;                          ///< この名前の呼び出しで失敗する("" なら失敗しない)
        ErrorCode fail_code = ErrorCode::DeviceError;
        int connected_devices = 1;
        std::string device_type = "SR5100";
        MeasurementInfo info{2448, 2048, 12.5, false};
        bool drop_first_spot = false;                 ///< 要求した最初のスポットのデータを返さない
        int image_width = 2448;
        int image_height = 2048;

        // --- 記録 ---
        std::vector<std::string> calls;
        int initialized = 0, finalized = 0, opened = 0, closed = 0, created = 0, destroyed = 0;
        std::string recipe_text, spot_text;
        std::string calibration_dir;
        LiveImageRequest live_request;

        int count(const std::string& name) const { return static_cast<int>(std::count(calls.begin(), calls.end(), name)); }

        Result<int> initialize(const std::string& dir) override {
            calls.push_back("initialize");
            calibration_dir = dir;
            if (auto e = maybeFail("initialize")) return tl::unexpected(*e);
            ++initialized;
            return connected_devices;
        }
        Result<DeviceInfo> openDevice(int) override {
            calls.push_back("openDevice");
            if (auto e = maybeFail("openDevice")) return tl::unexpected(*e);
            ++opened;
            return DeviceInfo{7, device_type, "TEST-0001"};
        }
        Status closeDevice(unsigned) override {
            calls.push_back("closeDevice");
            ++closed;
            return {};
        }
        Status finalize() override {
            calls.push_back("finalize");
            ++finalized;
            return {};
        }

        Result<int> createMeasurement(const fs::path& recipe_xml) override {
            calls.push_back("createMeasurement");
            recipe_text = readFile(recipe_xml);
            if (auto e = maybeFail("createMeasurement")) return tl::unexpected(*e);
            ++created;
            return 100 + created;
        }
        Status startMeasurement(unsigned, int, const ProgressFn& progress) override {
            calls.push_back("startMeasurement");
            if (progress) progress("scanning", 50.0f);
            if (auto e = maybeFail("startMeasurement")) return tl::unexpected(*e);
            return {};
        }
        Result<MeasurementInfo> measurementInfo(int) override {
            calls.push_back("measurementInfo");
            if (auto e = maybeFail("measurementInfo")) return tl::unexpected(*e);
            return info;
        }
        Status setSpotDefinition(int, const fs::path& spot_xml) override {
            calls.push_back("setSpotDefinition");
            spot_text = readFile(spot_xml);
            if (auto e = maybeFail("setSpotDefinition")) return tl::unexpected(*e);
            return {};
        }
        /// スポット XML に書かれた ID ごとに、値 = ID * 1000 + (波長 - 380) のスペクトルを返す。順番は逆にしておく。
        Result<std::vector<SpotValues>> spotValues(int) override {
            calls.push_back("spotValues");
            if (auto e = maybeFail("spotValues")) return tl::unexpected(*e);
            const auto spots = io::parseSpotXml(spot_text);
            REQUIRE(spots.has_value());
            std::vector<SpotValues> out;
            for (const SpotRequest& s : *spots) {
                SpotValues v;
                v.spot_id = s.id;
                v.location = s.pixel;
                for (int i = 0; i <= 400; ++i) v.spectrum.push_back(s.id * 1000.0 + i);
                out.push_back(std::move(v));
            }
            std::reverse(out.begin(), out.end());
            if (drop_first_spot && !spots->empty()) {
                const int dropped = spots->front().id;
                out.erase(std::remove_if(out.begin(), out.end(), [&](const SpotValues& v) { return v.spot_id == dropped; }), out.end());
            }
            return out;
        }
        Status destroyMeasurement(int) override {
            calls.push_back("destroyMeasurement");
            ++destroyed;
            return {};
        }

        Result<GrayImage> captureLiveImage(unsigned, const LiveImageRequest& request) override {
            calls.push_back("captureLiveImage");
            live_request = request;
            if (auto e = maybeFail("captureLiveImage")) return tl::unexpected(*e);
            return GrayImage(image_width, image_height, 42);
        }

    private:
        std::optional<Error> maybeFail(const std::string& name) const {
            if (fail_at == name) return Error{fail_code, name + " failed (fake)"};
            return std::nullopt;
        }
    };

    struct Fixture {
        std::shared_ptr<FakeApi> api = std::make_shared<FakeApi>();
        fs::path work_dir;
        TopconImagerConfig config;
        std::unique_ptr<TopconImager> imager;

        explicit Fixture(const std::string& name) {
            work_dir = fs::temp_directory_path() / ("brdf_test_topcon_" + name);
            fs::remove_all(work_dir);
            config.calibration_dir = "D:/fake/Calibration";
            config.device = "SR5100";
            config.binning = "BINNING_TYPE1";
            config.lens = "STANDARD";
            config.image_width = 2448;
            config.image_height = 2048;
            config.spot_size = 3.0;
            config.work_dir = work_dir;
            imager = std::make_unique<TopconImager>(api, config);
        }
        ~Fixture() {
            imager.reset();
            std::error_code ec;
            fs::remove_all(work_dir, ec);
        }

        CaptureRequest request() const {
            CaptureRequest r;
            r.measure_distance_mm = 1000.0;
            r.integration_time_ms = 0.0;
            r.wavelengths = {400.0, 420.0, 10.0};
            r.spots = {{1, Vec2(100.0, 200.0)}, {2, Vec2(1500.0, 900.0)}, {5, Vec2(2000.0, 1800.0)}};
            return r;
        }

        /// 作った測定データは、必ず同じ数だけ解放されている(巨大なので、解放漏れがない)
        void checkBalanced() const { CHECK(api->created == api->destroyed); }

        /// 切断したあとは、開いた装置も初期化した SDK も、すべて閉じている
        void checkAllReleased() const {
            CHECK(api->created == api->destroyed);
            CHECK(api->opened == api->closed);
            CHECK(api->initialized == api->finalized);
        }
    };

} // namespace

// ------------------------------------------------------------------ 接続

TEST_CASE("connect opens the first device and disconnect closes it", "[topcon][connect]") {
    Fixture f("connect");
    REQUIRE_FALSE(f.imager->isConnected());

    REQUIRE(f.imager->connect().has_value());
    CHECK(f.imager->isConnected());
    CHECK((f.api->calls == std::vector<std::string>{"initialize", "openDevice"}));
    CHECK(f.api->calibration_dir == "D:/fake/Calibration");

    f.imager->disconnect();
    CHECK_FALSE(f.imager->isConnected());
    CHECK((f.api->calls == std::vector<std::string>{"initialize", "openDevice", "closeDevice", "finalize"}));
    f.checkAllReleased();

    f.imager->disconnect();   // 2 回呼んでも何も起きない
    CHECK(f.api->calls.size() == 4);
}

TEST_CASE("connect is idempotent while connected", "[topcon][connect]") {
    Fixture f("connect_twice");
    REQUIRE(f.imager->connect().has_value());
    REQUIRE(f.imager->connect().has_value());
    CHECK(f.api->count("initialize") == 1);
}

TEST_CASE("the destructor releases the SDK", "[topcon][connect]") {
    auto api = std::make_shared<FakeApi>();
    {
        TopconImagerConfig config;
        config.calibration_dir = "x";
        TopconImager imager(api, config);
        REQUIRE(imager.connect().has_value());
    }
    CHECK(api->closed == 1);
    CHECK(api->finalized == 1);
}

TEST_CASE("connect failures give everything back and say what to check", "[topcon][connect]") {
    Fixture f("connect_fail");

    SECTION("initialize fails") {
        f.api->fail_at = "initialize";
        const auto s = f.imager->connect();
        REQUIRE_FALSE(s.has_value());
        CHECK_THAT(s.error().message, ContainsSubstring("calibration_dir"));
        CHECK(f.api->count("finalize") == 0);   // 初期化できていないので、終了処理も要らない
    }
    SECTION("no device is connected") {
        f.api->connected_devices = 0;
        const auto s = f.imager->connect();
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::DeviceError);
        CHECK_THAT(s.error().message, ContainsSubstring("no spectroradiometer"));
        CHECK(f.api->finalized == 1);
    }
    SECTION("opening the device fails") {
        f.api->fail_at = "openDevice";
        const auto s = f.imager->connect();
        REQUIRE_FALSE(s.has_value());
        CHECK_THAT(s.error().message, ContainsSubstring("another TOPCON program"));
        CHECK(f.api->finalized == 1);
    }
    SECTION("a different model is connected") {
        f.api->device_type = "SR5000";
        const auto s = f.imager->connect();
        REQUIRE_FALSE(s.has_value());
        CHECK_THAT(s.error().message, ContainsSubstring("SR5000"));
        CHECK_THAT(s.error().message, ContainsSubstring("SR5100"));
        CHECK(f.api->closed == 1);
        CHECK(f.api->finalized == 1);
    }
    CHECK_FALSE(f.imager->isConnected());
    f.checkAllReleased();

    // どの失敗のあとでも、もう一度つなげる(枠が返っている)
    f.api->fail_at.clear();
    f.api->connected_devices = 1;
    f.api->device_type = "SR5100";
    CHECK(f.imager->connect().has_value());
}

TEST_CASE("only one imager can use the SDK at a time", "[topcon][connect]") {
    Fixture a("one_a");
    Fixture b("one_b");
    REQUIRE(a.imager->connect().has_value());

    const auto s = b.imager->connect();
    REQUIRE_FALSE(s.has_value());
    CHECK_THAT(s.error().message, ContainsSubstring("already in use"));
    CHECK(b.api->calls.empty());   // 2 つ目は SDK に触らない

    a.imager->disconnect();
    CHECK(b.imager->connect().has_value());
}

// ------------------------------------------------------------------ capture

TEST_CASE("capture follows the SDK call order and releases the measurement data", "[topcon][capture]") {
    Fixture f("capture");
    REQUIRE(f.imager->connect().has_value());
    f.api->calls.clear();

    const auto r = f.imager->capture(f.request());
    REQUIRE(r.has_value());
    CHECK((f.api->calls == std::vector<std::string>{"createMeasurement", "startMeasurement", "measurementInfo", "setSpotDefinition",
                                                    "spotValues", "destroyMeasurement"}));
    f.checkBalanced();
    CHECK(f.api->created == 1);
}

TEST_CASE("capture returns the spectra by spot id, in the requested order and wavelengths", "[topcon][capture]") {
    Fixture f("capture_values");
    REQUIRE(f.imager->connect().has_value());

    const auto r = f.imager->capture(f.request());
    REQUIRE(r.has_value());

    // 波長は要求の範囲(400, 410, 420)
    REQUIRE(r->wavelengths.size() == 3);
    CHECK_THAT(r->wavelengths(0), WithinAbs(400.0, 1e-9));
    CHECK_THAT(r->wavelengths(2), WithinAbs(420.0, 1e-9));
    CHECK_THAT(r->integration_time_ms, WithinAbs(12.5, 1e-12));   // 装置が実際に使った積分時間
    CHECK_THAT(r->measure_distance_mm, WithinAbs(1000.0, 1e-12));

    // フェイクは順番を逆にして返すが、結果は要求の順(ID 1, 2, 5)で、ID ごとの値が入っている
    REQUIRE(r->spots.size() == 3);
    const int ids[] = {1, 2, 5};
    for (int i = 0; i < 3; ++i) {
        CHECK(r->spots[i].id == ids[i]);
        REQUIRE(r->spots[i].values.size() == 3);
        for (int k = 0; k < 3; ++k) {
            const double wavelength = 400.0 + 10.0 * k;
            CHECK_THAT(r->spots[i].values(k), WithinAbs(ids[i] * 1000.0 + (wavelength - 380.0), 1e-9));
        }
    }
    CHECK_THAT(r->spots[1].pixel.x(), WithinAbs(1500.0, 1e-12));
    CHECK_THAT(r->spots[1].pixel.y(), WithinAbs(900.0, 1e-12));
}

TEST_CASE("capture writes the recipe and the spot definition for the SDK", "[topcon][capture]") {
    Fixture f("capture_files");
    REQUIRE(f.imager->connect().has_value());

    auto req = f.request();
    req.measure_distance_mm = 1234.4;
    REQUIRE(f.imager->capture(req).has_value());

    // レシピ: 装置の種類・距離・波長・画像全体・積分時間は自動
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("<DeviceType type=\"SR5100\"/>"));
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("<MeasureDistance mm=\"1234\"/>"));
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("<MeasurementWaveLength min=\"400\" max=\"420\" step=\"10\"/>"));
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("right=\"2448\" bottom=\"2048\""));
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("integration_time_type=\"AUTO\""));

    // スポット定義: 3 点、画像の大きさと一辺の長さは設定どおり
    const auto spots = io::parseSpotXml(f.api->spot_text);
    REQUIRE(spots.has_value());
    CHECK(spots->size() == 3);
    CHECK_THAT(f.api->spot_text, ContainsSubstring("2448"));

    // ファイルは work_dir に残る(あとで見られる)
    CHECK(fs::exists(f.work_dir / "recipe.xml"));
    CHECK(fs::exists(f.work_dir / "spots.xml"));
}

TEST_CASE("a positive integration time is sent as manual", "[topcon][capture]") {
    Fixture f("capture_manual");
    REQUIRE(f.imager->connect().has_value());
    auto req = f.request();
    req.integration_time_ms = 37.5;
    REQUIRE(f.imager->capture(req).has_value());
    CHECK_THAT(f.api->recipe_text, ContainsSubstring("integration_time_type=\"MANUAL\" integration_time=\"37.5\""));
}

TEST_CASE("every failure after the measurement data is created still releases it", "[topcon][capture]") {
    Fixture f("capture_fail");
    REQUIRE(f.imager->connect().has_value());

    const std::string step = GENERATE(as<std::string>{}, "createMeasurement", "startMeasurement", "measurementInfo", "setSpotDefinition",
                                      "spotValues");
    CAPTURE(step);
    f.api->fail_at = step;
    f.api->fail_code = ErrorCode::DeviceError;

    const auto r = f.imager->capture(f.request());
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring(step));   // どの段階で失敗したか分かる
    f.checkBalanced();

    // 失敗のあとでも、次の撮影ができる
    f.api->fail_at.clear();
    CHECK(f.imager->capture(f.request()).has_value());
    f.checkBalanced();
}

TEST_CASE("the SDK's error code is kept", "[topcon][capture]") {
    Fixture f("capture_code");
    REQUIRE(f.imager->connect().has_value());
    f.api->fail_at = "startMeasurement";
    f.api->fail_code = ErrorCode::InvalidArgument;
    const auto r = f.imager->capture(f.request());
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("a saturated measurement is an error", "[topcon][capture]") {
    Fixture f("capture_saturated");
    REQUIRE(f.imager->connect().has_value());
    f.api->info.saturated = true;
    const auto r = f.imager->capture(f.request());
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::Saturated);
    CHECK_THAT(r.error().message, ContainsSubstring("saturated"));
    f.checkBalanced();
}

TEST_CASE("an image size different from setup.toml is an error", "[topcon][capture]") {
    Fixture f("capture_size");
    REQUIRE(f.imager->connect().has_value());
    f.api->info.image_width = 1376;
    f.api->info.image_height = 1024;
    const auto r = f.imager->capture(f.request());
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("1376x1024"));
    CHECK_THAT(r.error().message, ContainsSubstring("2448x2048"));
    f.checkBalanced();
}

TEST_CASE("a spot the SDK did not return is an error", "[topcon][capture]") {
    Fixture f("capture_missing");
    REQUIRE(f.imager->connect().has_value());
    f.api->drop_first_spot = true;
    const auto r = f.imager->capture(f.request());
    REQUIRE_FALSE(r.has_value());
    CHECK_THAT(r.error().message, ContainsSubstring("spot 1"));
    f.checkBalanced();
}

TEST_CASE("bad requests are refused before the SDK is touched", "[topcon][capture]") {
    Fixture f("capture_invalid");

    SECTION("not connected") {
        const auto r = f.imager->capture(f.request());
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::NotConnected);
        CHECK(f.api->calls.empty());
        return;
    }

    REQUIRE(f.imager->connect().has_value());
    f.api->calls.clear();
    auto req = f.request();
    ErrorCode expected = ErrorCode::InvalidArgument;

    SECTION("no spots") { req.spots.clear(); }
    SECTION("duplicate spot ids") { req.spots[1].id = req.spots[0].id; }
    SECTION("a spot outside the image") {
        req.spots[0].pixel = Vec2(2448.0, 10.0);
        expected = ErrorCode::OutOfRange;
    }
    SECTION("a spot with a negative position") {
        req.spots[0].pixel = Vec2(-1.0, 10.0);
        expected = ErrorCode::OutOfRange;
    }
    SECTION("negative integration time") { req.integration_time_ms = -1.0; }
    SECTION("zero distance") { req.measure_distance_mm = 0.0; }
    SECTION("wavelength below the SDK range") { req.wavelengths = {370.0, 420.0, 10.0}; }
    SECTION("wavelength above the SDK range") { req.wavelengths = {400.0, 790.0, 10.0}; }
    SECTION("fractional wavelengths") { req.wavelengths = {400.5, 420.0, 10.0}; }
    SECTION("fractional step") { req.wavelengths = {400.0, 420.0, 0.5}; }
    SECTION("end before start") { req.wavelengths = {420.0, 400.0, 1.0}; }

    const auto r = f.imager->capture(req);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == expected);
    CHECK(f.api->calls.empty());
}

TEST_CASE("the whole visible range at 1 nm works", "[topcon][capture]") {
    Fixture f("capture_full");
    REQUIRE(f.imager->connect().has_value());
    auto req = f.request();
    req.wavelengths = {380.0, 780.0, 1.0};
    const auto r = f.imager->capture(req);
    REQUIRE(r.has_value());
    CHECK(r->wavelengths.size() == 401);
    CHECK_THAT(r->spots[0].values(400), WithinAbs(1 * 1000.0 + 400.0, 1e-9));
}

// ------------------------------------------------------------------ captureImage

TEST_CASE("captureImage takes a live image and checks its size", "[topcon][image]") {
    Fixture f("image");

    SECTION("not connected") {
        const auto r = f.imager->captureImage(ImageRequest{});
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::NotConnected);
        return;
    }

    REQUIRE(f.imager->connect().has_value());

    SECTION("the default integration time is used when none is given") {
        ImageRequest req;
        req.integration_time_ms = 0.0;
        const auto r = f.imager->captureImage(req);
        REQUIRE(r.has_value());
        CHECK(r->width == 2448);
        CHECK(r->height == 2048);
        CHECK_THAT(f.api->live_request.integration_time_ms, WithinAbs(10.0, 1e-12));
        CHECK(f.api->live_request.binning == "BINNING_TYPE1");
    }
    SECTION("the requested integration time is passed on") {
        ImageRequest req;
        req.integration_time_ms = 25.0;
        REQUIRE(f.imager->captureImage(req).has_value());
        CHECK_THAT(f.api->live_request.integration_time_ms, WithinAbs(25.0, 1e-12));
    }
    SECTION("a negative integration time is refused") {
        ImageRequest req;
        req.integration_time_ms = -5.0;
        const auto r = f.imager->captureImage(req);
        REQUIRE_FALSE(r.has_value());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
    }
    SECTION("a different image size is an error") {
        f.api->image_width = 1376;
        f.api->image_height = 1024;
        const auto r = f.imager->captureImage(ImageRequest{});
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("1376x1024"));
    }
    SECTION("an SDK failure is reported with context") {
        f.api->fail_at = "captureLiveImage";
        const auto r = f.imager->captureImage(ImageRequest{});
        REQUIRE_FALSE(r.has_value());
        CHECK_THAT(r.error().message, ContainsSubstring("live image"));
    }
}

// ------------------------------------------------------------------ 設定からの組み立て

TEST_CASE("makeTopconConfig takes the settings from setup.toml", "[topcon][factory]") {
    io::SetupConfig setup;
    setup.imager.device = "SR5000";
    setup.imager.binning = "BINNING_TYPE2";
    setup.imager.lens = "WIDE";
    setup.imager.calibration_dir = "E:/x/Calibration";
    setup.spot_xml.spot_size = 4.0;

    const auto c = app::makeTopconConfig(setup, "some/dir");
    CHECK(c.device == "SR5000");
    CHECK(c.binning == "BINNING_TYPE2");
    CHECK(c.lens == "WIDE");
    CHECK(c.calibration_dir == "E:/x/Calibration");
    CHECK(c.image_width == 688);     // 1376 / 2
    CHECK(c.image_height == 512);    // 1024 / 2
    CHECK_THAT(c.spot_size, WithinAbs(4.0, 1e-12));
    CHECK(c.work_dir == fs::path("some/dir"));
}

#ifndef BRDF_WITH_TOPCON
TEST_CASE("a build without the TOPCON SDK says so", "[topcon][factory]") {
    const auto r = app::makeTopconImager(io::SetupConfig{}, "x");
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::NotImplemented);
    CHECK_THAT(r.error().message, ContainsSubstring("BRDF_WITH_TOPCON"));
}
#endif

#ifdef BRDF_WITH_TOPCON
TEST_CASE("a build with the TOPCON SDK makes an imager that is not connected yet", "[topcon][factory]") {
    const auto r = app::makeTopconImager(io::SetupConfig{}, "x");
    REQUIRE(r.has_value());
    CHECK_FALSE((*r)->isConnected());
}
#endif
