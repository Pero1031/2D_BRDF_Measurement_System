/*
* @file src/driver/topcon/TopconImager.cpp
* @brief ISpectralImager for the real TOPCON SR-5000 / SR-5100.
*/

#include "brdf/driver/topcon/TopconImager.hpp"

#include <atomic>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

#include <spdlog/spdlog.h>

#include "brdf/io/SpotXml.hpp"
#include "brdf/io/TopconRecipeXml.hpp"

namespace brdf::driver::topcon {

    namespace {

        namespace fs = std::filesystem;

        /// SDK は、プロセス全体で 1 つの接続しか扱えない。使用中の枠を持っているのが誰かを覚えておく。
        std::atomic<bool> g_sdk_in_use{false};

        /// SDK のスペクトルの範囲(添字 i = 波長 380 + i nm)
        constexpr int kFirstWavelength = 380;
        constexpr int kLastWavelength = 780;

        tl::unexpected<Error> withContext(const std::string& what, const Error& e) {
            return fail(e.code, "imager: " + what + ": " + e.message);
        }

        bool isInteger(Real v) { return std::abs(v - std::round(v)) < 1e-9; }

        Status writeFile(const fs::path& path, const std::string& text) {
            std::ofstream ofs(path, std::ios::binary);
            if (!ofs) return fail(ErrorCode::DeviceError, "imager: cannot write " + path.string());
            ofs.write(text.data(), static_cast<std::streamsize>(text.size()));
            ofs.close();
            if (!ofs) return fail(ErrorCode::DeviceError, "imager: cannot write " + path.string());
            return {};
        }

        /// 測定データは、使い終わったら必ず解放する(途中で失敗しても)。巨大(SR-5100 で約 10 GB)なので解放漏れは許されない。
        class MeasurementGuard {
        public:
            MeasurementGuard(ITopconApi& api, int id) : api_(api), id_(id) {}
            ~MeasurementGuard() {
                const Status s = api_.destroyMeasurement(id_);
                if (!s) spdlog::warn("imager: releasing the measurement data failed: {}", s.error().message);
            }
            MeasurementGuard(const MeasurementGuard&) = delete;
            MeasurementGuard& operator=(const MeasurementGuard&) = delete;

        private:
            ITopconApi& api_;
            int id_;
        };

    } // namespace

    TopconImager::TopconImager(std::shared_ptr<ITopconApi> api, TopconImagerConfig config)
        : api_(std::move(api)), config_(std::move(config)) {}

    TopconImager::~TopconImager() { disconnect(); }

    Status TopconImager::connect() {
        if (connected_) return {};
        if (!api_) return fail(ErrorCode::InvalidArgument, "imager: no SDK interface");
        if (g_sdk_in_use.exchange(true)) {
            return fail(ErrorCode::DeviceError,
                        "imager: the TOPCON SDK is already in use by another imager in this process (only one connection is possible)");
        }
        owns_sdk_ = true;

        // 失敗したら、ここまでにやったことを元に戻して、枠を返す
        const auto giveUp = [&](Error e, bool initialized, bool opened) -> Status {
            if (opened) (void)api_->closeDevice(device_.device_id);
            if (initialized) (void)api_->finalize();
            owns_sdk_ = false;
            g_sdk_in_use = false;
            return tl::unexpected(std::move(e));
        };

        const auto count = api_->initialize(config_.calibration_dir);
        if (!count) {
            return giveUp(Error{count.error().code, "imager: initialize: " + count.error().message
                                                        + " (check [imager] calibration_dir in setup.toml)"},
                          false, false);
        }
        if (*count <= 0) {
            return giveUp(Error{ErrorCode::DeviceError,
                                "imager: no spectroradiometer is connected (check the power, the cable, and that no other "
                                "TOPCON software is using it)"},
                          true, false);
        }

        auto device = api_->openDevice(0);
        if (!device) {
            return giveUp(Error{device.error().code, "imager: open device: " + device.error().message
                                                         + " (is another TOPCON program using the device?)"},
                          true, false);
        }
        device_ = *device;

        if (device_.device_type != config_.device) {
            Error e{ErrorCode::DeviceError, "imager: the connected device is " + device_.device_type + " but setup.toml says [imager] device = \""
                                                + config_.device + "\" (the image size and the spot positions depend on it)"};
            return giveUp(std::move(e), true, true);
        }

        connected_ = true;
        spdlog::info("imager: connected to {} (product id {})", device_.device_type, device_.product_id);
        return {};
    }

    void TopconImager::disconnect() {
        if (!connected_ && !owns_sdk_) return;
        if (connected_) {
            const Status s = api_->closeDevice(device_.device_id);
            if (!s) spdlog::warn("imager: closing the device failed: {}", s.error().message);
        }
        if (owns_sdk_) {
            const Status s = api_->finalize();
            if (!s) spdlog::warn("imager: finalize failed: {}", s.error().message);
            g_sdk_in_use = false;
        }
        connected_ = false;
        owns_sdk_ = false;
    }

    Result<CaptureResult> TopconImager::capture(const CaptureRequest& request) {
        if (!connected_) return fail(ErrorCode::NotConnected, "imager: not connected");

        // --- 要求の検査 ---
        if (request.spots.empty()) return fail(ErrorCode::InvalidArgument, "imager: no spots requested");
        if (request.integration_time_ms < 0.0) {
            return fail(ErrorCode::InvalidArgument, "imager: integration time must not be negative (0 = automatic)");
        }
        if (!(request.measure_distance_mm > 0.0)) {
            return fail(ErrorCode::InvalidArgument, "imager: measure distance must be positive");
        }
        const WavelengthRange& wr = request.wavelengths;
        if (!isInteger(wr.start) || !isInteger(wr.end) || !isInteger(wr.step) || !(wr.step >= 1.0) || !(wr.end >= wr.start)
            || wr.start < kFirstWavelength || wr.end > kLastWavelength) {
            std::ostringstream m;
            m << "imager: the wavelength range must be whole nanometers within " << kFirstWavelength << "-" << kLastWavelength
              << " nm with a step of at least 1 nm";
            return fail(ErrorCode::InvalidArgument, m.str());
        }
        std::map<int, Vec2> wanted;
        for (const SpotRequest& s : request.spots) {
            if (!wanted.emplace(s.id, s.pixel).second) {
                return fail(ErrorCode::InvalidArgument, "imager: duplicate spot id " + std::to_string(s.id));
            }
            if (!(s.pixel.x() >= 0.0 && s.pixel.x() < config_.image_width && s.pixel.y() >= 0.0 && s.pixel.y() < config_.image_height)) {
                std::ostringstream m;
                m << "imager: spot " << s.id << " at (" << s.pixel.x() << ", " << s.pixel.y() << ") is outside the " << config_.image_width
                  << "x" << config_.image_height << " image";
                return fail(ErrorCode::OutOfRange, m.str());
            }
        }

        // --- 1. レシピとスポット定義を書く ---
        std::error_code ec;
        fs::create_directories(config_.work_dir, ec);
        if (ec) return fail(ErrorCode::DeviceError, "imager: cannot create " + config_.work_dir.string() + ": " + ec.message());

        io::TopconRecipeSettings recipe;
        recipe.device = config_.device;
        recipe.binning = config_.binning;
        recipe.lens = config_.lens;
        recipe.measure_distance_mm = static_cast<int>(std::lround(request.measure_distance_mm));
        recipe.integration_time_ms = request.integration_time_ms;
        recipe.wavelengths = wr;
        recipe.roi_width = config_.image_width;
        recipe.roi_height = config_.image_height;
        const fs::path recipe_path = config_.work_dir / "recipe.xml";
        if (auto s = writeFile(recipe_path, io::toTopconRecipeXml(recipe)); !s) return tl::unexpected(s.error());

        io::SpotXmlSettings spot_settings;
        spot_settings.roi_width = config_.image_width;
        spot_settings.roi_height = config_.image_height;
        spot_settings.binning = config_.binning;
        spot_settings.spot_size = config_.spot_size;
        const fs::path spot_path = config_.work_dir / "spots.xml";
        if (auto s = writeFile(spot_path, io::toSpotXml(spot_settings, request.spots)); !s) return tl::unexpected(s.error());

        // --- 2. 測定データを作る → 3. 測る ---
        const auto measurement = api_->createMeasurement(recipe_path);
        if (!measurement) return withContext("create measurement data", measurement.error());
        const MeasurementGuard guard(*api_, *measurement);

        const ProgressFn progress = [](const std::string& stage, float value) { spdlog::debug("imager: {} {:.0f}%", stage, value); };
        if (auto s = api_->startMeasurement(device_.device_id, *measurement, progress); !s) {
            return withContext("measurement", s.error());
        }

        // --- 4. 結果の確認 ---
        const auto info = api_->measurementInfo(*measurement);
        if (!info) return withContext("read the measurement information", info.error());
        if (info->saturated) {
            return fail(ErrorCode::Saturated,
                        "imager: the measurement is saturated (too bright: lower the light, the integration time, or use an ND filter)");
        }
        if (info->image_width != config_.image_width || info->image_height != config_.image_height) {
            std::ostringstream m;
            m << "imager: the device image is " << info->image_width << "x" << info->image_height << " but setup.toml expects "
              << config_.image_width << "x" << config_.image_height << " (check [imager] device / binning)";
            return fail(ErrorCode::DeviceError, m.str());
        }

        // --- 5. スポットを設定 → 6. 値を取り出す ---
        if (auto s = api_->setSpotDefinition(*measurement, spot_path); !s) return withContext("set the spots", s.error());
        const auto values = api_->spotValues(*measurement);
        if (!values) return withContext("read the spot values", values.error());

        std::map<int, const SpotValues*> by_id;
        for (const SpotValues& v : *values) by_id[v.spot_id] = &v;

        CaptureResult result;
        const int n = wr.count();
        result.wavelengths.resize(n);
        for (int k = 0; k < n; ++k) result.wavelengths(k) = wr.start + wr.step * k;
        result.integration_time_ms = info->integration_time_ms;
        result.measure_distance_mm = request.measure_distance_mm;

        for (const SpotRequest& s : request.spots) {
            const auto it = by_id.find(s.id);
            if (it == by_id.end()) {
                return fail(ErrorCode::DeviceError, "imager: the SDK returned no data for spot " + std::to_string(s.id));
            }
            const std::vector<double>& spectrum = it->second->spectrum;
            SpotSpectrum out;
            out.id = s.id;
            out.pixel = s.pixel;
            out.values.resize(n);
            for (int k = 0; k < n; ++k) {
                const int index = static_cast<int>(std::lround(result.wavelengths(k))) - kFirstWavelength;
                if (index < 0 || static_cast<std::size_t>(index) >= spectrum.size()) {
                    return fail(ErrorCode::DeviceError, "imager: the SDK spectrum of spot " + std::to_string(s.id) + " is too short");
                }
                out.values(k) = spectrum[static_cast<std::size_t>(index)];
            }
            result.spots.push_back(std::move(out));
        }
        return result;
    }

    Result<GrayImage> TopconImager::captureImage(const ImageRequest& request) {
        if (!connected_) return fail(ErrorCode::NotConnected, "imager: not connected");
        if (request.integration_time_ms < 0.0) {
            return fail(ErrorCode::InvalidArgument, "imager: integration time must not be negative (0 = default)");
        }

        LiveImageRequest live;
        live.binning = config_.binning;
        live.integration_time_ms = request.integration_time_ms > 0.0 ? request.integration_time_ms : config_.live_integration_ms;

        auto image = api_->captureLiveImage(device_.device_id, live);
        if (!image) return withContext("live image", image.error());
        if (image->width != config_.image_width || image->height != config_.image_height) {
            std::ostringstream m;
            m << "imager: the live image is " << image->width << "x" << image->height << " but setup.toml expects " << config_.image_width
              << "x" << config_.image_height << " (check [imager] device / binning)";
            return fail(ErrorCode::DeviceError, m.str());
        }
        return image;
    }

} // namespace brdf::driver::topcon
