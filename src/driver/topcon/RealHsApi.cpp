/*
* @file src/driver/topcon/RealHsApi.cpp
* @brief ITopconApi on the real TOPCON SDK. The only file that includes hs_api.h (built when BRDF_WITH_TOPCON is ON).
*
* Not verified on a real device yet. The points that are still guesses are listed in docs/TopconSdkDesign.md §8;
* the ones that matter here are marked "UNVERIFIED".
*/

#include "brdf/driver/topcon/RealHsApi.hpp"

#include <windows.h>

#include <cstring>
#include <memory>
#include <set>
#include <sstream>

#include "hs_api.h"

namespace brdf::driver::topcon {

    namespace {

        // ---------------------------------------------------------------- エラー

        ErrorCode errorCodeOf(HsError e) {
            switch (e) {
            case HS_ERROR_NOT_INITIALIZED:
                return ErrorCode::NotConnected;
            case HS_ERROR_INVALID_PARAMETER:
            case HS_ERROR_INVALID_IMAGE_SIZE:
            case HS_ERROR_INVALID_INDEX:
            case HS_ERROR_ADJUST_SIZE:
            case HS_ERROR_CALIBRATION_PATH_NOT_FOUND:
            case HS_ERROR_PATH_DOES_NOT_EXIST:
            case HS_ERROR_CANNOT_FIND_FILE:
            case HS_ERROR_CANNOT_LOAD_FILE:
            case HS_ERROR_INVALID_FILE_FORMAT:
                return ErrorCode::InvalidArgument;
            case HS_ERROR_SATURATION:
            case HS_ERROR_OPTIMIZE_TOO_BRIGHT:
                return ErrorCode::Saturated;
            case HS_ERROR_PROCESS_TIMEOUT:
                return ErrorCode::Timeout;
            case HS_ERROR_NO_CONNECTABLE_DEVICE:
            case HS_ERROR_CANNOT_FIND_CONNECTED_DEVICE:
            case HS_ERROR_CANNOT_CONNECT_TO_DEVICE:
                return ErrorCode::Communication;
            default:
                return ErrorCode::DeviceError;
            }
        }

        /// "hsXxx failed: <SDK のメッセージ> (code N)"
        tl::unexpected<Error> sdkError(const char* function, HsError e) {
            char text[256] = {0};
            hsGetErrorString(e, sizeof(text), text);
            std::ostringstream m;
            m << function << " failed: " << (text[0] != '\0' ? text : "unknown error") << " (code " << static_cast<int>(e) << ")";
            return fail(errorCodeOf(e), m.str());
        }

        // ---------------------------------------------------------------- 文字列 ↔ SDK の型

        std::string deviceName(HsDeviceType t) {
            switch (t) {
            case HS_DEVICE_SR5000: return "SR5000";
            case HS_DEVICE_SR5100: return "SR5100";
            default: return "other(" + std::to_string(static_cast<int>(t)) + ")";
            }
        }

        Result<HsBinningType> binningOf(const std::string& s) {
            if (s == "BINNING_TYPE1") return HS_BINNING_TYPE1;
            if (s == "BINNING_TYPE2") return HS_BINNING_TYPE2;
            if (s == "BINNING_TYPE4") return HS_BINNING_TYPE4;
            return fail(ErrorCode::InvalidArgument, "unknown binning \"" + s + "\"");
        }

        /// SDK は ANSI(日本語 Windows では CP932)の文字列でパスを受け取る。変換で文字が壊れるパスは使えない。
        Result<std::string> sdkPath(const std::filesystem::path& p) {
            const std::string narrow = p.string();
            if (std::filesystem::path(narrow).wstring() != p.wstring()) {
                return fail(ErrorCode::InvalidArgument,
                            "the path contains characters the SDK cannot take (use a folder with plain ASCII letters): " + narrow);
            }
            return narrow;
        }

        // ---------------------------------------------------------------- 進捗

        const ProgressFn* g_progress = nullptr;

        const char* stageName(HsRunningStatus s) {
            switch (s) {
            case HS_STATUS_LOADING_DATA: return "loading data";
            case HS_STATUS_CREATING_SPOT_DATA: return "creating spot data";
            case HS_STATUS_WRITING_CSV: return "writing csv";
            case HS_STATUS_OPTIMIZING: return "optimizing";
            case HS_STATUS_SCANNING: return "scanning";
            case HS_STATUS_PROCESSING: return "processing";
            case HS_STATUS_SEARCH_SPECTRUM: return "search spectrum";
            case HS_STATUS_SIMULATION: return "simulation";
            default: return "working";
            }
        }

        /// SDK のコールバックにはユーザーデータを渡せないので、静的な変数で橋渡しする(装置は 1 台しか使わない)。
        void __stdcall progressThunk(HsRunningStatus status, float progress) {
            if (g_progress && *g_progress) (*g_progress)(stageName(status), progress);
        }

        // ---------------------------------------------------------------- 後始末

        // 作る(読む)のに成功したものだけを解放する。armed を立てるのは、成功を確かめたあと。
        struct RecipeGuard {
            HsRecipe recipe{};
            bool armed = false;
            ~RecipeGuard() { if (armed) hsDestroyRecipe(&recipe); }
        };

        struct SpotDefGuard {
            HsSpotDef def{};
            bool armed = false;
            ~SpotDefGuard() { if (armed) hsDestroySpotDef(&def); }
        };

        struct SpotDataListGuard {
            HsSpotDataList list{};
            bool armed = false;
            ~SpotDataListGuard() { if (armed) hsDestroySpotDataList(&list); }
        };

        struct Image8u3Guard {
            HsImage8u3 image{};
            bool armed = false;
            ~Image8u3Guard() { if (armed) hsDestroyImage8u3(&image); }
        };

    } // namespace

    struct RealHsApi::Impl {
        HsConfiguration config;
    };

    RealHsApi::RealHsApi() : impl_(std::make_unique<Impl>()) { std::memset(&impl_->config, 0, sizeof(impl_->config)); }
    RealHsApi::~RealHsApi() = default;

    Result<int> RealHsApi::initialize(const std::string& calibration_dir) {
        std::memset(&impl_->config, 0, sizeof(impl_->config));
        const HsError e = hsInitialize(calibration_dir.c_str(), &impl_->config);
        if (e != HS_ERROR_NONE) return sdkError("hsInitialize", e);
        return impl_->config.num_connected;
    }

    Result<DeviceInfo> RealHsApi::openDevice(int index) {
        if (index < 0 || index >= impl_->config.num_connected) {
            return fail(ErrorCode::InvalidArgument, "hsOpenDevice: no device with index " + std::to_string(index));
        }
        HsDeviceInfo info;
        std::memset(&info, 0, sizeof(info));
        const HsError e = hsOpenDevice(&impl_->config.connected_devices[index], &info);
        if (e != HS_ERROR_NONE) return sdkError("hsOpenDevice", e);

        DeviceInfo out;
        out.device_id = info.device_id;
        out.device_type = deviceName(info.device_type);
        out.product_id = info.product_id;
        return out;
    }

    Status RealHsApi::closeDevice(unsigned device_id) {
        const HsError e = hsCloseDevice(device_id);
        if (e != HS_ERROR_NONE) return sdkError("hsCloseDevice", e);
        return {};
    }

    Status RealHsApi::finalize() {
        const HsError e = hsFinalize();
        if (e != HS_ERROR_NONE) return sdkError("hsFinalize", e);
        return {};
    }

    Result<int> RealHsApi::createMeasurement(const std::filesystem::path& recipe_xml) {
        const auto path = sdkPath(recipe_xml);
        if (!path) return tl::unexpected(path.error());

        RecipeGuard guard;
        HsError e = hsCreateRecipe(&guard.recipe);
        if (e != HS_ERROR_NONE) return sdkError("hsCreateRecipe", e);
        guard.armed = true;
        e = hsLoadRecipe(path->c_str(), &guard.recipe);
        if (e != HS_ERROR_NONE) return sdkError("hsLoadRecipe", e);

        int id = 0;
        e = hsCreateMeasurementData(&guard.recipe, &id);
        if (e != HS_ERROR_NONE) return sdkError("hsCreateMeasurementData", e);
        return id;
    }

    Status RealHsApi::startMeasurement(unsigned device_id, int measurement_id, const ProgressFn& progress) {
        g_progress = &progress;
        const HsError e = hsStartMeasurement(device_id, measurement_id, 1, &progressThunk);
        g_progress = nullptr;
        if (e != HS_ERROR_NONE) return sdkError("hsStartMeasurement", e);
        return {};
    }

    Result<MeasurementInfo> RealHsApi::measurementInfo(int measurement_id) {
        // HsMeasurementData は大きい(スペクトルの統計を含む)ので、スタックに置かない
        auto data = std::make_unique<HsMeasurementData>();
        std::memset(data.get(), 0, sizeof(HsMeasurementData));
        const HsError e = hsGetMeasurementData(measurement_id, data.get());
        if (e != HS_ERROR_NONE) return sdkError("hsGetMeasurementData", e);

        MeasurementInfo info;
        info.image_width = data->image_width;
        info.image_height = data->image_height;
        info.integration_time_ms = data->integration_time[HS_FILTER_TYPE_SPECTRAL];
        info.saturated = data->is_saturated;
        return info;
    }

    Status RealHsApi::setSpotDefinition(int measurement_id, const std::filesystem::path& spot_xml) {
        const auto path = sdkPath(spot_xml);
        if (!path) return tl::unexpected(path.error());

        SpotDefGuard guard;
        HsError e = hsLoadSpotDef(path->c_str(), &guard.def);
        if (e != HS_ERROR_NONE) return sdkError("hsLoadSpotDef", e);
        guard.armed = true;
        e = hsSetMeasurementStandardSpotDef(measurement_id, &guard.def);
        if (e != HS_ERROR_NONE) return sdkError("hsSetMeasurementStandardSpotDef", e);
        return {};
    }

    Result<std::vector<SpotValues>> RealHsApi::spotValues(int measurement_id) {
        SpotDataListGuard guard;
        HsError e = hsCreateSpotDataList(measurement_id, HS_SPOT_DATA_TYPE_STANDARD, &progressThunk, &guard.list);
        if (e != HS_ERROR_NONE) return sdkError("hsCreateSpotDataList", e);
        guard.armed = true;

        std::vector<SpotValues> out;
        std::set<int> seen;
        auto data = std::make_unique<HsSpotData>();
        for (int i = 0; i < guard.list.count; ++i) {
            std::memset(data.get(), 0, sizeof(HsSpotData));
            // UNVERIFIED: hsGetSpotData は const のリストを受け取るので、取り出す点は spot_data.spot_id で指定するはず
            // (添字なのか、スポット XML の id なのかは未確認)。下の検査で、同じ点ばかり返ってくる場合は失敗にする。
            data->spot_id = i;
            e = hsGetSpotData(&guard.list, data.get());
            if (e != HS_ERROR_NONE) return sdkError("hsGetSpotData", e);
            if (!seen.insert(data->spot_id).second) {
                return fail(ErrorCode::DeviceError,
                            "hsGetSpotData returned spot " + std::to_string(data->spot_id)
                                + " more than once (the way to walk the spot list is not what this driver assumes)");
            }

            SpotValues v;
            v.spot_id = data->spot_id;
            v.location = Vec2(data->location.x, data->location.y);
            // UNVERIFIED: 添字 i = 波長 380 + i nm、値 = 分光放射輝度(単位は SDK のマニュアルで確認する)
            constexpr int kCount = 780 - HS_SPECTRAL_WAVELENGTH_MIN + 1;
            v.spectrum.reserve(kCount);
            for (int w = 0; w < kCount; ++w) v.spectrum.push_back(data->spectral_statistics[w].average);
            out.push_back(std::move(v));
        }
        return out;
    }

    Status RealHsApi::destroyMeasurement(int measurement_id) {
        const HsError e = hsDestroyMeasurementData(measurement_id);
        if (e != HS_ERROR_NONE) return sdkError("hsDestroyMeasurementData", e);
        return {};
    }

    Result<GrayImage> RealHsApi::captureLiveImage(unsigned device_id, const LiveImageRequest& request) {
        const auto binning = binningOf(request.binning);
        if (!binning) return tl::unexpected(binning.error());

        HsDeviceType device_type = HS_DEVICE_UNKNOWN;
        HsError e = hsGetDeviceType(device_id, &device_type);
        if (e != HS_ERROR_NONE) return sdkError("hsGetDeviceType", e);
        HsSize size{};
        e = hsGetImageSize(device_type, *binning, &size);
        if (e != HS_ERROR_NONE) return sdkError("hsGetImageSize", e);

        // いまの設定を読んで、必要な所だけ書き換える(live_view の例は、すべてを自分で設定している)
        HsLiveSetting live;
        std::memset(&live, 0, sizeof(live));
        e = hsGetLiveSetting(device_id, &live);
        if (e != HS_ERROR_NONE) return sdkError("hsGetLiveSetting", e);
        live.binning_type = *binning;
        live.integration_time = request.integration_time_ms;
        live.roi.x = 0;
        live.roi.y = 0;
        live.roi.width = size.width;
        live.roi.height = size.height;
        live.trigger_config.mode_type = HS_TRIGGER_MODE_TYPE_SOFT;
        live.trigger_config.activation_type = HS_TRIGGER_ACTIVATION_RISING_EDGE;
        live.trigger_config.delay_time = 0.0;
        e = hsSetLiveSetting(device_id, &live);
        if (e != HS_ERROR_NONE) return sdkError("hsSetLiveSetting", e);

        Image8u3Guard image;
        e = hsCreateImage8u3(size.width, size.height, &image.image);
        if (e != HS_ERROR_NONE) return sdkError("hsCreateImage8u3", e);
        image.armed = true;

        e = hsStartCapture(device_id);
        if (e != HS_ERROR_NONE) return sdkError("hsStartCapture", e);

        // 設定を変えた直後の画像は古いかもしれないので、数枚撮って最後の 1 枚を使う。取得に失敗したら撮り直す(live_view と同じ)
        constexpr int kFrames = 3;
        constexpr int kMaxAttempts = 30;
        HsFocusSetting focus{};
        focus.enabled = false;
        focus.focus_rect.x = 0;
        focus.focus_rect.y = 0;
        focus.focus_rect.width = size.width;
        focus.focus_rect.height = size.height;
        auto focus_result = std::make_unique<HsFocusResult>();

        int got = 0;
        HsError last = HS_ERROR_NONE;
        for (int attempt = 0; attempt < kMaxAttempts && got < kFrames; ++attempt) {
            last = hsCaptureLiveImage(device_id, &focus, &image.image, focus_result.get());
            if (last == HS_ERROR_NONE) ++got;
        }
        const HsError stop = hsStopCapture(device_id);
        if (got < kFrames) return sdkError("hsCaptureLiveImage", last);
        if (stop != HS_ERROR_NONE) return sdkError("hsStopCapture", stop);

        // 3 チャンネルの平均で明るさにする(R,G,B の並び順は未確認だが、平均なら順番によらない)
        GrayImage gray(size.width, size.height);
        const std::size_t n = static_cast<std::size_t>(size.width) * static_cast<std::size_t>(size.height);
        for (std::size_t i = 0; i < n; ++i) {
            const unsigned char* p = image.image.pixels + 3 * i;
            gray.pixels[i] = static_cast<std::uint8_t>((static_cast<int>(p[0]) + p[1] + p[2]) / 3);
        }
        return gray;
    }

} // namespace brdf::driver::topcon
