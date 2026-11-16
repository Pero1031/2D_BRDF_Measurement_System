/*
* @file include/brdf/driver/topcon/HsApi.hpp
* @brief The calls TopconImager needs from the TOPCON SDK (hs_api), as a small abstract interface.
*
* This header does not include hs_api.h. RealHsApi (built only when BRDF_WITH_TOPCON is ON) implements it with the
* SDK; the tests implement it with a fake. Handles are plain integers; every call returns a Result whose error
* already says which SDK function failed.
*/

#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/core/Image.hpp"

namespace brdf::driver::topcon {

    /// 接続した装置の情報。
    struct DeviceInfo {
        unsigned device_id = 0;
        std::string device_type;     ///< "SR5000" / "SR5100" / それ以外の機種名
        std::string product_id;
    };

    /// 測定が終わったあとの測定データの情報。
    struct MeasurementInfo {
        int image_width = 0;
        int image_height = 0;
        Real integration_time_ms = 0.0;    ///< 実際に使った積分時間(分光フィルタ)
        bool saturated = false;
    };

    /// 1 つのスポットの測定値。
    struct SpotValues {
        int spot_id = 0;
        Vec2 location = Vec2::Zero();      ///< 画像上の位置 [pixel]
        std::vector<double> spectrum;      ///< 添字 i が波長 (380 + i) nm の分光放射輝度
    };

    /// ライブ画像の撮影条件。
    struct LiveImageRequest {
        std::string binning = "BINNING_TYPE1";
        Real integration_time_ms = 10.0;
    };

    /// 測定の進み具合の通知。stage は "optimizing"、"scanning"、"processing" など。
    using ProgressFn = std::function<void(const std::string& stage, float progress)>;

    class ITopconApi {
    public:
        virtual ~ITopconApi() = default;

        /// hsInitialize。つながっている装置の数を返す。
        virtual Result<int> initialize(const std::string& calibration_dir) = 0;
        /// hsOpenDevice(index 番目の装置)。
        virtual Result<DeviceInfo> openDevice(int index) = 0;
        virtual Status closeDevice(unsigned device_id) = 0;
        /// hsFinalize。
        virtual Status finalize() = 0;

        /// レシピの XML を読み、測定データを作る(hsCreateRecipe + hsLoadRecipe + hsCreateMeasurementData)。
        /// 返す番号は、destroyMeasurement で必ず解放する。
        virtual Result<int> createMeasurement(const std::filesystem::path& recipe_xml) = 0;
        /// hsStartMeasurement。測定が終わるまで戻らない。
        virtual Status startMeasurement(unsigned device_id, int measurement_id, const ProgressFn& progress) = 0;
        virtual Result<MeasurementInfo> measurementInfo(int measurement_id) = 0;
        /// スポット定義の XML を読んで、標準スポットとして測定データに設定する。
        virtual Status setSpotDefinition(int measurement_id, const std::filesystem::path& spot_xml) = 0;
        /// 設定したスポットごとの値を取り出す(hsCreateSpotDataList + hsGetSpotData)。
        virtual Result<std::vector<SpotValues>> spotValues(int measurement_id) = 0;
        virtual Status destroyMeasurement(int measurement_id) = 0;

        /// ライブ画像を 1 枚撮る(設定 → 撮影開始 → 取得 → 撮影終了)。明るさ(グレー)で返す。
        virtual Result<GrayImage> captureLiveImage(unsigned device_id, const LiveImageRequest& request) = 0;
    };

} // namespace brdf::driver::topcon
