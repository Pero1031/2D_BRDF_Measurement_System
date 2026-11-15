/*
* @file include/brdf/driver/ISpectralImager.hpp
* @brief Abstract interface for a 2D spectroradiometer (spectral imager).
*/

#pragma once

#include <vector>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::driver {

    /// 測定したい点 1 つ。画像上の位置で指定する。
    struct SpotRequest {
        int id = 0;                    ///< 点の ID。結果の同じ ID と対応づけて使う(順番に頼らない)
        Vec2 pixel = Vec2::Zero();     ///< 画像上の位置 [pixel](x = 右、y = 下)
    };

    /// 波長の範囲 [nm]。旧システムのレシピは 380〜780 nm、1 nm ステップ。
    struct WavelengthRange {
        Real start = 380.0;
        Real end   = 780.0;
        Real step  = 1.0;

        /// 範囲に含まれる波長の個数。
        int count() const { return static_cast<int>((end - start) / step + 0.5) + 1; }
    };

    /// 1 回の撮影の要求。旧システムの「レシピ XML + スポット XML」に相当する。
    struct CaptureRequest {
        Real measure_distance_mm = 1000.0;     ///< カメラから試料までの距離(ピント合わせに使う)
        Real integration_time_ms = 100.0;      ///< 積分時間
        WavelengthRange wavelengths;           ///< 測定する波長範囲
        std::vector<SpotRequest> spots;        ///< 測定したい点(1 点以上)
    };

    /// 点ごとの測定結果。
    struct SpotSpectrum {
        int id = 0;                    ///< 要求の SpotRequest::id と同じ
        Vec2 pixel = Vec2::Zero();     ///< 要求した画像上の位置
        VecX values;                   ///< 分光値。要素数 = 波長の個数
    };

    /// 1 回の撮影の結果。
    struct CaptureResult {
        VecX wavelengths;                      ///< 波長 [nm]
        std::vector<SpotSpectrum> spots;       ///< 要求した順に並ぶ
        Real integration_time_ms = 0.0;        ///< 実際に使った積分時間
        Real measure_distance_mm = 0.0;        ///< 実際に使った測定距離
    };

    /// 2D 分光放射計のインターフェース。実機(TOPCON SR-5000H)とモックが実装する。
    class ISpectralImager {
    public:
        virtual ~ISpectralImager() = default;

        virtual Status connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

        /// 撮影して、指定した点のスペクトルを返す。完了してから戻る(ブロッキング)。
        virtual Result<CaptureResult> capture(const CaptureRequest& request) = 0;
    };

} // namespace brdf::driver
