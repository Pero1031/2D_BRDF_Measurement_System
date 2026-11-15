/*
* @file include/brdf/sim/MockSpectralImager.hpp
* @brief Simulated 2D spectroradiometer that "measures" a known BRDF.
*/

#pragma once

#include <cstdint>
#include <memory>

#include <optional>
#include <utility>

#include "brdf/calibration/BoardLayout.hpp"
#include "brdf/calibration/SampleToBoard.hpp"
#include "brdf/driver/ISpectralImager.hpp"
#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/SimulatedRig.hpp"

namespace brdf::sim {

    struct MockImagerOptions {
        Real light_intensity = 1.0e6;        ///< 光源の強さ(任意単位)。照度 = 強さ / 距離² [mm²]
        Real noise_relative = 0.0;           ///< ノイズの標準偏差(信号に対する割合)。0 ならノイズなし
        std::uint32_t seed = 12345;          ///< ノイズの乱数シード(同じシードなら同じ結果)
        Real focus_tolerance_mm = 20.0;      ///< 要求した測定距離と実際の距離の許容差。超えるとピントエラー
    };

    /// 試料面に取り付けられたキャリブボード。試料と同じ平面にある。
    struct SimCalibrationBoard {
        calibration::BoardLayout layout;
        /// ボードの取り付け方(試料座標 → ボード座標)。
        calibration::SampleToBoard mapping;
    };

    /// シミュレーション用の 2D 分光放射計。
    ///
    /// 撮影のたびに、SimulatedRig から「いまの光源・試料・カメラの位置」を読み、点ごとに
    ///   1. 画像上の位置 → カメラの光線 → 試料面との交点
    ///   2. 交点での入射角・観察角(optics::computeAngles)
    ///   3. 既知の BRDF モデル f と照度から、分光値  f · cosθi · I0 / d² · 積分時間
    /// を計算する。実機と同じインターフェースなので、測定の流れ全体を実機なしで試せる。
    class MockSpectralImager : public driver::ISpectralImager {
    public:
        MockSpectralImager(std::shared_ptr<const SimulatedRig> rig,
                           std::shared_ptr<const IBrdfModel> brdf,
                           MockImagerOptions options = {});

        Status connect() override;
        void disconnect() override;
        bool isConnected() const override { return connected_; }

        Result<driver::CaptureResult> capture(const driver::CaptureRequest& request) override;

        /// キャリブボードを試料面に取り付けたことにする。captureImage はこのボードを描く。
        void setCalibrationBoard(SimCalibrationBoard board) { board_ = std::move(board); }

        /// ボードを撮影した画像を描く: 紙は明るく、黒円は暗く、紙の外は暗い背景。
        Result<GrayImage> captureImage(const driver::ImageRequest& request) override;

    private:
        std::shared_ptr<const SimulatedRig> rig_;
        std::shared_ptr<const IBrdfModel> brdf_;
        MockImagerOptions options_;
        std::optional<SimCalibrationBoard> board_;
        bool connected_ = false;
        std::uint32_t capture_count_ = 0;   // 撮影ごとに乱数を変えるため
    };

} // namespace brdf::sim
