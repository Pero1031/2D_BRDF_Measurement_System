/*
* @file include/brdf/driver/topcon/TopconImager.hpp
* @brief ISpectralImager for the real TOPCON SR-5000 / SR-5100, on top of the SDK wrapper ITopconApi.
*/

#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "brdf/driver/ISpectralImager.hpp"
#include "brdf/driver/topcon/HsApi.hpp"

namespace brdf::driver::topcon {

    struct TopconImagerConfig {
        std::string calibration_dir;             ///< SDK の校正データのフォルダ
        std::string device = "SR5100";           ///< 期待する機種("SR5100" / "SR5000")。違う装置につながったら失敗する
        std::string binning = "BINNING_TYPE1";
        std::string lens = "STANDARD";
        int image_width = 2448;                  ///< setup.toml の画像の大きさ。SDK が返す大きさと合うか確かめる
        int image_height = 2048;
        Real spot_size = 2.0;                    ///< スポットの一辺 [pixel]
        Real live_integration_ms = 10.0;         ///< captureImage で、積分時間の指定がないときの値
        std::filesystem::path work_dir;          ///< レシピとスポット定義の XML を一時的に書く場所
    };

    /// 実機の分光放射計。SDK の呼び出し順は docs/TopconSdkDesign.md の §3。
    ///
    /// SDK はプロセス全体で 1 つの接続しか扱えないので、同時に接続できる TopconImager は 1 つだけ。
    class TopconImager final : public ISpectralImager {
    public:
        TopconImager(std::shared_ptr<ITopconApi> api, TopconImagerConfig config);
        ~TopconImager() override;

        TopconImager(const TopconImager&) = delete;
        TopconImager& operator=(const TopconImager&) = delete;

        Status connect() override;
        void disconnect() override;
        bool isConnected() const override { return connected_; }

        Result<CaptureResult> capture(const CaptureRequest& request) override;
        Result<GrayImage> captureImage(const ImageRequest& request) override;

    private:
        std::shared_ptr<ITopconApi> api_;
        TopconImagerConfig config_;
        bool connected_ = false;
        bool owns_sdk_ = false;      ///< このインスタンスが hsInitialize を済ませている(= 同時接続の枠を持っている)
        DeviceInfo device_;
    };

} // namespace brdf::driver::topcon
