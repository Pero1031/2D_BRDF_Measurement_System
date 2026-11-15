/*
* @file include/brdf/sim/PinholeCamera.hpp
* @brief Pinhole camera model used by the simulated spectral imager.
*/

#pragma once

#include "brdf/core/Algebra.hpp"

namespace brdf::sim {

    /// シミュレータ用のカメラの設定。
    struct PinholeCameraSetup {
        Vec3 position = Vec3(1136.0, 728.0, 560.0);   ///< カメラの位置(ワールド座標, mm)
        Vec3 target   = Vec3(0.0, 0.0, 0.0);          ///< カメラが向いている点(画像の中心に写る点)
        int image_width  = 1376;                      ///< 画像の幅 [pixel]
        int image_height = 1024;                      ///< 画像の高さ [pixel]
        /// 注視点の位置での画像の解像度 [pixel/mm]。旧システムの記録にある 1.81 を既定値にした。
        Real pixels_per_mm = 1.81;
    };

    /// ピンホールカメラ。画像上の位置 ↔ ワールド座標の光線、の対応を与える。
    /// 本物のカメラは固定されているので、向きは注視点から 1 回だけ決める。
    ///
    /// 画像の軸: u = 右、v = 下(OpenCV と同じ)。カメラの上方向はワールドの +Z。
    class PinholeCamera {
    public:
        explicit PinholeCamera(PinholeCameraSetup setup = {});

        const PinholeCameraSetup& setup() const { return setup_; }
        const Vec3& position() const { return setup_.position; }

        /// 画像の位置 (u, v) [pixel] を通る光線の向き(ワールド座標の単位ベクトル)。
        Vec3 rayDirection(const Vec2& pixel) const;

        /// ワールド座標の点が写る画像上の位置 [pixel]。カメラの後ろにある点は NaN を返す。
        Vec2 project(const Vec3& world_point) const;

        /// 画像の範囲内か。
        bool inImage(const Vec2& pixel) const;

    private:
        PinholeCameraSetup setup_;
        Vec3 forward_;
        Vec3 right_;
        Vec3 down_;
        Real focal_px_;
    };

} // namespace brdf::sim
