/*
* @file include/brdf/app/SpotProjector.hpp
* @brief Maps a point on the sample (world coordinates) to a position in the camera image.
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::app {

    /// ワールド座標の点 → 画像上の位置 [pixel]。
    ///
    /// 測定点を試料座標で指定し、撮影のときだけ画像の位置に直すための窓口。
    ///  - シミュレーション: シミュレータのカメラモデル(sim::CameraSpotProjector)
    ///  - 実機(P4): キャリブボードから求めたホモグラフィ
    class SpotProjector {
    public:
        virtual ~SpotProjector() = default;

        /// 画像の外やカメラの後ろになる点はエラー。
        virtual Result<Vec2> pixelFor(const Vec3& world_point) const = 0;
    };

} // namespace brdf::app
