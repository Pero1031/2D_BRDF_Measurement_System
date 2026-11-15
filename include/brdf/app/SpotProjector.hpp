/*
* @file include/brdf/app/SpotProjector.hpp
* @brief Maps a point on the sample to a position in the camera image.
*/

#pragma once

#include <string>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::app {

    /// 画像上の位置を知りたい測定点 1 つ。
    struct SpotQuery {
        std::string pose_id;                 ///< どのポーズでの撮影か(ホモグラフィはポーズごとに違う)
        Vec2 sample_mm = Vec2::Zero();       ///< 試料面の座標(紙の座標系, mm)
        Vec3 world_point = Vec3::Zero();     ///< 同じ点のワールド座標(幾何から計算済み)
    };

    /// 試料面の点 → 画像上の位置 [pixel]。
    ///
    /// 測定点を試料座標で指定し、撮影のときだけ画像の位置に直すための窓口。
    ///  - カメラモデル(sim::CameraSpotProjector): シミュレータの「真の」カメラで、ワールド座標から求める
    ///  - キャリブボード(calibration::HomographySpotProjector): ポーズごとのホモグラフィで、試料座標から求める
    class SpotProjector {
    public:
        virtual ~SpotProjector() = default;

        /// 画像の外やカメラの後ろになる点、校正データのないポーズはエラー。
        virtual Result<Vec2> pixelFor(const SpotQuery& query) const = 0;
    };

} // namespace brdf::app
