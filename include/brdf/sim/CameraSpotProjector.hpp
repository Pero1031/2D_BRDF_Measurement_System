/*
* @file include/brdf/sim/CameraSpotProjector.hpp
* @brief SpotProjector backed by the simulated camera model.
*/

#pragma once

#include <memory>

#include "brdf/app/SpotProjector.hpp"
#include "brdf/sim/SimulatedRig.hpp"

namespace brdf::sim {

    /// シミュレータのカメラモデルで、ワールド座標の点を画像上の位置に直す。
    /// 実機では、これをキャリブボードのホモグラフィ(P4)に置き換える。
    class CameraSpotProjector : public app::SpotProjector {
    public:
        explicit CameraSpotProjector(std::shared_ptr<const SimulatedRig> rig) : rig_(std::move(rig)) {}

        Result<Vec2> pixelFor(const Vec3& world_point) const override;

    private:
        std::shared_ptr<const SimulatedRig> rig_;
    };

} // namespace brdf::sim
