/*
* @file include/brdf/sim/SimulatedRig.hpp
* @brief The simulated world shared by the mock arms and the mock imager.
*/

#pragma once

#include "brdf/core/Error.hpp"
#include "brdf/geometry/Joints.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/sim/PinholeCamera.hpp"

namespace brdf::sim {

    enum class ArmSide { Left, Right };

    inline const char* toString(ArmSide side) { return side == ArmSide::Left ? "Left" : "Right"; }

    /// シミュレーション上の装置の状態。左右アームの関節角を保持する。
    ///
    /// MockRobotArm がアームを動かすと、ここの関節角が更新される。
    /// MockSpectralImager は、撮影のときにここから「いまの光源・試料・カメラの位置」を読む。
    /// こうして、アームの動きが撮影結果に反映される。
    class SimulatedRig {
    public:
        /// camera を省略すると、注視点を「右アームが典型的な測定姿勢 (-45,0,90,0,0,0) のときの試料位置」にする。
        explicit SimulatedRig(geometry::RigSetup setup = {});
        SimulatedRig(geometry::RigSetup setup, PinholeCameraSetup camera);

        const geometry::RigGeometry& geometry() const { return geometry_; }
        const PinholeCamera& camera() const { return camera_; }

        const geometry::Joints& joints(ArmSide side) const;

        /// アームを目標の関節角へ動かす(瞬時)。可動域外、またはもう一方のアームと干渉する姿勢はエラー。
        /// エラーのときは状態を変えない。
        Status moveTo(ArmSide side, const geometry::Joints& target);

        // --- 現在の状態から得られる、撮影に必要な量 ---
        Vec3 lightPosition() const;                  ///< 光源の位置(ワールド座標)
        geometry::SurfaceFrame sampleFrame() const;  ///< 試料の座標系(紙の座標系)

    private:
        static PinholeCameraSetup defaultCamera(const geometry::RigGeometry& geometry);

        geometry::RigGeometry geometry_;
        PinholeCamera camera_;
        geometry::Joints left_;
        geometry::Joints right_;
    };

} // namespace brdf::sim
