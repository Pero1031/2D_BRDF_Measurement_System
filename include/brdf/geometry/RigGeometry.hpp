/*
* @file include/brdf/geometry/RigGeometry.hpp
* @brief World-space geometry of the rig: where the light, the sample and the camera are.
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/geometry/Frame.hpp"
#include "brdf/geometry/Joints.hpp"
#include "brdf/geometry/RigSetup.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"

namespace brdf::geometry {

    /// 装置全体の幾何。左右アームの関節角から、光源・試料・カメラのワールド座標での位置と向きを返す。
    /// 計算だけを行う純粋なクラスで、実機でもモックでも同じものを使う。
    class RigGeometry {
    public:
        explicit RigGeometry(RigSetup setup = {});

        const RigSetup& setup() const { return setup_; }

        /// 左アーム(光源側)の先端姿勢(ワールド座標)。
        Isometry3 leftTipPose(const Joints& left) const;

        /// 光源の位置(ワールド座標)= 左アーム先端 + light_offset。
        Vec3 lightPosition(const Joints& left) const;

        /// 右アーム(試料側)の先端姿勢(ワールド座標)。試料面までのオフセット(アクリル+紙)を含む。
        Isometry3 sampleHolderPose(const Joints& right) const;

        /// カメラ(2D分光放射計)の位置(ワールド座標)。
        const Vec3& cameraPosition() const { return setup_.camera_position; }

        /// 右アーム先端の座標系そのままの試料座標系。軸 = アーム先端の X, Y, Z。
        /// 旧システムの角度表 (MakeAngleTable) はこの座標系で角度を出していた。
        SurfaceFrame tipFrame(const Joints& right) const;

        /// 紙の座標系。軸 = (先端の +Y, 先端の -X, 先端の +Z)。
        /// 旧システムの試料点ごとの角度(OutputAngles)はこの座標系で出していた。
        /// tipFrame とは φ が 90° ずれる (φ_paper = φ_tip - 90°)。新システムの標準はこちら。
        SurfaceFrame paperFrame(const Joints& right) const;

    private:
        RigSetup setup_;
        Rc8Kinematics kinematics_;
    };

} // namespace brdf::geometry
