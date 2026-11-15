/*
* @file include/brdf/geometry/Rc8Kinematics.hpp
* @brief Forward kinematics of the 6-axis DENSO RC8 arms used by the rig.
*/

#pragma once

#include <array>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/geometry/Joints.hpp"

namespace brdf::geometry {

    /// アームのリンク寸法 [mm]。旧システムの RobotParam に相当する。
    /// 括弧内は旧コードでの名前。既定値は DENSO VP-6242 の外形図の寸法と一致する
    /// (土台の高さ 280、J2-J3 間 210 と 75、前腕 210、手首〜ツール取付面 70)。
    struct ArmLinks {
        Real base_height       = 280.0;     ///< 土台から第1アームまでの高さ (BASE)
        Real upper_arm         = 210.0;     ///< 第1アームの長さ (ARM1)
        Real elbow_offset      = 75.0;      ///< アーム1とアーム2をつなぐジョイントの長さ (JOINT)
        Real forearm           = 210.0;     ///< 第2アームの長さ (ARM2)
        Real wrist             = 70.0;      ///< 第3アームの長さ (ARM3)
        Real elbow_fixed_angle = HALF_PI;   ///< アーム1と2のジョイントに固定された角度 [rad] (JOINT_FIX_ANGLE=90°)
    };

    /// 6軸アームの順運動学。関節角からアーム先端の位置・姿勢を計算する純粋な計算クラス
    /// (通信・表示・ファイルアクセスは一切しない)。
    ///
    /// 変換の連鎖(旧 Calc::calcLeftTip と同じ):
    ///   Trans(0,0,BASE) · RotZ(J1) · RotY(J2) · Trans(0,0,ARM1)
    ///   · RotY(-(90° - J3)) · Trans(0,0,JOINT) · RotY(固定角) · Trans(0,0,ARM2)
    ///   · RotZ(J4) · RotY(J5) · Trans(0,0,ARM3) · RotZ(J6)
    ///
    /// 結果は「アームの土台の中心を原点とする座標系」での先端姿勢。
    /// 土台がワールド座標のどこにあるかは RigGeometry が扱う。
    class Rc8Kinematics {
    public:
        explicit Rc8Kinematics(ArmLinks links = {}) : links_(links) {}

        const ArmLinks& links() const { return links_; }

        /// 関節角 [rad] → 先端姿勢(土台座標系)。
        Isometry3 tipPose(const Joints& q) const;

        /// アームを折れ線で表したときの 6 つの点(土台座標系)。
        /// [0]=土台の足元, [1]=第1アームの付け根, [2]=肩, [3]=肘, [4]=手首, [5]=先端。
        /// 干渉チェックでアームを細い棒の連なりとして扱うために使う。
        std::array<Vec3, 6> linkPoints(const Joints& q) const;

    private:
        Isometry3 chain(const Joints& q, std::array<Vec3, 6>* points) const;

        ArmLinks links_;
    };

} // namespace brdf::geometry
