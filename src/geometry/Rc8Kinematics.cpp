/*
* @file src/geometry/Rc8Kinematics.cpp
* @brief Forward kinematics of the 6-axis DENSO RC8 arms.
*/

#include "brdf/geometry/Rc8Kinematics.hpp"

namespace brdf::geometry {

    namespace {
        using Rot = Eigen::AngleAxis<Real>;
        const Vec3 kY = Vec3::UnitY();
        const Vec3 kZ = Vec3::UnitZ();
    }

    Isometry3 Rc8Kinematics::tipPose(const Joints& q) const {
        // Eigen の translate()/rotate() は「右から掛ける」ので、書いた順がそのまま変換の連鎖の順になる。
        Isometry3 t = Isometry3::Identity();

        t.translate(Vec3(0, 0, links_.base_height));
        t.rotate(Rot(q(0), kZ));                         // J1: 土台の旋回
        t.rotate(Rot(q(1), kY));                         // J2: 肩
        t.translate(Vec3(0, 0, links_.upper_arm));
        t.rotate(Rot(-(HALF_PI - q(2)), kY));            // J3: 肘 (90° が「腕を水平に伸ばした」姿勢)
        t.translate(Vec3(0, 0, links_.elbow_offset));
        t.rotate(Rot(links_.elbow_fixed_angle, kY));     // アーム1と2の間の固定角
        t.translate(Vec3(0, 0, links_.forearm));
        t.rotate(Rot(q(3), kZ));                         // J4: 手首ロール
        t.rotate(Rot(q(4), kY));                         // J5: 手首ピッチ
        t.translate(Vec3(0, 0, links_.wrist));
        t.rotate(Rot(q(5), kZ));                         // J6: 先端ロール

        return t;
    }

} // namespace brdf::geometry
