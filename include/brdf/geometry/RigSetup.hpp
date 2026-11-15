/*
* @file include/brdf/geometry/RigSetup.hpp
* @brief Physical layout of the measurement rig (arms, light, camera, sample holder).
*/

#pragma once

#include "brdf/core/Algebra.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"

namespace brdf::geometry {

    /// 測定装置の物理的な配置。旧システムでヘッダに直書きされていた定数
    /// (RobotParam / SpectrometerParam / LightSourcePos / calcRightTip 内の厚み)をここに集約する。
    ///
    /// ワールド座標系: 原点 = 左アームの土台の中心(床の養生テープが目印)、単位は mm。
    ///
    /// 注意: カメラ位置は測定日ごとに手で合わせ直す値で、旧システムではその都度ヘッダを書き換えていた。
    /// ここの既定値は「現在のヘッダ値」にすぎない。実際の測定では setup.toml(P3 で実装)から読み込み、
    /// 測定結果と一緒に保存すること。
    struct RigSetup {
        ArmLinks links;                                      ///< 左右アーム共通のリンク寸法

        Vec3 left_base_position  = Vec3(0.0,   0.0, 0.0);    ///< 左アーム(光源側)の土台位置 = 原点
        Vec3 right_base_position = Vec3(0.0, 945.0, 0.0);    ///< 右アーム(試料側)の土台位置 (RightX/Y/Z)

        /// 光源の位置: 左アーム先端の座標系で見たオフセット (旧 LightSourcePos の deltax, deltay)
        Vec3 light_offset = Vec3(-30.0, 108.0, 0.0);

        /// 2D分光放射計の位置(ワールド座標)。旧 SpectrometerParam の PosX/Y/Z (現在のヘッダ値)
        Vec3 camera_position = Vec3(1136.0, 728.0, 560.0);

        /// 右アーム先端から試料面までの距離(先端の Z 方向)。アクリル板 5mm + 紙 1mm (旧 calcRightTip の akuriru, paper)
        Real sample_surface_offset = 6.0;
    };

} // namespace brdf::geometry
