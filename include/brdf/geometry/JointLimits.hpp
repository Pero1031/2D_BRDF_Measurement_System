/*
* @file include/brdf/geometry/JointLimits.hpp
* @brief Joint range limits of a 6-axis arm.
*/

#pragma once

#include "brdf/geometry/Joints.hpp"

namespace brdf::geometry {

    /// 関節の可動域 [rad]。
    ///
    /// 既定値は DENSO VP-6242(6軸)のソフトウェアリミットの出荷時設定値
    /// (VP ロボット取扱説明書「ソフトウェアリミットの出荷時の設定値」)。
    ///   J1 ±160°、J2 ±120°、J3 +19°〜+160°、J4 ±160°、J5 ±120°、J6 ±360°
    /// J3 は 90° が「前腕が水平」で、外形図の 71°(上向き)と 70°(下向き)の余裕と一致する。
    /// 実機のコントローラでリミットを変更している場合は、setup.toml(P3 で実装)で上書きすること。
    struct JointLimits {
        Joints lower = jointsFromDegrees(-160.0, -120.0,  19.0, -160.0, -120.0, -360.0);
        Joints upper = jointsFromDegrees( 160.0,  120.0, 160.0,  160.0,  120.0,  360.0);

        /// 可動域を外れている最初の関節の番号 (0〜5)。全部範囲内なら -1。
        int firstViolation(const Joints& q) const {
            for (int i = 0; i < 6; ++i) {
                if (q(i) < lower(i) || q(i) > upper(i)) return i;
            }
            return -1;
        }
    };

} // namespace brdf::geometry
