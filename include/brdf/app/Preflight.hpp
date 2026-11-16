/*
* @file include/brdf/app/Preflight.hpp
* @brief Pre-flight check of a recipe: limits, collisions and angles, without moving any device.
*/

#pragma once

#include <string>
#include <vector>

#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/geometry/RigGeometry.hpp"

namespace brdf::app {

    enum class Severity { Warning, Error };

    struct Issue {
        Severity severity = Severity::Error;
        int pose_index = 0;          ///< 0 から始まるポーズの番号
        std::string pose_id;
        ErrorCode code = ErrorCode::InvalidArgument;
        std::string message;
    };

    struct PreflightReport {
        std::vector<Issue> issues;

        int errorCount() const;
        int warningCount() const;
        bool ok() const { return errorCount() == 0; }
    };

    /// 関節の範囲 [rad]。
    struct JointRange {
        geometry::Joints lower;
        geometry::Joints upper;
    };

    /// 旧システムの姿勢ファイル(19 個、370 行)に現れた関節の範囲。right = RJ(試料側)、left = LJ(光源側)。
    /// 測定のために作られた姿勢の値であり、実際に全部を動かしたとは限らない。「動かしたことのある範囲」であって、
    /// 安全な限界ではない(docs/SafetyProcedure.md の §2)。
    const JointRange& previousSystemRange(bool left_arm);

    struct PreflightOptions {
        /// true なら、旧システムの範囲から previous_range_margin_deg を超えて外れる関節を、警告にする(実機モード用)。
        bool warn_outside_previous_range = false;
        Real previous_range_margin_deg = 15.0;

        /// 姿勢の間の移動で、アーム同士の距離を調べる点の数。関節補間は、各関節が同時に目標へ向かうので、
        /// 途中でアームが大きく振れる。到達点だけを見ると、途中のぶつかりを見逃す。
        int path_samples = 20;
    };

    /// 実機を動かす前に、全ポーズを検査する。
    ///  エラー  : 関節の可動域外、左右アームの干渉(ポーズの到達時と、左→右の順に動く**途中**の両方)。
    ///  警告    : 光源またはカメラが試料面の裏側(θ ≥ 90°)になる測定点がある。
    ///            options で、旧システムの実績範囲の外の姿勢も警告にできる。
    /// 旧システムでは、これらを実機で試して初めて分かっていた。
    PreflightReport preflight(const geometry::RigGeometry& geometry, const domain::Recipe& recipe,
                              const PreflightOptions& options = {});

    /// 1 台だけを動かす(brdf arm move)ときの検査。2 台の実際の位置と、動かす側の目標から、次を調べる。
    ///  エラー  : 今の位置が可動域の外、目標が可動域の外、今の位置で既に干渉している、動く途中(目標までの途中の点を含む)で、
    ///            もう一方(止まったまま)と干渉する。
    PreflightReport checkSingleArmMove(const geometry::RigGeometry& geometry, const geometry::Joints& current_left,
                                       const geometry::Joints& current_right, bool move_left, const geometry::Joints& target,
                                       const PreflightOptions& options = {});

    /// 実機に接続した直後の、アームの**実際の位置**から、レシピの最初の姿勢までの動き(左 → 右の順)を検査する。
    /// preflight はレシピの姿勢の間しか見ないので、最初の移動の安全は、実際の位置を読まないと分からない。
    ///  エラー  : 今の位置が可動域の外、今の位置で既に干渉している、最初の姿勢までの途中で干渉する。
    PreflightReport checkStartPath(const geometry::RigGeometry& geometry, const geometry::Joints& current_left,
                                   const geometry::Joints& current_right, const domain::PoseStep& first,
                                   const PreflightOptions& options = {});

} // namespace brdf::app
