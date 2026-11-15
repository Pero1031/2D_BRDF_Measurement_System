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

    /// 実機を動かす前に、全ポーズを検査する。
    ///  エラー  : 関節の可動域外、左右アームの干渉(ポーズの到達時と、左→右の順に動く途中の両方)。
    ///  警告    : 光源またはカメラが試料面の裏側(θ ≥ 90°)になる測定点がある。
    /// 旧システムでは、これらを実機で試して初めて分かっていた。
    PreflightReport preflight(const geometry::RigGeometry& geometry, const domain::Recipe& recipe);

} // namespace brdf::app
