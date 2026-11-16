/*
* @file include/brdf/app/RealRun.hpp
* @brief The guards that apply when the real arms move: confirmation, the start-position check, pre-flight options.
*
* `brdf run` and `brdf calibrate` both move the arms, so they share these (docs/SafetyProcedure.md).
*/

#pragma once

#include <iosfwd>
#include <string>

#include "brdf/app/Preflight.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/driver/IRobotArm.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// 事前検査の設定。実機モードのときは、旧システムの実績範囲の外の姿勢も警告にする。
    PreflightOptions preflightOptionsFor(const io::SetupConfig& setup);

    /// 実機のアームを動かす前に、利用者に見せる文: 装置、実際に使う速度(上限を含む)、動かす範囲、中止の方法。
    std::string describeRealMotion(const io::SetupConfig& setup, const domain::Recipe& recipe);

    /// 実機モードのときだけ、describeRealMotion の文を表示して、OK の入力を求める。OK 以外なら、何も動かさずに失敗する。
    /// シミュレータ(mode = "sim")のときは、何もせず成功する。--yes でも省略できない。
    Status confirmRealMotion(const io::SetupConfig& setup, const domain::Recipe& recipe, std::istream& in);

    /// 接続したアームの実際の位置を読み、そこから最初の姿勢までの道のりを検査する(checkStartPath)。
    /// エラーがあれば、何も動かさずに失敗する。警告はログに出す。
    Status checkStartPosition(const geometry::RigGeometry& geometry, const driver::IRobotArm& left, const driver::IRobotArm& right,
                              const domain::PoseStep& first, const PreflightOptions& options);

} // namespace brdf::app
