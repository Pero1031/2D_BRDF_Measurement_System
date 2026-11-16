/*
* @file include/brdf/app/ArmCommands.hpp
* @brief `brdf arm`: read the real arms' positions and move one arm at a time (the first steps of docs/SafetyProcedure.md).
*/

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>

#include "brdf/app/DeviceFactory.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/driver/IRobotArm.hpp"
#include "brdf/geometry/Joints.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// 1 台を動かす目標。絶対角(absolute)か、関節 1 つの変化量(jog)のどちらか 1 つ。
    struct ArmTarget {
        std::optional<geometry::Joints> absolute;
        int jog_joint = -1;     ///< 0〜5(J1〜J6)
        Real jog_deg = 0.0;     ///< 現在の角度からの変化量 [度]
    };

    /// 旧システムの試験用の姿勢ファイルが、繰り返し使っている標準姿勢。右 (-45, 0, 90, 0, 0, 0)、左 (45, 0, 90, 0, 0, 0)。
    geometry::Joints standardPose(RealArm side);

    /// 関節角を "J1 = 45.00  J2 = 0.00 ..." の形(度)にする。
    std::string formatJoints(const geometry::Joints& q);

    /// 動かす前に利用者に見せる文: どのアーム、速度(上限を含む)、現在 → 目標、もう一方は動かさないこと。
    std::string describeArmMove(const io::SetupConfig& setup, RealArm side, const geometry::Joints& from, const geometry::Joints& to,
                                Real requested_speed);

    /// 接続済みの 2 台の現在位置を読み、side のアームだけを動かす。
    /// 流れ: 現在位置を読む → 目標を決める → 検査(可動域、干渉、途中の干渉。エラーなら何も動かさない)→ 確認(OK の入力)→ 移動。
    /// 速度は、setup.toml の [controllers] max_speed を超えない。
    Status moveOneArm(const io::SetupConfig& setup, driver::IRobotArm& left, driver::IRobotArm& right, RealArm side,
                      const ArmTarget& target, Real speed, std::istream& in);

    /// `brdf arm status`: 両方のアームに接続して、現在の関節角を表示して、切断する。動かさない。
    /// 実機モード(mode = "real")のプロジェクトだけ。
    Status armStatus(const std::filesystem::path& project_dir);

    /// `brdf arm move`: 両方のアームに接続して(もう一方の現在位置を、干渉の検査に使う)、side の 1 台だけを動かして、切断する。
    Status armMove(const std::filesystem::path& project_dir, RealArm side, const ArmTarget& target, Real speed, std::istream& in);

} // namespace brdf::app
