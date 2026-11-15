/*
* @file include/brdf/driver/IRobotArm.hpp
* @brief Abstract interface for a 6-axis robot arm.
*/

#pragma once

#include "brdf/core/Error.hpp"
#include "brdf/geometry/Joints.hpp"

namespace brdf::driver {

    /// 6軸ロボットアームのインターフェース。実機(RC8)とモックが同じインターフェースを実装する。
    ///
    /// 旧システムが使っていた操作だけに絞っている(関節空間の移動 J(...) と停止)。
    /// 角度は常にラジアン。度への変換は実機ドライバの内部で行う。
    class IRobotArm {
    public:
        virtual ~IRobotArm() = default;

        // --- 接続 ---
        virtual Status connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

        // --- 移動 ---
        /// 関節角で目標位置へ移動する。**動作が完了してから戻る(ブロッキング)**。
        /// speed は 0 より大きく 1 以下の割合(1 = 最大速度)。
        /// 可動域外の関節角や、干渉する姿勢はエラーになる。
        virtual Status moveJ(const geometry::Joints& target, Real speed) = 0;

        // --- 状態 ---
        virtual Result<geometry::Joints> currentJoints() const = 0;

        // --- 停止 ---
        /// 通常の停止。
        virtual void stop() = 0;
        /// 非常停止。以降は再接続するまで動かせない。
        virtual void emergencyStop() = 0;
    };

} // namespace brdf::driver
