/*
* @file include/brdf/driver/rc8/BcapApi.hpp
* @brief The calls Rc8Arm needs from the DENSO b-CAP client library, as a small abstract interface.
*
* This header does not include bcap_client.h. RealBcapApi (built only when BRDF_WITH_BCAP is ON) implements it with
* the library; the tests implement it with a fake. Every call returns a Result whose error already says which b-CAP
* call failed. Angles are in degrees here, because that is what the controller uses.
*/

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "brdf/core/Error.hpp"

namespace brdf::driver::rc8 {

    /// 6 軸の関節角 [度]。RC8 は J1..J6 のあとに拡張軸 2 つを返すが、VP-6242 は 6 軸なので使わない。
    using Degrees6 = std::array<double, 6>;

    /// コントローラの中のロボット(コントローラ名 "Arm")を指すハンドル。ロボットへの命令は、これを渡す。
    struct BcapRobot {
        int fd = -1;                    ///< ソケット(bCap_Open_Client が返す)
        std::uint32_t handle = 0;       ///< ロボットのハンドル(bCap_ControllerGetRobot が返す)
    };

    /// 関節移動の命令の文字列 "J(j1,j2,j3,j4,j5,j6,-1)"(度)。旧システムと同じ書式で、小数も送る
    /// (旧システムは整数に丸めていた)。数値の書式は、実行環境のロケールによらず、小数点は常に '.'。
    /// RealBcapApi が bCap_RobotMove に渡す。
    std::string jointMoveCommand(const Degrees6& degrees);

    /// 1 台の RC8 コントローラとの b-CAP 通信。
    ///
    /// 接続は「ソケットを開く → サービス開始 → コントローラに接続 → ロボットを取得」の 4 段階で、
    /// 閉じるときは逆順。Rc8Arm が、途中で失敗したときの巻き戻しも含めて順番を守る。
    class IBcapApi {
    public:
        virtual ~IBcapApi() = default;

        // --- 接続(上から順に開く。閉じるのは逆順) ---

        /// bCap_Open_Client。address は "tcp:<IP>:<ポート>"。timeout_ms は 1 回の命令を待つ時間、retry は再送回数。
        virtual Result<int> open(const std::string& address, int timeout_ms, int retry) = 0;
        /// bCap_ServiceStart。
        virtual Status serviceStart(int fd) = 0;
        /// bCap_ControllerConnect。
        virtual Result<std::uint32_t> controllerConnect(int fd, const std::string& provider, const std::string& machine) = 0;
        /// bCap_ControllerGetRobot。
        virtual Result<BcapRobot> getRobot(int fd, std::uint32_t controller, const std::string& name) = 0;

        virtual Status releaseRobot(const BcapRobot& robot) = 0;
        virtual Status controllerDisconnect(int fd, std::uint32_t controller) = 0;
        virtual Status serviceStop(int fd) = 0;
        virtual Status close(int fd) = 0;

        /// bCap_SetTimeout。以降の命令を待つ時間を変える(動作が終わるまで戻らない移動のときだけ、長くする)。
        virtual Status setTimeout(int fd, int timeout_ms) = 0;

        // --- ロボットの操作 ---

        /// "Takearm"。ロボットの制御権を取る(取っている間は、ティーチングペンダントなどからは動かせない)。
        virtual Status takeArm(const BcapRobot& robot) = 0;
        /// "Givearm"。制御権を返す。
        virtual Status giveArm(const BcapRobot& robot) = 0;
        /// "Motor"。モータの ON / OFF。OFF にするとブレーキがかかる。
        virtual Status setMotor(const BcapRobot& robot, bool on) = 0;
        /// "ExtSpeed"。外部速度 [%]。0.1〜100(RC8 のプログラマーズマニュアルによる)。
        virtual Status setExtSpeed(const BcapRobot& robot, double percent) = 0;
        /// bCap_RobotMove(関節補間)。**動作が終わるまで戻らない**。
        virtual Status moveJoint(const BcapRobot& robot, const Degrees6& target) = 0;
        /// "CurJnt"。エンコーダで検出した現在の関節角 [度]。
        virtual Result<Degrees6> currentJoint(const BcapRobot& robot) = 0;
        /// bCap_RobotHalt。動作を止める。
        virtual Status halt(const BcapRobot& robot) = 0;
    };

} // namespace brdf::driver::rc8
