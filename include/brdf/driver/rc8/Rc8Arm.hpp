/*
* @file include/brdf/driver/rc8/Rc8Arm.hpp
* @brief IRobotArm for a DENSO RC8 controller (VP-6242), on top of the b-CAP wrapper IBcapApi.
*/

#pragma once

#include <memory>
#include <string>

#include "brdf/driver/IRobotArm.hpp"
#include "brdf/driver/rc8/BcapApi.hpp"
#include "brdf/geometry/JointLimits.hpp"

namespace brdf::driver::rc8 {

    struct Rc8ArmConfig {
        std::string name = "Arm";                ///< ログと、エラーメッセージの識別用("LightArm" / "SampleArm" など)
        std::string host = "192.168.0.5";        ///< コントローラの IP アドレス
        int port = 5007;                         ///< b-CAP のポート(RC8 の既定値)
        std::string provider = "CaoProv.DENSO.VRC";   ///< RC8 に接続するときのプロバイダ名(固定)
        std::string machine = "localhost";            ///< 同上(固定)
        std::string robot_name = "Arm";               ///< コントローラの中のロボット名(旧システムと同じ)

        /// 1 回の命令を待つ時間 [ms]。モータの ON/OFF、速度の設定、角度の取得など、すぐ終わる命令用。
        int command_timeout_ms = 5000;
        /// 移動(動作が終わるまで戻らない)を待つ時間 [ms]。遅い速度で遠くへ動かすと長くかかるので、長めにしてある。
        int move_timeout_ms = 180000;
        /// 通信の再送回数。**0 のまま**にする: 再送すると、移動の命令が 2 回届くおそれがある(旧システムも 0)。
        int retry = 0;

        /// 関節の可動域。ここを外れる指令は、コントローラに送る前に JointLimit で断る(コントローラ側のリミットとは別の防御)。
        geometry::JointLimits limits;

        /// 移動が終わったあと、実際の関節角と目標の差がこれ [度] を超えたらエラーにする(命令が無視された、などを見つける)。
        double arrival_tolerance_deg = 1.0;
    };

    /// 実機の DENSO RC8 + VP-6242。b-CAP の呼び出し順は docs/Rc8ArmDesign.md の §2。
    ///
    /// - **接続は 1 回だけ**: connect() で制御権(Takearm)まで取り、disconnect() で返す。
    ///   旧システムは、1 回動かすたびに接続・切断していた。
    /// - モータは、移動の間だけ ON にする(旧システムと同じ。移動の前後はブレーキがかかっている)。
    /// - 通信が途切れた・時間切れになったときは、状態が分からないので、接続を切って再接続を求める。
    ///
    /// スレッド: 1 本のソケットを使うので、1 つのインスタンスは 1 つのスレッドから使う。
    /// moveJ の途中で別のスレッドから stop() を呼ぶことはできない(非常時は、ティーチングペンダントの非常停止を使う)。
    class Rc8Arm final : public IRobotArm {
    public:
        Rc8Arm(std::shared_ptr<IBcapApi> api, Rc8ArmConfig config);
        ~Rc8Arm() override;

        Rc8Arm(const Rc8Arm&) = delete;
        Rc8Arm& operator=(const Rc8Arm&) = delete;

        Status connect() override;
        void disconnect() override;
        bool isConnected() const override { return connected_; }

        Status moveJ(const geometry::Joints& target, Real speed) override;
        Result<geometry::Joints> currentJoints() const override;

        void stop() override;
        void emergencyStop() override;

    private:
        /// b-CAP の 4 段階の接続のうち、どこまで開いたか。release() が、開いたものだけを逆順に閉じる。
        struct Link {
            int fd = -1;
            std::uint32_t controller = 0;
            BcapRobot robot;
            bool socket_open = false;
            bool service_started = false;
            bool controller_open = false;
            bool robot_open = false;
            bool arm_taken = false;
        };

        /// link_ に開いてあるものを、すべて閉じる。graceful が false のときは、ソケットを閉じるだけ
        /// (通信が途切れたあとに、届かない命令を何度も待たないため)。
        void release(bool graceful);

        /// モータが ON の状態で、移動して、目標に着いたかを確かめる。モータの ON/OFF は呼び出し側(moveJ)の仕事。
        Status moveAndCheck(const Degrees6& target);

        /// 通信エラー(Timeout / Communication)なら、接続を捨てて再接続を求めるメッセージを足す。それ以外はそのまま返す。
        Error handleError(Error error, const std::string& what);

        std::string label() const { return config_.name; }

        std::shared_ptr<IBcapApi> api_;
        Rc8ArmConfig config_;
        Link link_;
        bool connected_ = false;
    };

    /// b-CAP の接続文字列 "tcp:<IP>:<ポート>"。
    std::string bcapAddress(const std::string& host, int port);

} // namespace brdf::driver::rc8
