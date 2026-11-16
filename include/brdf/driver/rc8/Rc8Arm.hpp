/*
* @file include/brdf/driver/rc8/Rc8Arm.hpp
* @brief IRobotArm for a DENSO RC8 controller (VP-6242), on top of the b-CAP wrapper IBcapApi.
*/

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "brdf/core/Cancel.hpp"
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

        /// 速度の上限(0 より大きく 1 以下の割合)。moveJ に、これより大きい速度が指定されても、この速度で動かす
        /// (警告を出す)。実機を初めて動かすときに、レシピの速度が速すぎても、速く動かさないための防御。
        /// ドライバの既定は上限なし。setup.toml の [controllers] max_speed が、実機モードでは 5% を既定にする。
        double max_speed = 1.0;

        /// true なら、移動を**非同期**で始めて、現在の関節角を見ながら待つ。待つ間に中止の要求(CancelToken)があれば、
        /// 止める(Halt)。false なら、動作が終わるまで戻る呼び出し 1 つで動かす(旧システムと同じで、実機で動いた実績がある。
        /// ただし、動作中に何もできない)。
        /// 非同期の移動(オプション "NEXT")は、実機でまだ確かめていないので、既定は false。確かめたら true にする。
        bool abortable_moves = false;
        int poll_interval_ms = 100;          ///< abortable_moves のとき、現在の関節角を読む間隔
        int settle_polls = 3;                ///< 連続でこの回数、関節角が止まっていたら、動作が終わったとみなす
        double settle_tolerance_deg = 0.05;  ///< 前回の読み取りとの差がこれ未満なら「止まっている」
        int start_grace_ms = 3000;           ///< 動き出すまでに待つ時間。これを過ぎても動かず、目標にも着いていなければエラー

        /// 待つ処理(テスト用。空なら、実際に待つ)。引数はミリ秒。
        std::function<void(int)> sleep_ms;
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
        /// cancel は、中止の要求を伝える旗(省略できる)。立っていると、次の移動を始めず(Cancelled)、
        /// abortable_moves のときは、動作中でも止める。
        Rc8Arm(std::shared_ptr<IBcapApi> api, Rc8ArmConfig config, std::shared_ptr<CancelToken> cancel = nullptr);
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

        /// moveAndCheck の、中止できる版(abortable_moves)。非同期で始めて、関節角を読みながら、止まるまで待つ。
        Status moveAbortable(const Degrees6& target);

        void pause(int ms) const;

        /// 通信エラー(Timeout / Communication)なら、接続を捨てて再接続を求めるメッセージを足す。それ以外はそのまま返す。
        Error handleError(Error error, const std::string& what);

        std::string label() const { return config_.name; }

        std::shared_ptr<IBcapApi> api_;
        Rc8ArmConfig config_;
        std::shared_ptr<CancelToken> cancel_;
        Link link_;
        bool connected_ = false;
        bool speed_capped_warned_ = false;
    };

    /// b-CAP の接続文字列 "tcp:<IP>:<ポート>"。
    std::string bcapAddress(const std::string& host, int port);

} // namespace brdf::driver::rc8
