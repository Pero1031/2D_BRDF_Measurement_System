#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/DeviceFactory.hpp"
#include "brdf/driver/rc8/Rc8Arm.hpp"
#ifdef BRDF_WITH_BCAP
#include "brdf/driver/rc8/RealBcapApi.hpp"
#endif

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::driver::rc8;

namespace {

    /// b-CAP ライブラリの代わりをするフェイク。呼ばれた順を記録し、指定した呼び出しで失敗させ、
    /// ソケットやハンドルの開閉の数と、モータの状態を数える。ロボットは、移動の命令どおりの位置に動く。
    class FakeBcap final : public IBcapApi {
    public:
        // --- 振る舞いの設定 ---
        std::string fail_at;                          ///< この名前の呼び出しで失敗する("" なら失敗しない)
        int fail_nth = 1;                             ///< その名前の呼び出しの、何回目で失敗するか(1 から数える)
        ErrorCode fail_code = ErrorCode::DeviceError;
        bool sticky = false;                          ///< true なら、失敗したあとの呼び出し(close と setTimeout 以外)も、同じ原因で失敗する
        bool ignore_moves = false;                    ///< 移動の命令が無視される(動かないのに成功を返す)
        double arrival_error_deg = 0.0;               ///< 移動のあと、J1 がこれだけ目標からずれる
        Degrees6 start{0.0, 0.0, 90.0, 0.0, 45.0, 0.0};

        // --- 記録 ---
        std::vector<std::string> calls;
        std::map<std::string, int> counts;
        int opened = 0, closed = 0, services = 0, services_stopped = 0, controllers = 0, controllers_closed = 0;
        int robots = 0, robots_released = 0, arm_taken = 0, arm_given = 0;
        bool motor_on = false;
        std::vector<bool> motor_log;
        std::vector<double> speeds;
        std::vector<int> timeouts;
        std::vector<Degrees6> moves;
        std::string address, provider, machine, robot_name;
        int open_timeout = 0, open_retry = -1;

        int count(const std::string& name) const {
            const auto it = counts.find(name);
            return it == counts.end() ? 0 : it->second;
        }
        bool called(const std::string& name) const { return count(name) > 0; }

        /// 呼び出しの名前だけを、順に並べた文字列(順序の確認用)
        std::string sequence() const {
            std::string s;
            for (const auto& c : calls) s += (s.empty() ? "" : ",") + c;
            return s;
        }

        Result<int> open(const std::string& a, int timeout_ms, int retry) override {
            if (auto e = enter("open")) return tl::unexpected(*e);
            address = a;
            open_timeout = timeout_ms;
            open_retry = retry;
            ++opened;
            return 11;
        }
        Status serviceStart(int) override {
            if (auto e = enter("serviceStart")) return tl::unexpected(*e);
            ++services;
            return {};
        }
        Result<std::uint32_t> controllerConnect(int, const std::string& p, const std::string& m) override {
            if (auto e = enter("controllerConnect")) return tl::unexpected(*e);
            provider = p;
            machine = m;
            ++controllers;
            return 21u;
        }
        Result<BcapRobot> getRobot(int fd, std::uint32_t, const std::string& name) override {
            if (auto e = enter("getRobot")) return tl::unexpected(*e);
            robot_name = name;
            ++robots;
            return BcapRobot{fd, 31u};
        }
        Status releaseRobot(const BcapRobot&) override {
            if (auto e = enter("releaseRobot")) return tl::unexpected(*e);
            ++robots_released;
            return {};
        }
        Status controllerDisconnect(int, std::uint32_t) override {
            if (auto e = enter("controllerDisconnect")) return tl::unexpected(*e);
            ++controllers_closed;
            return {};
        }
        Status serviceStop(int) override {
            if (auto e = enter("serviceStop")) return tl::unexpected(*e);
            ++services_stopped;
            return {};
        }
        Status close(int) override {
            calls.push_back("close");
            ++counts["close"];
            ++closed;   // close は、通信が壊れていても失敗させない
            return {};
        }
        Status setTimeout(int, int timeout_ms) override {
            calls.push_back("setTimeout");
            ++counts["setTimeout"];
            timeouts.push_back(timeout_ms);
            return {};
        }

        Status takeArm(const BcapRobot&) override {
            if (auto e = enter("takeArm")) return tl::unexpected(*e);
            ++arm_taken;
            return {};
        }
        Status giveArm(const BcapRobot&) override {
            if (auto e = enter("giveArm")) return tl::unexpected(*e);
            ++arm_given;
            return {};
        }
        Status setMotor(const BcapRobot&, bool on) override {
            if (auto e = enter("setMotor")) return tl::unexpected(*e);
            motor_on = on;
            motor_log.push_back(on);
            return {};
        }
        Status setExtSpeed(const BcapRobot&, double percent) override {
            if (auto e = enter("setExtSpeed")) return tl::unexpected(*e);
            speeds.push_back(percent);
            return {};
        }
        Status moveJoint(const BcapRobot&, const Degrees6& target) override {
            if (auto e = enter("moveJoint")) return tl::unexpected(*e);
            moves.push_back(target);
            if (!ignore_moves) {
                start = target;
                start[0] += arrival_error_deg;
            }
            return {};
        }
        Result<Degrees6> currentJoint(const BcapRobot&) override {
            if (auto e = enter("currentJoint")) return tl::unexpected(*e);
            return start;
        }
        Status halt(const BcapRobot&) override {
            if (auto e = enter("halt")) return tl::unexpected(*e);
            return {};
        }

        /// 開いたものが、すべて閉じている(通信が壊れた場合の、ソケットを閉じるだけの後始末は除く)
        void checkAllClosed() const {
            CHECK(opened == closed);
            CHECK(services == services_stopped);
            CHECK(controllers == controllers_closed);
            CHECK(robots == robots_released);
            CHECK(arm_taken == arm_given);
            CHECK_FALSE(motor_on);
        }

    private:
        bool dead_ = false;

        std::optional<Error> enter(const std::string& name) {
            calls.push_back(name);
            const int n = ++counts[name];
            if (dead_) return Error{fail_code, "no answer (link is down) in " + name};
            if (name == fail_at && n == fail_nth) {
                dead_ = sticky;
                return Error{fail_code, name + " failed (injected)"};
            }
            return std::nullopt;
        }
    };

    struct Fixture {
        std::shared_ptr<FakeBcap> api = std::make_shared<FakeBcap>();
        Rc8ArmConfig config;
        std::unique_ptr<Rc8Arm> arm;

        Fixture() { config.name = "SampleArm"; }

        Rc8Arm& make() {
            arm = std::make_unique<Rc8Arm>(api, config);
            return *arm;
        }
        Rc8Arm& connected() {
            Rc8Arm& a = make();
            REQUIRE(a.connect().has_value());
            api->calls.clear();
            api->counts.clear();
            return a;
        }
    };

    geometry::Joints pose(double j1 = 65, double j2 = 10, double j3 = 90, double j4 = 0, double j5 = 45, double j6 = 0) {
        return geometry::jointsFromDegrees(j1, j2, j3, j4, j5, j6);
    }

} // namespace

// ---------------------------------------------------------------------------
// 命令の文字列
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: the move command has the legacy format", "[rc8]") {
    CHECK(jointMoveCommand({65, 0, 90, 0, 45, 0}) == "J(65.0000,0.0000,90.0000,0.0000,45.0000,0.0000,-1)");
    // 旧システムは整数に丸めていた。新しいものは小数も送る
    CHECK(jointMoveCommand({12.3456, -7.5, 90.00004, 1e-9, -0.00001, 359.99994}) ==
          "J(12.3456,-7.5000,90.0000,0.0000,0.0000,359.9999,-1)");
}

TEST_CASE("Rc8: the b-CAP address carries the port", "[rc8]") {
    CHECK(bcapAddress("192.168.0.5", 5007) == "tcp:192.168.0.5:5007");
}

// ---------------------------------------------------------------------------
// 接続
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: connect opens the four levels in order and takes the arm once", "[rc8]") {
    Fixture f;
    f.config.host = "192.168.0.2";
    Rc8Arm& arm = f.make();

    REQUIRE(arm.connect().has_value());
    CHECK(arm.isConnected());
    CHECK(f.api->sequence() == "open,serviceStart,controllerConnect,getRobot,takeArm");
    CHECK(f.api->address == "tcp:192.168.0.2:5007");
    CHECK(f.api->provider == "CaoProv.DENSO.VRC");
    CHECK(f.api->machine == "localhost");
    CHECK(f.api->robot_name == "Arm");
    CHECK(f.api->open_timeout == 5000);
    CHECK(f.api->open_retry == 0);   // 再送すると、移動の命令が 2 回届くおそれがある

    // 2 回目の connect は、何もしない(接続は 1 回だけ)
    f.api->calls.clear();
    CHECK(arm.connect().has_value());
    CHECK(f.api->calls.empty());
}

TEST_CASE("Rc8: disconnect releases in reverse order and gives the arm back", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();

    arm.disconnect();
    CHECK_FALSE(arm.isConnected());
    // 念のためモータを OFF にして、制御権を返し、開いたものを逆順に閉じる
    CHECK(f.api->sequence() == "setMotor,giveArm,releaseRobot,controllerDisconnect,serviceStop,close");
    f.api->checkAllClosed();

    // 2 回目は何もしない
    f.api->calls.clear();
    arm.disconnect();
    CHECK(f.api->calls.empty());
}

TEST_CASE("Rc8: destroying a connected arm releases everything", "[rc8]") {
    Fixture f;
    f.connected();
    f.arm.reset();
    f.api->checkAllClosed();
}

TEST_CASE("Rc8: a failure at any level rolls back what was opened", "[rc8]") {
    const std::string step = GENERATE("open", "serviceStart", "controllerConnect", "getRobot", "takeArm");
    CAPTURE(step);

    Fixture f;
    f.api->fail_at = step;
    f.api->fail_code = ErrorCode::DeviceError;
    Rc8Arm& arm = f.make();

    const Status s = arm.connect();
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::DeviceError);
    CHECK_THAT(s.error().message, ContainsSubstring("SampleArm"));
    CHECK_THAT(s.error().message, ContainsSubstring(step + " failed (injected)"));
    CHECK_FALSE(arm.isConnected());
    f.api->checkAllClosed();   // どこで失敗しても、開いたものは閉じている

    // 失敗のあとで、同じインスタンスで接続し直せる
    f.api->fail_at.clear();
    CHECK(arm.connect().has_value());
    CHECK(arm.isConnected());
}

TEST_CASE("Rc8: taking the arm fails with a hint about the teach pendant", "[rc8]") {
    Fixture f;
    f.api->fail_at = "takeArm";
    const Status s = f.make().connect();
    REQUIRE_FALSE(s.has_value());
    CHECK_THAT(s.error().message, ContainsSubstring("teach pendant"));
}

TEST_CASE("Rc8: when the link is down during connect, only the socket is closed", "[rc8]") {
    const ErrorCode code = GENERATE(ErrorCode::Timeout, ErrorCode::Communication);
    Fixture f;
    f.api->fail_at = "getRobot";
    f.api->fail_code = code;
    f.api->sticky = true;   // 以降の命令は、届かない

    const Status s = f.make().connect();
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == code);   // 種類はそのまま返る
    // 届かない命令を何度も待たずに、ソケットを閉じるだけ
    CHECK(f.api->sequence() == "open,serviceStart,controllerConnect,getRobot,close");
    CHECK(f.api->opened == f.api->closed);
}

TEST_CASE("Rc8: connect rejects a bad configuration without touching the network", "[rc8]") {
    Fixture f;
    SECTION("empty address") { f.config.host.clear(); }
    SECTION("port zero") { f.config.port = 0; }
    SECTION("port too large") { f.config.port = 65536; }
    SECTION("zero timeout") { f.config.command_timeout_ms = 0; }
    SECTION("zero move timeout") { f.config.move_timeout_ms = 0; }
    SECTION("negative retry") { f.config.retry = -1; }

    const Status s = f.make().connect();
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::InvalidArgument);
    CHECK(f.api->calls.empty());
}

// ---------------------------------------------------------------------------
// 移動
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: moveJ sets the speed, turns the motor on only for the move, and checks the arrival", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();

    REQUIRE(arm.moveJ(pose(65, 10, 90, 0, 45, 0), 0.25).has_value());

    CHECK(f.api->sequence() == "setExtSpeed,setMotor,setTimeout,moveJoint,setTimeout,currentJoint,setMotor");
    REQUIRE(f.api->speeds.size() == 1);
    CHECK_THAT(f.api->speeds[0], WithinAbs(25.0, 1e-9));   // 割合 → パーセント
    REQUIRE(f.api->moves.size() == 1);
    const Degrees6 want{65, 10, 90, 0, 45, 0};
    for (std::size_t i = 0; i < 6; ++i) CHECK_THAT(f.api->moves[0][i], WithinAbs(want[i], 1e-9));   // ラジアン → 度
    CHECK(f.api->motor_log == std::vector<bool>{true, false});   // 移動の間だけ ON
    // 移動を待つ間だけ、待ち時間が長い
    CHECK(f.api->timeouts == std::vector<int>{180000, 5000});
    CHECK(arm.isConnected());   // 移動のたびに接続し直したりしない
}

TEST_CASE("Rc8: moveJ keeps fractional degrees (the legacy system rounded them)", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    REQUIRE(arm.moveJ(pose(65.4321, 10.5, 90.25, 0, 45, 0), 1.0).has_value());
    CHECK_THAT(f.api->moves[0][0], WithinAbs(65.4321, 1e-9));
    CHECK_THAT(f.api->moves[0][1], WithinAbs(10.5, 1e-9));
    CHECK_THAT(f.api->moves[0][2], WithinAbs(90.25, 1e-9));
}

TEST_CASE("Rc8: the speed fraction maps to the controller's 0.1-100 percent", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    REQUIRE(arm.moveJ(pose(), 1.0).has_value());
    REQUIRE(arm.moveJ(pose(), 0.001).has_value());
    CHECK_THAT(f.api->speeds[0], WithinAbs(100.0, 1e-9));
    CHECK_THAT(f.api->speeds[1], WithinAbs(0.1, 1e-9));
}

TEST_CASE("Rc8: currentJoints returns radians", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    REQUIRE(arm.moveJ(pose(65, 10, 90, 0, 45, 0), 0.5).has_value());

    const auto q = arm.currentJoints();
    REQUIRE(q.has_value());
    const auto want = pose(65, 10, 90, 0, 45, 0);
    for (int i = 0; i < 6; ++i) CHECK_THAT((*q)(i), WithinAbs(want(i), 1e-9));
}

TEST_CASE("Rc8: commands that are rejected before anything is sent", "[rc8]") {
    Fixture f;

    SECTION("not connected") {
        Rc8Arm& arm = f.make();
        const Status s = arm.moveJ(pose(), 0.5);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::NotConnected);
        CHECK(arm.currentJoints().error().code == ErrorCode::NotConnected);
        CHECK(f.api->calls.empty());
    }
    SECTION("bad speed") {
        Rc8Arm& arm = f.connected();
        const double speed = GENERATE(0.0, -0.5, 1.5, std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::infinity());
        const Status s = arm.moveJ(pose(), speed);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
        CHECK(f.api->calls.empty());
    }
    SECTION("target is not a number") {
        Rc8Arm& arm = f.connected();
        auto q = pose();
        q(2) = std::numeric_limits<double>::quiet_NaN();
        const Status s = arm.moveJ(q, 0.5);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
        CHECK(f.api->calls.empty());
    }
    SECTION("outside the joint range") {
        Rc8Arm& arm = f.connected();
        const Status s = arm.moveJ(pose(65, 10, 10 /* J3 の下限は 19° */, 0, 45, 0), 0.5);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::JointLimit);
        CHECK_THAT(s.error().message, ContainsSubstring("J3"));
        CHECK_THAT(s.error().message, ContainsSubstring("10.00"));
        CHECK(f.api->calls.empty());   // モータも ON にしていない
        CHECK(arm.isConnected());
    }
}

TEST_CASE("Rc8: the joint limits come from the configuration", "[rc8]") {
    Fixture f;
    f.config.limits.upper(0) = toRadians(30.0);   // J1 の上限を 30° にする
    Rc8Arm& arm = f.connected();
    CHECK(arm.moveJ(pose(29, 10, 90, 0, 45, 0), 0.5).has_value());
    const Status s = arm.moveJ(pose(31, 10, 90, 0, 45, 0), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::JointLimit);
}

TEST_CASE("Rc8: a rejected move leaves the motor off and the connection usable", "[rc8]") {
    // 何番目の命令で失敗しても、モータは OFF に戻り、待ち時間は元に戻る
    const std::string step = GENERATE("setExtSpeed", "setMotor", "moveJoint", "currentJoint");
    CAPTURE(step);

    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = step;
    f.api->fail_code = ErrorCode::DeviceError;   // コントローラが断った(通信は生きている)

    const Status s = arm.moveJ(pose(), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::DeviceError);
    CHECK_THAT(s.error().message, ContainsSubstring("SampleArm"));
    CHECK_THAT(s.error().message, ContainsSubstring(step + " failed (injected)"));
    CHECK(arm.isConnected());
    CHECK_FALSE(f.api->motor_on);
    if (f.api->called("moveJoint")) CHECK(f.api->timeouts.back() == 5000);

    // そのまま、次の移動ができる
    f.api->fail_at.clear();
    CHECK(arm.moveJ(pose(), 0.5).has_value());
}

TEST_CASE("Rc8: failing to turn the motor off after a move is reported", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "setMotor";
    f.api->fail_nth = 2;   // 1 回目 = ON、2 回目 = OFF

    const Status s = arm.moveJ(pose(), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK_THAT(s.error().message, ContainsSubstring("Motor off"));
    CHECK(arm.isConnected());
}

TEST_CASE("Rc8: a move that did not arrive is an error", "[rc8]") {
    Fixture f;

    SECTION("the command was ignored") {
        Rc8Arm& arm = f.connected();
        f.api->ignore_moves = true;
        const Status s = arm.moveJ(pose(65, 10, 90, 0, 45, 0), 0.5);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::DeviceError);
        CHECK_THAT(s.error().message, ContainsSubstring("J1"));
        CHECK_THAT(s.error().message, ContainsSubstring("target"));
        CHECK_FALSE(f.api->motor_on);
        CHECK(arm.isConnected());
    }
    SECTION("a small error is within the tolerance") {
        Rc8Arm& arm = f.connected();
        f.api->arrival_error_deg = 0.5;
        CHECK(arm.moveJ(pose(), 0.5).has_value());
    }
    SECTION("a larger error is not") {
        Rc8Arm& arm = f.connected();
        f.api->arrival_error_deg = 1.5;
        CHECK_FALSE(arm.moveJ(pose(), 0.5).has_value());
    }
    SECTION("the tolerance is configurable") {
        f.config.arrival_tolerance_deg = 2.0;
        Rc8Arm& arm = f.connected();
        f.api->arrival_error_deg = 1.5;
        CHECK(arm.moveJ(pose(), 0.5).has_value());
    }
}

// ---------------------------------------------------------------------------
// 通信の途切れ・時間切れ
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: a timeout during a move halts the arm, turns the motor off and drops the connection", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "moveJoint";
    f.api->fail_code = ErrorCode::Timeout;   // 動作中かもしれない。通信そのものは生きている

    const Status s = arm.moveJ(pose(), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::Timeout);
    CHECK_THAT(s.error().message, ContainsSubstring("connect again"));

    // 待ち時間を戻したあと、通信が生きているうちに、止める → モータ OFF。その後は、ソケットを閉じるだけ
    CHECK(f.api->sequence() == "setExtSpeed,setMotor,setTimeout,moveJoint,setTimeout,halt,setMotor,close");
    CHECK_FALSE(arm.isConnected());
    CHECK_FALSE(f.api->motor_on);
    CHECK(f.api->opened == f.api->closed);

    // 状態が分からないので、動かせない。接続し直せば、また使える
    CHECK(arm.moveJ(pose(), 0.5).error().code == ErrorCode::NotConnected);
    f.api->fail_at.clear();
    REQUIRE(arm.connect().has_value());
    CHECK(arm.moveJ(pose(), 0.5).has_value());
}

TEST_CASE("Rc8: when the link is completely down, the cleanup does not hang on dead calls", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "moveJoint";
    f.api->fail_code = ErrorCode::Communication;
    f.api->sticky = true;

    const Status s = arm.moveJ(pose(), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::Communication);
    CHECK_FALSE(arm.isConnected());
    // 止める・モータ OFF を試みるが(届かなくても)、その後の後始末はソケットを閉じるだけ
    CHECK(f.api->sequence() == "setExtSpeed,setMotor,setTimeout,moveJoint,setTimeout,halt,setMotor,close");
    CHECK(f.api->opened == f.api->closed);
}

TEST_CASE("Rc8: a communication error in any command drops the connection", "[rc8]") {
    const std::string step = GENERATE("setExtSpeed", "setMotor", "currentJoint");
    const ErrorCode code = GENERATE(ErrorCode::Timeout, ErrorCode::Communication);
    CAPTURE(step);

    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = step;
    f.api->fail_code = code;
    f.api->sticky = true;

    const Status s = arm.moveJ(pose(), 0.5);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == code);
    CHECK_FALSE(arm.isConnected());
    CHECK(f.api->opened == f.api->closed);
    CHECK(f.api->count("giveArm") == 0);   // 届かない命令は送らない
}

TEST_CASE("Rc8: currentJoints reports a read failure with its code", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "currentJoint";
    f.api->fail_code = ErrorCode::Timeout;
    const auto q = arm.currentJoints();
    REQUIRE_FALSE(q.has_value());
    CHECK(q.error().code == ErrorCode::Timeout);
    CHECK_THAT(q.error().message, ContainsSubstring("SampleArm"));
}

// ---------------------------------------------------------------------------
// 停止
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: stop halts the arm and stays connected", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    arm.stop();
    CHECK(f.api->sequence() == "halt");
    CHECK(arm.isConnected());
}

TEST_CASE("Rc8: stop when not connected does nothing", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.make();
    arm.stop();
    arm.emergencyStop();
    CHECK(f.api->calls.empty());
}

TEST_CASE("Rc8: stop that cannot reach the controller drops the connection", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "halt";
    f.api->fail_code = ErrorCode::Timeout;
    arm.stop();
    CHECK_FALSE(arm.isConnected());
    CHECK(f.api->opened == f.api->closed);
}

TEST_CASE("Rc8: emergency stop halts, turns the motor off, releases and refuses further moves", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();

    arm.emergencyStop();
    CHECK(f.api->sequence() == "halt,setMotor,giveArm,releaseRobot,controllerDisconnect,serviceStop,close");
    CHECK_FALSE(arm.isConnected());
    f.api->checkAllClosed();

    // 以降は、再接続するまで動かせない
    CHECK(arm.moveJ(pose(), 0.5).error().code == ErrorCode::NotConnected);
    CHECK(arm.connect().has_value());
    CHECK(arm.moveJ(pose(), 0.5).has_value());
}

TEST_CASE("Rc8: emergency stop still closes the socket when the halt does not get through", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "halt";
    f.api->fail_code = ErrorCode::Communication;
    f.api->sticky = true;

    arm.emergencyStop();
    CHECK_FALSE(arm.isConnected());
    CHECK(f.api->sequence() == "halt,close");
    CHECK(f.api->opened == f.api->closed);
}

// ---------------------------------------------------------------------------
// 切断の途中の失敗
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: disconnect carries on after a failing step", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "giveArm";
    f.api->fail_code = ErrorCode::DeviceError;   // 制御権を返せなかったが、通信は生きている

    arm.disconnect();
    CHECK_FALSE(arm.isConnected());
    // 残りも、順に閉じる
    CHECK(f.api->sequence() == "setMotor,giveArm,releaseRobot,controllerDisconnect,serviceStop,close");
    CHECK(f.api->robots == f.api->robots_released);
    CHECK(f.api->opened == f.api->closed);
}

TEST_CASE("Rc8: disconnect with a dead link only closes the socket", "[rc8]") {
    Fixture f;
    Rc8Arm& arm = f.connected();
    f.api->fail_at = "setMotor";
    f.api->fail_code = ErrorCode::Timeout;
    f.api->sticky = true;

    arm.disconnect();
    CHECK_FALSE(arm.isConnected());
    CHECK(f.api->sequence() == "setMotor,close");
    CHECK(f.api->opened == f.api->closed);
}

// ---------------------------------------------------------------------------
// 2 台のアーム
// ---------------------------------------------------------------------------

TEST_CASE("Rc8: two arms use separate connections", "[rc8]") {
    auto api_l = std::make_shared<FakeBcap>();
    auto api_r = std::make_shared<FakeBcap>();
    Rc8ArmConfig left, right;
    left.name = "LightArm";
    left.host = "192.168.0.2";
    right.name = "SampleArm";
    right.host = "192.168.0.5";
    Rc8Arm l(api_l, left), r(api_r, right);

    REQUIRE(l.connect().has_value());
    REQUIRE(r.connect().has_value());
    CHECK(api_l->address == "tcp:192.168.0.2:5007");
    CHECK(api_r->address == "tcp:192.168.0.5:5007");

    // 片方の失敗は、もう一方の接続に影響しない
    api_r->fail_at = "moveJoint";
    api_r->fail_code = ErrorCode::Timeout;
    CHECK_FALSE(r.moveJ(pose(), 0.5).has_value());
    CHECK_FALSE(r.isConnected());
    CHECK(l.isConnected());
    CHECK(l.moveJ(pose(), 0.5).has_value());
}

// ---------------------------------------------------------------------------
// 設定と工場関数
// ---------------------------------------------------------------------------

TEST_CASE("makeRc8Config takes the controller settings from setup.toml", "[rc8][factory]") {
    io::SetupConfig setup;
    setup.controllers.left_host = "10.0.0.2";
    setup.controllers.right_host = "10.0.0.5";
    setup.controllers.port = 6000;
    setup.controllers.robot_name = "Robot1";
    setup.controllers.command_timeout_ms = 1234;
    setup.controllers.move_timeout_ms = 56789;
    setup.controllers.arrival_tolerance_deg = 0.25;
    setup.rig.joint_limits.upper(0) = toRadians(100.0);

    const auto left = app::makeRc8Config(setup, app::RealArm::Left);
    CHECK(left.name == "LightArm");
    CHECK(left.host == "10.0.0.2");
    const auto right = app::makeRc8Config(setup, app::RealArm::Right);
    CHECK(right.name == "SampleArm");
    CHECK(right.host == "10.0.0.5");

    for (const auto* c : {&left, &right}) {
        CHECK(c->port == 6000);
        CHECK(c->robot_name == "Robot1");
        CHECK(c->command_timeout_ms == 1234);
        CHECK(c->move_timeout_ms == 56789);
        CHECK_THAT(c->arrival_tolerance_deg, WithinAbs(0.25, 1e-12));
        CHECK_THAT(c->limits.upper(0), WithinAbs(toRadians(100.0), 1e-12));   // [arm.limits] が、送る前の可動域チェックに使われる
        CHECK(c->retry == 0);
    }
}

TEST_CASE("the default controller addresses are the ones the legacy system used", "[rc8][factory]") {
    const io::SetupConfig setup;
    CHECK(app::makeRc8Config(setup, app::RealArm::Left).host == "192.168.0.2");
    CHECK(app::makeRc8Config(setup, app::RealArm::Right).host == "192.168.0.5");
    CHECK(app::makeRc8Config(setup, app::RealArm::Left).port == 5007);
    CHECK(app::makeRc8Config(setup, app::RealArm::Left).robot_name == "Arm");
}

#ifndef BRDF_WITH_BCAP
TEST_CASE("without the b-CAP SDK, makeRc8Arm says how to enable it", "[rc8][factory]") {
    const auto r = app::makeRc8Arm(io::SetupConfig{}, app::RealArm::Left);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::NotImplemented);
    CHECK_THAT(r.error().message, ContainsSubstring("BRDF_WITH_BCAP"));
}
#else
TEST_CASE("with the b-CAP SDK, makeRc8Arm returns an arm that is not connected yet", "[rc8][factory]") {
    const auto r = app::makeRc8Arm(io::SetupConfig{}, app::RealArm::Left);
    REQUIRE(r.has_value());
    CHECK_FALSE((*r)->isConnected());
}

// ---------------------------------------------------------------------------
// 実際の b-CAP ライブラリ(装置につながなくても確かめられること)
// ---------------------------------------------------------------------------

TEST_CASE("RealBcapApi: connecting to a port nobody listens on fails cleanly", "[rc8][bcap]") {
    RealBcapApi api;
    const auto fd = api.open(bcapAddress("127.0.0.1", 59999), 500, 0);
    REQUIRE_FALSE(fd.has_value());
    CHECK((fd.error().code == ErrorCode::Communication || fd.error().code == ErrorCode::Timeout));
    CHECK_THAT(fd.error().message, ContainsSubstring("bCap_Open_Client"));
}

TEST_CASE("RealBcapApi: names that are not ASCII are refused before anything is sent", "[rc8][bcap]") {
    RealBcapApi api;
    const auto c = api.controllerConnect(-1, "\xE6\x97\xA5\xE6\x9C\xAC", "localhost");
    REQUIRE_FALSE(c.has_value());
    CHECK(c.error().code == ErrorCode::InvalidArgument);
    const auto r = api.getRobot(-1, 0, "\xE6\x97\xA5\xE6\x9C\xAC");
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("Rc8Arm with the real library: connecting to nothing leaves nothing open", "[rc8][bcap]") {
    Rc8ArmConfig config;
    config.name = "TestArm";
    config.host = "127.0.0.1";
    config.port = 59999;
    config.command_timeout_ms = 500;
    Rc8Arm arm(std::make_shared<RealBcapApi>(), config);

    const Status s = arm.connect();
    REQUIRE_FALSE(s.has_value());
    CHECK((s.error().code == ErrorCode::Communication || s.error().code == ErrorCode::Timeout));
    CHECK_THAT(s.error().message, ContainsSubstring("TestArm"));
    CHECK_THAT(s.error().message, ContainsSubstring("tcp:127.0.0.1:59999"));
    CHECK_FALSE(arm.isConnected());

    // つながっていない状態で、何を呼んでも安全
    CHECK(arm.moveJ(pose(), 0.5).error().code == ErrorCode::NotConnected);
    arm.stop();
    arm.emergencyStop();
    arm.disconnect();
}
#endif
