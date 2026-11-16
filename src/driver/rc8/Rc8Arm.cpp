/*
* @file src/driver/rc8/Rc8Arm.cpp
* @brief IRobotArm for a DENSO RC8 controller (VP-6242).
*/

#include "brdf/driver/rc8/Rc8Arm.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "brdf/core/MathUtils.hpp"

namespace brdf::driver::rc8 {

    namespace {

        /// 通信そのものが壊れた(あるいは時間切れで状態が分からない)エラーか。
        bool isLinkError(ErrorCode code) { return code == ErrorCode::Timeout || code == ErrorCode::Communication; }

        Degrees6 toDegreeArray(const geometry::Joints& q) {
            Degrees6 d{};
            for (int i = 0; i < 6; ++i) d[static_cast<std::size_t>(i)] = toDegrees(static_cast<double>(q(i)));
            return d;
        }

        geometry::Joints fromDegreeArray(const Degrees6& d) {
            return geometry::jointsFromDegrees(d[0], d[1], d[2], d[3], d[4], d[5]);
        }

        double maxAbsDiff(const Degrees6& a, const Degrees6& b) {
            double d = 0.0;
            for (std::size_t i = 0; i < 6; ++i) d = std::max(d, std::abs(a[i] - b[i]));
            return d;
        }

        std::string fixed(double v, int digits) {
            std::ostringstream o;
            o.imbue(std::locale::classic());
            o.setf(std::ios::fixed);
            o.precision(digits);
            o << v;
            return o.str();
        }

    } // namespace

    std::string bcapAddress(const std::string& host, int port) {
        return "tcp:" + host + ":" + std::to_string(port);
    }

    std::string jointMoveCommand(const Degrees6& degrees) {
        std::string s = "J(";
        for (const double v : degrees) {
            // 小数点以下 4 桁(0.0001 度)。"-0.0000" にならないように、丸めると 0 になる値は 0 にする
            const double x = std::abs(v) < 0.00005 ? 0.0 : v;
            char buf[64];
            const auto r = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::fixed, 4);
            s.append(buf, r.ptr);
            s += ',';
        }
        s += "-1)";   // 7 番目の値。旧システムが付けていたもの(旧システムで動いている書式をそのまま使う)
        return s;
    }

    Rc8Arm::Rc8Arm(std::shared_ptr<IBcapApi> api, Rc8ArmConfig config, std::shared_ptr<CancelToken> cancel)
        : api_(std::move(api)), config_(std::move(config)), cancel_(std::move(cancel)) {}

    Rc8Arm::~Rc8Arm() { disconnect(); }

    Status Rc8Arm::connect() {
        if (connected_) return {};

        if (config_.host.empty()) return fail(ErrorCode::InvalidArgument, label() + ": the controller address is empty");
        if (config_.port < 1 || config_.port > 65535) return fail(ErrorCode::InvalidArgument, label() + ": the port must be 1..65535");
        if (config_.command_timeout_ms <= 0 || config_.move_timeout_ms <= 0 || config_.retry < 0) {
            return fail(ErrorCode::InvalidArgument, label() + ": the timeouts must be positive and retry must not be negative");
        }

        if (!(config_.max_speed > 0.0 && config_.max_speed <= 1.0)) {
            return fail(ErrorCode::InvalidArgument, label() + ": max_speed must be in (0, 1]");
        }
        if (config_.poll_interval_ms <= 0 || config_.settle_polls < 1 || config_.start_grace_ms < 0 ||
            !(config_.settle_tolerance_deg > 0.0)) {
            return fail(ErrorCode::InvalidArgument, label() + ": the polling settings must be positive");
        }

        const std::string address = bcapAddress(config_.host, config_.port);
        link_ = Link{};

        // 途中で失敗したら、そこまでに開いたものを逆順に閉じてから、エラーを返す
        auto giveUp = [&](const Error& e, const std::string& what) -> tl::unexpected<Error> {
            release(!isLinkError(e.code));
            return fail(e.code, label() + ": " + what + ": " + e.message);
        };

        const auto fd = api_->open(address, config_.command_timeout_ms, config_.retry);
        if (!fd) return giveUp(fd.error(), "cannot connect to " + address);
        link_.fd = *fd;
        link_.socket_open = true;

        if (const auto s = api_->serviceStart(link_.fd); !s) return giveUp(s.error(), "b-CAP service did not start");
        link_.service_started = true;

        const auto controller = api_->controllerConnect(link_.fd, config_.provider, config_.machine);
        if (!controller) return giveUp(controller.error(), "cannot connect to the controller");
        link_.controller = *controller;
        link_.controller_open = true;

        const auto robot = api_->getRobot(link_.fd, link_.controller, config_.robot_name);
        if (!robot) return giveUp(robot.error(), "cannot get the robot \"" + config_.robot_name + "\"");
        link_.robot = *robot;
        link_.robot_open = true;

        // 制御権。取れないときは、ティーチングペンダントがマニュアルモードだったり、別のプログラムが持っていたりする
        if (const auto s = api_->takeArm(link_.robot); !s) {
            return giveUp(s.error(), "cannot take control of the arm (is the teach pendant in manual mode, or is another program connected?)");
        }
        link_.arm_taken = true;

        connected_ = true;
        spdlog::info("[{}] Connected to {} (b-CAP).", label(), address);
        return {};
    }

    void Rc8Arm::disconnect() {
        if (!connected_) return;
        release(true);
        spdlog::info("[{}] Disconnected.", label());
    }

    void Rc8Arm::release(bool graceful) {
        auto note = [&](const Status& s, const char* what) {
            if (s) return;
            spdlog::warn("[{}] {} failed: {}", label(), what, s.error().message);
            // 通信が壊れたなら、以降の命令は届かない。待たずに、ソケットを閉じるだけにする
            if (isLinkError(s.error().code)) graceful = false;
        };

        // 念のため。移動のあとに OFF にし損ねていても、ここで止める
        if (graceful && link_.arm_taken) note(api_->setMotor(link_.robot, false), "Motor off");
        if (graceful && link_.arm_taken) note(api_->giveArm(link_.robot), "Givearm");
        if (graceful && link_.robot_open) note(api_->releaseRobot(link_.robot), "releasing the robot");
        if (graceful && link_.controller_open) note(api_->controllerDisconnect(link_.fd, link_.controller), "disconnecting the controller");
        if (graceful && link_.service_started) note(api_->serviceStop(link_.fd), "stopping the b-CAP service");
        if (link_.socket_open) note(api_->close(link_.fd), "closing the socket");

        link_ = Link{};
        connected_ = false;
    }

    Error Rc8Arm::handleError(Error error, const std::string& what) {
        error.message = label() + ": " + what + ": " + error.message;
        if (isLinkError(error.code)) {
            error.message += " (the connection was closed because its state is unknown; connect again before moving)";
            release(false);
        }
        return error;
    }

    Status Rc8Arm::moveAndCheck(const Degrees6& target) {
        // 動作が終わるまで戻らないので、待ち時間を長くする。移動の結果にかかわらず、あとで元に戻す
        if (const auto s = api_->setTimeout(link_.fd, config_.move_timeout_ms); !s) return s;
        const Status moved = api_->moveJoint(link_.robot, target);
        if (const auto s = api_->setTimeout(link_.fd, config_.command_timeout_ms); !s) {
            spdlog::warn("[{}] restoring the command timeout failed: {}", label(), s.error().message);
        }
        if (!moved) return moved;

        // 命令が無視されていないかを、エンコーダの値で確かめる
        const auto now = api_->currentJoint(link_.robot);
        if (!now) return tl::unexpected(now.error());
        for (std::size_t i = 0; i < 6; ++i) {
            const double diff = std::abs((*now)[i] - target[i]);
            if (!(diff <= config_.arrival_tolerance_deg)) {
                return fail(ErrorCode::DeviceError,
                            "the move finished but J" + std::to_string(i + 1) + " is at " + fixed((*now)[i], 3) +
                                " deg, not at the target " + fixed(target[i], 3) + " deg");
            }
        }
        return {};
    }

    void Rc8Arm::pause(int ms) const {
        if (config_.sleep_ms) {
            config_.sleep_ms(ms);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
    }

    Status Rc8Arm::moveAbortable(const Degrees6& target) {
        if (const auto s = api_->moveJointAsync(link_.robot, target); !s) return s;

        const int interval = config_.poll_interval_ms;
        const int max_polls = (config_.move_timeout_ms + interval - 1) / interval;
        const int grace_polls = (config_.start_grace_ms + interval - 1) / interval;

        std::optional<Degrees6> previous;
        int stable = 0;          // 連続で、関節角が止まっていた回数
        bool moved = false;      // 一度でも、動いているのを見たか

        for (int poll = 0; poll < max_polls; ++poll) {
            // 中止の要求があれば、止める。ここで止めるために、移動を非同期にしている
            if (cancel_ && cancel_->requested()) {
                if (const auto h = api_->halt(link_.robot); !h) return tl::unexpected(h.error());
                return fail(ErrorCode::Cancelled, "cancelled while the arm was moving; it was halted before the target");
            }

            pause(interval);
            const auto now = api_->currentJoint(link_.robot);
            if (!now) return tl::unexpected(now.error());

            if (previous) {
                if (maxAbsDiff(*now, *previous) < config_.settle_tolerance_deg) {
                    ++stable;
                } else {
                    stable = 0;
                    moved = true;
                }
            }
            previous = *now;

            if (stable >= config_.settle_polls) {
                if (maxAbsDiff(*now, target) <= config_.arrival_tolerance_deg) return {};   // 目標で止まった
                // 止まっているのに、目標にいない: 動き出す前(grace の間)でなければ、動作が途中で止まった
                if (moved || poll >= grace_polls) {
                    for (std::size_t i = 0; i < 6; ++i) {
                        if (std::abs((*now)[i] - target[i]) > config_.arrival_tolerance_deg) {
                            return fail(ErrorCode::DeviceError,
                                        "the arm stopped but J" + std::to_string(i + 1) + " is at " + fixed((*now)[i], 3) +
                                            " deg, not at the target " + fixed(target[i], 3) + " deg");
                        }
                    }
                }
            }
        }
        return fail(ErrorCode::Timeout, "the move did not finish within " + std::to_string(config_.move_timeout_ms) + " ms");
    }

    Status Rc8Arm::moveJ(const geometry::Joints& target, Real speed) {
        if (!connected_) return fail(ErrorCode::NotConnected, label() + ": not connected");
        if (cancel_ && cancel_->requested()) {
            return fail(ErrorCode::Cancelled, label() + ": cancelled before the move (nothing was sent)");
        }
        if (!(speed > 0.0 && speed <= 1.0)) {
            return fail(ErrorCode::InvalidArgument, label() + ": speed must be in (0, 1]");
        }
        if (!target.allFinite()) {
            return fail(ErrorCode::InvalidArgument, label() + ": the target joint angles are not finite numbers");
        }

        // コントローラに送る前に、可動域を確かめる(コントローラ側のリミットとは別の防御)
        if (const int bad = config_.limits.firstViolation(target); bad >= 0) {
            const auto i = static_cast<Eigen::Index>(bad);
            return fail(ErrorCode::JointLimit,
                        label() + ": J" + std::to_string(bad + 1) + " = " + fixed(toDegrees(static_cast<double>(target(i))), 2) +
                            " deg is outside [" + fixed(toDegrees(static_cast<double>(config_.limits.lower(i))), 2) + ", " +
                            fixed(toDegrees(static_cast<double>(config_.limits.upper(i))), 2) + "] deg");
        }

        const Degrees6 degrees = toDegreeArray(target);

        // 速度の上限。超えていたら、上限の速度で動かす
        Real effective = speed;
        if (speed > config_.max_speed) {
            effective = config_.max_speed;
            if (!speed_capped_warned_) {
                speed_capped_warned_ = true;
                spdlog::warn("[{}] the requested speed {:.0f}% is above the limit; moving at {:.0f}% (setup.toml [controllers] max_speed)",
                             label(), speed * 100.0, config_.max_speed * 100.0);
            }
        }

        if (const auto s = api_->setExtSpeed(link_.robot, effective * 100.0); !s) {
            return tl::unexpected(handleError(s.error(), "setting the speed"));
        }
        if (const auto s = api_->setMotor(link_.robot, true); !s) {
            return tl::unexpected(handleError(s.error(), "Motor on"));
        }

        // ここからは、結果にかかわらず、モータを OFF に戻す(OFF のあいだは、ブレーキがかかっている)
        const Status result = config_.abortable_moves ? moveAbortable(degrees) : moveAndCheck(degrees);
        if (!result && isLinkError(result.error().code)) {
            // 時間切れのときは、まだ動いているかもしれない。通信が生きているうちに、止めに行く
            if (const auto h = api_->halt(link_.robot); !h) {
                spdlog::warn("[{}] halt after the failed move also failed: {}", label(), h.error().message);
            }
        }
        const Status off = api_->setMotor(link_.robot, false);
        if (!off) spdlog::warn("[{}] Motor off failed: {}", label(), off.error().message);

        if (!result) return tl::unexpected(handleError(result.error(), "moveJ"));
        if (!off) return tl::unexpected(handleError(off.error(), "the move finished, but Motor off"));

        spdlog::debug("[{}] moveJ done (speed {:.2f})", label(), effective);
        return {};
    }

    Result<geometry::Joints> Rc8Arm::currentJoints() const {
        if (!connected_) return fail(ErrorCode::NotConnected, label() + ": not connected");
        const auto d = api_->currentJoint(link_.robot);
        if (!d) return fail(d.error().code, label() + ": reading the joint angles: " + d.error().message);
        return fromDegreeArray(*d);
    }

    void Rc8Arm::stop() {
        if (!connected_) return;
        const Status h = api_->halt(link_.robot);
        if (h) {
            spdlog::info("[{}] STOP.", label());
        } else {
            spdlog::warn("[{}] STOP failed: {}", label(), h.error().message);
            if (isLinkError(h.error().code)) release(false);
        }
    }

    void Rc8Arm::emergencyStop() {
        spdlog::critical("[{}] EMERGENCY STOP!", label());
        if (!connected_) return;
        const Status h = api_->halt(link_.robot);
        if (!h) spdlog::warn("[{}] halt failed: {}", label(), h.error().message);
        // 止めたあと、モータを OFF にして制御権を返す(release の中で行う)。以降は、再接続するまで動かせない
        release(h || !isLinkError(h.error().code));
    }

} // namespace brdf::driver::rc8
