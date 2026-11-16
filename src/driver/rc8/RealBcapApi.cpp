/*
* @file src/driver/rc8/RealBcapApi.cpp
* @brief IBcapApi implemented with the DENSO b-CAP client library.
*
* The library defines its own HRESULT / VARIANT / BSTR types (and SysAllocString, SafeArray* and so on), so this file
* must not include <windows.h>. Every call here follows the sequence the legacy system used on the real arms
* (b-CAP command names, VARIANT types and the J(...) move string); each place where the library's documentation
* or the controller manual leaves a choice says why this one was chosen.
*/

#include "brdf/driver/rc8/RealBcapApi.hpp"

#include <cstdio>
#include <string>
#include <utility>

#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#include "bcap_client.h"
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

namespace brdf::driver::rc8 {

    namespace {

        constexpr std::uint32_t kTimeout = 0x80000900u;        // E_TIMEOUT
        constexpr std::uint32_t kNotConnected = 0x80000902u;   // E_NOT_CONNECTED
        constexpr std::uint32_t kOsErrorMask = 0xFFFF0000u;    // OSERR2HRESULT: 0x8091xxxx = OS(ソケット)のエラー
        constexpr std::uint32_t kOsError = 0x80910000u;

        std::string hex(HRESULT hr) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "0x%08X", static_cast<unsigned>(hr));
            return buf;
        }

        /// HRESULT をエラーに直す。時間切れは Timeout、通信そのものの失敗は Communication、それ以外はコントローラの返したエラー。
        /// opening が true のとき(ソケットを開く段階)は、時間切れ以外の失敗もすべて通信の失敗として扱う。
        Error toError(const char* function, HRESULT hr, bool opening = false) {
            const auto code = static_cast<std::uint32_t>(hr);
            ErrorCode kind = ErrorCode::DeviceError;
            std::string meaning;
            if (code == kTimeout) {
                kind = ErrorCode::Timeout;
                meaning = "timeout";
            } else if (code == kNotConnected) {
                kind = ErrorCode::Communication;
                meaning = "not connected";
            } else if ((code & kOsErrorMask) == kOsError) {
                kind = ErrorCode::Communication;
                meaning = "socket error " + std::to_string(code & 0xFFFFu);
            } else if (opening) {
                kind = ErrorCode::Communication;
            } else if (code == 0x80004005u) {
                meaning = "E_FAIL";
            } else if (code == 0x80070005u) {
                meaning = "access denied";
            } else if (code == 0x80070057u) {
                meaning = "invalid argument";
            } else if (code == 0x80070006u) {
                meaning = "invalid handle";
            }
            return Error{kind, std::string(function) + " failed: " + hex(hr) + (meaning.empty() ? "" : " (" + meaning + ")")};
        }

        Status check(const char* function, HRESULT hr) {
            if (FAILED(hr)) return tl::unexpected(toError(function, hr));
            return {};
        }

        bool isAscii(const std::string& s) {
            for (const unsigned char c : s) {
                if (c > 0x7F) return false;
            }
            return true;
        }

        std::wstring widen(const std::string& ascii) { return std::wstring(ascii.begin(), ascii.end()); }

        /// BSTR を必ず解放する。ライブラリの SysAllocString を使う(ライブラリが読む BSTR と同じ形式)。
        class Bstr {
        public:
            explicit Bstr(const std::string& ascii) : s_(SysAllocString(widen(ascii).c_str())) {}
            ~Bstr() {
                if (s_) SysFreeString(s_);
            }
            Bstr(const Bstr&) = delete;
            Bstr& operator=(const Bstr&) = delete;
            BSTR get() const { return s_; }
            bool ok() const { return s_ != nullptr; }

        private:
            BSTR s_;
        };

        /// VARIANT を必ず解放する(中の BSTR や配列も)。
        class Variant {
        public:
            Variant() { VariantInit(&v_); }
            ~Variant() { VariantClear(&v_); }
            Variant(const Variant&) = delete;
            Variant& operator=(const Variant&) = delete;
            VARIANT& get() { return v_; }

            /// 文字列(BSTR)を入れる。
            bool setString(const std::string& ascii) {
                v_.bstrVal = SysAllocString(widen(ascii).c_str());
                v_.vt = VT_BSTR;
                return v_.bstrVal != nullptr;
            }
            /// double 1 個の配列を入れる。
            bool setDouble(double value) {
                v_.parray = SafeArrayCreateVector(VT_R8, 0, 1);
                if (!v_.parray) return false;
                v_.vt = static_cast<uint16_t>(VT_R8 | VT_ARRAY);
                double* data = nullptr;
                if (FAILED(SafeArrayAccessData(v_.parray, reinterpret_cast<void**>(&data))) || !data) return false;
                data[0] = value;
                SafeArrayUnaccessData(v_.parray);
                return true;
            }

        private:
            VARIANT v_;
        };

        Error memoryError(const char* what) { return Error{ErrorCode::DeviceError, std::string("out of memory while preparing ") + what}; }

        /// ロボットへの命令(bCap_RobotExecute)。param は呼び出し側が用意し、result は成功したときだけ使う。
        Status execute(const BcapRobot& robot, const char* command, Variant& param, Variant& result) {
            const Bstr name{command};
            if (!name.ok()) return tl::unexpected(memoryError(command));
            return check(command, bCap_RobotExecute(robot.fd, robot.handle, name.get(), param.get(), &result.get()));
        }

        /// 引数が文字列 1 個(または空)の命令。
        Status executeWithString(const BcapRobot& robot, const char* command, const std::string& argument) {
            Variant param, result;
            if (!param.setString(argument)) return tl::unexpected(memoryError(command));
            return execute(robot, command, param, result);
        }

        /// bCap_RobotMove(補間 1 = 関節補間)。option は "" か "NEXT"。
        Status moveWithOption(const BcapRobot& robot, const Degrees6& target, const char* option_text) {
            Variant pose;
            if (!pose.setString(jointMoveCommand(target))) return tl::unexpected(memoryError("bCap_RobotMove"));
            const Bstr option{option_text};
            if (!option.ok()) return tl::unexpected(memoryError("bCap_RobotMove"));
            return check("bCap_RobotMove", bCap_RobotMove(robot.fd, robot.handle, 1, pose.get(), option.get()));
        }

    } // namespace

    Result<int> RealBcapApi::open(const std::string& address, int timeout_ms, int retry) {
        int fd = 0;
        const HRESULT hr = bCap_Open_Client(address.c_str(), static_cast<uint32_t>(timeout_ms), static_cast<unsigned int>(retry), &fd);
        if (FAILED(hr)) return tl::unexpected(toError("bCap_Open_Client", hr, true));
        return fd;
    }

    Status RealBcapApi::serviceStart(int fd) { return check("bCap_ServiceStart", bCap_ServiceStart(fd, nullptr)); }

    Result<std::uint32_t> RealBcapApi::controllerConnect(int fd, const std::string& provider, const std::string& machine) {
        if (!isAscii(provider) || !isAscii(machine)) {
            return fail(ErrorCode::InvalidArgument, "the b-CAP provider and machine names must be ASCII");
        }
        const Bstr name{""}, prov{provider}, mach{machine}, option{""};
        if (!name.ok() || !prov.ok() || !mach.ok() || !option.ok()) return tl::unexpected(memoryError("bCap_ControllerConnect"));
        uint32_t handle = 0;
        const HRESULT hr = bCap_ControllerConnect(fd, name.get(), prov.get(), mach.get(), option.get(), &handle);
        if (FAILED(hr)) return tl::unexpected(toError("bCap_ControllerConnect", hr));
        return handle;
    }

    Result<BcapRobot> RealBcapApi::getRobot(int fd, std::uint32_t controller, const std::string& robot_name) {
        if (!isAscii(robot_name)) return fail(ErrorCode::InvalidArgument, "the robot name must be ASCII");
        const Bstr name{robot_name}, option{""};
        if (!name.ok() || !option.ok()) return tl::unexpected(memoryError("bCap_ControllerGetRobot"));
        uint32_t handle = 0;
        const HRESULT hr = bCap_ControllerGetRobot(fd, controller, name.get(), option.get(), &handle);
        if (FAILED(hr)) return tl::unexpected(toError("bCap_ControllerGetRobot", hr));
        return BcapRobot{fd, handle};
    }

    Status RealBcapApi::releaseRobot(const BcapRobot& robot) {
        uint32_t handle = robot.handle;
        return check("bCap_RobotRelease", bCap_RobotRelease(robot.fd, &handle));
    }

    Status RealBcapApi::controllerDisconnect(int fd, std::uint32_t controller) {
        uint32_t handle = controller;
        return check("bCap_ControllerDisconnect", bCap_ControllerDisconnect(fd, &handle));
    }

    Status RealBcapApi::serviceStop(int fd) { return check("bCap_ServiceStop", bCap_ServiceStop(fd)); }

    Status RealBcapApi::close(int fd) {
        int f = fd;
        return check("bCap_Close_Client", bCap_Close_Client(&f));
    }

    Status RealBcapApi::setTimeout(int fd, int timeout_ms) {
        return check("bCap_SetTimeout", bCap_SetTimeout(fd, static_cast<uint32_t>(timeout_ms)));
    }

    // 旧システムと同じく、引数なし(空の文字列)の "Takearm"。Keep = 0(省略時)なので、制御権を取った時点で、
    // 内部の速度・加速度・減速度が 100 に、ツール座標系とワーク座標系が 0 に初期化される(RC8 プログラマーズマニュアル)。
    // → 実効の速度が、前に動かしたプログラムの設定に左右されず、ExtSpeed だけで決まる。
    Status RealBcapApi::takeArm(const BcapRobot& robot) { return executeWithString(robot, "Takearm", ""); }

    Status RealBcapApi::giveArm(const BcapRobot& robot) { return executeWithString(robot, "Givearm", ""); }

    // 旧システムと同じ、文字列 "1" / "0"
    Status RealBcapApi::setMotor(const BcapRobot& robot, bool on) { return executeWithString(robot, "Motor", on ? "1" : "0"); }

    // 旧システムと同じ、double 1 個の配列(外部速度 [%]。マニュアルでは 0.1〜100)
    Status RealBcapApi::setExtSpeed(const BcapRobot& robot, double percent) {
        Variant param, result;
        if (!param.setDouble(percent)) return tl::unexpected(memoryError("ExtSpeed"));
        return execute(robot, "ExtSpeed", param, result);
    }

    // 旧システムと同じ、補間方法 1(関節補間)と、文字列 "J(...)" の目標。オプションは空。
    // 動作が終わるまで戻らない(旧システムはこれを前提に、次の測定へ進んでいた)。
    Status RealBcapApi::moveJoint(const BcapRobot& robot, const Degrees6& target) {
        return moveWithOption(robot, target, "");
    }

    // 動作の完了を待たない移動。オプション "NEXT" が、旧システムでは使っていない、この実装で初めて使うもの。
    // 本当に待たずに戻るかは、実機で確かめる(待ってしまっても、Rc8Arm の監視は正しく終わる)。
    Status RealBcapApi::moveJointAsync(const BcapRobot& robot, const Degrees6& target) {
        return moveWithOption(robot, target, "NEXT");
    }

    // DENSO のサンプル(bCapSlaveMove.c)と同じ、引数なしの "CurJnt"。double の配列(J1..J8)が返る。
    Result<Degrees6> RealBcapApi::currentJoint(const BcapRobot& robot) {
        Variant param, result;
        if (auto s = execute(robot, "CurJnt", param, result); !s) return tl::unexpected(s.error());

        VARIANT& v = result.get();
        if (v.vt != static_cast<uint16_t>(VT_R8 | VT_ARRAY) || !v.parray) {
            return fail(ErrorCode::DeviceError, "CurJnt returned an unexpected data type");
        }
        int32_t lower = 0, upper = -1;
        if (FAILED(SafeArrayGetLBound(v.parray, 1, &lower)) || FAILED(SafeArrayGetUBound(v.parray, 1, &upper)) || upper - lower + 1 < 6) {
            return fail(ErrorCode::DeviceError, "CurJnt returned fewer than 6 joint angles");
        }
        double* data = nullptr;
        if (FAILED(SafeArrayAccessData(v.parray, reinterpret_cast<void**>(&data))) || !data) {
            return fail(ErrorCode::DeviceError, "CurJnt returned data that cannot be read");
        }
        Degrees6 out{};
        for (std::size_t i = 0; i < 6; ++i) out[i] = data[i];
        SafeArrayUnaccessData(v.parray);
        return out;
    }

    Status RealBcapApi::halt(const BcapRobot& robot) {
        const Bstr option{""};
        if (!option.ok()) return tl::unexpected(memoryError("bCap_RobotHalt"));
        return check("bCap_RobotHalt", bCap_RobotHalt(robot.fd, robot.handle, option.get()));
    }

} // namespace brdf::driver::rc8
