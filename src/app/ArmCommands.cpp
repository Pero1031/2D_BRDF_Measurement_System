/*
* @file src/app/ArmCommands.cpp
* @brief `brdf arm`: read the real arms' positions and move one arm at a time.
*/

#include "brdf/app/ArmCommands.hpp"

#include <algorithm>
#include <format>
#include <iostream>
#include <memory>
#include <sstream>

#include <spdlog/spdlog.h>

#include "brdf/app/Commands.hpp"
#include "brdf/app/Interrupt.hpp"
#include "brdf/app/Preflight.hpp"
#include "brdf/app/RealRun.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/geometry/RigGeometry.hpp"

namespace brdf::app {

    namespace {

        const char* sideName(RealArm side) { return side == RealArm::Left ? "left (light source)" : "right (sample)"; }

        std::string hostOf(const io::SetupConfig& setup, RealArm side) {
            const auto& ct = setup.controllers;
            return (side == RealArm::Left ? ct.left_host : ct.right_host) + ":" + std::to_string(ct.port);
        }

        Status requireRealMode(const io::SetupConfig& setup) {
            if (setup.mode != io::DeviceMode::Real) {
                return fail(ErrorCode::InvalidArgument,
                            "brdf arm works on the real arms; set [devices] mode = \"real\" in setup.toml");
            }
            return {};
        }

    } // namespace

    geometry::Joints standardPose(RealArm side) {
        return side == RealArm::Left ? geometry::jointsFromDegrees(45, 0, 90, 0, 0, 0) : geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0);
    }

    std::string formatJoints(const geometry::Joints& q) {
        std::ostringstream o;
        for (int j = 0; j < 6; ++j) o << (j == 0 ? "" : "  ") << "J" << (j + 1) << " = " << std::format("{:.2f}", toDegrees(q(j)));
        return o.str();
    }

    std::string describeArmMove(const io::SetupConfig& setup, RealArm side, const geometry::Joints& from, const geometry::Joints& to,
                                Real requested_speed) {
        const Real effective = std::min(requested_speed, setup.controllers.max_speed);
        Real biggest = 0.0;
        for (int j = 0; j < 6; ++j) biggest = std::max(biggest, std::abs(toDegrees(to(j) - from(j))));

        std::ostringstream o;
        o << "実機のアームを 1 台だけ動かします:\n";
        o << "  アーム: " << (side == RealArm::Left ? "左(光源側)" : "右(試料側)") << "  " << hostOf(setup, side) << "\n";
        o << "  速度: " << std::format("{:.0f}", effective * 100.0) << "%";
        if (requested_speed > setup.controllers.max_speed) {
            o << "(指定の " << std::format("{:.0f}", requested_speed * 100.0) << "% を、setup.toml の max_speed = "
              << std::format("{:.0f}", setup.controllers.max_speed * 100.0) << "% に制限)";
        }
        o << "\n";
        o << "  現在: " << formatJoints(from) << "\n";
        o << "  目標: " << formatJoints(to) << "\n";
        o << "  関節の変化が最大のもの: " << std::format("{:.2f}", biggest) << " 度\n";
        o << "  もう一方のアームは動かしません。\n";
        if (setup.controllers.abortable_moves) {
            o << "  中止: Ctrl+C で、動作中でも止まります(止める → モータ OFF)。非常停止の代わりにはなりません。\n";
        } else {
            o << "  中止: 動作中は Ctrl+C で止められません。止めるときは、ティーチングペンダントの非常停止を使ってください。\n";
        }
        o << "  アームの届く範囲に、人・カメラ・試料の板・荷物がなく、非常停止スイッチに手が届くことを確認してください。\n";
        return o.str();
    }

    Status moveOneArm(const io::SetupConfig& setup, driver::IRobotArm& left, driver::IRobotArm& right, RealArm side,
                      const ArmTarget& target, Real speed, std::istream& in) {
        if (!(speed > 0.0 && speed <= 1.0)) return fail(ErrorCode::InvalidArgument, "the speed must be greater than 0 and at most 1");
        if (!target.absolute && !(target.jog_joint >= 0 && target.jog_joint < 6)) {
            return fail(ErrorCode::InvalidArgument, "give the target: absolute joint angles, or one joint to jog");
        }

        const auto l = left.currentJoints();
        if (!l) return fail(l.error().code, "left arm: reading the current position: " + l.error().message);
        const auto r = right.currentJoints();
        if (!r) return fail(r.error().code, "right arm: reading the current position: " + r.error().message);

        const geometry::Joints from = side == RealArm::Left ? *l : *r;
        geometry::Joints to = from;
        if (target.absolute) {
            to = *target.absolute;
        } else {
            to(target.jog_joint) += toRadians(target.jog_deg);
        }

        // 検査: 可動域、干渉、途中の干渉。もう一方は、実際の位置で止まったままとして調べる
        const geometry::RigGeometry geometry(setup.rig);
        const PreflightReport report =
            checkSingleArmMove(geometry, *l, *r, side == RealArm::Left, to, preflightOptionsFor(setup));
        for (const Issue& i : report.issues) spdlog::error("[{}] {}", toString(i.code), i.message);
        if (!report.ok()) {
            return fail(report.issues.front().code, report.issues.front().message + " (" + std::to_string(report.errorCount()) +
                                                         " error(s); nothing was moved)");
        }

        // 確認
        std::cout << describeArmMove(setup, side, from, to, speed);
        std::cout << "動かしてよければ OK と入力してください: " << std::flush;
        std::string answer;
        std::getline(in, answer);
        if (answer != "OK") {
            return fail(ErrorCode::InvalidArgument, "cancelled: moving the arm was not confirmed (nothing was moved)");
        }

        driver::IRobotArm& arm = side == RealArm::Left ? left : right;
        if (auto s = arm.moveJ(to, speed); !s) return s;

        if (const auto now = arm.currentJoints(); now) {
            spdlog::info("{} arm is now at: {}", sideName(side), formatJoints(*now));
        }
        return {};
    }

    namespace {

        /// 両方のアームを作って接続する。切断は、戻り値のオブジェクトを捨てるときに行う。
        struct ConnectedArms {
            std::shared_ptr<driver::IRobotArm> left, right;
            ~ConnectedArms() {
                if (right) right->disconnect();
                if (left) left->disconnect();
            }
        };

        Result<std::unique_ptr<ConnectedArms>> connectArms(const io::SetupConfig& setup, const std::shared_ptr<CancelToken>& cancel) {
            auto left = makeRc8Arm(setup, RealArm::Left, cancel);
            if (!left) return tl::unexpected(left.error());
            auto right = makeRc8Arm(setup, RealArm::Right, cancel);
            if (!right) return tl::unexpected(right.error());

            auto arms = std::make_unique<ConnectedArms>();
            arms->left = *left;
            arms->right = *right;
            if (auto s = arms->left->connect(); !s) return fail(s.error().code, "left arm: " + s.error().message);
            if (auto s = arms->right->connect(); !s) return fail(s.error().code, "right arm: " + s.error().message);
            return arms;
        }

    } // namespace

    Status armStatus(const std::filesystem::path& project_dir) {
        auto loaded = loadProject(project_dir);
        if (!loaded) return tl::unexpected(loaded.error());
        const io::SetupConfig& setup = loaded->setup;
        if (auto s = requireRealMode(setup); !s) return s;

        auto arms = connectArms(setup, std::make_shared<CancelToken>());
        if (!arms) return tl::unexpected(arms.error());

        for (const RealArm side : {RealArm::Left, RealArm::Right}) {
            driver::IRobotArm& arm = side == RealArm::Left ? *(*arms)->left : *(*arms)->right;
            const auto q = arm.currentJoints();
            if (!q) return fail(q.error().code, std::string(sideName(side)) + " arm: " + q.error().message);
            const int j = setup.rig.joint_limits.firstViolation(*q);
            spdlog::info("{} arm ({}): {}{}", sideName(side), hostOf(setup, side), formatJoints(*q),
                         j >= 0 ? "   <-- J" + std::to_string(j + 1) + " is outside [arm.limits]" : "");
        }
        return {};
    }

    Status armMove(const std::filesystem::path& project_dir, RealArm side, const ArmTarget& target, Real speed, std::istream& in) {
        auto loaded = loadProject(project_dir);
        if (!loaded) return tl::unexpected(loaded.error());
        const io::SetupConfig& setup = loaded->setup;
        if (auto s = requireRealMode(setup); !s) return s;

        const auto cancel = std::make_shared<CancelToken>();
        auto arms = connectArms(setup, cancel);
        if (!arms) return tl::unexpected(arms.error());

        ScopedInterruptHandler interrupt(cancel);
        const Status result = moveOneArm(setup, *(*arms)->left, *(*arms)->right, side, target, speed, in);
        if (!result) {
            // 失敗したときは、動作中だったかもしれないので、切断する前に止める
            (*arms)->left->stop();
            (*arms)->right->stop();
        }
        return result;
    }

} // namespace brdf::app
