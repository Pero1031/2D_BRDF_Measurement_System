/*
* @file src/app/RealRun.cpp
* @brief The guards that apply when the real arms move.
*/

#include "brdf/app/RealRun.hpp"

#include <algorithm>
#include <format>
#include <iostream>
#include <sstream>

#include <spdlog/spdlog.h>

#include "brdf/core/MathUtils.hpp"

namespace brdf::app {

    PreflightOptions preflightOptionsFor(const io::SetupConfig& setup) {
        PreflightOptions options;
        options.warn_outside_previous_range = setup.mode == io::DeviceMode::Real;
        return options;
    }

    namespace {

        /// 全ポーズでの、関節ごとの範囲 [度]。"J1 [45, 65]  J2 [0, 20] ..."
        std::string rangeText(const domain::Recipe& recipe, bool left_arm) {
            std::ostringstream o;
            for (int j = 0; j < 6; ++j) {
                Real lo = 1e9, hi = -1e9;
                for (const domain::PoseStep& p : recipe.poses) {
                    const Real v = toDegrees((left_arm ? p.left : p.right)(j));
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
                o << (j == 0 ? "" : "  ") << "J" << (j + 1) << " [" << std::format("{:.0f}", lo) << ", " << std::format("{:.0f}", hi) << "]";
            }
            return o.str();
        }

    } // namespace

    std::string describeRealMotion(const io::SetupConfig& setup, const domain::Recipe& recipe) {
        const auto& ct = setup.controllers;
        const Real requested = recipe.capture.arm_speed;
        const Real effective = std::min(requested, ct.max_speed);

        std::ostringstream o;
        o << "実機のアームを動かします:\n";
        o << "  左アーム(光源側): " << ct.left_host << ":" << ct.port << "\n";
        o << "  右アーム(試料側): " << ct.right_host << ":" << ct.port << "\n";
        o << "  速度: " << std::format("{:.0f}", effective * 100.0) << "%";
        if (requested > ct.max_speed) {
            o << "(recipe.toml の arm_speed = " << std::format("{:.0f}", requested * 100.0)
              << "% を、setup.toml の max_speed = " << std::format("{:.0f}", ct.max_speed * 100.0) << "% に制限)";
        }
        o << "\n";
        o << "  姿勢: " << recipe.poses.size() << " 個\n";
        o << "  動かす範囲(度):\n";
        o << "    左: " << rangeText(recipe, true) << "\n";
        o << "    右: " << rangeText(recipe, false) << "\n";
        if (ct.abortable_moves) {
            o << "  中止: Ctrl+C で、動作中でも止まります(止める → モータ OFF)。ただし、非常停止の代わりにはなりません。\n";
        } else {
            o << "  中止: 動作中は Ctrl+C で止められません(次の姿勢に進む前には止まります)。止めるときは、ティーチングペンダントの非常停止を使ってください。\n";
        }
        o << "  アームの届く範囲に、人・カメラ・試料の板・荷物がなく、非常停止スイッチに手が届くことを確認してください。\n";
        return o.str();
    }

    Status confirmRealMotion(const io::SetupConfig& setup, const domain::Recipe& recipe, std::istream& in) {
        if (setup.mode != io::DeviceMode::Real) return {};
        std::cout << describeRealMotion(setup, recipe);
        std::cout << "動かしてよければ OK と入力してください: " << std::flush;
        std::string answer;
        std::getline(in, answer);
        if (answer != "OK") {
            return fail(ErrorCode::InvalidArgument, "cancelled: moving the real arms was not confirmed (nothing was moved)");
        }
        return {};
    }

    Status checkStartPosition(const geometry::RigGeometry& geometry, const driver::IRobotArm& left, const driver::IRobotArm& right,
                              const domain::PoseStep& first, const PreflightOptions& options) {
        const auto l = left.currentJoints();
        if (!l) return fail(l.error().code, "left arm: reading the current position: " + l.error().message);
        const auto r = right.currentJoints();
        if (!r) return fail(r.error().code, "right arm: reading the current position: " + r.error().message);

        const PreflightReport report = checkStartPath(geometry, *l, *r, first, options);
        for (const Issue& i : report.issues) {
            if (i.severity == Severity::Error) {
                spdlog::error("start check: [{}] {}", toString(i.code), i.message);
            } else {
                spdlog::warn("start check: {}", i.message);
            }
        }
        if (!report.ok()) {
            const auto first_error = std::find_if(report.issues.begin(), report.issues.end(),
                                                  [](const Issue& i) { return i.severity == Severity::Error; });
            return fail(first_error->code, "start check: " + first_error->message + " (" + std::to_string(report.errorCount()) +
                                               " error(s); nothing was moved)");
        }
        spdlog::info("Start check passed: the path from the arms' current position to the first pose is clear");
        return {};
    }

} // namespace brdf::app
