/*
* @file src/app/Preflight.cpp
* @brief Pre-flight check of a recipe: limits, collisions and angles, without moving any device.
*/

#include "brdf/app/Preflight.hpp"

#include <algorithm>
#include <format>
#include <tuple>

#include "brdf/app/AngleTable.hpp"
#include "brdf/core/MathUtils.hpp"

namespace brdf::app {

    int PreflightReport::errorCount() const {
        return static_cast<int>(std::count_if(issues.begin(), issues.end(),
            [](const Issue& i) { return i.severity == Severity::Error; }));
    }

    int PreflightReport::warningCount() const {
        return static_cast<int>(issues.size()) - errorCount();
    }

    namespace {

        geometry::Joints lerp(const geometry::Joints& a, const geometry::Joints& b, Real t) { return a + (b - a) * t; }

        /// 左アームが from → to へ動き、右アームが right で止まっているときの、アーム同士の最短距離(途中の点も含む)。
        Real minDistanceLeftMoving(const geometry::RigGeometry& g, const geometry::Joints& from, const geometry::Joints& to,
                                   const geometry::Joints& right, int samples) {
            Real d = g.minArmDistance(from, right);
            for (int k = 1; k <= samples; ++k) {
                d = std::min(d, g.minArmDistance(lerp(from, to, static_cast<Real>(k) / samples), right));
            }
            return d;
        }

        /// 右アームが from → to へ動き、左アームが left で止まっているとき。
        Real minDistanceRightMoving(const geometry::RigGeometry& g, const geometry::Joints& left, const geometry::Joints& from,
                                    const geometry::Joints& to, int samples) {
            Real d = g.minArmDistance(left, from);
            for (int k = 1; k <= samples; ++k) {
                d = std::min(d, g.minArmDistance(left, lerp(from, to, static_cast<Real>(k) / samples)));
            }
            return d;
        }

    } // namespace

    const JointRange& previousSystemRange(bool left_arm) {
        static const JointRange right{geometry::jointsFromDegrees(-85, -20, 20, -20, -20, -30),
                                      geometry::jointsFromDegrees(-30, 45, 125, 15, 15, 30)};
        static const JointRange left{geometry::jointsFromDegrees(0, -20, 90, -45, -115, -70),
                                     geometry::jointsFromDegrees(85, 40, 110, 40, 0, 65)};
        return left_arm ? left : right;
    }

    PreflightReport checkStartPath(const geometry::RigGeometry& geometry, const geometry::Joints& current_left,
                                   const geometry::Joints& current_right, const domain::PoseStep& first,
                                   const PreflightOptions& options) {
        PreflightReport report;
        const auto& setup = geometry.setup();
        const Real clearance = 2.0 * setup.arm_radius;
        auto add = [&](ErrorCode code, std::string msg) {
            report.issues.push_back({Severity::Error, 0, first.id, code, "from the current position: " + std::move(msg)});
        };

        // 今の位置が、可動域の中か(外なら、設定かコントローラの設定が、実機と合っていない)
        for (const auto& [name, q] : {std::pair{"left", current_left}, std::pair{"right", current_right}}) {
            const int j = setup.joint_limits.firstViolation(q);
            if (j >= 0) {
                add(ErrorCode::JointLimit,
                    std::format("the {} arm is at J{} = {:.2f} deg, outside the limits [{:.1f}, {:.1f}] deg; "
                                "check [arm.limits] and the controller's settings",
                                name, j + 1, toDegrees(q(j)), toDegrees(setup.joint_limits.lower(j)),
                                toDegrees(setup.joint_limits.upper(j))));
            }
        }

        // 今の位置で、既に近すぎないか
        const Real now = geometry.minArmDistance(current_left, current_right);
        if (now < clearance) {
            add(ErrorCode::Collision,
                std::format("the arms are already within {:.1f} mm of each other (limit {:.1f} mm)", now, clearance));
        }

        // 左アームを先に動かす途中、そのあと右アームを動かす途中
        const Real left_phase = minDistanceLeftMoving(geometry, current_left, first.left, current_right, options.path_samples);
        if (left_phase < clearance) {
            add(ErrorCode::Collision,
                std::format("while moving the left arm to the first pose, the arms come within {:.1f} mm (limit {:.1f} mm)",
                            left_phase, clearance));
        }
        const Real right_phase = minDistanceRightMoving(geometry, first.left, current_right, first.right, options.path_samples);
        if (right_phase < clearance) {
            add(ErrorCode::Collision,
                std::format("while moving the right arm to the first pose, the arms come within {:.1f} mm (limit {:.1f} mm)",
                            right_phase, clearance));
        }
        return report;
    }

    PreflightReport checkSingleArmMove(const geometry::RigGeometry& geometry, const geometry::Joints& current_left,
                                       const geometry::Joints& current_right, bool move_left, const geometry::Joints& target,
                                       const PreflightOptions& options) {
        PreflightReport report;
        const auto& setup = geometry.setup();
        const Real clearance = 2.0 * setup.arm_radius;
        auto add = [&](ErrorCode code, std::string msg) {
            report.issues.push_back({Severity::Error, 0, "arm move", code, std::move(msg)});
        };
        const char* moving = move_left ? "left" : "right";

        for (const auto& [name, q] : {std::pair{"left", current_left}, std::pair{"right", current_right}}) {
            const int j = setup.joint_limits.firstViolation(q);
            if (j >= 0) {
                add(ErrorCode::JointLimit,
                    std::format("the {} arm is at J{} = {:.2f} deg, outside the limits [{:.1f}, {:.1f}] deg; "
                                "check [arm.limits] and the controller's settings",
                                name, j + 1, toDegrees(q(j)), toDegrees(setup.joint_limits.lower(j)),
                                toDegrees(setup.joint_limits.upper(j))));
            }
        }
        if (const int j = setup.joint_limits.firstViolation(target); j >= 0) {
            add(ErrorCode::JointLimit,
                std::format("the target of the {} arm is J{} = {:.2f} deg, outside the limits [{:.1f}, {:.1f}] deg", moving, j + 1,
                            toDegrees(target(j)), toDegrees(setup.joint_limits.lower(j)), toDegrees(setup.joint_limits.upper(j))));
        }

        const Real now = geometry.minArmDistance(current_left, current_right);
        if (now < clearance) {
            add(ErrorCode::Collision,
                std::format("the arms are already within {:.1f} mm of each other (limit {:.1f} mm)", now, clearance));
        }

        const Real path = move_left ? minDistanceLeftMoving(geometry, current_left, target, current_right, options.path_samples)
                                    : minDistanceRightMoving(geometry, current_left, current_right, target, options.path_samples);
        if (path < clearance) {
            add(ErrorCode::Collision,
                std::format("while the {} arm moves to the target, the arms come within {:.1f} mm (limit {:.1f} mm)", moving, path,
                            clearance));
        }
        return report;
    }

    PreflightReport preflight(const geometry::RigGeometry& geometry, const domain::Recipe& recipe,
                              const PreflightOptions& options) {
        PreflightReport report;
        const auto& setup = geometry.setup();
        const Real clearance = 2.0 * setup.arm_radius;

        auto add = [&](Severity sev, int index, const domain::PoseStep& pose, ErrorCode code, std::string msg) {
            report.issues.push_back({sev, index, pose.id, code, std::move(msg)});
        };

        for (std::size_t i = 0; i < recipe.poses.size(); ++i) {
            const domain::PoseStep& pose = recipe.poses[i];
            const int idx = static_cast<int>(i);

            // --- 関節の可動域 ---
            bool limits_ok = true;
            for (const auto& [name, q] : {std::pair{"left", pose.left}, std::pair{"right", pose.right}}) {
                const int j = setup.joint_limits.firstViolation(q);
                if (j >= 0) {
                    limits_ok = false;
                    add(Severity::Error, idx, pose, ErrorCode::JointLimit,
                        std::format("{} arm: J{} = {:.2f} deg is outside the limits [{:.1f}, {:.1f}] deg",
                                    name, j + 1, toDegrees(q(j)),
                                    toDegrees(setup.joint_limits.lower(j)), toDegrees(setup.joint_limits.upper(j))));
                }
            }

            // --- 左右アームの干渉(到達したとき) ---
            const Real arrived = geometry.minArmDistance(pose.left, pose.right);
            if (arrived < clearance) {
                add(Severity::Error, idx, pose, ErrorCode::Collision,
                    std::format("the arms come within {:.1f} mm of each other at this pose (limit {:.1f} mm)",
                                arrived, clearance));
            }

            // --- 動く途中(左アームを先に、そのあと右アーム。途中の点も調べる) ---
            if (i > 0) {
                const domain::PoseStep& prev = recipe.poses[i - 1];
                const Real left_phase = minDistanceLeftMoving(geometry, prev.left, pose.left, prev.right, options.path_samples);
                if (left_phase < clearance) {
                    add(Severity::Error, idx, pose, ErrorCode::Collision,
                        std::format("while moving the left arm to this pose, the arms come within {:.1f} mm "
                                    "(limit {:.1f} mm)", left_phase, clearance));
                }
                const Real right_phase = minDistanceRightMoving(geometry, pose.left, prev.right, pose.right, options.path_samples);
                if (right_phase < clearance) {
                    add(Severity::Error, idx, pose, ErrorCode::Collision,
                        std::format("while moving the right arm to this pose, the arms come within {:.1f} mm "
                                    "(limit {:.1f} mm)", right_phase, clearance));
                }
            }

            // --- 旧システムの実績範囲の外(実機モードのときだけ。警告) ---
            if (options.warn_outside_previous_range && limits_ok) {
                const Real margin = toRadians(options.previous_range_margin_deg);
                for (const auto& [name, q, is_left] : {std::tuple{"left", pose.left, true}, std::tuple{"right", pose.right, false}}) {
                    const JointRange& range = previousSystemRange(is_left);
                    for (int j = 0; j < 6; ++j) {
                        if (q(j) < range.lower(j) - margin || q(j) > range.upper(j) + margin) {
                            add(Severity::Warning, idx, pose, ErrorCode::OutOfRange,
                                std::format("{} arm: J{} = {:.1f} deg is outside the range the previous system used "
                                            "([{:.0f}, {:.0f}] deg, margin {:.0f} deg); this has not been tried on the real arm",
                                            name, j + 1, toDegrees(q(j)), toDegrees(range.lower(j)), toDegrees(range.upper(j)),
                                            options.previous_range_margin_deg));
                        }
                    }
                }
            }

            // --- 角度(可動域が正しいポーズだけ。範囲外の姿勢の角度は意味がない) ---
            if (limits_ok) {
                const auto patches = computePoseAngles(geometry, pose, recipe.spots);
                int light_behind = 0, camera_behind = 0;
                for (const PatchAngles& p : patches) {
                    if (p.angles.incidence.theta >= HALF_PI)   ++light_behind;
                    if (p.angles.observation.theta >= HALF_PI) ++camera_behind;
                }
                if (light_behind > 0) {
                    add(Severity::Warning, idx, pose, ErrorCode::OutOfRange,
                        std::format("the light is behind the sample surface (theta_i >= 90 deg) for {} of {} spots",
                                    light_behind, patches.size()));
                }
                if (camera_behind > 0) {
                    add(Severity::Warning, idx, pose, ErrorCode::OutOfRange,
                        std::format("the camera is behind the sample surface (theta_o >= 90 deg) for {} of {} spots",
                                    camera_behind, patches.size()));
                }
            }
        }
        return report;
    }

} // namespace brdf::app
