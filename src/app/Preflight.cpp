/*
* @file src/app/Preflight.cpp
* @brief Pre-flight check of a recipe: limits, collisions and angles, without moving any device.
*/

#include "brdf/app/Preflight.hpp"

#include <algorithm>
#include <format>

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

    PreflightReport preflight(const geometry::RigGeometry& geometry, const domain::Recipe& recipe) {
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

            // --- 左アームを先に動かすときの途中(新しい左 + 前のポーズの右) ---
            if (i > 0) {
                const Real transit = geometry.minArmDistance(pose.left, recipe.poses[i - 1].right);
                if (transit < clearance) {
                    add(Severity::Error, idx, pose, ErrorCode::Collision,
                        std::format("while moving to this pose (left arm first), the arms come within {:.1f} mm "
                                    "(limit {:.1f} mm)", transit, clearance));
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
