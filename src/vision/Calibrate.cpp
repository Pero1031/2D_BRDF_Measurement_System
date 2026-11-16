/*
* @file src/vision/Calibrate.cpp
* @brief Calibration of every pose of a recipe from images of the calibration board.
*/

#include "brdf/app/Calibrate.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iostream>

#include <spdlog/spdlog.h>

#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/DeviceFactory.hpp"
#include "brdf/app/Interrupt.hpp"
#include "brdf/app/Preflight.hpp"
#include "brdf/app/RealRun.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/vision/CircleGridDetector.hpp"
#include "brdf/vision/HomographyFit.hpp"
#include "brdf/vision/ImageIO.hpp"

namespace brdf::app {

    namespace fs = std::filesystem;

    namespace {

        struct Guard {
            std::function<void()> action;
            ~Guard() { if (action) action(); }
        };

    } // namespace

    Result<std::vector<io::PoseCalibration>> calibrateProject(const fs::path& dir) {
        return calibrateProject(dir, std::cin);
    }

    Result<std::vector<io::PoseCalibration>> calibrateProject(const fs::path& dir, std::istream& in) {
        const ProjectLayout layout(dir);
        auto project = loadProject(dir);
        if (!project) return tl::unexpected(project.error());
        const io::SetupConfig& setup = project->setup;
        const domain::Recipe& recipe = project->recipe;

        // 実機を動かす前の検査(可動域・干渉)
        const geometry::RigGeometry geometry(setup.rig);
        const PreflightOptions preflight_options = preflightOptionsFor(setup);
        const PreflightReport report = preflight(geometry, recipe, preflight_options);
        logReport(report);
        if (!report.ok()) {
            return fail(ErrorCode::InvalidArgument,
                        "pre-flight check found " + std::to_string(report.errorCount()) + " error(s); nothing was moved");
        }

        const bool real = setup.mode == io::DeviceMode::Real;
        const auto cancel = std::make_shared<CancelToken>();
        std::error_code work_ec;
        if (real) fs::create_directories(layout.workDir(), work_ec);
        if (work_ec) return fail(ErrorCode::DeviceError, "cannot create " + layout.workDir().string());
        auto devices = makeDevices(setup, domain::MeasurementKind::Sample, real ? layout.workDir() : fs::path{}, cancel);
        if (!devices) return tl::unexpected(devices.error());
        if (!devices->left_arm || !devices->right_arm || !devices->imager) {
            return fail(ErrorCode::InvalidArgument, "calibration: the device set is incomplete");
        }

        // 実機のアームを動かす前の確認(装置、速度、動かす範囲、中止の方法)
        if (auto s = confirmRealMotion(setup, recipe, in); !s) return tl::unexpected(s.error());

        if (auto s = devices->left_arm->connect(); !s)  return fail(s.error().code, "left arm: " + s.error().message);
        if (auto s = devices->right_arm->connect(); !s) return fail(s.error().code, "right arm: " + s.error().message);
        if (auto s = devices->imager->connect(); !s)    return fail(s.error().code, "imager: " + s.error().message);
        Guard guard{[&] {
            devices->imager->disconnect();
            devices->right_arm->disconnect();
            devices->left_arm->disconnect();
        }};

        // 実機: アームの実際の位置から、最初の姿勢までの道のりが安全かを、動かす前に確かめる
        if (real && !recipe.poses.empty()) {
            if (auto s = checkStartPosition(geometry, *devices->left_arm, *devices->right_arm, recipe.poses.front(), preflight_options);
                !s) {
                return tl::unexpected(s.error());
            }
        }
        ScopedInterruptHandler interrupt(cancel);   // Ctrl+C で、次の姿勢へ進まずに止まる

        std::error_code ec;
        fs::create_directories(layout.calibrationImagesDir(), ec);
        if (ec) return fail(ErrorCode::DeviceError, "cannot create " + layout.calibrationImagesDir().string());

        const auto& board = setup.calibration.board;
        const std::vector<Vec2> board_points = board.circleCenters();

        std::vector<io::PoseCalibration> done;
        std::vector<std::string> failures;
        const int total = static_cast<int>(recipe.poses.size());

        for (int i = 0; i < total; ++i) {
            const domain::PoseStep& pose = recipe.poses[static_cast<std::size_t>(i)];
            if (cancel->requested()) {
                failures.push_back("cancelled before pose \"" + pose.id + "\"");
                break;
            }
            spdlog::info("calibrating pose {}/{} (\"{}\")", i + 1, total, pose.id);

            auto fail_pose = [&](const std::string& why) {
                spdlog::error("pose \"{}\": {}", pose.id, why);
                failures.push_back("\"" + pose.id + "\" (" + why + ")");
            };

            // アームの移動の失敗は、検出の失敗とは違う(アームがどこにいるか分からなくなる)。次の姿勢へ進まずに、止める
            if (auto s = devices->left_arm->moveJ(pose.left, recipe.capture.arm_speed); !s) {
                fail_pose("moving the left arm: " + s.error().message);
                devices->left_arm->stop();
                devices->right_arm->stop();
                break;
            }
            if (auto s = devices->right_arm->moveJ(pose.right, recipe.capture.arm_speed); !s) {
                fail_pose("moving the right arm: " + s.error().message);
                devices->left_arm->stop();
                devices->right_arm->stop();
                break;
            }

            driver::ImageRequest request;
            request.measure_distance_mm = measureDistance(geometry, pose);
            request.integration_time_ms = recipe.capture.integration_time_ms;
            auto image = devices->imager->captureImage(request);
            if (!image) {
                fail_pose("capturing: " + image.error().message);
                continue;
            }

            const fs::path image_path = layout.calibrationImagesDir() / (fileSafeName(pose.id) + ".png");
            if (auto s = vision::saveGrayImage(image_path, *image); !s) {
                fail_pose(s.error().message);
                continue;
            }

            auto centers = vision::detectCircleGrid(*image, board);
            if (!centers) {
                fail_pose(centers.error().message);
                continue;
            }

            auto fit = vision::fitHomography(board_points, *centers);
            if (!fit) {
                fail_pose(fit.error().message);
                continue;
            }
            if (fit->rms_px > setup.calibration.max_rms_px) {
                fail_pose("the reprojection error is too large (rms " + std::to_string(fit->rms_px) + " px, limit "
                          + std::to_string(setup.calibration.max_rms_px) + " px)");
                continue;
            }

            io::PoseCalibration c;
            c.pose_id = pose.id;
            c.homography = fit->homography;
            c.rms_px = fit->rms_px;
            c.max_px = fit->max_px;
            c.circles = fit->points;
            spdlog::info("  {} circles, reprojection error rms {:.3f} px (max {:.3f} px)", c.circles, c.rms_px, c.max_px);
            done.push_back(std::move(c));
        }

        // 成功したポーズの結果は、失敗があっても保存する
        {
            std::ofstream ofs(layout.calibrationFile(), std::ios::binary);
            if (!ofs) return fail(ErrorCode::DeviceError, "cannot write " + layout.calibrationFile().string());
            ofs << io::toCsv(done);
        }
        spdlog::info("Saved {} calibration(s) to {}", done.size(), layout.calibrationFile().string());

        if (!failures.empty()) {
            std::string list;
            for (const std::string& f : failures) list += (list.empty() ? "" : "; ") + f;
            return fail(ErrorCode::DeviceError,
                        "calibration failed for " + std::to_string(failures.size()) + " pose(s): " + list);
        }
        return done;
    }

} // namespace brdf::app
