/*
* @file src/app/Commands.cpp
* @brief The operations behind the `brdf` command line.
*/

#include "brdf/app/Commands.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>

#include "brdf/app/DeviceFactory.hpp"
#include "brdf/app/MeasurementSession.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/calibration/HomographySpotProjector.hpp"
#include "brdf/io/CalibrationFile.hpp"
#include "brdf/io/SpotXml.hpp"
#include "brdf/app/AngleTable.hpp"
#include "brdf/app/ResultFiles.hpp"
#include "brdf/io/RecipeFile.hpp"
#include "brdf/io/SetupConfig.hpp"

#ifndef BRDF_VERSION
#define BRDF_VERSION "0.0.0"
#endif

namespace brdf::app {

    namespace fs = std::filesystem;

    namespace {

        Status writeText(const fs::path& path, const std::string& text) {
            std::ofstream ofs(path, std::ios::binary);
            if (!ofs) return fail(ErrorCode::DeviceError, "cannot write " + path.string());
            ofs << text;
            if (!ofs) return fail(ErrorCode::DeviceError, "failed while writing " + path.string());
            return {};
        }

        // キャリブボードの校正結果から、ポーズごとのホモグラフィで投影する SpotProjector を作る。
        // レシピの全ポーズに校正結果がなければ、何も動かす前にエラーにする。
        Result<std::shared_ptr<calibration::HomographySpotProjector>> loadBoardProjector(
            const ProjectLayout& layout, const LoadedProject& project) {
            auto loaded = io::loadCalibration(layout.calibrationFile());
            if (!loaded) return tl::unexpected(loaded.error());

            auto mapping = project.setup.calibration.mapping();
            if (!mapping) return tl::unexpected(mapping.error());
            auto projector = std::make_shared<calibration::HomographySpotProjector>(
                *mapping, project.setup.camera.image_width, project.setup.camera.image_height);
            for (const io::PoseCalibration& c : *loaded) projector->setHomography(c.pose_id, c.homography);

            std::string missing;
            for (const domain::PoseStep& pose : project.recipe.poses) {
                if (!projector->has(pose.id)) missing += (missing.empty() ? "" : ", ") + ("\"" + pose.id + "\"");
            }
            if (!missing.empty()) {
                return fail(ErrorCode::InvalidArgument,
                            "no calibration for pose(s) " + missing + " in " + layout.calibrationFile().string()
                            + "; run `brdf calibrate`");
            }
            return projector;
        }

    } // namespace

    std::string currentTimestamp() {
        const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &now);
#else
        localtime_r(&now, &tm);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
        return buf;
    }

    domain::Recipe defaultRecipe() {
        domain::Recipe r;
        r.name = "example";
        r.operator_name = "";
        r.description = "旧システムの最初の 3 ポーズによる、シミュレーション用のサンプル";
        r.kind = domain::MeasurementKind::Sample;

        // 測定点: 試料面の中心まわり 3×3(間隔 20 mm)
        int id = 1;
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                r.spots.push_back({id++, Vec2((i - 1) * 20.0, (j - 1) * 20.0)});
            }
        }

        // ポーズ: 旧システムの Posdata.csv の最初の 3 行(光源アームの J6 だけが変わる)
        const geometry::Joints right = geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0);
        const Real j6[] = {-30, -25, -20};
        for (int k = 0; k < 3; ++k) {
            r.poses.push_back({std::to_string(k + 1), geometry::jointsFromDegrees(45, 0, 90, 0, 0, j6[k]), right});
        }

        r.checklist = {
            "分光放射輝度計のセッティング(ピント、位置調節)はしましたか",
            "カメラの位置を setup.toml に書き込みましたか",
            "台紙のセッティングはしましたか",
            "暗幕をたらし、暗室にしましたか",
        };
        return r;
    }

    Result<LoadedProject> loadProject(const fs::path& dir) {
        const ProjectLayout layout(dir);
        auto setup = io::loadSetup(layout.setupFile());
        if (!setup) return tl::unexpected(setup.error());
        auto recipe = io::loadRecipe(layout.recipeFile());
        if (!recipe) return tl::unexpected(recipe.error());
        return LoadedProject{std::move(*setup), std::move(*recipe)};
    }

    void logReport(const PreflightReport& report) {
        for (const Issue& i : report.issues) {
            const std::string where = "pose " + std::to_string(i.pose_index + 1) + " (\"" + i.pose_id + "\")";
            if (i.severity == Severity::Error) {
                spdlog::error("{}: [{}] {}", where, toString(i.code), i.message);
            } else {
                spdlog::warn("{}: {}", where, i.message);
            }
        }
    }

    Result<fs::path> initProject(const fs::path& dir) {
        const ProjectLayout layout(dir);
        for (const fs::path& f : {layout.setupFile(), layout.recipeFile()}) {
            std::error_code ec;
            if (fs::exists(f, ec)) {
                return fail(ErrorCode::InvalidArgument, f.string() + " already exists; it was not overwritten");
            }
        }
        if (auto s = layout.createFolders(); !s) return tl::unexpected(s.error());
        if (auto s = writeText(layout.setupFile(), io::toToml(io::SetupConfig{})); !s) return tl::unexpected(s.error());
        if (auto s = writeText(layout.recipeFile(), io::toToml(defaultRecipe())); !s) return tl::unexpected(s.error());
        return dir;
    }

    Result<PreflightReport> checkProject(const fs::path& dir) {
        auto loaded = loadProject(dir);
        if (!loaded) return tl::unexpected(loaded.error());
        PreflightReport report = preflight(geometry::RigGeometry(loaded->setup.rig), loaded->recipe);

        // ボードで投影する設定なら、全ポーズに校正結果があることも確認する
        if (loaded->setup.calibration.projector == "board") {
            auto projector = loadBoardProjector(ProjectLayout(dir), *loaded);
            if (!projector) {
                report.issues.push_back({Severity::Error, 0, loaded->recipe.poses.front().id,
                                         projector.error().code, projector.error().message});
            }
        }
        return report;
    }

    Result<fs::path> runProject(const fs::path& dir, const RunOptions& options, std::istream& in) {
        const ProjectLayout layout(dir);

        auto loaded = loadProject(dir);
        if (!loaded) return tl::unexpected(loaded.error());
        io::SetupConfig& setup = loaded->setup;
        domain::Recipe& recipe = loaded->recipe;
        if (options.kind) recipe.kind = *options.kind;

        // 1. 事前検査: 実機を動かす前に、可動域・干渉を確認する
        const geometry::RigGeometry geometry(setup.rig);
        const PreflightReport report = preflight(geometry, recipe);
        logReport(report);
        if (!report.ok()) {
            return fail(ErrorCode::InvalidArgument,
                        "pre-flight check found " + std::to_string(report.errorCount())
                        + " error(s); nothing was moved. Run `brdf check` for details");
        }

        // 2. 装置の組み立て(real のときはここでエラーになる)と、画像上の位置への変換
        auto devices = makeDevices(setup);
        if (!devices) return tl::unexpected(devices.error());
        if (setup.calibration.projector == "board") {
            auto projector = loadBoardProjector(layout, *loaded);
            if (!projector) return tl::unexpected(projector.error());
            devices->projector = *projector;
        }

        // 3. 測定前の確認(旧システムの「準備はできましたか → OK」)
        if (!options.assume_yes && !recipe.checklist.empty()) {
            std::cout << "測定前の確認:\n";
            for (std::size_t i = 0; i < recipe.checklist.size(); ++i) {
                std::cout << "  " << (i + 1) << ". " << recipe.checklist[i] << "\n";
            }
            std::cout << "すべて確認できたら OK と入力してください: " << std::flush;
            std::string answer;
            std::getline(in, answer);
            if (answer != "OK") {
                return fail(ErrorCode::InvalidArgument, "cancelled: the checklist was not confirmed (nothing was moved)");
            }
        }

        // 4. 測定
        spdlog::info("Running \"{}\" ({}): {} pose(s) x {} spot(s), devices = {}",
                     recipe.name, domain::toString(recipe.kind), recipe.poses.size(), recipe.spots.size(),
                     io::toString(setup.mode));
        MeasurementSession session(geometry, std::move(*devices), recipe);
        auto result = session.run([](int done, int total, const domain::PoseStep& next) {
            if (done < total) spdlog::info("pose {}/{} (\"{}\")", done + 1, total, next.id);
        });
        if (!result) return tl::unexpected(result.error());

        // 5. 保存(測定が成功してから結果フォルダを作る。失敗した測定の空フォルダを残さない)
        const std::string timestamp = options.timestamp.empty() ? currentTimestamp() : options.timestamp;
        auto run_dir = layout.createRunDir(timestamp, domain::toString(recipe.kind));
        if (!run_dir) return tl::unexpected(run_dir.error());

        RunManifest manifest;
        manifest.tool_version = BRDF_VERSION;
        manifest.timestamp = timestamp;
        manifest.kind = domain::toString(recipe.kind);
        manifest.device_mode = io::toString(setup.mode);
        manifest.recipe_name = recipe.name;
        manifest.operator_name = recipe.operator_name;
        manifest.pose_count = static_cast<int>(recipe.poses.size());
        manifest.spot_count = static_cast<int>(recipe.spots.size());
        manifest.row_count = static_cast<int>(result->rows.size());
        manifest.wavelength_start = recipe.capture.wavelengths.start;
        manifest.wavelength_end = recipe.capture.wavelengths.end;
        manifest.wavelength_step = recipe.capture.wavelengths.step;

        if (auto s = writeText(*run_dir / "measurements.csv", measurementsCsv(*result)); !s) return tl::unexpected(s.error());
        if (auto s = writeText(*run_dir / "run_manifest.toml", toToml(manifest)); !s) return tl::unexpected(s.error());

        // 実行時の設定のコピー(後から、どの条件で測ったか追えるように)
        std::error_code ec;
        fs::copy_file(layout.setupFile(), *run_dir / "setup.toml", fs::copy_options::overwrite_existing, ec);
        if (ec) return fail(ErrorCode::DeviceError, "cannot copy setup.toml: " + ec.message());
        fs::copy_file(layout.recipeFile(), *run_dir / "recipe.toml", fs::copy_options::overwrite_existing, ec);
        if (ec) return fail(ErrorCode::DeviceError, "cannot copy recipe.toml: " + ec.message());

        spdlog::info("Saved the result to {}", run_dir->string());
        return *run_dir;
    }

    Result<fs::path> exportAngles(const fs::path& dir, const std::string& timestamp) {
        const ProjectLayout layout(dir);
        auto loaded = loadProject(dir);
        if (!loaded) return tl::unexpected(loaded.error());

        const geometry::RigGeometry geometry(loaded->setup.rig);
        auto run_dir = layout.createRunDir(timestamp.empty() ? currentTimestamp() : timestamp, "angles");
        if (!run_dir) return tl::unexpected(run_dir.error());

        if (auto s = writeText(*run_dir / "angles.csv", anglesCsv(geometry, loaded->recipe)); !s) {
            return tl::unexpected(s.error());
        }
        return *run_dir;
    }

    Result<fs::path> exportSpots(const fs::path& dir, const std::string& timestamp) {
        const ProjectLayout layout(dir);
        auto loaded = loadProject(dir);
        if (!loaded) return tl::unexpected(loaded.error());

        if (loaded->setup.calibration.projector != "board") {
            return fail(ErrorCode::InvalidArgument,
                        "brdf spots needs [calibration] projector = \"board\" in setup.toml (the positions come from the "
                        "calibration-board homographies; run `brdf calibrate` first)");
        }
        auto projector = loadBoardProjector(layout, *loaded);
        if (!projector) return tl::unexpected(projector.error());

        const geometry::RigGeometry geometry(loaded->setup.rig);
        const std::string stamp = timestamp.empty() ? currentTimestamp() : timestamp;

        // 先にすべての位置を計算する(画像の外に出る点があれば、ファイルを 1 つも作らずに止める)
        std::vector<std::pair<std::string, std::vector<driver::SpotRequest>>> per_pose;
        for (const domain::PoseStep& pose : loaded->recipe.poses) {
            std::vector<driver::SpotRequest> spots;
            for (const PatchAngles& p : computePoseAngles(geometry, pose, loaded->recipe.spots)) {
                auto pixel = (*projector)->pixelFor(SpotQuery{pose.id, p.sample_mm, p.world_point});
                if (!pixel) {
                    return fail(pixel.error().code, "pose \"" + pose.id + "\", spot " + std::to_string(p.spot_id) + ": "
                                                    + pixel.error().message);
                }
                spots.push_back({p.spot_id, *pixel});
            }
            per_pose.emplace_back(pose.id, std::move(spots));
        }

        io::SpotXmlSettings settings;
        settings.roi_width = loaded->setup.camera.image_width;
        settings.roi_height = loaded->setup.camera.image_height;
        settings.binning = loaded->setup.spot_xml.binning;
        settings.spot_size = loaded->setup.spot_xml.spot_size;
        settings.datetime = io::spotXmlDatetime(stamp);

        auto run_dir = layout.createRunDir(stamp, "spots");
        if (!run_dir) return tl::unexpected(run_dir.error());
        for (const auto& [pose_id, spots] : per_pose) {
            if (auto s = writeText(*run_dir / (fileSafeName(pose_id) + "_spot.xml"), io::toSpotXml(settings, spots)); !s) {
                return tl::unexpected(s.error());
            }
        }
        spdlog::info("Wrote {} spot definition file(s) to {}", per_pose.size(), run_dir->string());
        return *run_dir;
    }

    Status importPoses(const fs::path& legacy_csv, const fs::path& recipe_out, bool force) {
        std::error_code ec;
        if (fs::exists(recipe_out, ec) && !force) {
            return fail(ErrorCode::InvalidArgument,
                        recipe_out.string() + " already exists; use --force to overwrite it");
        }
        auto poses = io::loadLegacyPoseCsv(legacy_csv);
        if (!poses) return tl::unexpected(poses.error());

        domain::Recipe recipe = defaultRecipe();
        recipe.name = legacy_csv.stem().string();
        recipe.description = "旧システムのポーズ CSV (" + legacy_csv.filename().string() + ") から取り込み";
        recipe.poses = std::move(*poses);

        return writeText(recipe_out, io::toToml(recipe));
    }

} // namespace brdf::app
