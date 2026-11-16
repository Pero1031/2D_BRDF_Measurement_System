/*
* @file src/cli/main.cpp
* @brief The `brdf` command line.
*
*   brdf init  <dir>                         プロジェクトフォルダとひな型を作る
*   brdf check <dir>                         実機を動かさずに可動域・干渉・角度を検査する
*   brdf calibrate <dir>                     キャリブボードを撮影して、ポーズごとのホモグラフィを求める
*   brdf run   <dir> [--kind K] [--yes]      測定を実行して結果を保存する
*   brdf arm status <dir>                    実機のアームの現在の関節角を表示する(動かさない)
*   brdf arm move <dir> --arm A (--to ... | --standard | --jog N=DEG)  実機のアームを 1 台だけ動かす
*   brdf angles <dir>                        角度だけを出力する
*   brdf analyze <dir> [--sample R] [--white R]  試料と白色板の測定から、放射輝度率と BRDF を計算する
*   brdf view <dir> [--steps N]              ポーズの動きを、ブラウザで見られる 3D アニメーション(HTML)にする
*   brdf spots <dir>                         SR-5000 用のスポット定義ファイル(XML)を、ポーズごとに出力する
*   brdf import-poses <csv> [--out F]        旧システムのポーズ CSV から recipe.toml を作る
*/

#include <iostream>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include "brdf/app/Analyze.hpp"
#include "brdf/app/ArmCommands.hpp"
#include "brdf/app/Calibrate.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/ViewExport.hpp"

#ifndef BRDF_VERSION
#define BRDF_VERSION "0.0.0"
#endif

namespace {

    int reportError(const brdf::Error& e) {
        spdlog::error("[{}] {}", brdf::toString(e.code), e.message);
        return 1;
    }

} // namespace

int main(int argc, char** argv) {
    spdlog::set_pattern("[%H:%M:%S] [%^%l%$] %v");

    CLI::App app{"2D BRDF measurement system"};
    app.set_version_flag("--version", BRDF_VERSION);
    app.require_subcommand(1);

    int exit_code = 0;

    // --- init ---
    std::string init_dir;
    auto* init = app.add_subcommand("init", "Create a project folder with template setup.toml and recipe.toml");
    init->add_option("dir", init_dir, "project folder")->required();
    init->callback([&] {
        auto r = brdf::app::initProject(init_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
        spdlog::info("Created the project at {}", r->string());
        spdlog::info("Next: edit setup.toml / recipe.toml, then `brdf check {0}` and `brdf run {0}`", init_dir);
    });

    // --- check ---
    std::string check_dir;
    auto* check = app.add_subcommand("check", "Check joint limits, collisions and angles without moving any device");
    check->add_option("dir", check_dir, "project folder")->required();
    check->callback([&] {
        auto r = brdf::app::checkProject(check_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
        brdf::app::logReport(*r);
        spdlog::info("Check finished: {} error(s), {} warning(s)", r->errorCount(), r->warningCount());
        if (!r->ok()) exit_code = 2;
    });

    // --- calibrate ---
    std::string calib_dir;
    auto* calibrate = app.add_subcommand("calibrate", "Photograph the calibration board at every pose and compute the homographies");
    calibrate->add_option("dir", calib_dir, "project folder")->required();
    calibrate->callback([&] {
        auto r = brdf::app::calibrateProject(calib_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- run ---
    std::string run_dir, run_kind;
    bool run_yes = false;
    auto* run = app.add_subcommand("run", "Run the measurement and save the result");
    run->add_option("dir", run_dir, "project folder")->required();
    run->add_option("--kind", run_kind, "override the kind in recipe.toml: sample | white | calib")
        ->check(CLI::IsMember({"sample", "white", "calib"}));
    run->add_flag("--yes,-y", run_yes, "skip the confirmation of the checklist");
    run->callback([&] {
        brdf::app::RunOptions options;
        options.assume_yes = run_yes;
        if (!run_kind.empty()) options.kind = brdf::domain::parseKind(run_kind);
        auto r = brdf::app::runProject(run_dir, options, std::cin);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- arm ---
    auto* arm = app.add_subcommand("arm", "Read the real arms' positions, or move one arm at a time (docs/SafetyProcedure.md)");
    arm->require_subcommand(1);

    std::string arm_status_dir;
    auto* arm_status = arm->add_subcommand("status", "Show the current joint angles of both arms (nothing moves)");
    arm_status->add_option("dir", arm_status_dir, "project folder (setup.toml with mode = \"real\")")->required();
    arm_status->callback([&] {
        auto r = brdf::app::armStatus(arm_status_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    std::string arm_move_dir, arm_move_side, arm_move_jog;
    std::vector<double> arm_move_to;
    bool arm_move_standard = false;
    double arm_move_speed = 0.05;
    auto* arm_move = arm->add_subcommand("move", "Move one arm: to given joint angles, to the standard pose, or by jogging one joint");
    arm_move->add_option("dir", arm_move_dir, "project folder (setup.toml with mode = \"real\")")->required();
    arm_move->add_option("--arm", arm_move_side, "left (light source) | right (sample)")->required()
        ->check(CLI::IsMember({"left", "right"}));
    auto* to_option = arm_move->add_option("--to", arm_move_to, "target J1..J6 in degrees, comma separated; write it as --to=-45,0,90,0,0,0")
                          ->delimiter(',');
    auto* standard_option = arm_move->add_flag("--standard", arm_move_standard, "the standard pose of the previous system (right -45,0,90,0,0,0; left 45,0,90,0,0,0)");
    auto* jog_option = arm_move->add_option("--jog", arm_move_jog, "move one joint by some degrees: N=DEG (N = 1..6), e.g. --jog 1=2.5");
    to_option->excludes(standard_option)->excludes(jog_option);
    standard_option->excludes(jog_option);
    arm_move->add_option("--speed", arm_move_speed, "speed, greater than 0 and at most 1 (never above max_speed in setup.toml)")
        ->capture_default_str()->check(CLI::Range(0.001, 1.0));
    arm_move->callback([&] {
        brdf::app::ArmTarget target;
        const auto side = arm_move_side == "left" ? brdf::app::RealArm::Left : brdf::app::RealArm::Right;
        if (arm_move_standard) {
            target.absolute = brdf::app::standardPose(side);
        } else if (!arm_move_to.empty()) {
            if (arm_move_to.size() != 6) {
                exit_code = reportError({brdf::ErrorCode::InvalidArgument, "--to needs exactly 6 angles (J1..J6)"});
                return;
            }
            target.absolute = brdf::geometry::jointsFromDegrees(arm_move_to[0], arm_move_to[1], arm_move_to[2],
                                                                arm_move_to[3], arm_move_to[4], arm_move_to[5]);
        } else if (!arm_move_jog.empty()) {
            const auto eq = arm_move_jog.find('=');
            try {
                if (eq == std::string::npos) throw std::invalid_argument("no '='");
                target.jog_joint = std::stoi(arm_move_jog.substr(0, eq)) - 1;
                target.jog_deg = std::stod(arm_move_jog.substr(eq + 1));
            } catch (const std::exception&) {
                exit_code = reportError({brdf::ErrorCode::InvalidArgument, "--jog must look like N=DEG, e.g. 1=2.5"});
                return;
            }
            if (target.jog_joint < 0 || target.jog_joint > 5) {
                exit_code = reportError({brdf::ErrorCode::InvalidArgument, "--jog: the joint number must be 1..6"});
                return;
            }
        } else {
            exit_code = reportError({brdf::ErrorCode::InvalidArgument, "give a target: --to, --standard or --jog"});
            return;
        }
        auto r = brdf::app::armMove(arm_move_dir, side, target, arm_move_speed, std::cin);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- angles ---
    std::string angles_dir;
    auto* angles = app.add_subcommand("angles", "Write the incidence / observation angles of every pose and spot");
    angles->add_option("dir", angles_dir, "project folder")->required();
    angles->callback([&] {
        auto r = brdf::app::exportAngles(angles_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
        spdlog::info("Wrote {}", (*r / "angles.csv").string());
    });

    // --- analyze ---
    std::string an_dir, an_sample, an_white;
    bool an_correct = false;
    auto* analyze = app.add_subcommand("analyze", "Compute the radiance factor and the BRDF from a sample run and a white-plate run");
    analyze->add_option("dir", an_dir, "project folder")->required();
    analyze->add_option("--sample", an_sample, "result folder of the sample run (default: the latest)");
    analyze->add_option("--white", an_white, "result folder of the white-plate run (default: the latest)");
    analyze->add_flag("--irradiance-correction", an_correct,
                      "correct for the different irradiance at the sample point and the white reference point");
    analyze->callback([&] {
        brdf::app::AnalyzeOptions options;
        options.sample_run = an_sample;
        options.white_run = an_white;
        if (an_correct) options.irradiance_correction = true;
        auto r = brdf::app::analyzeProject(an_dir, options);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- view ---
    std::string view_dir;
    int view_steps = 15;
    auto* view = app.add_subcommand("view", "Export the poses as a 3D animation (one HTML file, opens in any browser)");
    view->add_option("dir", view_dir, "project folder")->required();
    view->add_option("--steps", view_steps, "frames per arm movement (smoother, bigger file)")->capture_default_str()->check(CLI::Range(1, 200));
    view->callback([&] {
        brdf::app::ViewOptions options;
        options.steps_per_move = view_steps;
        auto r = brdf::app::exportView(view_dir, options);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- spots ---
    std::string spots_dir;
    auto* spots = app.add_subcommand("spots", "Write SR-5000 spot definition files (XML), one per pose, from the calibration");
    spots->add_option("dir", spots_dir, "project folder")->required();
    spots->callback([&] {
        auto r = brdf::app::exportSpots(spots_dir);
        if (!r) { exit_code = reportError(r.error()); return; }
    });

    // --- import-poses ---
    std::string import_csv, import_out = "recipe.toml";
    bool import_force = false;
    auto* import = app.add_subcommand("import-poses", "Create a recipe.toml from the previous system's pose CSV");
    import->add_option("csv", import_csv, "pose CSV (header RJ1..RJ6,LJ1..LJ6)")->required()->check(CLI::ExistingFile);
    import->add_option("--out", import_out, "recipe file to write")->capture_default_str();
    import->add_flag("--force", import_force, "overwrite the recipe file if it exists");
    import->callback([&] {
        auto r = brdf::app::importPoses(import_csv, import_out, import_force);
        if (!r) { exit_code = reportError(r.error()); return; }
        spdlog::info("Wrote {}", import_out);
    });

    CLI11_PARSE(app, argc, argv);
    return exit_code;
}
