/*
* @file src/cli/main.cpp
* @brief The `brdf` command line.
*
*   brdf init  <dir>                         プロジェクトフォルダとひな型を作る
*   brdf check <dir>                         実機を動かさずに可動域・干渉・角度を検査する
*   brdf calibrate <dir>                     キャリブボードを撮影して、ポーズごとのホモグラフィを求める
*   brdf run   <dir> [--kind K] [--yes]      測定を実行して結果を保存する
*   brdf angles <dir>                        角度だけを出力する
*   brdf analyze <dir> [--sample R] [--white R]  試料と白色板の測定から、放射輝度率と BRDF を計算する
*   brdf view <dir> [--steps N]              ポーズの動きを、ブラウザで見られる 3D アニメーション(HTML)にする
*   brdf spots <dir>                         SR-5000 用のスポット定義ファイル(XML)を、ポーズごとに出力する
*   brdf import-poses <csv> [--out F]        旧システムのポーズ CSV から recipe.toml を作る
*/

#include <iostream>
#include <string>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include "brdf/app/Analyze.hpp"
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
