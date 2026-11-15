/*
* @file src/app/Analyze.cpp
* @brief Computes the radiance factor and BRDF from a sample run and a white-plate run.
*/

#include "brdf/app/Analyze.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

#include <spdlog/spdlog.h>

#include "brdf/analysis/RadianceFactor.hpp"
#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Commands.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/io/MeasurementsCsv.hpp"
#include "brdf/io/RecipeFile.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    namespace fs = std::filesystem;

    namespace {

        std::string readText(const fs::path& p) {
            std::ifstream ifs(p, std::ios::binary);
            std::stringstream ss;
            ss << ifs.rdbuf();
            return ss.str();
        }

        // 結果フォルダの名前が "<日時>_<種類>" または "<日時>_<種類>_<番号>" か
        bool isRunOfKind(const std::string& name, const std::string& kind) {
            const std::string tag = "_" + kind;
            const auto pos = name.find(tag);
            if (pos == std::string::npos) return false;
            const std::string rest = name.substr(pos + tag.size());
            if (rest.empty()) return true;
            return rest[0] == '_' && rest.size() > 1 && std::all_of(rest.begin() + 1, rest.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); });
        }

        Result<fs::path> findRun(const ProjectLayout& layout, const std::string& requested, const std::string& kind) {
            std::error_code ec;
            if (!requested.empty()) {
                const fs::path p = layout.resultRoot() / requested;
                if (!fs::is_directory(p, ec)) {
                    return fail(ErrorCode::InvalidArgument, "the result folder " + p.string() + " does not exist");
                }
                return p;
            }
            std::string latest;
            if (fs::is_directory(layout.resultRoot(), ec)) {
                for (const auto& e : fs::directory_iterator(layout.resultRoot(), ec)) {
                    const std::string name = e.path().filename().string();
                    if (e.is_directory() && isRunOfKind(name, kind) && name > latest) latest = name;
                }
            }
            if (latest.empty()) {
                return fail(ErrorCode::InvalidArgument,
                            "no " + kind + " measurement was found in " + layout.resultRoot().string()
                            + "; run `brdf run <dir> --kind " + kind + "` first");
            }
            return layout.resultRoot() / latest;
        }

        // run_manifest.toml の kind が期待どおりか(測定のとりちがえを防ぐ)
        Status checkKind(const fs::path& run, const std::string& kind) {
            const fs::path manifest = run / "run_manifest.toml";
            std::error_code ec;
            if (!fs::exists(manifest, ec)) {
                return fail(ErrorCode::InvalidArgument, run.string() + " has no run_manifest.toml (is it a measurement result?)");
            }
            if (readText(manifest).find("kind = \"" + kind + "\"") == std::string::npos) {
                return fail(ErrorCode::InvalidArgument,
                            run.filename().string() + " is not a \"" + kind + "\" measurement (see its run_manifest.toml)");
            }
            return {};
        }

        // 2 つの測定が、同じポーズ・同じ装置の配置で測られたか
        Status checkSameConditions(const io::SetupConfig& a_setup, const domain::Recipe& a_recipe,
                                   const io::SetupConfig& b_setup, const domain::Recipe& b_recipe,
                                   const std::string& a_name, const std::string& b_name) {
            auto differs = [&](const std::string& what) {
                return fail(ErrorCode::InvalidArgument,
                            "the sample (" + a_name + ") and the white plate (" + b_name + ") were not measured under the "
                            "same conditions: " + what + " differs");
            };
            const auto& ra = a_setup.rig;
            const auto& rb = b_setup.rig;
            const Real tol = 1e-9;
            if ((ra.camera_position - rb.camera_position).norm() > tol) return differs("camera_position");
            if ((ra.light_offset - rb.light_offset).norm() > tol) return differs("light_offset");
            if ((ra.left_base_position - rb.left_base_position).norm() > tol) return differs("left_base");
            if ((ra.right_base_position - rb.right_base_position).norm() > tol) return differs("right_base");
            if (std::abs(ra.sample_surface_offset - rb.sample_surface_offset) > tol) return differs("sample_surface_offset");

            if (a_recipe.poses.size() != b_recipe.poses.size()) return differs("the number of poses");
            for (std::size_t i = 0; i < a_recipe.poses.size(); ++i) {
                const auto& pa = a_recipe.poses[i];
                const auto& pb = b_recipe.poses[i];
                if (pa.id != pb.id) return differs("the name of pose " + std::to_string(i + 1));
                if ((pa.left - pb.left).norm() > tol || (pa.right - pb.right).norm() > tol) {
                    return differs("pose \"" + pa.id + "\"");
                }
            }
            return {};
        }

        // E(P) / E(基準点)。点光源として、E ∝ cosθi / d²。
        analysis::IrradianceRatioFn makeIrradianceRatio(const geometry::RigGeometry& geometry, const domain::Recipe& recipe) {
            return [&geometry, &recipe](const std::string& pose_id, const Vec2& sample_mm) -> Result<Real> {
                const auto it = std::find_if(recipe.poses.begin(), recipe.poses.end(),
                                             [&](const domain::PoseStep& p) { return p.id == pose_id; });
                if (it == recipe.poses.end()) {
                    return fail(ErrorCode::InvalidArgument, "pose \"" + pose_id + "\" is not in the recipe");
                }
                const auto patches = computePoseAngles(geometry, *it, {{1, sample_mm}, {2, Vec2(0.0, 0.0)}});
                const Vec3 light = geometry.lightPosition(it->left);

                auto irradiance = [&](const PatchAngles& p) {
                    const Real d = (light - p.world_point).norm();
                    return std::max(0.0, std::cos(p.angles.incidence.theta)) / (d * d);
                };
                const Real reference = irradiance(patches[1]);
                if (!(reference > 0.0)) {
                    return fail(ErrorCode::OutOfRange,
                                "the light does not reach the white plate's reference point at pose \"" + pose_id + "\"");
                }
                return irradiance(patches[0]) / reference;
            };
        }

    } // namespace

    Result<fs::path> analyzeProject(const fs::path& dir, const AnalyzeOptions& options) {
        const ProjectLayout layout(dir);
        auto project = loadProject(dir);
        if (!project) return tl::unexpected(project.error());
        const io::AnalysisConfig& cfg = project->setup.analysis;

        auto sample_dir = findRun(layout, options.sample_run, "sample");
        if (!sample_dir) return tl::unexpected(sample_dir.error());
        auto white_dir = findRun(layout, options.white_run, "white");
        if (!white_dir) return tl::unexpected(white_dir.error());
        const std::string sample_name = sample_dir->filename().string();
        const std::string white_name = white_dir->filename().string();

        if (auto s = checkKind(*sample_dir, "sample"); !s) return tl::unexpected(s.error());
        if (auto s = checkKind(*white_dir, "white"); !s) return tl::unexpected(s.error());

        // 2 つの測定の、実行時の設定のコピー
        auto sample_setup = io::loadSetup(*sample_dir / "setup.toml");
        auto sample_recipe = io::loadRecipe(*sample_dir / "recipe.toml");
        auto white_setup = io::loadSetup(*white_dir / "setup.toml");
        auto white_recipe = io::loadRecipe(*white_dir / "recipe.toml");
        if (!sample_setup) return tl::unexpected(sample_setup.error());
        if (!sample_recipe) return tl::unexpected(sample_recipe.error());
        if (!white_setup) return tl::unexpected(white_setup.error());
        if (!white_recipe) return tl::unexpected(white_recipe.error());
        if (auto s = checkSameConditions(*sample_setup, *sample_recipe, *white_setup, *white_recipe, sample_name, white_name); !s) {
            return tl::unexpected(s.error());
        }

        auto sample = io::loadMeasurementsCsv(*sample_dir / "measurements.csv");
        auto white = io::loadMeasurementsCsv(*white_dir / "measurements.csv");
        if (!sample) return tl::unexpected(sample.error());
        if (!white) return tl::unexpected(white.error());

        analysis::AnalysisOptions opt;
        opt.white_reflectance = cfg.white_reflectance;
        opt.min_wavelength_nm = cfg.min_wavelength_nm;
        opt.max_wavelength_nm = cfg.max_wavelength_nm;
        const bool correct = options.irradiance_correction.value_or(cfg.irradiance_correction);

        // 照度の補正には、試料を測ったときの装置の配置を使う
        const geometry::RigGeometry geometry(sample_setup->rig);
        analysis::IrradianceRatioFn ratio;
        if (correct) ratio = makeIrradianceRatio(geometry, *sample_recipe);

        auto table = analysis::computeRadianceFactor(*sample, *white, opt, ratio);
        if (!table) return tl::unexpected(table.error());
        if (table->invalid_values > 0) {
            spdlog::warn("{} value(s) could not be computed and are written as nan (the white plate radiance is zero or "
                         "the light does not reach the point)", table->invalid_values);
        }

        const std::string stamp = options.timestamp.empty() ? currentTimestamp() : options.timestamp;
        auto out_dir = layout.createRunDir(stamp, "analysis");
        if (!out_dir) return tl::unexpected(out_dir.error());

        auto write = [&](const std::string& name, const std::string& text) -> Status {
            std::ofstream ofs(*out_dir / name, std::ios::binary);
            if (!ofs) return fail(ErrorCode::DeviceError, "cannot write " + (*out_dir / name).string());
            ofs << text;
            return {};
        };
        if (auto s = write("radiance_factor.csv", analysis::toCsv(*table, analysis::Quantity::RadianceFactor)); !s) return tl::unexpected(s.error());
        if (auto s = write("brdf.csv", analysis::toCsv(*table, analysis::Quantity::Brdf)); !s) return tl::unexpected(s.error());

        std::ostringstream m;
        m << "# analysis_manifest.toml - この解析の記録(自動生成)\n\n[analysis]\n"
          << "timestamp = \"" << stamp << "\"\n"
          << "sample_run = \"" << sample_name << "\"\n"
          << "white_run = \"" << white_name << "\"\n"
          << "white_reflectance = " << opt.white_reflectance << "\n"
          << "wavelength_min_nm = " << table->wavelengths(0) << "\n"
          << "wavelength_max_nm = " << table->wavelengths(table->wavelengths.size() - 1) << "\n"
          << "irradiance_correction = " << (correct ? "true" : "false") << "\n"
          << "rows = " << table->rows.size() << "\n"
          << "invalid_values = " << table->invalid_values << "\n";
        if (auto s = write("analysis_manifest.toml", m.str()); !s) return tl::unexpected(s.error());

        spdlog::info("Analyzed {} row(s): sample = {}, white = {}, irradiance correction = {}",
                     table->rows.size(), sample_name, white_name, correct ? "on" : "off");
        spdlog::info("Saved the result to {}", out_dir->string());
        return *out_dir;
    }

} // namespace brdf::app
