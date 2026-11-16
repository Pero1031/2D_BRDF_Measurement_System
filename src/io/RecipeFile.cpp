/*
* @file src/io/RecipeFile.cpp
* @brief recipe.toml reading / writing, and import of the previous system's pose CSV.
*/

#include "brdf/io/RecipeFile.hpp"

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>

#include "TomlReader.hpp"
#include "brdf/core/MathUtils.hpp"

namespace brdf::io {

    using namespace detail;

    namespace {

        Status readJoints(Section& s, std::string_view key, geometry::Joints& out) {
            std::array<Real, 6> deg{};
            BRDF_TRY(s.numbers(key, deg));
            out = geometry::jointsFromDegrees(deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);
            return {};
        }

        std::string degreesArray(const geometry::Joints& q) {
            std::string out = "[";
            for (int i = 0; i < 6; ++i) {
                out += num(std::stod(std::format("{:.10g}", toDegrees(q(i)))));
                out += i < 5 ? ", " : "]";
            }
            return out;
        }

        Status validate(const domain::Recipe& r, std::string_view origin) {
            auto err = [&](const std::string& m) { return fail(ErrorCode::InvalidArgument, std::string(origin) + ": " + m); };

            if (r.poses.empty()) return err("no poses: add at least one [[poses]] entry");
            if (r.spots.empty()) return err("no spots: add [[spots]] entries or a [spot_grid]");

            const auto& c = r.capture;
            if (!(c.integration_time_ms >= 0.0)) return err("[capture] integration_time_ms must not be negative (0 = automatic)");
            if (!(c.arm_speed > 0.0 && c.arm_speed <= 1.0)) return err("[capture] arm_speed must be in (0, 1]");
            const auto& w = c.wavelengths;
            if (!(w.step > 0.0) || !(w.end >= w.start) || !(w.start > 0.0)) {
                return err("[capture] wavelength_start / wavelength_end / wavelength_step are invalid");
            }

            std::set<int> spot_ids;
            for (const auto& s : r.spots) {
                if (!spot_ids.insert(s.id).second) return err("duplicate spot id " + std::to_string(s.id));
            }
            std::set<std::string> pose_ids;
            for (const auto& p : r.poses) {
                if (!pose_ids.insert(p.id).second) return err("duplicate pose id \"" + p.id + "\"");
            }
            return {};
        }

    } // namespace

    Result<domain::Recipe> parseRecipe(std::string_view text, std::string_view origin) {
        auto parsed = parseToml(text, origin);
        if (!parsed) return tl::unexpected(parsed.error());

        domain::Recipe r;
        Section top(&*parsed, std::string(origin), "");

        // --- [recipe] ---
        {
            auto s = top.child("recipe");
            if (!s) return tl::unexpected(s.error());
            BRDF_TRY(s->string("name", r.name));
            BRDF_TRY(s->string("operator", r.operator_name));
            BRDF_TRY(s->string("description", r.description));
            std::string kind = domain::toString(r.kind);
            BRDF_TRY(s->string("kind", kind));
            auto k = domain::parseKind(kind);
            if (!k) return s->bad("kind", "must be \"sample\", \"white\" or \"calib\"");
            r.kind = *k;
            BRDF_TRY(s->stringArray("checklist", r.checklist));
            BRDF_TRY(s->finish());
        }

        // --- [capture] ---
        {
            auto s = top.child("capture");
            if (!s) return tl::unexpected(s.error());
            auto& c = r.capture;
            BRDF_TRY(s->number("integration_time_ms", c.integration_time_ms));
            BRDF_TRY(s->number("arm_speed", c.arm_speed));
            BRDF_TRY(s->number("wavelength_start", c.wavelengths.start));
            BRDF_TRY(s->number("wavelength_end", c.wavelengths.end));
            BRDF_TRY(s->number("wavelength_step", c.wavelengths.step));
            BRDF_TRY(s->finish());
        }

        // --- [[spots]] ---
        {
            auto list = top.tableArray("spots");
            if (!list) return tl::unexpected(list.error());
            for (Section& s : *list) {
                domain::SpotDefinition spot;
                spot.id = static_cast<int>(r.spots.size()) + 1;
                BRDF_TRY(s.integer("id", spot.id));
                BRDF_TRY(s.number("x", spot.sample_mm.x()));
                BRDF_TRY(s.number("y", spot.sample_mm.y()));
                BRDF_TRY(s.finish());
                r.spots.push_back(spot);
            }
        }

        // --- [spot_grid] ---
        {
            auto s = top.child("spot_grid");
            if (!s) return tl::unexpected(s.error());
            if (s->present()) {
                Vec2 center = Vec2::Zero();
                Vec2 step(10.0, 10.0);
                std::array<Real, 2> count{1.0, 1.0};
                BRDF_TRY(s->vec2("center", center));
                BRDF_TRY(s->vec2("step", step));
                BRDF_TRY(s->numbers("count", count));
                BRDF_TRY(s->finish());

                const int nx = static_cast<int>(count[0]);
                const int ny = static_cast<int>(count[1]);
                if (nx < 1 || ny < 1 || count[0] != nx || count[1] != ny) {
                    return s->bad("count", "must be two positive integers [nx, ny]");
                }
                int next_id = 1;
                for (const auto& sp : r.spots) next_id = std::max(next_id, sp.id + 1);
                for (int j = 0; j < ny; ++j) {
                    for (int i = 0; i < nx; ++i) {
                        domain::SpotDefinition spot;
                        spot.id = next_id++;
                        spot.sample_mm = Vec2(center.x() + (i - (nx - 1) * 0.5) * step.x(),
                                              center.y() + (j - (ny - 1) * 0.5) * step.y());
                        r.spots.push_back(spot);
                    }
                }
            }
        }

        // --- [[poses]] ---
        {
            auto list = top.tableArray("poses");
            if (!list) return tl::unexpected(list.error());
            for (Section& s : *list) {
                domain::PoseStep pose;
                pose.id = std::to_string(r.poses.size() + 1);
                pose.left = geometry::jointsFromDegrees(0, 0, 90, 0, 0, 0);
                pose.right = geometry::jointsFromDegrees(0, 0, 90, 0, 0, 0);

                // left / right は必須。無いまま進むと、気づかないうちに初期姿勢で測ってしまう。
                if (!s.has("left"))  return s.bad("left", "is required (6 joint angles in degrees)");
                if (!s.has("right")) return s.bad("right", "is required (6 joint angles in degrees)");

                BRDF_TRY(s.string("id", pose.id));
                BRDF_TRY(readJoints(s, "left", pose.left));
                BRDF_TRY(readJoints(s, "right", pose.right));
                BRDF_TRY(s.finish());
                r.poses.push_back(std::move(pose));
            }
        }

        BRDF_TRY(top.finish());
        BRDF_TRY(validate(r, origin));
        return r;
    }

    Result<domain::Recipe> loadRecipe(const std::filesystem::path& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return fail(ErrorCode::InvalidArgument, "cannot open " + path.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        return parseRecipe(ss.str(), path.filename().string());
    }

    std::string toToml(const domain::Recipe& r) {
        std::ostringstream o;

        o << "# recipe.toml - 測定の手順書\n"
          << "# ポーズ(左右アームの関節角, 度)、測定点(試料面の座標, mm)、撮影条件をここに書く。\n\n";

        o << "[recipe]\n"
          << "name = " << quote(r.name) << "\n"
          << "operator = " << quote(r.operator_name) << "\n"
          << "description = " << quote(r.description) << "\n"
          << "kind = " << quote(domain::toString(r.kind)) << "   # \"sample\" | \"white\" | \"calib\"\n";
        o << "# 測定前に人が確認する項目(brdf run が表示し、OK の入力を待つ)\n"
          << "checklist = [\n";
        for (const auto& item : r.checklist) o << "    " << quote(item) << ",\n";
        o << "]\n\n";

        o << "[capture]\n"
          << "integration_time_ms = " << num(r.capture.integration_time_ms) << "   # 0 = 装置が自動で決める\n"
          << "arm_speed = " << num(r.capture.arm_speed) << "   # 0 より大きく 1 以下\n"
          << "wavelength_start = " << num(r.capture.wavelengths.start) << "\n"
          << "wavelength_end = " << num(r.capture.wavelengths.end) << "\n"
          << "wavelength_step = " << num(r.capture.wavelengths.step) << "\n\n";

        o << "# 測定点: 試料面の座標 (x, y) [mm]。紙の座標系(原点 = 試料面の基準点)\n"
          << "# [spot_grid] で格子状に指定することもできる:\n"
          << "#   [spot_grid]\n"
          << "#   center = [0.0, 0.0]\n"
          << "#   step = [10.0, 10.0]\n"
          << "#   count = [5, 5]\n";
        for (const auto& s : r.spots) {
            o << "\n[[spots]]\n"
              << "id = " << s.id << "\n"
              << "x = " << num(s.sample_mm.x()) << "\n"
              << "y = " << num(s.sample_mm.y()) << "\n";
        }

        o << "\n# ポーズ: left = 左アーム(光源側)、right = 右アーム(試料側)。J1〜J6 の順、度\n";
        for (const auto& p : r.poses) {
            o << "\n[[poses]]\n"
              << "id = " << quote(p.id) << "\n"
              << "left = " << degreesArray(p.left) << "\n"
              << "right = " << degreesArray(p.right) << "\n";
        }
        return o.str();
    }

    Result<std::vector<domain::PoseStep>> parseLegacyPoseCsv(std::string_view text, std::string_view origin) {
        auto err = [&](int line, const std::string& m) {
            return fail(ErrorCode::InvalidArgument, std::string(origin) + ":" + std::to_string(line) + ": " + m);
        };

        std::istringstream in{std::string(text)};
        std::string line;
        if (!std::getline(in, line)) return err(1, "the file is empty");
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);
        if (line != "RJ1,RJ2,RJ3,RJ4,RJ5,RJ6,LJ1,LJ2,LJ3,LJ4,LJ5,LJ6") {
            return err(1, "the header must be RJ1,RJ2,RJ3,RJ4,RJ5,RJ6,LJ1,LJ2,LJ3,LJ4,LJ5,LJ6");
        }

        std::vector<domain::PoseStep> poses;
        int lineno = 1;
        while (std::getline(in, line)) {
            ++lineno;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            std::vector<Real> v;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                try {
                    std::size_t used = 0;
                    v.push_back(std::stod(cell, &used));
                    if (used != cell.size()) throw std::invalid_argument("trailing");
                } catch (const std::exception&) {
                    return err(lineno, "\"" + cell + "\" is not a number");
                }
            }
            if (v.size() != 12) return err(lineno, "expected 12 values, found " + std::to_string(v.size()));

            domain::PoseStep p;
            p.id = std::to_string(poses.size() + 1);
            p.right = geometry::jointsFromDegrees(v[0], v[1], v[2], v[3], v[4], v[5]);
            p.left = geometry::jointsFromDegrees(v[6], v[7], v[8], v[9], v[10], v[11]);
            poses.push_back(std::move(p));
        }
        if (poses.empty()) return err(lineno, "no pose rows");
        return poses;
    }

    Result<std::vector<domain::PoseStep>> loadLegacyPoseCsv(const std::filesystem::path& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return fail(ErrorCode::InvalidArgument, "cannot open " + path.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        return parseLegacyPoseCsv(ss.str(), path.filename().string());
    }

} // namespace brdf::io
