/*
* @file src/app/ResultFiles.cpp
* @brief Text formats of the files a measurement run produces.
*/

#include "brdf/app/ResultFiles.hpp"

#include <format>
#include <sstream>

#include "brdf/app/AngleTable.hpp"
#include "brdf/core/MathUtils.hpp"

namespace brdf::app {

    namespace {

        std::string g(Real v) { return std::format("{:.9g}", v); }

        std::string tomlString(const std::string& s) {
            std::string out = "\"";
            for (char c : s) {
                if (c == '"' || c == '\\') out += '\\';
                out += c;
            }
            return out + "\"";
        }

        std::string tomlNumber(Real v) {
            std::string s = std::format("{:.12g}", v);
            if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
            return s;
        }

        // CSV のセルには、カンマや引用符を含む文字列を引用符で囲んで入れる
        std::string cell(const std::string& s) {
            if (s.find_first_of(",\"\n") == std::string::npos) return s;
            std::string out = "\"";
            for (char c : s) {
                if (c == '"') out += '"';
                out += c;
            }
            return out + "\"";
        }

    } // namespace

    std::string measurementsCsv(const SessionResult& result) {
        std::ostringstream o;
        o << "pose,spot,x_mm,y_mm,pixel_x,pixel_y,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg";
        for (Eigen::Index i = 0; i < result.wavelengths.size(); ++i) {
            o << ",wl_" << g(result.wavelengths(i));
        }
        o << "\n";

        for (const SpotMeasurement& m : result.rows) {
            o << cell(m.pose_id) << ',' << m.spot_id << ',' << g(m.sample_mm.x()) << ',' << g(m.sample_mm.y()) << ','
              << g(m.pixel.x()) << ',' << g(m.pixel.y()) << ','
              << g(toDegrees(m.angles.incidence.theta)) << ',' << g(toDegrees(m.angles.incidence.phi)) << ','
              << g(toDegrees(m.angles.observation.theta)) << ',' << g(toDegrees(m.angles.observation.phi));
            for (Eigen::Index i = 0; i < m.spectrum.size(); ++i) o << ',' << g(m.spectrum(i));
            o << "\n";
        }
        return o.str();
    }

    std::string anglesCsv(const geometry::RigGeometry& geometry, const domain::Recipe& recipe) {
        std::ostringstream o;
        o << "pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg\n";
        for (const domain::PoseStep& pose : recipe.poses) {
            for (const PatchAngles& p : computePoseAngles(geometry, pose, recipe.spots)) {
                o << cell(pose.id) << ',' << p.spot_id << ',' << g(p.sample_mm.x()) << ',' << g(p.sample_mm.y()) << ','
                  << g(toDegrees(p.angles.incidence.theta)) << ',' << g(toDegrees(p.angles.incidence.phi)) << ','
                  << g(toDegrees(p.angles.observation.theta)) << ',' << g(toDegrees(p.angles.observation.phi)) << "\n";
            }
        }
        return o.str();
    }

    std::string toToml(const RunManifest& m) {
        std::ostringstream o;
        o << "# run_manifest.toml - この測定の記録(自動生成)\n\n"
          << "[run]\n"
          << "tool_version = " << tomlString(m.tool_version) << "\n"
          << "timestamp = " << tomlString(m.timestamp) << "\n"
          << "kind = " << tomlString(m.kind) << "\n"
          << "device_mode = " << tomlString(m.device_mode) << "\n"
          << "recipe_name = " << tomlString(m.recipe_name) << "\n"
          << "operator = " << tomlString(m.operator_name) << "\n"
          << "pose_count = " << m.pose_count << "\n"
          << "spot_count = " << m.spot_count << "\n"
          << "row_count = " << m.row_count << "\n"
          << "wavelength_start = " << tomlNumber(m.wavelength_start) << "\n"
          << "wavelength_end = " << tomlNumber(m.wavelength_end) << "\n"
          << "wavelength_step = " << tomlNumber(m.wavelength_step) << "\n"
          << "\n# このフォルダの setup.toml / recipe.toml は、実行時の設定のコピー。\n";
        return o.str();
    }

} // namespace brdf::app
