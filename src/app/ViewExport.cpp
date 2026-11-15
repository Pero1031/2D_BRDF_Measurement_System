/*
* @file src/app/ViewExport.cpp
* @brief Exports the poses of a recipe as a self-contained 3D animation (one HTML file).
*/

#include "brdf/app/ViewExport.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Preflight.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/io/SpotXml.hpp"
#include "brdf/sim/PinholeCamera.hpp"
#include "brdf/sim/SimulatedRig.hpp"

// resources/view_template.html を、ビルド時に CMake が埋め込んだもの(build/generated/ViewTemplate.cpp)
namespace brdf::app::detail {
    extern const unsigned char kViewTemplateBytes[];
    extern const std::size_t kViewTemplateSize;
}

namespace brdf::app {

    namespace fs = std::filesystem;

    namespace {

        // ---- JSON の部品 ----
        std::string str(const std::string& s) {
            std::string out = "\"";
            for (const unsigned char c : s) {
                switch (c) {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                // <script> を壊さないよう、HTML で特別な文字は \u00XX にする
                case '<': case '>': case '&': out += std::format("\\u{:04x}", static_cast<unsigned>(c)); break;
                default:
                    if (c < 0x20) out += std::format("\\u{:04x}", static_cast<unsigned>(c));
                    else out += static_cast<char>(c);
                }
            }
            return out + "\"";
        }

        std::string n1(Real v) { return std::format("{:.1f}", v); }
        std::string n2(Real v) { return std::format("{:.2f}", v); }

        std::string vec(const Vec3& v) { return "[" + n1(v.x()) + "," + n1(v.y()) + "," + n1(v.z()) + "]"; }

        // アームの折れ線(7 点 = 21 個の数)
        std::string points(const std::vector<geometry::Segment>& segs) {
            std::string out = "[";
            for (std::size_t i = 0; i < segs.size(); ++i) {
                const Vec3& a = segs[i].a;
                out += n1(a.x()) + "," + n1(a.y()) + "," + n1(a.z()) + ",";
            }
            const Vec3& last = segs.back().b;
            out += n1(last.x()) + "," + n1(last.y()) + "," + n1(last.z()) + "]";
            return out;
        }

        std::string degrees(const geometry::Joints& q) {
            std::string out = "[";
            for (int i = 0; i < 6; ++i) out += n2(toDegrees(q(i))) + (i < 5 ? "," : "]");
            return out;
        }

        struct Frame {
            int pose = 0;
            char stage = 'S';              // S = 初期位置、L = 左アームの移動、R = 右アームの移動、M = 測定
            geometry::Joints left, right;
            Real distance = 0.0;           // 左右アームの最短距離
        };

        geometry::Joints lerp(const geometry::Joints& a, const geometry::Joints& b, Real t) { return a + (b - a) * t; }

        std::string utf8Template() {
            return std::string(reinterpret_cast<const char*>(detail::kViewTemplateBytes), detail::kViewTemplateSize);
        }

    } // namespace

    Result<std::string> buildViewHtml(const LoadedProject& project, const ViewOptions& options) {
        const io::SetupConfig& setup = project.setup;
        const domain::Recipe& recipe = project.recipe;
        const geometry::RigGeometry geometry(setup.rig);
        const int K = std::max(1, options.steps_per_move);
        const Real clearance = 2.0 * setup.rig.arm_radius;

        // ---- 1. 全コマの関節角と、左右アームの距離 ----
        std::vector<Frame> frames;
        const geometry::Joints home = geometry::jointsFromDegrees(0, 0, 90, 0, 0, 0);
        auto push = [&](int pose, char stage, const geometry::Joints& l, const geometry::Joints& r) {
            frames.push_back({pose, stage, l, r, geometry.minArmDistance(l, r)});
        };

        geometry::Joints prevL = home, prevR = home;
        push(0, 'S', prevL, prevR);
        for (std::size_t i = 0; i < recipe.poses.size(); ++i) {
            const domain::PoseStep& pose = recipe.poses[i];
            const int pi = static_cast<int>(i);
            if ((pose.left - prevL).norm() > 1e-9) {                 // 左アームが動くときだけ
                for (int k = 1; k <= K; ++k) push(pi, 'L', lerp(prevL, pose.left, static_cast<Real>(k) / K), prevR);
            }
            if ((pose.right - prevR).norm() > 1e-9) {
                for (int k = 1; k <= K; ++k) push(pi, 'R', pose.left, lerp(prevR, pose.right, static_cast<Real>(k) / K));
            }
            push(pi, 'M', pose.left, pose.right);
            prevL = pose.left;
            prevR = pose.right;
        }

        // ---- 2. 事前検査 + 動いている途中の距離 ----
        const PreflightReport report = preflight(geometry, recipe);
        struct Note { std::string sev, msg; };
        std::vector<std::vector<Note>> notes(recipe.poses.size());
        for (const Issue& is : report.issues) {
            notes[static_cast<std::size_t>(is.pose_index)].push_back(
                {is.severity == Severity::Error ? "error" : "warning", is.message});
        }
        int path_problems = 0;
        for (std::size_t i = 0; i < recipe.poses.size(); ++i) {
            Real worst = std::numeric_limits<Real>::max();
            for (const Frame& f : frames) {
                if (f.pose == static_cast<int>(i) && (f.stage == 'L' || f.stage == 'R')) worst = std::min(worst, f.distance);
            }
            if (worst < clearance) {
                ++path_problems;
                notes[i].push_back({"error", std::format("動いている途中で、左右のアームが {:.0f} mm まで近づきます(基準 {:.0f} mm)。"
                                                         "ポーズ間の動きを確認してください", worst, clearance)});
            }
        }

        // ---- 3. 表示のための範囲 ----
        Vec3 lo = Vec3::Constant(std::numeric_limits<Real>::max()), hi = Vec3::Constant(std::numeric_limits<Real>::lowest());
        for (const Frame& f : frames) {
            for (const auto& segs : {geometry.leftArmSegments(f.left), geometry.rightArmSegments(f.right)}) {
                for (const auto& s : segs) {
                    lo = lo.cwiseMin(s.a).cwiseMin(s.b);
                    hi = hi.cwiseMax(s.a).cwiseMax(s.b);
                }
            }
        }
        const Vec3 target = 0.5 * (lo + hi);
        const Real extent = (hi - lo).maxCoeff();

        const Vec3& camera_position = geometry.cameraPosition();
        const Real grid_min = std::floor((std::min({lo.x(), lo.y(), camera_position.x(), camera_position.y(), 0.0}) - 100.0) / 200.0) * 200.0;
        const Real grid_max = std::ceil((std::max({hi.x(), hi.y(), camera_position.x(), camera_position.y()}) + 100.0) / 200.0) * 200.0;

        // カメラの視野(シミュレータのカメラモデル)
        sim::PinholeCameraSetup cs = sim::SimulatedRig::defaultCamera(geometry);
        cs.image_width = setup.imager.width();
        cs.image_height = setup.imager.height();
        cs.pixels_per_mm = setup.sim.camera.pixels_per_mm;
        if (setup.sim.camera.target) cs.target = *setup.sim.camera.target;
        const sim::PinholeCamera camera(cs);
        const Real reach = (cs.target - cs.position).norm();
        std::string frustum = "[";
        {
            const Vec2 corners[4] = {{0, 0}, {Real(cs.image_width), 0}, {Real(cs.image_width), Real(cs.image_height)}, {0, Real(cs.image_height)}};
            for (int i = 0; i < 4; ++i) frustum += vec(cs.position + reach * camera.rayDirection(corners[i])) + (i < 3 ? "," : "]");
        }

        // 試料の板(紙)の四隅(試料座標)
        std::string plate = "[";
        {
            const auto& b = setup.calibration.board;
            const Vec2 board_corners[4] = {{0, 0}, {b.paper_width, 0}, {b.paper_width, b.paper_height}, {0, b.paper_height}};
            const auto mapping = setup.calibration.mapping();
            for (int i = 0; i < 4; ++i) {
                const Vec2 s = mapping ? mapping->inverse(board_corners[i])
                                       : board_corners[i] - Vec2(b.paper_width * 0.5, b.paper_height * 0.5);
                plate += "[" + n1(s.x()) + "," + n1(s.y()) + "]" + (i < 3 ? "," : "]");
            }
        }

        // ---- 4. JSON ----
        std::ostringstream o;
        o << "{\"name\":" << str(recipe.name)
          << ",\"generated\":" << str(options.timestamp.empty() ? "" : [&] {
                 const std::string d = io::spotXmlDatetime(options.timestamp);
                 return d.size() >= 16 ? d.substr(0, 16) : d;
             }())
          << ",\"clearance\":" << n1(clearance)
          << ",\"radius\":" << n1(setup.rig.arm_radius)
          << ",\"view\":{\"dist\":" << n1(extent * 1.9 + 500.0) << ",\"target\":" << vec(target) << "}"
          << ",\"grid\":{\"min\":" << n1(grid_min) << ",\"max\":" << n1(grid_max) << ",\"step\":200}"
          << ",\"limits\":{\"lower\":" << degrees(setup.rig.joint_limits.lower) << ",\"upper\":" << degrees(setup.rig.joint_limits.upper) << "}"
          << ",\"plate\":" << plate
          << ",\"camera\":{\"position\":" << vec(cs.position) << ",\"frustum\":" << frustum << "}";

        o << ",\"spots\":[";
        for (std::size_t i = 0; i < recipe.spots.size(); ++i) {
            const auto& s = recipe.spots[i];
            o << (i ? "," : "") << "{\"id\":" << s.id << ",\"x\":" << n1(s.sample_mm.x()) << ",\"y\":" << n1(s.sample_mm.y()) << "}";
        }
        o << "],\"poses\":[";
        for (std::size_t i = 0; i < recipe.poses.size(); ++i) {
            const auto& pose = recipe.poses[i];
            o << (i ? "," : "") << "{\"id\":" << str(pose.id) << ",\"angles\":[";
            const auto patches = computePoseAngles(geometry, pose, recipe.spots);
            for (std::size_t k = 0; k < patches.size(); ++k) {
                const auto& a = patches[k].angles;
                o << (k ? "," : "") << "[" << n2(toDegrees(a.incidence.theta)) << "," << n2(toDegrees(a.incidence.phi)) << ","
                  << n2(toDegrees(a.observation.theta)) << "," << n2(toDegrees(a.observation.phi)) << "]";
            }
            o << "],\"issues\":[";
            for (std::size_t k = 0; k < notes[i].size(); ++k) {
                o << (k ? "," : "") << "{\"sev\":" << str(notes[i][k].sev) << ",\"msg\":" << str(notes[i][k].msg) << "}";
            }
            o << "]}";
        }
        o << "],\"frames\":[";
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const Frame& f = frames[i];
            const geometry::SurfaceFrame sf = geometry.paperFrame(f.right);
            o << (i ? "," : "") << "{\"p\":" << f.pose << ",\"s\":\"" << f.stage << "\""
              << ",\"l\":" << points(geometry.leftArmSegments(f.left))
              << ",\"r\":" << points(geometry.rightArmSegments(f.right))
              << ",\"a\":[" << n2(sf.xAxis().x()) << "," << n2(sf.xAxis().y()) << "," << n2(sf.xAxis().z()) << ","
                           << n2(sf.yAxis().x()) << "," << n2(sf.yAxis().y()) << "," << n2(sf.yAxis().z()) << ","
                           << n2(sf.zAxis().x()) << "," << n2(sf.zAxis().y()) << "," << n2(sf.zAxis().z()) << "]"
              << ",\"ld\":" << degrees(f.left) << ",\"rd\":" << degrees(f.right)
              << ",\"d\":" << n1(f.distance) << "}";
        }
        o << "]}";

        // ---- 5. ひな型に埋め込む ----
        std::string html = utf8Template();
        const std::string marker = "/*__BRDF_DATA__*/null";
        const auto pos = html.find(marker);
        if (pos == std::string::npos) {
            return fail(ErrorCode::DeviceError, "the view template has no data marker (the build is broken)");
        }
        html.replace(pos, marker.size(), o.str());

        spdlog::info("View: {} pose(s), {} frame(s); {} pose(s) with a path problem", recipe.poses.size(), frames.size(), path_problems);
        return html;
    }

    Result<fs::path> exportView(const fs::path& dir, const ViewOptions& options) {
        const ProjectLayout layout(dir);
        auto project = loadProject(dir);
        if (!project) return tl::unexpected(project.error());

        ViewOptions opt = options;
        if (opt.timestamp.empty()) opt.timestamp = currentTimestamp();

        auto html = buildViewHtml(*project, opt);
        if (!html) return tl::unexpected(html.error());

        auto run_dir = layout.createRunDir(opt.timestamp, "view");
        if (!run_dir) return tl::unexpected(run_dir.error());
        const fs::path file = *run_dir / "view.html";
        std::ofstream ofs(file, std::ios::binary);
        if (!ofs) return fail(ErrorCode::DeviceError, "cannot write " + file.string());
        ofs << *html;
        spdlog::info("Wrote {} ({} KB)", file.string(), html->size() / 1024);
        return file;
    }

} // namespace brdf::app
