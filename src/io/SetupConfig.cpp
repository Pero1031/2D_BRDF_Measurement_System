/*
* @file src/io/SetupConfig.cpp
* @brief setup.toml: the physical layout of the rig and the device mode.
*/

#include "brdf/io/SetupConfig.hpp"

#include <fstream>
#include <sstream>

#include "TomlReader.hpp"
#include "brdf/core/MathUtils.hpp"

namespace brdf::io {

    using namespace detail;

    namespace {

        Status readJointArray(Section& s, std::string_view key, geometry::Joints& out) {
            std::array<Real, 6> deg{};
            for (int i = 0; i < 6; ++i) deg[i] = toDegrees(out(i));
            BRDF_TRY(s.numbers(key, deg));
            out = geometry::jointsFromDegrees(deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);
            return {};
        }

        Status validate(const SetupConfig& c) {
            const auto& l = c.rig.links;
            auto positive = [](Real v) { return v > 0.0; };

            if (!positive(l.base_height) || !positive(l.upper_arm) || !positive(l.elbow_offset)
                || !positive(l.forearm) || !positive(l.wrist)) {
                return fail(ErrorCode::InvalidArgument, "setup: [arm.links] every link length must be positive");
            }
            if (!positive(c.rig.arm_radius)) {
                return fail(ErrorCode::InvalidArgument, "setup: [arm] radius_mm must be positive");
            }
            for (int i = 0; i < 6; ++i) {
                if (!(c.rig.joint_limits.lower(i) < c.rig.joint_limits.upper(i))) {
                    return fail(ErrorCode::InvalidArgument,
                                "setup: [arm.limits] lower_deg[" + std::to_string(i + 1) + "] must be smaller than upper_deg");
                }
            }
            if (c.rig.sample_surface_offset < 0.0) {
                return fail(ErrorCode::InvalidArgument, "setup: [rig] sample_surface_offset must not be negative");
            }
            if (c.camera.image_width <= 0 || c.camera.image_height <= 0 || !positive(c.camera.pixels_per_mm)) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [sim.camera] image_width, image_height and pixels_per_mm must be positive");
            }
            if (!positive(c.imager.light_intensity) || !positive(c.imager.focus_tolerance_mm) || c.imager.noise_relative < 0.0) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [sim.imager] light_intensity and focus_tolerance_mm must be positive, noise_relative must not be negative");
            }
            if (c.sample.model != "lambert" && c.sample.model != "phong") {
                return fail(ErrorCode::InvalidArgument, "setup: [sim.sample] model must be \"lambert\" or \"phong\"");
            }
            if (!positive(c.sample.width_nm) || !positive(c.sample.shininess)) {
                return fail(ErrorCode::InvalidArgument, "setup: [sim.sample] width_nm and shininess must be positive");
            }
            return {};
        }

    } // namespace

    Result<SetupConfig> parseSetup(std::string_view text, std::string_view origin) {
        auto parsed = parseToml(text, origin);
        if (!parsed) return tl::unexpected(parsed.error());

        SetupConfig c;
        Section top(&*parsed, std::string(origin), "");

        // --- [devices] ---
        {
            auto devices = top.child("devices");
            if (!devices) return tl::unexpected(devices.error());
            std::string mode = toString(c.mode);
            BRDF_TRY(devices->string("mode", mode));
            if (mode == "sim")       c.mode = DeviceMode::Sim;
            else if (mode == "real") c.mode = DeviceMode::Real;
            else return devices->bad("mode", "must be \"sim\" or \"real\"");
            BRDF_TRY(devices->finish());
        }

        // --- [arm], [arm.links], [arm.limits] ---
        {
            auto arm = top.child("arm");
            if (!arm) return tl::unexpected(arm.error());
            BRDF_TRY(arm->number("radius_mm", c.rig.arm_radius));

            auto links = arm->child("links");
            if (!links) return tl::unexpected(links.error());
            auto& l = c.rig.links;
            BRDF_TRY(links->number("base_height", l.base_height));
            BRDF_TRY(links->number("upper_arm", l.upper_arm));
            BRDF_TRY(links->number("elbow_offset", l.elbow_offset));
            BRDF_TRY(links->number("forearm", l.forearm));
            BRDF_TRY(links->number("wrist", l.wrist));
            Real fixed_deg = toDegrees(l.elbow_fixed_angle);
            BRDF_TRY(links->number("elbow_fixed_angle_deg", fixed_deg));
            l.elbow_fixed_angle = toRadians(fixed_deg);
            BRDF_TRY(links->finish());

            auto limits = arm->child("limits");
            if (!limits) return tl::unexpected(limits.error());
            BRDF_TRY(readJointArray(*limits, "lower_deg", c.rig.joint_limits.lower));
            BRDF_TRY(readJointArray(*limits, "upper_deg", c.rig.joint_limits.upper));
            BRDF_TRY(limits->finish());

            BRDF_TRY(arm->finish());
        }

        // --- [rig] ---
        {
            auto rig = top.child("rig");
            if (!rig) return tl::unexpected(rig.error());
            BRDF_TRY(rig->vec3("left_base", c.rig.left_base_position));
            BRDF_TRY(rig->vec3("right_base", c.rig.right_base_position));
            BRDF_TRY(rig->vec3("light_offset", c.rig.light_offset));
            BRDF_TRY(rig->vec3("camera_position", c.rig.camera_position));
            BRDF_TRY(rig->number("sample_surface_offset", c.rig.sample_surface_offset));
            BRDF_TRY(rig->finish());
        }

        // --- [sim.*] ---
        {
            auto sim = top.child("sim");
            if (!sim) return tl::unexpected(sim.error());

            auto camera = sim->child("camera");
            if (!camera) return tl::unexpected(camera.error());
            BRDF_TRY(camera->integer("image_width", c.camera.image_width));
            BRDF_TRY(camera->integer("image_height", c.camera.image_height));
            BRDF_TRY(camera->number("pixels_per_mm", c.camera.pixels_per_mm));
            if (camera->has("target")) {
                Vec3 target = Vec3::Zero();
                BRDF_TRY(camera->vec3("target", target));
                c.camera.target = target;
            }
            BRDF_TRY(camera->finish());

            auto imager = sim->child("imager");
            if (!imager) return tl::unexpected(imager.error());
            BRDF_TRY(imager->number("light_intensity", c.imager.light_intensity));
            BRDF_TRY(imager->number("noise_relative", c.imager.noise_relative));
            int seed = static_cast<int>(c.imager.seed);
            BRDF_TRY(imager->integer("seed", seed));
            c.imager.seed = static_cast<std::uint32_t>(seed);
            BRDF_TRY(imager->number("focus_tolerance_mm", c.imager.focus_tolerance_mm));
            BRDF_TRY(imager->finish());

            auto sample = sim->child("sample");
            if (!sample) return tl::unexpected(sample.error());
            BRDF_TRY(sample->string("model", c.sample.model));
            BRDF_TRY(sample->number("peak_nm", c.sample.peak_nm));
            BRDF_TRY(sample->number("width_nm", c.sample.width_nm));
            BRDF_TRY(sample->number("peak", c.sample.peak));
            BRDF_TRY(sample->number("base", c.sample.base));
            BRDF_TRY(sample->number("specular", c.sample.specular));
            BRDF_TRY(sample->number("shininess", c.sample.shininess));
            BRDF_TRY(sample->finish());

            BRDF_TRY(sim->finish());
        }

        BRDF_TRY(top.finish());
        BRDF_TRY(validate(c));
        return c;
    }

    Result<SetupConfig> loadSetup(const std::filesystem::path& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) {
            return fail(ErrorCode::InvalidArgument, "cannot open " + path.string());
        }
        std::stringstream ss;
        ss << ifs.rdbuf();
        return parseSetup(ss.str(), path.filename().string());
    }

    std::string toToml(const SetupConfig& c) {
        std::ostringstream o;
        const auto& l = c.rig.links;
        auto deg = [](const geometry::Joints& q) {
            std::string out = "[";
            for (int i = 0; i < 6; ++i) {
                out += num(std::stod(std::format("{:.10g}", toDegrees(q(i)))));
                out += i < 5 ? ", " : "]";
            }
            return out;
        };

        o << "# setup.toml - 装置の配置と、使うデバイス\n"
          << "# 長さは mm、角度は度。\n"
          << "# 注意: カメラ位置(camera_position)は測定日ごとに合わせ直す値です。\n"
          << "#       測定結果と一緒に、この設定のコピーが保存されます。\n\n";

        o << "[devices]\n"
          << "mode = " << quote(toString(c.mode)) << "   # \"sim\" = シミュレーション、\"real\" = 実機(未対応)\n\n";

        o << "[arm]\n"
          << "radius_mm = " << num(c.rig.arm_radius) << "   # 干渉チェックで、アームを太さのある棒として扱うときの半径\n\n";

        o << "[arm.links]   # DENSO VP-6242 の外形図の寸法\n"
          << "base_height = " << num(l.base_height) << "\n"
          << "upper_arm = " << num(l.upper_arm) << "\n"
          << "elbow_offset = " << num(l.elbow_offset) << "\n"
          << "forearm = " << num(l.forearm) << "\n"
          << "wrist = " << num(l.wrist) << "\n"
          << "elbow_fixed_angle_deg = " << num(std::stod(std::format("{:.10g}", toDegrees(l.elbow_fixed_angle)))) << "\n\n";

        o << "[arm.limits]   # VP-6242 のソフトウェアリミット(出荷時設定値)。J1〜J6 の順\n"
          << "lower_deg = " << deg(c.rig.joint_limits.lower) << "\n"
          << "upper_deg = " << deg(c.rig.joint_limits.upper) << "\n\n";

        o << "[rig]\n"
          << "left_base = " << arr3(c.rig.left_base_position) << "   # 左アーム(光源側)の土台位置 = 世界座標の原点\n"
          << "right_base = " << arr3(c.rig.right_base_position) << "   # 右アーム(試料側)の土台位置\n"
          << "light_offset = " << arr3(c.rig.light_offset) << "   # 左アーム先端の座標系で見た光源の位置\n"
          << "camera_position = " << arr3(c.rig.camera_position) << "   # 2D分光放射計の位置\n"
          << "sample_surface_offset = " << num(c.rig.sample_surface_offset) << "   # 右アーム先端から試料面まで(アクリル 5 + 紙 1)\n\n";

        o << "# ---- ここから下はシミュレーション(mode = \"sim\")のときだけ使う設定 ----\n\n";

        o << "[sim.camera]\n"
          << "image_width = " << c.camera.image_width << "\n"
          << "image_height = " << c.camera.image_height << "\n"
          << "pixels_per_mm = " << num(c.camera.pixels_per_mm) << "\n";
        if (c.camera.target) {
            o << "target = " << arr3(*c.camera.target) << "\n";
        } else {
            o << "# target = [0.0, 0.0, 0.0]   # 省略すると、典型的な測定姿勢での試料位置を向く\n";
        }
        o << "\n";

        o << "[sim.imager]\n"
          << "light_intensity = " << num(c.imager.light_intensity) << "\n"
          << "noise_relative = " << num(c.imager.noise_relative) << "   # 0 ならノイズなし\n"
          << "seed = " << c.imager.seed << "\n"
          << "focus_tolerance_mm = " << num(c.imager.focus_tolerance_mm) << "\n\n";

        o << "[sim.sample]   # シミュレーションで測られる試料\n"
          << "model = " << quote(c.sample.model) << "   # \"lambert\" または \"phong\"\n"
          << "peak_nm = " << num(c.sample.peak_nm) << "\n"
          << "width_nm = " << num(c.sample.width_nm) << "\n"
          << "peak = " << num(c.sample.peak) << "\n"
          << "base = " << num(c.sample.base) << "\n"
          << "specular = " << num(c.sample.specular) << "   # phong のみ\n"
          << "shininess = " << num(c.sample.shininess) << "   # phong のみ\n";

        return o.str();
    }

} // namespace brdf::io
