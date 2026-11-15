/*
* @file src/io/SetupConfig.cpp
* @brief setup.toml: the physical layout of the rig and the device mode.
*/

#include "brdf/io/SetupConfig.hpp"

#include <algorithm>
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
            const auto& im = c.imager;
            if (im.device != "SR5100" && im.device != "SR5000") {
                return fail(ErrorCode::InvalidArgument, "setup: [imager] device must be \"SR5100\" or \"SR5000\"");
            }
            if (im.binning != "BINNING_TYPE1" && im.binning != "BINNING_TYPE2" && im.binning != "BINNING_TYPE4") {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [imager] binning must be \"BINNING_TYPE1\", \"BINNING_TYPE2\" or \"BINNING_TYPE4\"");
            }
            if (im.lens != "WIDE" && im.lens != "STANDARD" && im.lens != "TELE" && im.lens != "MACRO") {
                return fail(ErrorCode::InvalidArgument, "setup: [imager] lens must be \"WIDE\", \"STANDARD\", \"TELE\" or \"MACRO\"");
            }
            if (im.width() <= 0 || im.height() <= 0) {
                return fail(ErrorCode::InvalidArgument, "setup: [imager] image_width and image_height must be positive");
            }
            if (!positive(c.sim.camera.pixels_per_mm)) {
                return fail(ErrorCode::InvalidArgument, "setup: [sim.camera] pixels_per_mm must be positive");
            }
            if (!positive(c.sim.imager.light_intensity) || !positive(c.sim.imager.focus_tolerance_mm)
                || c.sim.imager.noise_relative < 0.0) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [sim.imager] light_intensity and focus_tolerance_mm must be positive, noise_relative must not be negative");
            }
            const auto& cal = c.calibration;
            if (cal.projector != "camera-model" && cal.projector != "board") {
                return fail(ErrorCode::InvalidArgument, "setup: [calibration] projector must be \"camera-model\" or \"board\"");
            }
            if (cal.projector == "camera-model" && c.mode == DeviceMode::Real) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [calibration] projector = \"camera-model\" is only for the simulator; real devices need \"board\"");
            }
            if (!positive(cal.max_rms_px)) {
                return fail(ErrorCode::InvalidArgument, "setup: [calibration] max_rms_px must be positive");
            }
            const auto& b = cal.board;
            if (b.columns < 3 || b.rows < 3) {
                return fail(ErrorCode::InvalidArgument, "setup: [calibration.board] columns and rows must be at least 3");
            }
            if (!positive(b.pitch_x) || !positive(b.pitch_y) || !positive(b.paper_width) || !positive(b.paper_height)
                || !positive(b.circle_diameter)) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [calibration.board] pitch, paper size and circle_diameter must be positive");
            }
            if (b.circle_diameter >= std::min(b.pitch_x, b.pitch_y)) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [calibration.board] circle_diameter must be smaller than the pitch");
            }
            const Real last_x = b.first_x + (b.columns - 1) * b.pitch_x;
            const Real last_y = b.first_y + (b.rows - 1) * b.pitch_y;
            if (b.first_x < 0.0 || b.first_y < 0.0 || last_x > b.paper_width || last_y > b.paper_height) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [calibration.board] the circles do not fit on the paper (check first_x, first_y, pitch, columns, rows)");
            }
            if (auto m = cal.mapping(); !m) {
                return fail(ErrorCode::InvalidArgument, "setup: [calibration.board] " + m.error().message);
            }
            if (!positive(c.spot_xml.spot_size)) {
                return fail(ErrorCode::InvalidArgument, "setup: [spot_xml] spot_size must be positive");
            }
            const auto& an = c.analysis;
            if (!positive(an.white_reflectance) || an.white_reflectance > 2.0) {
                return fail(ErrorCode::InvalidArgument, "setup: [analysis] white_reflectance must be in (0, 2]");
            }
            if (!(an.min_wavelength_nm < an.max_wavelength_nm) || !positive(an.min_wavelength_nm)) {
                return fail(ErrorCode::InvalidArgument,
                            "setup: [analysis] min_wavelength_nm must be positive and smaller than max_wavelength_nm");
            }
            if (!positive(c.sim.white_reflectance) || c.sim.white_reflectance > 2.0) {
                return fail(ErrorCode::InvalidArgument, "setup: [sim.white] reflectance must be in (0, 2]");
            }
            if (c.sim.sample.model != "lambert" && c.sim.sample.model != "phong") {
                return fail(ErrorCode::InvalidArgument, "setup: [sim.sample] model must be \"lambert\" or \"phong\"");
            }
            if (!positive(c.sim.sample.width_nm) || !positive(c.sim.sample.shininess)) {
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

        // --- [calibration], [calibration.board] ---
        {
            auto cal = top.child("calibration");
            if (!cal) return tl::unexpected(cal.error());
            BRDF_TRY(cal->string("projector", c.calibration.projector));
            BRDF_TRY(cal->number("max_rms_px", c.calibration.max_rms_px));

            auto board = cal->child("board");
            if (!board) return tl::unexpected(board.error());
            auto& b = c.calibration.board;
            BRDF_TRY(board->integer("columns", b.columns));
            BRDF_TRY(board->integer("rows", b.rows));
            BRDF_TRY(board->number("pitch_x", b.pitch_x));
            BRDF_TRY(board->number("pitch_y", b.pitch_y));
            BRDF_TRY(board->number("first_x", b.first_x));
            BRDF_TRY(board->number("first_y", b.first_y));
            BRDF_TRY(board->number("paper_width", b.paper_width));
            BRDF_TRY(board->number("paper_height", b.paper_height));
            BRDF_TRY(board->number("circle_diameter", b.circle_diameter));
            BRDF_TRY(board->string("board_u", c.calibration.board_u));
            BRDF_TRY(board->string("board_v", c.calibration.board_v));
            if (board->has("sample_origin")) {
                Vec2 sample_origin = Vec2::Zero();
                BRDF_TRY(board->vec2("sample_origin", sample_origin));
                c.calibration.sample_origin = sample_origin;
            }
            BRDF_TRY(board->finish());
            BRDF_TRY(cal->finish());
        }

        // --- [imager] ---
        {
            auto s = top.child("imager");
            if (!s) return tl::unexpected(s.error());
            BRDF_TRY(s->string("device", c.imager.device));
            BRDF_TRY(s->string("binning", c.imager.binning));
            BRDF_TRY(s->string("lens", c.imager.lens));
            if (s->has("image_width")) {
                int v = 0;
                BRDF_TRY(s->integer("image_width", v));
                c.imager.image_width = v;
            }
            if (s->has("image_height")) {
                int v = 0;
                BRDF_TRY(s->integer("image_height", v));
                c.imager.image_height = v;
            }
            BRDF_TRY(s->finish());
        }

        // --- [analysis] ---
        {
            auto s = top.child("analysis");
            if (!s) return tl::unexpected(s.error());
            BRDF_TRY(s->number("white_reflectance", c.analysis.white_reflectance));
            BRDF_TRY(s->number("min_wavelength_nm", c.analysis.min_wavelength_nm));
            BRDF_TRY(s->number("max_wavelength_nm", c.analysis.max_wavelength_nm));
            BRDF_TRY(s->boolean("irradiance_correction", c.analysis.irradiance_correction));
            BRDF_TRY(s->finish());
        }

        // --- [spot_xml] ---
        {
            auto s = top.child("spot_xml");
            if (!s) return tl::unexpected(s.error());
            BRDF_TRY(s->number("spot_size", c.spot_xml.spot_size));
            BRDF_TRY(s->finish());
        }

        // --- [sim.*] ---
        {
            auto sim = top.child("sim");
            if (!sim) return tl::unexpected(sim.error());

            auto camera = sim->child("camera");
            if (!camera) return tl::unexpected(camera.error());
            BRDF_TRY(camera->number("pixels_per_mm", c.sim.camera.pixels_per_mm));
            if (camera->has("target")) {
                Vec3 target = Vec3::Zero();
                BRDF_TRY(camera->vec3("target", target));
                c.sim.camera.target = target;
            }
            BRDF_TRY(camera->finish());

            auto imager = sim->child("imager");
            if (!imager) return tl::unexpected(imager.error());
            auto& si = c.sim.imager;
            BRDF_TRY(imager->number("light_intensity", si.light_intensity));
            BRDF_TRY(imager->number("noise_relative", si.noise_relative));
            int seed = static_cast<int>(si.seed);
            BRDF_TRY(imager->integer("seed", seed));
            si.seed = static_cast<std::uint32_t>(seed);
            BRDF_TRY(imager->number("focus_tolerance_mm", si.focus_tolerance_mm));
            BRDF_TRY(imager->finish());

            auto sample = sim->child("sample");
            if (!sample) return tl::unexpected(sample.error());
            auto& ss = c.sim.sample;
            BRDF_TRY(sample->string("model", ss.model));
            BRDF_TRY(sample->number("peak_nm", ss.peak_nm));
            BRDF_TRY(sample->number("width_nm", ss.width_nm));
            BRDF_TRY(sample->number("peak", ss.peak));
            BRDF_TRY(sample->number("base", ss.base));
            BRDF_TRY(sample->number("specular", ss.specular));
            BRDF_TRY(sample->number("shininess", ss.shininess));
            auto white = sim->child("white");
            if (!white) return tl::unexpected(white.error());
            BRDF_TRY(white->number("reflectance", c.sim.white_reflectance));
            BRDF_TRY(white->finish());
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

        const auto& cal = c.calibration;
        const auto& bd = cal.board;
        o << "[calibration]\n"
          << "projector = " << quote(cal.projector)
          << "   # \"camera-model\" = シミュレータのカメラで直接(sim のみ)、\"board\" = キャリブボードのホモグラフィ(実機)\n"
          << "max_rms_px = " << num(cal.max_rms_px) << "   # 校正の再投影誤差(rms)の許容値\n\n";

        o << "[calibration.board]   # キャリブボード(黒円の格子)。座標は紙の角が原点、mm\n"
          << "columns = " << bd.columns << "\n"
          << "rows = " << bd.rows << "\n"
          << "pitch_x = " << num(bd.pitch_x) << "\n"
          << "pitch_y = " << num(bd.pitch_y) << "\n"
          << "first_x = " << num(bd.first_x) << "   # 最初の円の中心(座標が最小の円)\n"
          << "first_y = " << num(bd.first_y) << "\n"
          << "paper_width = " << num(bd.paper_width) << "\n"
          << "paper_height = " << num(bd.paper_height) << "\n"
          << "circle_diameter = " << num(bd.circle_diameter) << "\n";
        o << "board_u = " << quote(cal.board_u) << "   # ボードを正面から見て右向きの軸は、試料座標のどの軸か(\"+x\" \"-x\" \"+y\" \"-y\")\n"
          << "board_v = " << quote(cal.board_v) << "   # 同、下向きの軸\n";
        if (cal.sample_origin) {
            o << "sample_origin = [" << num(cal.sample_origin->x()) << ", " << num(cal.sample_origin->y()) << "]\n";
        } else {
            o << "# sample_origin = [225.0, 151.0]   # 試料座標の原点がボード座標のどこか。省略すると紙の中心\n";
        }
        o << "\n";

        o << "[imager]   # 2D 分光放射計。画像の大きさは機種とビニングで決まる\n"
          << "device = " << quote(c.imager.device) << "   # \"SR5100\"(最新、2448x2048) または \"SR5000\"(1376x1024)\n"
          << "binning = " << quote(c.imager.binning) << "   # \"BINNING_TYPE1\" / \"BINNING_TYPE2\" / \"BINNING_TYPE4\"\n"
          << "lens = " << quote(c.imager.lens) << "   # \"WIDE\" / \"STANDARD\" / \"TELE\" / \"MACRO\"\n";
        if (c.imager.image_width) o << "image_width = " << *c.imager.image_width << "\n";
        else o << "# image_width = " << c.imager.width() << "   # 省略すると、機種とビニングから決まる\n";
        if (c.imager.image_height) o << "image_height = " << *c.imager.image_height << "\n";
        else o << "# image_height = " << c.imager.height() << "\n";
        o << "\n";

        o << "[analysis]   # brdf analyze の設定\n"
          << "white_reflectance = " << num(c.analysis.white_reflectance) << "   # 標準拡散反射板の反射率(BRDF = 放射輝度率 x 反射率 / pi)\n"
          << "min_wavelength_nm = " << num(c.analysis.min_wavelength_nm) << "   # 出力する波長の範囲。LED の出力が足りない 380 nm 付近と 770 nm 付近は信頼性が低い\n"
          << "max_wavelength_nm = " << num(c.analysis.max_wavelength_nm) << "\n"
          << "irradiance_correction = " << (c.analysis.irradiance_correction ? "true" : "false")
          << "   # 試料の点と反射板の基準点の照度の違い(cos θi / d²)を補正する。false は旧システムと同じ\n\n";

        o << "[spot_xml]   # brdf spots が作る、スポット定義ファイル(ROI の大きさとビニングは [imager] の値)\n"
          << "spot_size = " << num(c.spot_xml.spot_size) << "   # 各スポットの一辺 [pixel]\n\n";

        o << "# ---- ここから下はシミュレーション(mode = \"sim\")のときだけ使う設定 ----\n\n";

        o << "[sim.camera]\n"
          << "pixels_per_mm = " << num(c.sim.camera.pixels_per_mm) << "   # 注視点での解像度(画像の大きさは [imager])\n";
        if (c.sim.camera.target) {
            o << "target = " << arr3(*c.sim.camera.target) << "\n";
        } else {
            o << "# target = [0.0, 0.0, 0.0]   # 省略すると、典型的な測定姿勢での試料位置を向く\n";
        }
        o << "\n";

        o << "[sim.imager]\n"
          << "light_intensity = " << num(c.sim.imager.light_intensity) << "\n"
          << "noise_relative = " << num(c.sim.imager.noise_relative) << "   # 0 ならノイズなし\n"
          << "seed = " << c.sim.imager.seed << "\n"
          << "focus_tolerance_mm = " << num(c.sim.imager.focus_tolerance_mm) << "\n\n";

        o << "[sim.sample]   # シミュレーションで測られる試料\n"
          << "model = " << quote(c.sim.sample.model) << "   # \"lambert\" または \"phong\"\n"
          << "peak_nm = " << num(c.sim.sample.peak_nm) << "\n"
          << "width_nm = " << num(c.sim.sample.width_nm) << "\n"
          << "peak = " << num(c.sim.sample.peak) << "\n"
          << "base = " << num(c.sim.sample.base) << "\n"
          << "specular = " << num(c.sim.sample.specular) << "   # phong のみ\n"
          << "shininess = " << num(c.sim.sample.shininess) << "   # phong のみ\n\n";

        o << "[sim.white]   # シミュレータの標準拡散反射板(brdf run --kind white で測られる)\n"
          << "reflectance = " << num(c.sim.white_reflectance) << "\n";

        return o.str();
    }

} // namespace brdf::io
