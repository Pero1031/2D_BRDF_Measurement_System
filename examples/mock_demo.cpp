// モックだけで、1 ポーズの模擬測定を最後まで通すデモ。
//
//   アーム 2 台を接続 → ポーズへ移動 → 2D 分光放射計で撮影 → 点ごとの角度とスペクトルを表示
//
// 実機はまったく使わない。光源・試料・カメラの位置から入射角・観察角を計算し、
// 既知の BRDF(ここでは緑色の拡散面)から分光値を作っている。

#include <cstdlib>
#include <memory>

#include <spdlog/spdlog.h>

#include "brdf/core/MathUtils.hpp"
#include "brdf/optics/Angles.hpp"
#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/MockSpectralImager.hpp"
#include "brdf/sim/SimulatedRig.hpp"

using namespace brdf;

namespace {

    // 失敗したらメッセージを出して終了する(デモなので簡単に)。
    void check(const Status& s, const char* what) {
        if (!s) {
            spdlog::error("{} failed [{}]: {}", what, toString(s.error().code), s.error().message);
            std::exit(1);
        }
    }

} // namespace

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== Simulated measurement demo ===");

    // 1. 装置の世界(左右アーム、カメラ)と、そこで動くモックを用意する
    auto rig = std::make_shared<sim::SimulatedRig>();
    sim::MockRobotArm light_arm("LightArm", rig, sim::ArmSide::Left);
    sim::MockRobotArm sample_arm("SampleArm", rig, sim::ArmSide::Right);

    // 測りたい試料: 緑色の拡散面(550 nm にピーク)
    auto brdf_model = std::make_shared<sim::LambertBrdf>(sim::gaussianCurve(550.0, 60.0, 0.8, 0.05));
    sim::MockSpectralImager imager(rig, brdf_model);

    check(light_arm.connect(), "connect LightArm");
    check(sample_arm.connect(), "connect SampleArm");
    check(imager.connect(), "connect imager");

    // 2. ポーズへ移動(旧システムの測定で最初に使われるポーズ)
    check(light_arm.moveJ(geometry::jointsFromDegrees(45, 0, 90, 0, 0, -30), 0.5), "move LightArm");
    check(sample_arm.moveJ(geometry::jointsFromDegrees(-45, 0, 90, 0, 0, 0), 0.5), "move SampleArm");

    // 3. 画像の中心とその周りの 4 点を撮影する
    driver::CaptureRequest request;
    request.measure_distance_mm = (rig->sampleFrame().origin - rig->camera().position()).norm();
    request.integration_time_ms = 100.0;
    const Vec2 centre(688, 512);
    const Vec2 offsets[] = {{0, 0}, {-150, 0}, {150, 0}, {0, -120}, {0, 120}};
    int id = 1;
    for (const Vec2& o : offsets) request.spots.push_back({id++, centre + o});

    auto result = imager.capture(request);
    if (!result) {
        spdlog::error("capture failed [{}]: {}", toString(result.error().code), result.error().message);
        return 1;
    }

    // 4. 点ごとに、角度とスペクトルのピークを表示する
    //    (角度はここで独立に計算し直したもの。測定の流れでは角度は別に記録しておく)
    const auto frame = rig->sampleFrame();
    const Vec3 light = rig->lightPosition();
    const Vec3 cam = rig->camera().position();

    spdlog::info("measure distance: {:.1f} mm", request.measure_distance_mm);
    spdlog::info("{:>3} {:>14} {:>9} {:>9} {:>9} {:>9} {:>12}", "id", "pixel", "theta_i", "phi_i", "theta_o",
                 "phi_o", "peak value");

    for (const driver::SpotSpectrum& spot : result->spots) {
        const Vec3 ray = rig->camera().rayDirection(spot.pixel);
        const Real t = frame.zAxis().dot(frame.origin - cam) / frame.zAxis().dot(ray);
        const auto a = optics::computeAngles(frame, cam + t * ray, light, cam);

        Eigen::Index peak = 0;
        const Real peak_value = spot.values.maxCoeff(&peak);
        spdlog::info("{:>3} ({:>5.0f},{:>5.0f}) {:>9.2f} {:>9.2f} {:>9.2f} {:>9.2f} {:>12.4g} @{:.0f}nm",
                     spot.id, spot.pixel.x(), spot.pixel.y(),
                     toDegrees(a.incidence.theta), toDegrees(a.incidence.phi),
                     toDegrees(a.observation.theta), toDegrees(a.observation.phi),
                     peak_value, result->wavelengths(peak));
    }

    // 5. エラーの例: 可動域を超える指令は拒否される
    const Status bad = sample_arm.moveJ(geometry::jointsFromDegrees(0, 0, 200, 0, 0, 0), 0.5);
    if (!bad) {
        spdlog::info("Expected rejection [{}]: {}", toString(bad.error().code), bad.error().message);
    }

    imager.disconnect();
    sample_arm.disconnect();
    light_arm.disconnect();
    spdlog::info("=== Demo finished ===");
    return 0;
}
