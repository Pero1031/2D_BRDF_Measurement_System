/*
* @file src/sim/MockSpectralImager.cpp
* @brief Simulated 2D spectroradiometer that "measures" a known BRDF.
*/

#include "brdf/sim/MockSpectralImager.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <utility>

#include "brdf/optics/Angles.hpp"

namespace brdf::sim {

    MockSpectralImager::MockSpectralImager(std::shared_ptr<const SimulatedRig> rig,
                                           std::shared_ptr<const IBrdfModel> brdf,
                                           MockImagerOptions options)
        : rig_(std::move(rig)), brdf_(std::move(brdf)), options_(options) {}

    Status MockSpectralImager::connect() {
        connected_ = true;
        spdlog::info("[Imager] Connected (mock).");
        return {};
    }

    void MockSpectralImager::disconnect() {
        connected_ = false;
        spdlog::info("[Imager] Disconnected.");
    }

    Result<GrayImage> MockSpectralImager::captureImage(const driver::ImageRequest& request) {
        if (!connected_) return fail(ErrorCode::NotConnected, "imager: not connected");
        if (!board_) {
            return fail(ErrorCode::InvalidArgument,
                        "imager: no calibration board is set up in the simulator (see [calibration.board] in setup.toml)");
        }
        if (request.integration_time_ms < 0.0) {
            return fail(ErrorCode::InvalidArgument, "imager: integration time must not be negative (0 = automatic)");
        }

        const geometry::SurfaceFrame sample = rig_->sampleFrame();
        const PinholeCamera& camera = rig_->camera();
        const Vec3& cam_pos = camera.position();

        const Real actual_distance = (sample.origin - cam_pos).norm();
        if (std::abs(request.measure_distance_mm - actual_distance) > options_.focus_tolerance_mm) {
            std::ostringstream msg;
            msg << "imager: measure distance " << request.measure_distance_mm
                << " mm does not match the actual distance " << actual_distance << " mm";
            return fail(ErrorCode::FocusError, msg.str());
        }

        const auto& layout = board_->layout;
        const Real radius = layout.circle_diameter * 0.5;
        const Real denom_scale = sample.zAxis().dot(sample.origin - cam_pos);

        const int w = camera.setup().image_width;
        const int h = camera.setup().image_height;
        GrayImage image(w, h, 40);   // 暗い背景

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const Vec3 ray = camera.rayDirection(Vec2(x + 0.5, y + 0.5));
                const Real denom = sample.zAxis().dot(ray);
                if (denom == 0.0) continue;
                const Real t = denom_scale / denom;
                if (t <= 0.0) continue;

                // 試料面の座標 → ボード座標
                const Vec3 d = cam_pos + t * ray - sample.origin;
                const Vec2 b = board_->mapping.apply(Vec2(d.dot(sample.xAxis()), d.dot(sample.yAxis())));
                if (b.x() < 0.0 || b.y() < 0.0 || b.x() > layout.paper_width || b.y() > layout.paper_height) continue;

                std::uint8_t v = 210;   // 紙
                const int i = static_cast<int>(std::lround((b.x() - layout.first_x) / layout.pitch_x));
                const int j = static_cast<int>(std::lround((b.y() - layout.first_y) / layout.pitch_y));
                if (i >= 0 && i < layout.columns && j >= 0 && j < layout.rows) {
                    const Vec2 centre(layout.first_x + i * layout.pitch_x, layout.first_y + j * layout.pitch_y);
                    if ((b - centre).norm() < radius) v = 20;   // 黒円
                }
                image.at(x, y) = v;
            }
        }
        return image;
    }

    Result<driver::CaptureResult> MockSpectralImager::capture(const driver::CaptureRequest& request) {
        if (!connected_) {
            return fail(ErrorCode::NotConnected, "imager: not connected");
        }

        // --- 要求の検査 ---
        if (request.spots.empty()) {
            return fail(ErrorCode::InvalidArgument, "imager: no spots requested");
        }
        if (request.integration_time_ms < 0.0) {
            return fail(ErrorCode::InvalidArgument, "imager: integration time must not be negative (0 = automatic)");
        }
        const auto& wr = request.wavelengths;
        if (!(wr.step > 0.0) || !(wr.end >= wr.start)) {
            return fail(ErrorCode::InvalidArgument, "imager: invalid wavelength range");
        }

        // --- いまの幾何 ---
        const geometry::SurfaceFrame sample = rig_->sampleFrame();
        const Vec3 light = rig_->lightPosition();
        const PinholeCamera& camera = rig_->camera();
        const Vec3& cam_pos = camera.position();

        // ピント: 要求された距離が、カメラ〜試料の実際の距離と合っているか
        const Real actual_distance = (sample.origin - cam_pos).norm();
        if (std::abs(request.measure_distance_mm - actual_distance) > options_.focus_tolerance_mm) {
            std::ostringstream msg;
            msg << "imager: measure distance " << request.measure_distance_mm
                << " mm does not match the actual distance " << actual_distance << " mm";
            return fail(ErrorCode::FocusError, msg.str());
        }

        // --- 波長 ---
        driver::CaptureResult result;
        const int n = wr.count();
        result.wavelengths.resize(n);
        for (int i = 0; i < n; ++i) result.wavelengths(i) = wr.start + wr.step * i;
        // 自動(0)のとき、シミュレータは 100 ms を使ったことにする(放射輝度は積分時間によらない)
        result.integration_time_ms = request.integration_time_ms > 0.0 ? request.integration_time_ms : 100.0;
        result.measure_distance_mm = request.measure_distance_mm;

        std::mt19937 rng(options_.seed + capture_count_++);
        std::normal_distribution<Real> gauss(0.0, 1.0);

        // --- 点ごとの計算 ---
        for (const driver::SpotRequest& spot : request.spots) {
            if (!camera.inImage(spot.pixel)) {
                std::ostringstream msg;
                msg << "imager: spot " << spot.id << " (" << spot.pixel.x() << ", " << spot.pixel.y()
                    << ") is outside the image";
                return fail(ErrorCode::OutOfRange, msg.str());
            }

            // 画像上の位置 → 光線 → 試料面との交点
            const Vec3 ray = camera.rayDirection(spot.pixel);
            const Real denom = sample.zAxis().dot(ray);
            const Real t = denom != 0.0 ? sample.zAxis().dot(sample.origin - cam_pos) / denom : -1.0;
            if (t <= 0.0) {
                std::ostringstream msg;
                msg << "imager: spot " << spot.id << " does not hit the sample plane";
                return fail(ErrorCode::OutOfRange, msg.str());
            }
            const Vec3 patch = cam_pos + t * ray;

            // 交点での角度と照度
            const optics::GeometryAngles angles = optics::computeAngles(sample, patch, light, cam_pos);
            const Real light_distance = (light - patch).norm();
            const Real cos_i = std::max(Real(0), std::cos(angles.incidence.theta));   // 面の裏から当たる光は 0
            const Real irradiance = options_.light_intensity * cos_i / (light_distance * light_distance);

            driver::SpotSpectrum out;
            out.id = spot.id;
            out.pixel = spot.pixel;
            out.values.resize(n);
            for (int i = 0; i < n; ++i) {
                const Real radiance = brdf_->value(result.wavelengths(i), angles) * irradiance;
                Real signal = radiance;   // 実機は校正済みの分光放射輝度を返す(積分時間によらない)
                if (options_.noise_relative > 0.0) {
                    signal *= 1.0 + options_.noise_relative * gauss(rng);
                }
                out.values(i) = signal;
            }
            result.spots.push_back(std::move(out));
        }

        spdlog::debug("[Imager] captured {} spot(s)", result.spots.size());
        return result;
    }

} // namespace brdf::sim
