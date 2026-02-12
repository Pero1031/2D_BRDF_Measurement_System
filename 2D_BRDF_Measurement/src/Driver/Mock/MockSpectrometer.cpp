#include "Driver/Mock/MockSpectrometer.hpp"
#include <thread>
#include <chrono>
#include <cmath> // std::exp, std::pow

namespace brdf::driver {

    MockSpectrometer::MockSpectrometer() {
    }

    tl::expected<void, std::string> MockSpectrometer::connect() {
        spdlog::info("[Spectrometer] Connecting (Mock)...");

        // 実機っぽさを出すため、少しウェイトを入れます（0.5秒）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        is_connected_ = true;
        spdlog::info("[Spectrometer] Connected.");
        return {};
    }

    void MockSpectrometer::disconnect() {
        spdlog::info("[Spectrometer] Disconnecting...");
        is_connected_ = false;
    }

    bool MockSpectrometer::isConnected() const {
        return is_connected_;
    }

    tl::expected<Spectrum, std::string> MockSpectrometer::measure() {
        if (!is_connected_) {
            return tl::unexpected("Not connected");
        }

        spdlog::info("[Spectrometer] Measuring... (IntTime: {:.1f} ms)", integration_time_ms_);

        // 露光時間分だけ待機するフリをします
        // ※開発中は長すぎると待たされるので、最大1秒程度にキャップしても良いですわ
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(integration_time_ms_)));

        // ダミーデータの生成 (380nm - 780nm, 1nmステップ)
        Spectrum spec;
        spec.integration_time_ms = integration_time_ms_;

        const double start_wl = 380.0;
        const double end_wl = 780.0;
        const int steps = static_cast<int>(end_wl - start_wl) + 1; // 401点

        spec.wavelengths.reserve(steps);
        spec.intensities.resize(steps);

        for (int i = 0; i < steps; ++i) {
            double wl = start_wl + static_cast<double>(i);
            spec.wavelengths.push_back(wl);

            // ガウス関数で適当なピークを作ります (550nm中心の緑っぽい光)
            // exp( - (x - mu)^2 / (2 * sigma^2) )
            double val = std::exp(-std::pow(wl - 550.0, 2) / (2.0 * 50.0 * 50.0));

            // 露光時間が長いほど値が大きくなるように調整
            spec.intensities(i) = val * (integration_time_ms_ / 100.0);
        }

        spdlog::info("[Spectrometer] Measurement done. Generated {} points.", steps);
        return spec;
    }

    tl::expected<void, std::string> MockSpectrometer::setIntegrationTime(double ms) {
        if (ms <= 0) return tl::unexpected("Integration time must be positive");

        integration_time_ms_ = ms;
        spdlog::info("[Spectrometer] Integration time set to {:.1f} ms", ms);
        return {};
    }

    tl::expected<double, std::string> MockSpectrometer::getIntegrationTime() const {
        return integration_time_ms_;
    }

} // namespace brdf::driver