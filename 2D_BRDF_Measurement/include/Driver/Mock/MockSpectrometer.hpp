#pragma once
#include "Driver/ISpectrometer.hpp"
#include <spdlog/spdlog.h>

namespace brdf::driver {

    class MockSpectrometer : public ISpectrometer {
    public:
        MockSpectrometer();
        ~MockSpectrometer() override = default;

        tl::expected<void, std::string> connect() override;
        void disconnect() override;
        bool isConnected() const override;

        // 測定を行い、ダミーのスペクトルデータを返します
        tl::expected<Spectrum, std::string> measure() override;

        tl::expected<void, std::string> setIntegrationTime(double ms) override;
        tl::expected<double, std::string> getIntegrationTime() const override;

    private:
        bool is_connected_ = false;
        double integration_time_ms_ = 100.0; // デフォルト値
    };

} // namespace brdf::driver