/*
* @file include/Driver/ISpectrometer.hpp
* @brief Abstract interface for spectrometer devices.
*/

#pragma once

#include <Eigen/Dense>
#include <tl/expected.hpp>
#include <string>
#include <vector>

#include "Math/Algebra.hpp"

namespace brdf::driver {

    // 分光測定結果
    struct Spectrum {
        std::vector<double> wavelengths;  // [nm] 波長
        VecX intensities;                 // [W/sr/m²/nm] 分光放射輝度
        double integration_time_ms;       // 積分時間
    };

    // 分光放射計の抽象インターフェース
    class ISpectrometer {
    public:
        virtual ~ISpectrometer() = default;

        // 接続管理
        virtual tl::expected<void, std::string> connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;

        // 測定
        virtual tl::expected<Spectrum, std::string> measure() = 0;

        // 設定
        virtual tl::expected<void, std::string> setIntegrationTime(double ms) = 0;
        virtual tl::expected<double, std::string> getIntegrationTime() const = 0;
    };

} // namespace brdf::driver