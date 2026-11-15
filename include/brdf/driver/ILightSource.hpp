/*
* @file include/brdf/driver/ILightSource.hpp
*/

#pragma once

#include <tl/expected.hpp>
#include <string>

namespace brdf::driver {

    class ILightSource {
    public:
        virtual ~ILightSource() = default;

        // 将来実装
        virtual tl::expected<void, std::string> turnOn() = 0;
        virtual tl::expected<void, std::string> turnOff() = 0;
        virtual tl::expected<void, std::string> setIntensity(double percent) = 0;
    };

    // ManualLightSource
    class ManualLightSource : public ILightSource {
    public:
        tl::expected<void, std::string> turnOn() override {
            // 手動で操作する
            return {};
        }

        tl::expected<void, std::string> turnOff() override {
            return {};
        }

        tl::expected<void, std::string> setIntensity(double) override {
            return {};
        }
    };

} // namespace brdf::driver