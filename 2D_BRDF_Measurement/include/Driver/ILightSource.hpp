// ILightSource.hpp（将来用の枠）
#pragma once
#include <tl/expected.hpp>
#include <string>

namespace brdf::driver {

    class ILightSource {
    public:
        virtual ~ILightSource() = default;

        // 将来実装予定（今はダミーでOK）
        virtual tl::expected<void, std::string> turnOn() = 0;
        virtual tl::expected<void, std::string> turnOff() = 0;
        virtual tl::expected<void, std::string> setIntensity(double percent) = 0;
    };

    // ManualLightSource（手動用の実装）
    class ManualLightSource : public ILightSource {
    public:
        tl::expected<void, std::string> turnOn() override {
            // 何もしない（手動で操作するので）
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