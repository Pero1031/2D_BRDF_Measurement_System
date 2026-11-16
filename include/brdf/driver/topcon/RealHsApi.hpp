/*
* @file include/brdf/driver/topcon/RealHsApi.hpp
* @brief ITopconApi on the real TOPCON SDK. Only available when built with BRDF_WITH_TOPCON=ON.
*/

#pragma once

#include <memory>

#include "brdf/driver/topcon/HsApi.hpp"

namespace brdf::driver::topcon {

    class RealHsApi final : public ITopconApi {
    public:
        RealHsApi();
        ~RealHsApi() override;

        Result<int> initialize(const std::string& calibration_dir) override;
        Result<DeviceInfo> openDevice(int index) override;
        Status closeDevice(unsigned device_id) override;
        Status finalize() override;

        Result<int> createMeasurement(const std::filesystem::path& recipe_xml) override;
        Status startMeasurement(unsigned device_id, int measurement_id, const ProgressFn& progress) override;
        Result<MeasurementInfo> measurementInfo(int measurement_id) override;
        Status setSpotDefinition(int measurement_id, const std::filesystem::path& spot_xml) override;
        Result<std::vector<SpotValues>> spotValues(int measurement_id) override;
        Status destroyMeasurement(int measurement_id) override;

        Result<GrayImage> captureLiveImage(unsigned device_id, const LiveImageRequest& request) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;   ///< hs_api.h の型をヘッダに出さないための入れ物
    };

} // namespace brdf::driver::topcon
