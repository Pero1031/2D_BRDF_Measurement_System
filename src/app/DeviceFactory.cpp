/*
* @file src/app/DeviceFactory.cpp
* @brief Builds the devices for the mode chosen in setup.toml.
*/

#include "brdf/app/DeviceFactory.hpp"

#include <memory>

#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/CameraSpotProjector.hpp"
#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/MockSpectralImager.hpp"
#include "brdf/sim/SimulatedRig.hpp"

namespace brdf::app {

    namespace {

        std::shared_ptr<sim::IBrdfModel> makeBrdf(const io::SimSampleConfig& s) {
            const auto albedo = sim::gaussianCurve(s.peak_nm, s.width_nm, s.peak, s.base);
            if (s.model == "phong") {
                return std::make_shared<sim::PhongBrdf>(albedo, s.specular, s.shininess);
            }
            return std::make_shared<sim::LambertBrdf>(albedo);
        }

    } // namespace

    Result<Devices> makeDevices(const io::SetupConfig& setup, domain::MeasurementKind kind) {
        if (setup.mode == io::DeviceMode::Real) {
            return fail(ErrorCode::NotImplemented,
                        "devices.mode = \"real\" is not supported yet (real drivers are planned for P6); use \"sim\"");
        }

        // カメラ: 位置は装置の配置から、向きは設定、無ければ既定
        const geometry::RigGeometry geometry(setup.rig);
        sim::PinholeCameraSetup camera = sim::SimulatedRig::defaultCamera(geometry);
        camera.image_width = setup.imager.width();
        camera.image_height = setup.imager.height();
        camera.pixels_per_mm = setup.sim.camera.pixels_per_mm;
        if (setup.sim.camera.target) camera.target = *setup.sim.camera.target;

        auto rig = std::make_shared<sim::SimulatedRig>(setup.rig, camera);

        Devices d;
        d.left_arm = std::make_shared<sim::MockRobotArm>("LightArm", rig, sim::ArmSide::Left);
        d.right_arm = std::make_shared<sim::MockRobotArm>("SampleArm", rig, sim::ArmSide::Right);
        // 白色板の測定では、試料の代わりに標準拡散反射板(完全拡散)を測る
        std::shared_ptr<sim::IBrdfModel> target =
            kind == domain::MeasurementKind::White
                ? std::shared_ptr<sim::IBrdfModel>(std::make_shared<sim::LambertBrdf>(sim::flatCurve(setup.sim.white_reflectance)))
                : makeBrdf(setup.sim.sample);
        auto imager = std::make_shared<sim::MockSpectralImager>(rig, target, setup.sim.imager);
        auto mapping = setup.calibration.mapping();
        if (!mapping) return tl::unexpected(mapping.error());
        imager->setCalibrationBoard({setup.calibration.board, *mapping});
        d.imager = imager;

        // "board" のときは、校正結果(ホモグラフィ)を読み込んだ側が projector を設定する
        if (setup.calibration.projector == "camera-model") {
            d.projector = std::make_shared<sim::CameraSpotProjector>(rig);
        }
        return d;
    }

} // namespace brdf::app
