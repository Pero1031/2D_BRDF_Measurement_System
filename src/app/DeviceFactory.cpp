/*
* @file src/app/DeviceFactory.cpp
* @brief Builds the devices for the mode chosen in setup.toml.
*/

#include "brdf/app/DeviceFactory.hpp"

#include <memory>

#ifdef BRDF_WITH_TOPCON
#include "brdf/driver/topcon/RealHsApi.hpp"
#endif
#ifdef BRDF_WITH_BCAP
#include "brdf/driver/rc8/RealBcapApi.hpp"
#endif

#include "brdf/driver/topcon/TopconImager.hpp"
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
                        "devices.mode = \"real\" is not connected to the measurement run yet (the arm and spectroradiometer drivers exist but have not been tried on the real devices); use \"sim\"");
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

driver::topcon::TopconImagerConfig makeTopconConfig(const io::SetupConfig& setup, const std::filesystem::path& work_dir) {
        driver::topcon::TopconImagerConfig c;
        c.calibration_dir = setup.imager.calibration_dir;
        c.device = setup.imager.device;
        c.binning = setup.imager.binning;
        c.lens = setup.imager.lens;
        c.image_width = setup.imager.width();
        c.image_height = setup.imager.height();
        c.spot_size = setup.spot_xml.spot_size;
        c.work_dir = work_dir;
        return c;
    }

    driver::rc8::Rc8ArmConfig makeRc8Config(const io::SetupConfig& setup, RealArm arm) {
        const auto& ct = setup.controllers;
        driver::rc8::Rc8ArmConfig c;
        c.name = arm == RealArm::Left ? "LightArm" : "SampleArm";
        c.host = arm == RealArm::Left ? ct.left_host : ct.right_host;
        c.port = ct.port;
        c.robot_name = ct.robot_name;
        c.command_timeout_ms = ct.command_timeout_ms;
        c.move_timeout_ms = ct.move_timeout_ms;
        c.limits = setup.rig.joint_limits;
        c.arrival_tolerance_deg = ct.arrival_tolerance_deg;
        return c;
    }

    Result<std::shared_ptr<driver::IRobotArm>> makeRc8Arm(const io::SetupConfig& setup, RealArm arm) {
#ifdef BRDF_WITH_BCAP
        return std::make_shared<driver::rc8::Rc8Arm>(std::make_shared<driver::rc8::RealBcapApi>(), makeRc8Config(setup, arm));
#else
        (void)setup;
        (void)arm;
        return fail(ErrorCode::NotImplemented,
                    "this build has no b-CAP support (configure with -DBRDF_WITH_BCAP=ON -DBCAP_SDK_DIR=<b-CAP SDK folder>)");
#endif
    }

    Result<std::shared_ptr<driver::ISpectralImager>> makeTopconImager(const io::SetupConfig& setup,
                                                                      const std::filesystem::path& work_dir) {
#ifdef BRDF_WITH_TOPCON
        return std::make_shared<driver::topcon::TopconImager>(std::make_shared<driver::topcon::RealHsApi>(),
                                                              makeTopconConfig(setup, work_dir));
#else
        (void)setup;
        (void)work_dir;
        return fail(ErrorCode::NotImplemented,
                    "this build has no TOPCON SDK support (configure with -DBRDF_WITH_TOPCON=ON -DHS_SDK_DIR=<SDK folder>)");
#endif
    }

} // namespace brdf::app
