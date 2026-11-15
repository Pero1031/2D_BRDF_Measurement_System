// 公開ヘッダがそれぞれ単独で(他のヘッダを先に include しなくても)コンパイルできることの確認。
// ヘッダを移動・編集したときに include の誤りを見つけるためのテスト。
#include "brdf/core/Types.hpp"
#include "brdf/core/Algebra.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/driver/IRobotArm.hpp"
#include "brdf/driver/ISpectrometer.hpp"
#include "brdf/driver/ILightSource.hpp"
#include "brdf/driver/mock/MockRobotArm.hpp"
#include "brdf/driver/mock/MockSpectrometer.hpp"
#include "brdf/geometry/Frame.hpp"
#include "brdf/geometry/Joints.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"
#include "brdf/geometry/RigSetup.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/optics/Angles.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("public headers compile and basic mocks work", "[headers]") {
    brdf::driver::ManualLightSource light;
    CHECK(light.turnOn().has_value());
    CHECK(light.setIntensity(50.0).has_value());
    CHECK(light.turnOff().has_value());

    brdf::driver::MockRobotArm arm("test");
    CHECK_FALSE(arm.isConnected());
    CHECK(arm.connect().has_value());
    CHECK(arm.isConnected());
}
