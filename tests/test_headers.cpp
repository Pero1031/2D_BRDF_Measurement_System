// 公開ヘッダがそれぞれ単独で(他のヘッダを先に include しなくても)コンパイルできることの確認。
// ヘッダを移動・編集したときに include の誤りを見つけるためのテスト。
// 各ヘッダを 1 つずつ、先頭に置いて確認するため、ここでは「どれか 1 つでも壊れていれば失敗する」形にしている。
#include "brdf/core/Types.hpp"
#include "brdf/core/Algebra.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/driver/IRobotArm.hpp"
#include "brdf/driver/ISpectralImager.hpp"
#include "brdf/geometry/Frame.hpp"
#include "brdf/geometry/Joints.hpp"
#include "brdf/geometry/JointLimits.hpp"
#include "brdf/geometry/Segment.hpp"
#include "brdf/geometry/Rc8Kinematics.hpp"
#include "brdf/geometry/RigSetup.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/optics/Angles.hpp"
#include "brdf/sim/PinholeCamera.hpp"
#include "brdf/sim/BrdfModel.hpp"
#include "brdf/sim/SimulatedRig.hpp"
#include "brdf/sim/MockRobotArm.hpp"
#include "brdf/sim/MockSpectralImager.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("public headers compile", "[headers]") {
    brdf::Error e{brdf::ErrorCode::NotConnected, "x"};
    CHECK(std::string(brdf::toString(e.code)) == "NotConnected");

    const brdf::Status ok{};
    CHECK(ok.has_value());

    const brdf::Status ng = brdf::fail(brdf::ErrorCode::Collision, "boom");
    REQUIRE_FALSE(ng.has_value());
    CHECK(ng.error().message == "boom");
}
