#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "brdf/app/Commands.hpp"
#include "brdf/app/ArmCommands.hpp"
#include "brdf/app/DeviceFactory.hpp"
#include "brdf/app/MeasurementSession.hpp"
#include "brdf/app/ProjectLayout.hpp"
#include "brdf/app/RealRun.hpp"
#include "brdf/core/MathUtils.hpp"
#include "brdf/geometry/Joints.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using namespace brdf;
using namespace brdf::app;
using geometry::jointsFromDegrees;

namespace {

    /// 実機のアームの代わりをするフェイク。現在位置を答え、移動を記録し、指定した移動で失敗し、止められた回数を数える。
    class FakeArm final : public driver::IRobotArm {
    public:
        geometry::Joints current = jointsFromDegrees(65, 0, 90, 0, 0, 0);
        std::vector<geometry::Joints> moves;
        std::vector<Real> speeds;
        int fail_on_move = 0;                              ///< この回数目の移動で失敗する(0 なら失敗しない)
        ErrorCode fail_code = ErrorCode::DeviceError;
        std::function<void()> on_move;                     ///< 移動に成功するたびに呼ぶ
        int stops = 0, connects = 0, disconnects = 0;
        mutable int reads = 0;
        bool connected = false;

        Status connect() override {
            ++connects;
            connected = true;
            return {};
        }
        void disconnect() override {
            ++disconnects;
            connected = false;
        }
        bool isConnected() const override { return connected; }

        Status moveJ(const geometry::Joints& target, Real speed) override {
            if (fail_on_move > 0 && static_cast<int>(moves.size()) + 1 == fail_on_move) {
                return fail(fail_code, "move refused (injected)");
            }
            moves.push_back(target);
            speeds.push_back(speed);
            current = target;
            if (on_move) on_move();
            return {};
        }
        Result<geometry::Joints> currentJoints() const override {
            ++reads;
            return current;
        }
        void stop() override { ++stops; }
        void emergencyStop() override {}
    };

    /// 分光放射計の代わりをするフェイク。シミュレータの分光放射計は、シミュレータ内のアームの位置と測定距離を照合するので、
    /// フェイクのアームとは組み合わせられない。要求された点ごとに、一定のスペクトルを返す。
    class FakeImager final : public driver::ISpectralImager {
    public:
        Status connect() override { connected = true; return {}; }
        void disconnect() override { connected = false; }
        bool isConnected() const override { return connected; }
        Result<driver::CaptureResult> capture(const driver::CaptureRequest& request) override {
            ++captures;
            driver::CaptureResult r;
            const int n = request.wavelengths.count();
            r.wavelengths = VecX::LinSpaced(n, request.wavelengths.start, request.wavelengths.end);
            for (const driver::SpotRequest& s : request.spots) r.spots.push_back({s.id, s.pixel, VecX::Constant(n, 1.0)});
            r.integration_time_ms = request.integration_time_ms;
            r.measure_distance_mm = request.measure_distance_mm;
            return r;
        }
        Result<GrayImage> captureImage(const driver::ImageRequest&) override { return GrayImage(8, 8, 0); }
        bool connected = false;
        int captures = 0;
    };

    struct Fixture {
        io::SetupConfig setup;
        domain::Recipe recipe = defaultRecipe();
        Devices devices;
        std::shared_ptr<FakeArm> left = std::make_shared<FakeArm>();
        std::shared_ptr<FakeArm> right = std::make_shared<FakeArm>();
        std::shared_ptr<CancelToken> cancel = std::make_shared<CancelToken>();

        Fixture() {
            auto d = makeDevices(setup);   // 撮影は、シミュレータの分光放射計
            REQUIRE(d.has_value());
            devices = *d;
            devices.left_arm = left;
            devices.right_arm = right;
            devices.imager = std::make_shared<FakeImager>();
            // 試料側(右)の現在位置は、最初の姿勢の近く
            right->current = recipe.poses.front().right;
            left->current = recipe.poses.front().left;
        }

        MeasurementSession session(SessionOptions options = {}) {
            options.cancel = cancel;
            return MeasurementSession(geometry::RigGeometry(setup.rig), devices, recipe, options);
        }
    };

} // namespace

// ---------------------------------------------------------------------------
// 動かす前の確認
// ---------------------------------------------------------------------------

TEST_CASE("the real-motion description names the controllers, the effective speed and the range", "[realrun]") {
    io::SetupConfig setup;
    setup.mode = io::DeviceMode::Real;
    domain::Recipe recipe = defaultRecipe();

    SECTION("a requested speed above the limit is shown as limited") {
        recipe.capture.arm_speed = 0.5;
        setup.controllers.max_speed = 0.05;
        const std::string text = describeRealMotion(setup, recipe);
        CHECK_THAT(text, ContainsSubstring("192.168.0.2:5007"));
        CHECK_THAT(text, ContainsSubstring("192.168.0.5:5007"));
        CHECK_THAT(text, ContainsSubstring("速度: 5%"));
        CHECK_THAT(text, ContainsSubstring("arm_speed = 50%"));
        CHECK_THAT(text, ContainsSubstring("max_speed = 5%"));
        CHECK_THAT(text, ContainsSubstring("姿勢: 3 個"));
        CHECK_THAT(text, ContainsSubstring("J1 ["));
    }
    SECTION("a requested speed below the limit is used as it is") {
        recipe.capture.arm_speed = 0.02;
        setup.controllers.max_speed = 0.05;
        const std::string text = describeRealMotion(setup, recipe);
        CHECK_THAT(text, ContainsSubstring("速度: 2%"));
        CHECK(text.find("max_speed") == std::string::npos);
    }
    SECTION("what Ctrl+C does depends on the setting") {
        setup.controllers.abortable_moves = false;
        CHECK_THAT(describeRealMotion(setup, recipe), ContainsSubstring("動作中は Ctrl+C で止められません"));
        setup.controllers.abortable_moves = true;
        CHECK_THAT(describeRealMotion(setup, recipe), ContainsSubstring("動作中でも止まります"));
    }
    SECTION("the range comes from the poses") {
        recipe.poses = {{"a", jointsFromDegrees(10, 0, 90, 0, 0, 0), jointsFromDegrees(-70, 0, 90, 0, 0, 0)},
                        {"b", jointsFromDegrees(30, 0, 90, 0, 0, 0), jointsFromDegrees(-50, 0, 90, 0, 0, 0)}};
        const std::string text = describeRealMotion(setup, recipe);
        CHECK_THAT(text, ContainsSubstring("左: J1 [10, 30]"));
        CHECK_THAT(text, ContainsSubstring("右: J1 [-70, -50]"));
    }
}

TEST_CASE("moving the real arms needs an explicit OK", "[realrun]") {
    const domain::Recipe recipe = defaultRecipe();

    SECTION("the simulator is not asked") {
        io::SetupConfig setup;   // sim
        std::istringstream in;
        CHECK(confirmRealMotion(setup, recipe, in).has_value());
    }
    SECTION("real mode with OK") {
        io::SetupConfig setup;
        setup.mode = io::DeviceMode::Real;
        std::istringstream in("OK\n");
        CHECK(confirmRealMotion(setup, recipe, in).has_value());
    }
    SECTION("anything else cancels, and says nothing was moved") {
        io::SetupConfig setup;
        setup.mode = io::DeviceMode::Real;
        for (const char* answer : {"\n", "ok\n", "yes\n", "OK please\n", ""}) {
            std::istringstream in(answer);
            const Status s = confirmRealMotion(setup, recipe, in);
            REQUIRE_FALSE(s.has_value());
            CHECK(s.error().code == ErrorCode::InvalidArgument);
            CHECK_THAT(s.error().message, ContainsSubstring("nothing was moved"));
        }
    }
}

TEST_CASE("only real mode warns about poses outside the previous system's range", "[realrun]") {
    io::SetupConfig setup;
    CHECK_FALSE(preflightOptionsFor(setup).warn_outside_previous_range);
    setup.mode = io::DeviceMode::Real;
    CHECK(preflightOptionsFor(setup).warn_outside_previous_range);
}

// ---------------------------------------------------------------------------
// 接続した直後の位置のチェック
// ---------------------------------------------------------------------------

TEST_CASE("a session with the start check moves when the path from the current position is clear", "[realrun][session]") {
    Fixture f;
    SessionOptions options;
    options.check_start_path = true;

    const auto result = f.session(options).run();
    REQUIRE(result.has_value());
    CHECK(f.left->reads == 1);    // 接続した直後に、実際の位置を読んでいる
    CHECK(f.right->reads == 1);
    CHECK(f.left->moves.size() == 3);
    CHECK(f.right->moves.size() == 3);
}

TEST_CASE("a session without the start check does not read the position", "[realrun][session]") {
    Fixture f;
    REQUIRE(f.session().run().has_value());
    CHECK(f.left->reads == 0);
}

TEST_CASE("the start check refuses to move when an arm is outside its limits", "[realrun][session]") {
    Fixture f;
    f.left->current = jointsFromDegrees(65, 0, 10, 0, 0, 0);   // J3 の下限は 19°
    SessionOptions options;
    options.check_start_path = true;

    const auto result = f.session(options).run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::JointLimit);
    CHECK_THAT(result.error().message, ContainsSubstring("nothing was moved"));
    CHECK(f.left->moves.empty());       // 1 回も動かしていない
    CHECK(f.right->moves.empty());
    CHECK(f.left->disconnects == 1);    // 接続は閉じている
    CHECK(f.right->disconnects == 1);
}

TEST_CASE("the start check refuses to move when the arms are already too close", "[realrun][session]") {
    Fixture f;
    f.setup.rig.right_base_position = Vec3(0, 60, 0);   // 右アームを左アームのすぐそばに置いた配置
    f.left->current = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    f.right->current = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    SessionOptions options;
    options.check_start_path = true;

    const auto result = f.session(options).run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::Collision);
    CHECK_THAT(result.error().message, ContainsSubstring("already"));
    CHECK(f.left->moves.empty());
    CHECK(f.right->moves.empty());
}

// ---------------------------------------------------------------------------
// 失敗したとき・中止したとき
// ---------------------------------------------------------------------------

TEST_CASE("when an arm move fails, both arms are stopped and the devices are disconnected", "[realrun][session]") {
    Fixture f;
    f.right->fail_on_move = 2;   // 2 つ目のポーズで、右アームが失敗する

    const auto result = f.session().run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::DeviceError);
    CHECK_THAT(result.error().message, ContainsSubstring("moving the right arm"));
    CHECK_THAT(result.error().message, ContainsSubstring("\"2\""));

    CHECK(f.left->stops == 1);       // 切断する前に、両方を止めている(動作中だったかもしれない)
    CHECK(f.right->stops == 1);
    CHECK(f.left->disconnects == 1);
    CHECK(f.right->disconnects == 1);
    CHECK(f.left->moves.size() == 2);    // 左は、2 つ目のポーズへ動いた(右が失敗する前)
    CHECK(f.right->moves.size() == 1);   // 右は 1 つ目だけ
}

TEST_CASE("a successful session does not stop the arms", "[realrun][session]") {
    Fixture f;
    REQUIRE(f.session().run().has_value());
    CHECK(f.left->stops == 0);
    CHECK(f.right->stops == 0);
    CHECK(f.left->disconnects == 1);
}

TEST_CASE("a cancel request stops the session before the next pose", "[realrun][session]") {
    Fixture f;
    f.right->on_move = [&] { f.cancel->request(); };   // 1 つ目のポーズへ動いたあとで、中止を要求する

    const auto result = f.session().run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::Cancelled);
    CHECK_THAT(result.error().message, ContainsSubstring("cancelled before pose \"2\""));
    CHECK_THAT(result.error().message, ContainsSubstring("1 of 3"));

    CHECK(f.left->moves.size() == 1);    // 2 つ目のポーズへは動かさない
    CHECK(f.right->moves.size() == 1);
    CHECK(f.left->stops == 1);
    CHECK(f.right->stops == 1);
    CHECK(f.left->disconnects == 1);
}

TEST_CASE("a cancel that an arm reports in the middle of a move ends the session as cancelled", "[realrun][session]") {
    Fixture f;
    f.left->fail_on_move = 2;
    f.left->fail_code = ErrorCode::Cancelled;   // アームのドライバが、動作中に中止を受けた

    const auto result = f.session().run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ErrorCode::Cancelled);
    CHECK(f.left->stops == 1);
    CHECK(f.right->stops == 1);
}

// ---------------------------------------------------------------------------
// 装置の組み立て(実機)
// ---------------------------------------------------------------------------

TEST_CASE("makeDevices in real mode reports which driver is missing", "[realrun][factory]") {
    io::SetupConfig setup;
    setup.mode = io::DeviceMode::Real;

#if !(defined(BRDF_WITH_BCAP) && defined(BRDF_WITH_TOPCON))
    const auto d = makeDevices(setup);
    REQUIRE_FALSE(d.has_value());
    CHECK(d.error().code == ErrorCode::NotImplemented);
#ifndef BRDF_WITH_BCAP
    CHECK_THAT(d.error().message, ContainsSubstring("BRDF_WITH_BCAP"));
#else
    CHECK_THAT(d.error().message, ContainsSubstring("BRDF_WITH_TOPCON"));
#endif
#else
    // 両方の SDK 付きのビルド。作るだけで、接続はしない
    SECTION("the spectroradiometer needs a work folder") {
        const auto d = makeDevices(setup);
        REQUIRE_FALSE(d.has_value());
        CHECK(d.error().code == ErrorCode::InvalidArgument);
    }
    SECTION("with a work folder, the devices are built and not connected") {
        const auto d = makeDevices(setup, domain::MeasurementKind::Sample, "some/work/dir", std::make_shared<CancelToken>());
        REQUIRE(d.has_value());
        CHECK(d->left_arm);
        CHECK(d->right_arm);
        CHECK(d->imager);
        CHECK_FALSE(d->left_arm->isConnected());
        CHECK_FALSE(d->right_arm->isConnected());
        CHECK_FALSE(d->imager->isConnected());
        CHECK_FALSE(d->projector);   // ボードの校正結果を読み込んだ側が設定する
    }
#endif
}

TEST_CASE("a simulated run does not create the work folder", "[realrun][factory]") {
    const auto dir = std::filesystem::temp_directory_path() / "brdf_test_workdir";
    std::filesystem::remove_all(dir);
    REQUIRE(initProject(dir).has_value());

    RunOptions options;
    options.assume_yes = true;
    std::istringstream in;
    REQUIRE(runProject(dir, options, in).has_value());
    CHECK_FALSE(std::filesystem::exists(ProjectLayout(dir).workDir()));   // 作るのは、実機モードのときだけ
    CHECK(ProjectLayout(dir).workDir() == dir / "work");

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

// ---------------------------------------------------------------------------
// brdf arm: 1 台ずつ動かす
// ---------------------------------------------------------------------------

namespace {

    struct ArmFixture {
        io::SetupConfig setup;
        std::shared_ptr<FakeArm> left = std::make_shared<FakeArm>();
        std::shared_ptr<FakeArm> right = std::make_shared<FakeArm>();

        ArmFixture() {
            setup.mode = io::DeviceMode::Real;
            left->current = standardPose(RealArm::Left);
            right->current = standardPose(RealArm::Right);
        }

        Status move(RealArm side, const ArmTarget& target, Real speed = 0.05, const std::string& input = "OK\n") {
            std::istringstream in(input);
            return moveOneArm(setup, *left, *right, side, target, speed, in);
        }
    };

    ArmTarget jog(int joint, Real deg) {
        ArmTarget t;
        t.jog_joint = joint;
        t.jog_deg = deg;
        return t;
    }

    ArmTarget absolute(const geometry::Joints& q) {
        ArmTarget t;
        t.absolute = q;
        return t;
    }

} // namespace

TEST_CASE("the standard pose is the one the previous system used", "[arm]") {
    const auto right = standardPose(RealArm::Right);
    const auto left = standardPose(RealArm::Left);
    CHECK_THAT(toDegrees(right(0)), WithinAbs(-45.0, 1e-9));
    CHECK_THAT(toDegrees(left(0)), WithinAbs(45.0, 1e-9));
    for (int j = 1; j < 6; ++j) {
        CHECK_THAT(toDegrees(left(j)), WithinAbs(j == 2 ? 90.0 : 0.0, 1e-9));
        CHECK_THAT(toDegrees(right(j)), WithinAbs(j == 2 ? 90.0 : 0.0, 1e-9));
    }
}

TEST_CASE("joint angles are shown in degrees", "[arm]") {
    CHECK(formatJoints(jointsFromDegrees(-45, 0, 90, 0, 0, 12.345)) == "J1 = -45.00  J2 = 0.00  J3 = 90.00  J4 = 0.00  J5 = 0.00  J6 = 12.35");
}

TEST_CASE("the arm-move description shows the arm, the speed limit and what will not move", "[arm]") {
    io::SetupConfig setup;
    setup.controllers.max_speed = 0.05;
    const std::string text = describeArmMove(setup, RealArm::Right, standardPose(RealArm::Right), jointsFromDegrees(-42.5, 0, 90, 0, 0, 0), 0.5);
    CHECK_THAT(text, ContainsSubstring("右(試料側)"));
    CHECK_THAT(text, ContainsSubstring("192.168.0.5:5007"));
    CHECK_THAT(text, ContainsSubstring("速度: 5%"));
    CHECK_THAT(text, ContainsSubstring("指定の 50%"));
    CHECK_THAT(text, ContainsSubstring("J1 = -45.00"));
    CHECK_THAT(text, ContainsSubstring("J1 = -42.50"));
    CHECK_THAT(text, ContainsSubstring("2.50 度"));
    CHECK_THAT(text, ContainsSubstring("もう一方のアームは動かしません"));
}

TEST_CASE("jogging one joint moves only that arm, by that amount", "[arm]") {
    ArmFixture f;
    REQUIRE(f.move(RealArm::Right, jog(0, 2.5), 0.05).has_value());

    REQUIRE(f.right->moves.size() == 1);
    CHECK_THAT(toDegrees(f.right->moves[0](0)), WithinAbs(-42.5, 1e-9));     // 現在 -45° から +2.5°
    for (int j = 1; j < 6; ++j) CHECK_THAT(f.right->moves[0](j), WithinAbs(standardPose(RealArm::Right)(j), 1e-12));
    CHECK_THAT(f.right->speeds[0], WithinAbs(0.05, 1e-12));
    CHECK(f.left->moves.empty());                                              // もう一方は動かさない
    CHECK(f.left->reads == 1);                                                 // ただし、干渉の検査のために、実際の位置は読む
}

TEST_CASE("an absolute target moves the chosen arm", "[arm]") {
    ArmFixture f;
    REQUIRE(f.move(RealArm::Left, absolute(jointsFromDegrees(50, 0, 90, 0, 0, 0)), 0.05).has_value());
    REQUIRE(f.left->moves.size() == 1);
    CHECK_THAT(toDegrees(f.left->moves[0](0)), WithinAbs(50.0, 1e-9));
    CHECK(f.right->moves.empty());
}

TEST_CASE("nothing moves unless the answer is exactly OK", "[arm]") {
    ArmFixture f;
    for (const char* answer : {"\n", "ok\n", "yes\n", ""}) {
        const Status s = f.move(RealArm::Right, jog(0, 2.5), 0.05, answer);
        REQUIRE_FALSE(s.has_value());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
        CHECK_THAT(s.error().message, ContainsSubstring("nothing was moved"));
    }
    CHECK(f.right->moves.empty());
    CHECK(f.left->moves.empty());
}

TEST_CASE("a target outside the joint limits is refused before asking", "[arm]") {
    ArmFixture f;
    const Status s = f.move(RealArm::Left, absolute(jointsFromDegrees(45, 0, 10, 0, 0, 0)));   // J3 の下限は 19°
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::JointLimit);
    CHECK_THAT(s.error().message, ContainsSubstring("nothing was moved"));
    CHECK(f.left->moves.empty());
}

TEST_CASE("a jog that would leave the joint limits is refused", "[arm]") {
    ArmFixture f;
    f.right->current = jointsFromDegrees(-45, 0, 22, 0, 0, 0);   // J3 は下限 19° の近く
    const Status s = f.move(RealArm::Right, jog(2, -5.0), 0.05);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::JointLimit);
    CHECK(f.right->moves.empty());
}

TEST_CASE("arms that are already too close are not moved", "[arm]") {
    ArmFixture f;
    f.setup.rig.right_base_position = Vec3(0, 60, 0);
    f.left->current = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    f.right->current = jointsFromDegrees(0, 0, 90, 0, 0, 0);
    const Status s = f.move(RealArm::Right, jog(0, 2.0), 0.05);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::Collision);
    CHECK(f.right->moves.empty());
}

TEST_CASE("a refused move is reported to the caller", "[arm]") {
    ArmFixture f;
    f.right->fail_on_move = 1;
    f.right->fail_code = ErrorCode::Timeout;
    const Status s = f.move(RealArm::Right, jog(0, 2.5), 0.05);
    REQUIRE_FALSE(s.has_value());
    CHECK(s.error().code == ErrorCode::Timeout);
}

TEST_CASE("bad requests to the arm command are refused", "[arm]") {
    ArmFixture f;
    CHECK(f.move(RealArm::Right, jog(0, 1.0), 0.0).error().code == ErrorCode::InvalidArgument);
    CHECK(f.move(RealArm::Right, jog(0, 1.0), 1.5).error().code == ErrorCode::InvalidArgument);
    CHECK(f.move(RealArm::Right, ArmTarget{}, 0.05).error().code == ErrorCode::InvalidArgument);   // 目標がない
    CHECK(f.move(RealArm::Right, jog(6, 1.0), 0.05).error().code == ErrorCode::InvalidArgument);   // J7 はない
    CHECK(f.right->moves.empty());
}

TEST_CASE("the arm commands only work on a real-mode project", "[arm]") {
    const auto dir = std::filesystem::temp_directory_path() / "brdf_test_armcmd";
    std::filesystem::remove_all(dir);
    REQUIRE(initProject(dir).has_value());   // mode = "sim"

    const Status status = armStatus(dir);
    REQUIRE_FALSE(status.has_value());
    CHECK(status.error().code == ErrorCode::InvalidArgument);
    CHECK_THAT(status.error().message, ContainsSubstring("mode = \"real\""));

    std::istringstream in("OK\n");
    const Status move = armMove(dir, RealArm::Left, jog(0, 1.0), 0.05, in);
    REQUIRE_FALSE(move.has_value());
    CHECK(move.error().code == ErrorCode::InvalidArgument);

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}
