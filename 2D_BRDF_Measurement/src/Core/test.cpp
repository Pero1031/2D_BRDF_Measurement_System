#include <iostream>
#include <spdlog/spdlog.h>

#include "Math/Algebra.hpp"
#include "Driver/Mock/MockRobotArm.hpp"

// 名前空間を省略して記述をスッキリさせますわ
using namespace brdf::driver;

int main() {
    // ログレベル設定（Info以上を表示）
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== BRDF Measurement System Test Start ===");

    // 1. インスタンス化：2台のロボットを用意します
    MockRobotArm sourceArm("SourceArm"); // 光源用
    MockRobotArm sampleArm("SampleArm"); // サンプル用

    // 2. 接続テスト
    // tl::expected は bool キャストで成功判定、.error() でエラー内容取得が可能
    if (!sourceArm.connect()) {
        spdlog::error("Failed to connect SourceArm");
        return -1;
    }
    if (!sampleArm.connect()) {
        spdlog::error("Failed to connect SampleArm");
        return -1;
    }

    // 3. 移動テスト (MoveJ) - 光源アーム
    spdlog::info("--- Testing MoveJ (SourceArm) ---");
    JointAngles targetJoints;
    targetJoints.angles = brdf::VecX::Zero(6);
    targetJoints.angles(0) = 1.57; // 1軸目を90度(rad)に

    auto moveResult = sourceArm.moveJ(targetJoints, 0.5); // 速度50%
    if (!moveResult) {
        spdlog::error("MoveJ failed: {}", moveResult.error());
    }

    // 4. 状態確認テスト - 正しく値が更新されたか？
    auto currentJoints = sourceArm.getCurrentJoints();
    if (currentJoints) {
        // 値が更新されているかチェック (簡易的な検証)
        if (std::abs(currentJoints->angles(0) - 1.57) < 0.001) {
            spdlog::info("Verification OK: Joint angle is 1.57 rad.");
        }
        else {
            spdlog::error("Verification FAILED: Joint angle is not updated!");
        }
    }

    // 5. 移動テスト (MoveL) - サンプルアーム
    spdlog::info("--- Testing MoveL (SampleArm) ---");
    RobotPose targetPose;
    targetPose.position = brdf::Vec3(100.0, 200.0, 300.0); // x, y, z
    targetPose.orientation = brdf::Vec3(0.0, 0.0, 0.0);

    sampleArm.moveL(targetPose);

    // 6. エラーハンドリングテスト - 切断後の操作
    spdlog::info("--- Testing Error Handling ---");
    sampleArm.disconnect();

    auto failTest = sampleArm.moveL(targetPose);
    if (!failTest) {
        spdlog::info("Expected Error Caught: {}", failTest.error()); // ここで "Not connected" が出れば正解
    }
    else {
        spdlog::error("Test FAILED: Move command should fail when disconnected!");
    }

    spdlog::info("=== Test Finished ===");
    return 0;
}