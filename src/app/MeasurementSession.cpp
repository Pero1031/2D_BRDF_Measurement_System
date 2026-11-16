/*
* @file src/app/MeasurementSession.cpp
* @brief Runs a recipe on a set of devices and collects per-spot angles and spectra.
*/

#include "brdf/app/MeasurementSession.hpp"

#include <map>
#include <utility>

#include "brdf/app/RealRun.hpp"
namespace brdf::app {

    namespace {

        // エラーの前に「どのポーズか」を付ける
        tl::unexpected<Error> atPose(const domain::PoseStep& pose, const std::string& stage, const Error& e) {
            return fail(e.code, "pose \"" + pose.id + "\" (" + stage + "): " + e.message);
        }

        // スコープを抜けるときに装置を切断する
        struct DisconnectGuard {
            std::function<void()> action;
            ~DisconnectGuard() { if (action) action(); }
        };

    } // namespace

    MeasurementSession::MeasurementSession(geometry::RigGeometry geometry, Devices devices, domain::Recipe recipe,
                                           SessionOptions options)
        : geometry_(std::move(geometry)), devices_(std::move(devices)), recipe_(std::move(recipe)), options_(std::move(options)) {}

    Status MeasurementSession::connectAll() {
        if (!devices_.left_arm || !devices_.right_arm || !devices_.imager || !devices_.projector) {
            return fail(ErrorCode::InvalidArgument, "session: the device set is incomplete");
        }
        if (auto s = devices_.left_arm->connect(); !s)  return fail(s.error().code, "left arm: " + s.error().message);
        if (auto s = devices_.right_arm->connect(); !s) return fail(s.error().code, "right arm: " + s.error().message);
        if (auto s = devices_.imager->connect(); !s)    return fail(s.error().code, "imager: " + s.error().message);
        return {};
    }

    void MeasurementSession::disconnectAll() {
        if (devices_.imager)   devices_.imager->disconnect();
        if (devices_.right_arm) devices_.right_arm->disconnect();
        if (devices_.left_arm)  devices_.left_arm->disconnect();
    }

    Result<SessionResult> MeasurementSession::run(const ProgressFn& progress) {
        if (auto s = connectAll(); !s) {
            disconnectAll();
            return tl::unexpected(s.error());
        }
        DisconnectGuard guard{[this] { disconnectAll(); }};

        // 実機: アームの実際の位置から、最初の姿勢までの道のりが安全かを、動かす前に確かめる
        if (options_.check_start_path && !recipe_.poses.empty()) {
            if (auto s = checkStartPosition(geometry_, *devices_.left_arm, *devices_.right_arm, recipe_.poses.front(),
                                            options_.preflight);
                !s) {
                return tl::unexpected(s.error());
            }
        }

        auto result = runPoses(progress);
        if (!result) {
            // 失敗したときは、動作中だったかもしれないので、切断する前に両方のアームを止める
            devices_.left_arm->stop();
            devices_.right_arm->stop();
        }
        return result;
    }

    Result<SessionResult> MeasurementSession::runPoses(const ProgressFn& progress) {
        SessionResult result;
        const int total = static_cast<int>(recipe_.poses.size());

        for (int i = 0; i < total; ++i) {
            const domain::PoseStep& pose = recipe_.poses[i];
            if (cancelled()) {
                return fail(ErrorCode::Cancelled, "cancelled before pose \"" + pose.id + "\" (" + std::to_string(i) + " of " +
                                                      std::to_string(total) + " poses done)");
            }
            if (progress) progress(i, total, pose);

            // 1. 移動(左アーム → 右アームの順。事前検査も同じ順で途中の干渉を確認している)
            if (auto s = devices_.left_arm->moveJ(pose.left, recipe_.capture.arm_speed); !s) {
                return atPose(pose, "moving the left arm", s.error());
            }
            if (auto s = devices_.right_arm->moveJ(pose.right, recipe_.capture.arm_speed); !s) {
                return atPose(pose, "moving the right arm", s.error());
            }

            // 2. 幾何: 測定点の角度とワールド座標、試料までの距離
            const std::vector<PatchAngles> patches = computePoseAngles(geometry_, pose, recipe_.spots);

            // 3. 測定点 → 画像上の位置
            driver::CaptureRequest request;
            request.measure_distance_mm = measureDistance(geometry_, pose);
            request.integration_time_ms = recipe_.capture.integration_time_ms;
            request.wavelengths = recipe_.capture.wavelengths;
            for (const PatchAngles& p : patches) {
                auto pixel = devices_.projector->pixelFor(SpotQuery{pose.id, p.sample_mm, p.world_point});
                if (!pixel) {
                    return atPose(pose, "locating spot " + std::to_string(p.spot_id), pixel.error());
                }
                request.spots.push_back({p.spot_id, *pixel});
            }

            // 4. 撮影
            auto captured = devices_.imager->capture(request);
            if (!captured) return atPose(pose, "capturing", captured.error());

            // 5. 結果をまとめる。点の対応は ID で取る(順番に頼らない)
            std::map<int, const driver::SpotSpectrum*> by_id;
            for (const driver::SpotSpectrum& s : captured->spots) by_id[s.id] = &s;

            if (result.wavelengths.size() == 0) result.wavelengths = captured->wavelengths;

            for (const PatchAngles& p : patches) {
                auto it = by_id.find(p.spot_id);
                if (it == by_id.end()) {
                    return fail(ErrorCode::DeviceError, "pose \"" + pose.id + "\": the imager returned no data for spot "
                                                        + std::to_string(p.spot_id));
                }
                SpotMeasurement m;
                m.pose_index = i;
                m.pose_id = pose.id;
                m.spot_id = p.spot_id;
                m.sample_mm = p.sample_mm;
                m.pixel = it->second->pixel;
                m.angles = p.angles;
                m.spectrum = it->second->values;
                result.rows.push_back(std::move(m));
            }
        }

        if (progress) progress(total, total, recipe_.poses.back());
        return result;
    }

} // namespace brdf::app
