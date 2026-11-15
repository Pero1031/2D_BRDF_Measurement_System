/*
* @file include/brdf/app/MeasurementSession.hpp
* @brief Runs a recipe on a set of devices and collects per-spot angles and spectra.
*/

#pragma once

#include <functional>
#include <vector>

#include "brdf/app/AngleTable.hpp"
#include "brdf/app/Devices.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/geometry/RigGeometry.hpp"

namespace brdf::app {

    /// 1 つの(ポーズ, 測定点)の結果。
    struct SpotMeasurement {
        int pose_index = 0;
        std::string pose_id;
        int spot_id = 0;
        Vec2 sample_mm = Vec2::Zero();
        Vec2 pixel = Vec2::Zero();           ///< 撮影した画像上の位置
        optics::GeometryAngles angles;       ///< 入射角・観察角(ラジアン、紙の座標系)
        VecX spectrum;                       ///< 分光値
    };

    struct SessionResult {
        VecX wavelengths;                          ///< 波長 [nm]
        std::vector<SpotMeasurement> rows;         ///< ポーズ順、その中で測定点順
    };

    /// 測定の進み具合の通知: (終わったポーズ数, 全ポーズ数, いま取りかかるポーズ)。
    using ProgressFn = std::function<void(int done, int total, const domain::PoseStep& next)>;

    /// レシピを装置で実行する。旧システムの Measuring() に相当する。
    ///
    /// ポーズごとに、(1) 左アーム → 右アームの順に移動、(2) 試料までの距離を計算、
    /// (3) 測定点を画像上の位置へ変換、(4) 撮影、(5) 点ごとの角度を計算、して結果にまとめる。
    /// 途中で失敗したら、どのポーズ・どの段階かを含むエラーで止まる。装置は必ず切断する。
    class MeasurementSession {
    public:
        MeasurementSession(geometry::RigGeometry geometry, Devices devices, domain::Recipe recipe);

        Result<SessionResult> run(const ProgressFn& progress = {});

    private:
        Status connectAll();
        void disconnectAll();

        geometry::RigGeometry geometry_;
        Devices devices_;
        domain::Recipe recipe_;
    };

} // namespace brdf::app
