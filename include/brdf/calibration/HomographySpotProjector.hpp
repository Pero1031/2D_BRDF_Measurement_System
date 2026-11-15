/*
* @file include/brdf/calibration/HomographySpotProjector.hpp
* @brief SpotProjector that uses one homography per pose, found from calibration-board images.
*/

#pragma once

#include <map>
#include <string>

#include "brdf/app/SpotProjector.hpp"
#include "brdf/calibration/Homography.hpp"
#include "brdf/calibration/SampleToBoard.hpp"

namespace brdf::calibration {

    /// 試料面の点 → 画像上の位置を、ポーズごとのホモグラフィで求める。実機で使う方式。
    ///
    ///   ボード座標 = SampleToBoard(試料座標)     ← ボードの取り付け方(軸の向きと原点)
    ///   pixel      = H(ポーズ) · ボード座標
    ///
    /// 前提: キャリブボードと試料は同じ平面にある。
    class HomographySpotProjector : public app::SpotProjector {
    public:
        /// mapping: ボードの取り付け方(試料座標 → ボード座標)。
        /// image_width / image_height: 画像の大きさ。範囲外に出る点はエラーにする。
        HomographySpotProjector(SampleToBoard mapping, int image_width, int image_height)
            : mapping_(mapping), width_(image_width), height_(image_height) {}

        void setHomography(const std::string& pose_id, const Homography& h) { by_pose_[pose_id] = h; }
        bool has(const std::string& pose_id) const { return by_pose_.contains(pose_id); }
        std::size_t size() const { return by_pose_.size(); }

        Result<Vec2> pixelFor(const app::SpotQuery& query) const override;

    private:
        SampleToBoard mapping_;
        int width_;
        int height_;
        std::map<std::string, Homography> by_pose_;
    };

} // namespace brdf::calibration
