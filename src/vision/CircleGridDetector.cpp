/*
* @file src/vision/CircleGridDetector.cpp
* @brief Finds the black circles of the calibration board in a camera image.
*/

#include "brdf/vision/CircleGridDetector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "brdf/vision/HomographyFit.hpp"

namespace brdf::vision {

    namespace {

        // ボード中心での、ボードの X 軸・Y 軸が画像でどちらを向くか。
        struct Orientation {
            Real x_rightness = 0.0;     ///< X 軸の向きの右向き成分(右向き = 1、左向き = -1)
            Real handedness = 0.0;      ///< X 軸 × Y 軸 の向き。正なら、ボードを正面から見た(反転していない)画像
        };

        Orientation orientationOf(const calibration::Homography& h, const calibration::BoardLayout& layout) {
            const Vec2 centre(layout.paper_width * 0.5, layout.paper_height * 0.5);
            const Vec2 o = h.apply(centre);
            const Vec2 dx = h.apply(centre + Vec2(1.0, 0.0)) - o;
            const Vec2 dy = h.apply(centre + Vec2(0.0, 1.0)) - o;
            Orientation r;
            r.x_rightness = dx.norm() > 0.0 ? dx.x() / dx.norm() : 0.0;
            r.handedness = dx.x() * dy.y() - dx.y() * dy.x();
            return r;
        }

        // 円格子の並びの別解。対称な格子なので、回転(180°)も反転(行内を逆、行の順を逆)も、同じ点の集合を作る。
        std::vector<std::vector<Vec2>> alternatives(const std::vector<Vec2>& p, int columns, int rows) {
            std::vector<std::vector<Vec2>> out;
            out.push_back(p);                                                // そのまま
            out.emplace_back(p.rbegin(), p.rend());                          // 180° 回転
            std::vector<Vec2> flip_x, flip_y;
            for (int j = 0; j < rows; ++j) {
                for (int i = 0; i < columns; ++i) {
                    flip_x.push_back(p[static_cast<std::size_t>(j) * columns + (columns - 1 - i)]);       // 行内を逆
                    flip_y.push_back(p[static_cast<std::size_t>(rows - 1 - j) * columns + i]);            // 行の順を逆
                }
            }
            out.push_back(std::move(flip_x));
            out.push_back(std::move(flip_y));
            return out;
        }

    } // namespace

    Result<std::vector<Vec2>> detectCircleGrid(const GrayImage& image,
                                               const calibration::BoardLayout& layout,
                                               const DetectorOptions& options) {
        if (image.empty() || image.pixels.size() != static_cast<std::size_t>(image.width) * image.height) {
            return fail(ErrorCode::InvalidArgument, "circle detection: the image is empty or inconsistent");
        }
        if (layout.columns < 2 || layout.rows < 2) {
            return fail(ErrorCode::InvalidArgument, "circle detection: the board needs at least 2x2 circles");
        }

        // OpenCV の画像として見る(データはコピーしない)
        cv::Mat gray(image.height, image.width, CV_8UC1, const_cast<std::uint8_t*>(image.pixels.data()));

        cv::SimpleBlobDetector::Params params;
        params.filterByColor = true;
        params.blobColor = 0;                       // 黒い円
        params.filterByArea = true;
        params.minArea = static_cast<float>(options.min_area_px);
        params.maxArea = static_cast<float>(options.max_area_px);
        params.filterByCircularity = true;
        params.minCircularity = static_cast<float>(options.min_circularity);
        params.filterByInertia = false;
        params.filterByConvexity = false;
        cv::Ptr<cv::FeatureDetector> blobs = cv::SimpleBlobDetector::create(params);

        std::vector<cv::Point2f> found;
        const bool ok = cv::findCirclesGrid(gray, cv::Size(layout.columns, layout.rows), found,
                                            cv::CALIB_CB_SYMMETRIC_GRID, blobs);
        if (!ok || static_cast<int>(found.size()) != layout.count()) {
            return fail(ErrorCode::DeviceError,
                        "circle detection: the " + std::to_string(layout.columns) + "x" + std::to_string(layout.rows)
                        + " circle grid was not found in the image (check the focus, the exposure and that the whole "
                          "board is visible)");
        }

        std::vector<Vec2> found_pts;
        for (const cv::Point2f& p : found) found_pts.emplace_back(p.x, p.y);

        // 向きの決定:
        //  1. 反転した並び(ボードを裏から見たことになる並び)は、ホモグラフィの向きが逆になるので除く
        //  2. 残った回転違いの並びから、ボードの X 軸がより右向きに写る方を選ぶ
        const std::vector<Vec2> board = layout.circleCenters();
        const std::vector<Vec2>* best = nullptr;
        Real best_rightness = -2.0;
        std::vector<std::vector<Vec2>> candidates = alternatives(found_pts, layout.columns, layout.rows);
        for (const std::vector<Vec2>& cand : candidates) {
            auto fit = fitHomography(board, cand);
            if (!fit) continue;
            const Orientation o = orientationOf(fit->homography, layout);
            if (o.handedness <= 0.0) continue;
            if (o.x_rightness > best_rightness) {
                best_rightness = o.x_rightness;
                best = &cand;
            }
        }
        if (!best) {
            return fail(ErrorCode::DeviceError,
                        "circle detection: no consistent ordering of the circles was found (the board may be seen "
                        "from behind, or the detected points are not a regular grid)");
        }
        return *best;
    }

} // namespace brdf::vision
