/*
* @file src/vision/HomographyFit.cpp
* @brief Least-squares fit of a homography to point correspondences.
*/

#include "brdf/vision/HomographyFit.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace brdf::vision {

    Result<HomographyFit> fitHomography(const std::vector<Vec2>& board_points,
                                        const std::vector<Vec2>& image_points) {
        if (board_points.size() != image_points.size()) {
            return fail(ErrorCode::InvalidArgument, "homography: the two point lists have different lengths");
        }
        if (board_points.size() < 4) {
            return fail(ErrorCode::InvalidArgument, "homography: at least 4 point pairs are needed");
        }

        std::vector<cv::Point2d> src, dst;
        src.reserve(board_points.size());
        dst.reserve(board_points.size());
        for (std::size_t i = 0; i < board_points.size(); ++i) {
            src.emplace_back(board_points[i].x(), board_points[i].y());
            dst.emplace_back(image_points[i].x(), image_points[i].y());
        }

        // method = 0: 全点の最小二乗(+ 再投影誤差の精密化)
        cv::Mat h = cv::findHomography(src, dst, 0);
        if (h.empty() || h.rows != 3 || h.cols != 3) {
            return fail(ErrorCode::InvalidArgument,
                        "homography: could not be computed (are the points collinear or repeated?)");
        }

        HomographyFit fit;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) fit.homography.matrix(r, c) = h.at<double>(r, c);
        }
        // OpenCV は H(2,2) = 1 になるよう正規化して返すが、念のため
        if (std::abs(fit.homography.matrix(2, 2)) > 1e-12) fit.homography.matrix /= fit.homography.matrix(2, 2);

        Real sum_sq = 0.0;
        for (std::size_t i = 0; i < board_points.size(); ++i) {
            const Vec2 p = fit.homography.apply(board_points[i]);
            if (std::isnan(p.x()) || std::isnan(p.y())) {
                return fail(ErrorCode::InvalidArgument, "homography: the result is degenerate");
            }
            const Real e = (p - image_points[i]).norm();
            sum_sq += e * e;
            fit.max_px = std::max(fit.max_px, e);
        }
        fit.points = static_cast<int>(board_points.size());
        fit.rms_px = std::sqrt(sum_sq / board_points.size());
        return fit;
    }

} // namespace brdf::vision
