/*
* @file src/vision/OpenCvQuiet.hpp
* @brief Keeps OpenCV's start-up INFO messages out of the console. (internal)
*/

#pragma once

#include <opencv2/core/utils/logger.hpp>

namespace brdf::vision::detail {

    /// OpenCV は最初に並列処理を使うとき、プラグインの読み込みに失敗した旨の INFO を大量に出す(動作には無関係)。
    /// 警告以上だけを表示するようにする。最初の 1 回だけ設定する。
    inline void quietOpenCv() {
        static const bool once = [] {
            cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
            return true;
        }();
        (void)once;
    }

} // namespace brdf::vision::detail
