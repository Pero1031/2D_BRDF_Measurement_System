/*
* @file include/brdf/vision/ImageIO.hpp
* @brief Reading and writing image files as GrayImage.
*/

#pragma once

#include <filesystem>

#include "brdf/core/Error.hpp"
#include "brdf/core/Image.hpp"

namespace brdf::vision {

    /// 画像ファイル(PNG など)をグレースケールで読む。
    Result<GrayImage> loadGrayImage(const std::filesystem::path& path);

    /// PNG として保存する。
    Status saveGrayImage(const std::filesystem::path& path, const GrayImage& image);

} // namespace brdf::vision
