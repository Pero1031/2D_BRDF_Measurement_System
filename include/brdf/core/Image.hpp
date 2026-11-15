/*
* @file include/brdf/core/Image.hpp
* @brief A minimal 8-bit grayscale image (keeps OpenCV out of the public headers).
*/

#pragma once

#include <cstdint>
#include <vector>

namespace brdf {

    /// 8 ビットのグレースケール画像。行優先、左上が原点。
    /// キャリブボードの撮影(明るさだけが必要)に使う。OpenCV の型を公開ヘッダに出さないための薄い入れ物。
    struct GrayImage {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> pixels;   ///< 要素数 = width × height

        GrayImage() = default;
        GrayImage(int w, int h, std::uint8_t fill = 0)
            : width(w), height(h), pixels(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), fill) {}

        bool empty() const { return width <= 0 || height <= 0 || pixels.empty(); }

        std::uint8_t& at(int x, int y) { return pixels[static_cast<std::size_t>(y) * width + x]; }
        std::uint8_t at(int x, int y) const { return pixels[static_cast<std::size_t>(y) * width + x]; }
    };

} // namespace brdf
