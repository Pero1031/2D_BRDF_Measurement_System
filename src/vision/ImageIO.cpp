/*
* @file src/vision/ImageIO.cpp
* @brief Reading and writing image files as GrayImage.
*/

#include "brdf/vision/ImageIO.hpp"

#include <fstream>
#include <iterator>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "OpenCvQuiet.hpp"

namespace brdf::vision {

    Result<GrayImage> loadGrayImage(const std::filesystem::path& path) {
        detail::quietOpenCv();
        // パスに日本語が含まれても読めるよう、ファイルを自分で読んでからデコードする
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return fail(ErrorCode::InvalidArgument, "cannot open " + path.string());
        const std::vector<char> bytes((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

        const cv::Mat decoded = cv::imdecode(cv::Mat(1, static_cast<int>(bytes.size()), CV_8UC1,
                                                     const_cast<char*>(bytes.data())), cv::IMREAD_GRAYSCALE);
        if (decoded.empty()) {
            return fail(ErrorCode::InvalidArgument, path.string() + " is not a readable image");
        }

        GrayImage out(decoded.cols, decoded.rows);
        for (int y = 0; y < decoded.rows; ++y) {
            const std::uint8_t* row = decoded.ptr<std::uint8_t>(y);
            std::copy(row, row + decoded.cols, out.pixels.begin() + static_cast<std::ptrdiff_t>(y) * out.width);
        }
        return out;
    }

    Status saveGrayImage(const std::filesystem::path& path, const GrayImage& image) {
        detail::quietOpenCv();
        if (image.empty()) return fail(ErrorCode::InvalidArgument, "cannot save an empty image");

        const cv::Mat mat(image.height, image.width, CV_8UC1, const_cast<std::uint8_t*>(image.pixels.data()));
        std::vector<std::uint8_t> encoded;
        if (!cv::imencode(".png", mat, encoded)) {
            return fail(ErrorCode::DeviceError, "failed to encode the image as PNG");
        }
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs) return fail(ErrorCode::DeviceError, "cannot write " + path.string());
        ofs.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
        if (!ofs) return fail(ErrorCode::DeviceError, "failed while writing " + path.string());
        return {};
    }

} // namespace brdf::vision
