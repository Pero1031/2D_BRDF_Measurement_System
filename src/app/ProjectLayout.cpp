/*
* @file src/app/ProjectLayout.cpp
* @brief Folder layout of a measurement project.
*/

#include "brdf/app/ProjectLayout.hpp"

#include <string>
#include <system_error>

namespace brdf::app {

    namespace fs = std::filesystem;

    Status ProjectLayout::createFolders() const {
        std::error_code ec;
        for (const fs::path& dir : {calibrationDir(), resultRoot()}) {
            fs::create_directories(dir, ec);
            if (ec) return fail(ErrorCode::DeviceError, "cannot create " + dir.string() + ": " + ec.message());
        }
        return {};
    }

    Result<fs::path> ProjectLayout::createRunDir(std::string_view timestamp, std::string_view label) const {
        const std::string base = std::string(timestamp) + "_" + std::string(label);
        std::error_code ec;

        for (int n = 1; n < 1000; ++n) {
            const fs::path dir = resultRoot() / (n == 1 ? base : base + "_" + std::to_string(n));
            if (fs::exists(dir, ec)) continue;
            if (!fs::create_directories(dir, ec) || ec) {
                return fail(ErrorCode::DeviceError, "cannot create " + dir.string() + ": " + ec.message());
            }
            return dir;
        }
        return fail(ErrorCode::DeviceError, "too many result folders named " + base);
    }

} // namespace brdf::app
