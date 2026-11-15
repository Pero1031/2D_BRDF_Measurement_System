/*
* @file src/io/MeasurementsCsv.cpp
* @brief Reading back the measurements.csv that a measurement run wrote.
*/

#include "brdf/io/MeasurementsCsv.hpp"

#include <fstream>
#include <sstream>

#include "CsvLine.hpp"

namespace brdf::io {

    namespace {
        constexpr std::size_t kFixedColumns = 10;
        const char* const kFixedHeader[kFixedColumns] = {"pose", "spot", "x_mm", "y_mm", "pixel_x", "pixel_y",
                                                         "theta_i_deg", "phi_i_deg", "theta_o_deg", "phi_o_deg"};
    }

    Result<MeasurementTable> parseMeasurementsCsv(std::string_view text, std::string_view origin) {
        auto err = [&](int line, const std::string& m) {
            return fail(ErrorCode::InvalidArgument, std::string(origin) + ":" + std::to_string(line) + ": " + m);
        };

        std::istringstream in{std::string(text)};
        std::string line;
        if (!std::getline(in, line)) return err(1, "the file is empty");
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);

        const auto header = detail::splitCsvLine(line);
        if (header.size() <= kFixedColumns) return err(1, "the header has no wavelength columns (wl_<nm>)");
        for (std::size_t i = 0; i < kFixedColumns; ++i) {
            if (header[i] != kFixedHeader[i]) {
                return err(1, "column " + std::to_string(i + 1) + " must be \"" + kFixedHeader[i] + "\" (found \"" + header[i] + "\")");
            }
        }

        MeasurementTable table;
        table.wavelengths.resize(static_cast<Eigen::Index>(header.size() - kFixedColumns));
        for (std::size_t i = kFixedColumns; i < header.size(); ++i) {
            if (!header[i].starts_with("wl_")) return err(1, "column " + std::to_string(i + 1) + " must be wl_<nm>");
            try {
                table.wavelengths(static_cast<Eigen::Index>(i - kFixedColumns)) = std::stod(header[i].substr(3));
            } catch (const std::exception&) {
                return err(1, "\"" + header[i] + "\" is not a wavelength column");
            }
        }

        int lineno = 1;
        while (std::getline(in, line)) {
            ++lineno;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            const auto cells = detail::splitCsvLine(line);
            if (cells.size() != header.size()) {
                return err(lineno, "expected " + std::to_string(header.size()) + " columns, found " + std::to_string(cells.size()));
            }

            MeasuredRow row;
            row.pose_id = cells[0];
            row.spectrum.resize(table.wavelengths.size());
            try {
                row.spot_id = std::stoi(cells[1]);
                row.sample_mm = Vec2(std::stod(cells[2]), std::stod(cells[3]));
                row.pixel = Vec2(std::stod(cells[4]), std::stod(cells[5]));
                for (int k = 0; k < 4; ++k) row.angles_deg[static_cast<std::size_t>(k)] = std::stod(cells[6 + static_cast<std::size_t>(k)]);
                for (Eigen::Index i = 0; i < table.wavelengths.size(); ++i) {
                    row.spectrum(i) = std::stod(cells[kFixedColumns + static_cast<std::size_t>(i)]);
                }
            } catch (const std::exception&) {
                return err(lineno, "a value is not a number");
            }
            table.rows.push_back(std::move(row));
        }
        if (table.rows.empty()) return err(lineno, "the file has no data rows");
        return table;
    }

    Result<MeasurementTable> loadMeasurementsCsv(const std::filesystem::path& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return fail(ErrorCode::InvalidArgument, "cannot open " + path.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        return parseMeasurementsCsv(ss.str(), path.filename().string());
    }

} // namespace brdf::io
