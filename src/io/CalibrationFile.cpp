/*
* @file src/io/CalibrationFile.cpp
* @brief calibration/homographies.csv: one homography per pose.
*/

#include "brdf/io/CalibrationFile.hpp"

#include <format>
#include <fstream>
#include <set>
#include <sstream>

#include "CsvLine.hpp"

namespace brdf::io {

    namespace {

        std::string field(const std::string& s) {
            if (s.find_first_of(",\"\n") == std::string::npos) return s;
            std::string out = "\"";
            for (char c : s) {
                if (c == '"') out += '"';
                out += c;
            }
            return out + "\"";
        }

        const char* const kHeader = "pose,h11,h12,h13,h21,h22,h23,h31,h32,h33,rms_px,max_px,circles";

    } // namespace

    std::string toCsv(const std::vector<PoseCalibration>& calibrations) {
        std::ostringstream o;
        o << kHeader << "\n";
        for (const PoseCalibration& c : calibrations) {
            o << field(c.pose_id);
            for (int r = 0; r < 3; ++r) {
                for (int k = 0; k < 3; ++k) o << ',' << std::format("{:.17g}", c.homography.matrix(r, k));
            }
            o << ',' << std::format("{:.6g}", c.rms_px) << ',' << std::format("{:.6g}", c.max_px)
              << ',' << c.circles << "\n";
        }
        return o.str();
    }

    Result<std::vector<PoseCalibration>> parseCalibrationCsv(std::string_view text, std::string_view origin) {
        auto err = [&](int line, const std::string& m) {
            return fail(ErrorCode::InvalidArgument, std::string(origin) + ":" + std::to_string(line) + ": " + m);
        };

        std::istringstream in{std::string(text)};
        std::string line;
        if (!std::getline(in, line)) return err(1, "the file is empty");
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);
        if (line != kHeader) return err(1, std::string("the header must be ") + kHeader);

        std::vector<PoseCalibration> out;
        std::set<std::string> ids;
        int lineno = 1;
        while (std::getline(in, line)) {
            ++lineno;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;

            const auto cells = detail::splitCsvLine(line);
            if (cells.size() != 13) return err(lineno, "expected 13 columns, found " + std::to_string(cells.size()));

            PoseCalibration c;
            c.pose_id = cells[0];
            try {
                for (int r = 0; r < 3; ++r) {
                    for (int k = 0; k < 3; ++k) c.homography.matrix(r, k) = std::stod(cells[1 + r * 3 + k]);
                }
                c.rms_px = std::stod(cells[10]);
                c.max_px = std::stod(cells[11]);
                c.circles = std::stoi(cells[12]);
            } catch (const std::exception&) {
                return err(lineno, "a value is not a number");
            }
            if (!ids.insert(c.pose_id).second) return err(lineno, "duplicate pose \"" + c.pose_id + "\"");
            out.push_back(std::move(c));
        }
        return out;
    }

    Result<std::vector<PoseCalibration>> loadCalibration(const std::filesystem::path& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) {
            return fail(ErrorCode::InvalidArgument,
                        "cannot open " + path.string() + " (run `brdf calibrate` to create it)");
        }
        std::stringstream ss;
        ss << ifs.rdbuf();
        return parseCalibrationCsv(ss.str(), path.filename().string());
    }

} // namespace brdf::io
