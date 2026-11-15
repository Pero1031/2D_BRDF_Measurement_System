/*
* @file src/io/CsvLine.hpp
* @brief Splitting one CSV line with quoted cells. (internal)
*/

#pragma once

#include <string>
#include <vector>

namespace brdf::io::detail {

    /// 1 行を、カンマで分割する。引用符("...")で囲まれたセルでは、カンマや "" (引用符 1 つ)を含められる。
    inline std::vector<std::string> splitCsvLine(const std::string& line) {
        std::vector<std::string> cells;
        std::string cur;
        bool quoted = false;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (quoted) {
                if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') { cur += '"'; ++i; }
                else if (c == '"') quoted = false;
                else cur += c;
            } else if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                cells.push_back(cur);
                cur.clear();
            } else {
                cur += c;
            }
        }
        cells.push_back(cur);
        return cells;
    }

} // namespace brdf::io::detail
