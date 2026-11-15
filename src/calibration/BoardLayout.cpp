/*
* @file src/calibration/BoardLayout.cpp
* @brief Geometry of the calibration board.
*/

#include "brdf/calibration/BoardLayout.hpp"

namespace brdf::calibration {

    std::vector<Vec2> BoardLayout::circleCenters() const {
        std::vector<Vec2> out;
        out.reserve(static_cast<std::size_t>(count()));
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < columns; ++i) {
                out.emplace_back(first_x + i * pitch_x, first_y + j * pitch_y);
            }
        }
        return out;
    }

    BoardLayout BoardLayout::legacy2020() {
        BoardLayout b;
        b.columns = 11;
        b.rows = 7;
        b.pitch_x = 30.0;
        b.pitch_y = 40.0;
        b.paper_width = 467.0;
        b.paper_height = 317.0;
        // 旧 Paperparam.h (2020): XDEF=55, YDEF=35
        b.first_x = b.paper_width  - 55.0 - (b.columns - 1) * b.pitch_x;   // 112
        b.first_y = b.paper_height - 35.0 - (b.rows - 1) * b.pitch_y;      // 42
        b.circle_diameter = 17.0;
        return b;
    }

    BoardLayout BoardLayout::legacy2023() {
        BoardLayout b;   // 既定値がそのまま 2023 年のボード
        b.columns = 15;
        b.rows = 7;
        b.pitch_x = 25.0;
        b.pitch_y = 25.0;
        b.paper_width = 450.0;
        b.paper_height = 302.0;
        // 旧 Paperparam.h (2023): XDEF=25, YDEF=51
        b.first_x = b.paper_width  - 25.0 - (b.columns - 1) * b.pitch_x;   // 75
        b.first_y = b.paper_height - 51.0 - (b.rows - 1) * b.pitch_y;      // 101
        return b;
    }

} // namespace brdf::calibration
