/*
* @file src/calibration/SampleToBoard.cpp
* @brief How the calibration board is mounted relative to the sample coordinates.
*/

#include "brdf/calibration/SampleToBoard.hpp"

#include <string>

namespace brdf::calibration {

    namespace {

        // "+x" "-x" "+y" "-y" → 試料座標での単位ベクトル
        bool parseAxis(std::string_view name, Vec2& out) {
            if (name == "+x" || name == "x")   { out = Vec2(1, 0);  return true; }
            if (name == "-x")                  { out = Vec2(-1, 0); return true; }
            if (name == "+y" || name == "y")   { out = Vec2(0, 1);  return true; }
            if (name == "-y")                  { out = Vec2(0, -1); return true; }
            return false;
        }

    } // namespace

    Result<SampleToBoard> SampleToBoard::make(std::string_view board_u, std::string_view board_v, const Vec2& origin) {
        Vec2 u, v;
        if (!parseAxis(board_u, u)) {
            return fail(ErrorCode::InvalidArgument,
                        "board_u must be one of \"+x\", \"-x\", \"+y\", \"-y\" (got \"" + std::string(board_u) + "\")");
        }
        if (!parseAxis(board_v, v)) {
            return fail(ErrorCode::InvalidArgument,
                        "board_v must be one of \"+x\", \"-x\", \"+y\", \"-y\" (got \"" + std::string(board_v) + "\")");
        }
        if (u.dot(v) != 0.0) {
            return fail(ErrorCode::InvalidArgument, "board_u and board_v must be different axes (one x, one y)");
        }

        SampleToBoard m;
        m.axes.row(0) = u.transpose();
        m.axes.row(1) = v.transpose();
        m.origin = origin;
        return m;
    }

} // namespace brdf::calibration
