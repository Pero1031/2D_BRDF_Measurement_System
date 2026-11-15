/*
* @file include/brdf/geometry/Segment.hpp
* @brief Line segment and the distance between two segments.
*/

#pragma once

#include "brdf/core/Algebra.hpp"

namespace brdf::geometry {

    /// 線分(アームのリンクを細い棒として近似するのに使う)。
    struct Segment {
        Vec3 a;
        Vec3 b;
    };

    /// 2 本の線分の最短距離 [mm]。
    Real segmentDistance(const Segment& s1, const Segment& s2);

} // namespace brdf::geometry
