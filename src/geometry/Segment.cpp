/*
* @file src/geometry/Segment.cpp
* @brief Distance between two line segments.
*/

#include "brdf/geometry/Segment.hpp"

#include <algorithm>

namespace brdf::geometry {

    namespace {
        Real clamp01(Real v) { return std::clamp(v, Real(0), Real(1)); }
    }

    // Ericson, "Real-Time Collision Detection", closest point between two segments.
    Real segmentDistance(const Segment& s1, const Segment& s2) {
        const Vec3 d1 = s1.b - s1.a;
        const Vec3 d2 = s2.b - s2.a;
        const Vec3 r = s1.a - s2.a;
        const Real a = d1.dot(d1);
        const Real e = d2.dot(d2);
        const Real f = d2.dot(r);
        const Real eps = 1e-12;

        Real s = 0, t = 0;
        if (a <= eps && e <= eps) {
            return r.norm();                       // 両方とも点
        }
        if (a <= eps) {
            t = clamp01(f / e);                    // s1 が点
        } else {
            const Real c = d1.dot(r);
            if (e <= eps) {
                s = clamp01(-c / a);               // s2 が点
            } else {
                const Real b = d1.dot(d2);
                const Real denom = a * e - b * b;  // 平行なら 0
                s = denom > eps ? clamp01((b * f - c * e) / denom) : 0;
                t = (b * s + f) / e;
                if (t < 0) {
                    t = 0;
                    s = clamp01(-c / a);
                } else if (t > 1) {
                    t = 1;
                    s = clamp01((b - c) / a);
                }
            }
        }
        return ((s1.a + d1 * s) - (s2.a + d2 * t)).norm();
    }

} // namespace brdf::geometry
