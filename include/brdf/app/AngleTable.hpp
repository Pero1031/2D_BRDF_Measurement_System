/*
* @file include/brdf/app/AngleTable.hpp
* @brief Per-spot incidence / observation angles for the poses of a recipe (pure geometry).
*/

#pragma once

#include <vector>

#include "brdf/domain/Recipe.hpp"
#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/optics/Angles.hpp"

namespace brdf::app {

    /// 1 つの測定点の、ワールド座標での位置と角度。
    struct PatchAngles {
        int spot_id = 0;
        Vec2 sample_mm = Vec2::Zero();      ///< 試料面の座標(紙の座標系)
        Vec3 world_point = Vec3::Zero();    ///< ワールド座標
        optics::GeometryAngles angles;      ///< 入射角・観察角(ラジアン、紙の座標系)
    };

    /// 試料面の座標 (x, y) → ワールド座標。試料面の基準点 + 紙の X, Y 軸方向。
    Vec3 samplePointToWorld(const geometry::SurfaceFrame& paper_frame, const Vec2& sample_mm);

    /// 1 つのポーズでの、全測定点の角度。機器は使わず、幾何だけで計算する。
    std::vector<PatchAngles> computePoseAngles(const geometry::RigGeometry& geometry,
                                               const domain::PoseStep& pose,
                                               const std::vector<domain::SpotDefinition>& spots);

    /// カメラから試料面の基準点までの距離 [mm](撮影のピント合わせに使う)。
    Real measureDistance(const geometry::RigGeometry& geometry, const domain::PoseStep& pose);

} // namespace brdf::app
