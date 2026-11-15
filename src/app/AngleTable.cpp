/*
* @file src/app/AngleTable.cpp
* @brief Per-spot incidence / observation angles for the poses of a recipe (pure geometry).
*/

#include "brdf/app/AngleTable.hpp"

namespace brdf::app {

    Vec3 samplePointToWorld(const geometry::SurfaceFrame& frame, const Vec2& sample_mm) {
        return frame.origin + sample_mm.x() * frame.xAxis() + sample_mm.y() * frame.yAxis();
    }

    std::vector<PatchAngles> computePoseAngles(const geometry::RigGeometry& geometry,
                                               const domain::PoseStep& pose,
                                               const std::vector<domain::SpotDefinition>& spots) {
        const geometry::SurfaceFrame frame = geometry.paperFrame(pose.right);
        const Vec3 light = geometry.lightPosition(pose.left);
        const Vec3& camera = geometry.cameraPosition();

        std::vector<PatchAngles> out;
        out.reserve(spots.size());
        for (const domain::SpotDefinition& s : spots) {
            PatchAngles p;
            p.spot_id = s.id;
            p.sample_mm = s.sample_mm;
            p.world_point = samplePointToWorld(frame, s.sample_mm);
            p.angles = optics::computeAngles(frame, p.world_point, light, camera);
            out.push_back(p);
        }
        return out;
    }

    Real measureDistance(const geometry::RigGeometry& geometry, const domain::PoseStep& pose) {
        return (geometry.paperFrame(pose.right).origin - geometry.cameraPosition()).norm();
    }

} // namespace brdf::app
