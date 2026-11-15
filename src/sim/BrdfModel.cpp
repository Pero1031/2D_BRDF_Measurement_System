/*
* @file src/sim/BrdfModel.cpp
* @brief Known BRDF models used as ground truth by the simulator.
*/

#include "brdf/sim/BrdfModel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "brdf/core/MathUtils.hpp"

namespace brdf::sim {

    SpectralCurve flatCurve(Real value) {
        return [value](Real) { return value; };
    }

    SpectralCurve gaussianCurve(Real center_nm, Real width_nm, Real peak, Real base) {
        return [=](Real wl) {
            const Real x = (wl - center_nm) / width_nm;
            return base + peak * std::exp(-x * x);
        };
    }

    LambertBrdf::LambertBrdf(SpectralCurve albedo) : albedo_(std::move(albedo)) {}

    Real LambertBrdf::value(Real wavelength_nm, const optics::GeometryAngles&) const {
        return albedo_(wavelength_nm) * INV_PI;
    }

    PhongBrdf::PhongBrdf(SpectralCurve diffuse, Real specular, Real shininess)
        : diffuse_(std::move(diffuse)), specular_(specular), shininess_(shininess) {}

    Real PhongBrdf::value(Real wavelength_nm, const optics::GeometryAngles& a) const {
        const Vec3 wi = sphericalToCartesian(a.incidence.theta, a.incidence.phi);
        const Vec3 wo = sphericalToCartesian(a.observation.theta, a.observation.phi);
        const Vec3 reflected(-wi.x(), -wi.y(), wi.z());   // 面の法線 (Z軸) について鏡面反射

        const Real cos_alpha = std::max(Real(0), reflected.dot(wo));
        const Real lobe = (shininess_ + 2.0) / TWO_PI * std::pow(cos_alpha, shininess_);

        return diffuse_(wavelength_nm) * INV_PI + specular_ * lobe;
    }

} // namespace brdf::sim
