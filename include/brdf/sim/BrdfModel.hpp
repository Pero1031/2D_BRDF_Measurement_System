/*
* @file include/brdf/sim/BrdfModel.hpp
* @brief Known BRDF models used as ground truth by the simulator.
*/

#pragma once

#include <functional>

#include "brdf/core/Algebra.hpp"
#include "brdf/optics/Angles.hpp"

namespace brdf::sim {

    /// 波長 [nm] → 値(反射率など)の関数。
    using SpectralCurve = std::function<Real(Real)>;

    /// 波長によらず一定の値。
    SpectralCurve flatCurve(Real value);

    /// 山形(ガウス形)の分光反射率。base + peak · exp(-((λ - center) / width)²)
    SpectralCurve gaussianCurve(Real center_nm, Real width_nm, Real peak, Real base = 0.0);

    /// シミュレータが「真の値」として使う BRDF。単位は [1/sr]。
    /// 角度は試料面の座標系(紙の座標系)で与える。
    class IBrdfModel {
    public:
        virtual ~IBrdfModel() = default;
        virtual Real value(Real wavelength_nm, const optics::GeometryAngles& angles) const = 0;
    };

    /// 完全拡散面(Lambert)。f = ρ(λ) / π。角度によらない。
    class LambertBrdf : public IBrdfModel {
    public:
        explicit LambertBrdf(SpectralCurve albedo);
        Real value(Real wavelength_nm, const optics::GeometryAngles& angles) const override;

    private:
        SpectralCurve albedo_;
    };

    /// 拡散 + 鏡面の簡易モデル(正規化 Phong)。
    ///   f = kd(λ)/π + ks · (n+2)/(2π) · max(0, R·wo)^n
    /// R は入射方向を面で鏡面反射させた向き、wo は観察方向、n は鋭さ(大きいほど鋭いハイライト)。
    class PhongBrdf : public IBrdfModel {
    public:
        PhongBrdf(SpectralCurve diffuse, Real specular, Real shininess);
        Real value(Real wavelength_nm, const optics::GeometryAngles& angles) const override;

    private:
        SpectralCurve diffuse_;
        Real specular_;
        Real shininess_;
    };

} // namespace brdf::sim
