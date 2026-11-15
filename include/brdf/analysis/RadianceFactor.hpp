/*
* @file include/brdf/analysis/RadianceFactor.hpp
* @brief Radiance factor (sample / white plate) and the BRDF derived from it.
*/

#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/io/MeasurementsCsv.hpp"

namespace brdf::analysis {

    struct AnalysisOptions {
        /// 標準拡散反射板の反射率 ρ(全波長で一定とみなす)。
        Real white_reflectance = 0.99;
        /// 出力する波長の範囲 [nm]。LED の出力が足りない 380 nm 付近や 770 nm 付近は信頼性が低い
        /// (旧システムの論文)ので、必要なら狭める。
        Real min_wavelength_nm = 380.0;
        Real max_wavelength_nm = 780.0;
    };

    /// 試料の測定点 P での照度が、反射板の測定点(基準点)での照度の何倍か: E(P) / E(基準点)。
    /// 試料と反射板は同じポーズで測るが、測る位置が違うため、照度が違う(旧システムの系統誤差)。
    /// この関数を渡すと、その分を補正する。渡さなければ補正しない(旧システムと同じ)。
    using IrradianceRatioFn = std::function<Result<Real>(const std::string& pose_id, const Vec2& sample_mm)>;

    struct RadianceFactorRow {
        std::string pose_id;
        int spot_id = 0;
        Vec2 sample_mm = Vec2::Zero();
        std::array<Real, 4> angles_deg{};      ///< θi, φi, θo, φo [deg]
        VecX radiance_factor;                  ///< 放射輝度率 L_sample / L_white(波長ごと)。無効な値は NaN
        VecX brdf;                             ///< BRDF [1/sr] = 放射輝度率 × ρ / π
    };

    struct RadianceFactorTable {
        VecX wavelengths;                      ///< 出力する波長(範囲で絞り込み済み)
        std::vector<RadianceFactorRow> rows;
        int invalid_values = 0;                ///< NaN にした値の数(反射板の放射輝度が 0 以下、など)
    };

    /// 放射輝度率と BRDF を計算する。
    ///
    ///   放射輝度率 f_ref(λ) = L_sample(P, λ) / L_white(λ)        同じポーズの、試料と反射板
    ///   BRDF       f(λ)     = f_ref(λ) × ρ / π                   反射板を完全拡散(Lambert)とみなす
    ///   (照度の補正をするときは、f_ref を E(P) / E(基準点) で割る)
    ///
    /// 反射板は、ポーズごとに基準点(クロスヘア位置)で測った値を代表値とする。同じポーズに複数の行があれば、平均する。
    /// 試料のどのポーズにも反射板の測定が必要。波長が一致しない、ポーズが足りない場合はエラー。
    Result<RadianceFactorTable> computeRadianceFactor(const io::MeasurementTable& sample,
                                                      const io::MeasurementTable& white,
                                                      const AnalysisOptions& options = {},
                                                      const IrradianceRatioFn& irradiance_ratio = {});

    enum class Quantity { RadianceFactor, Brdf };

    /// CSV に書き出す。列: pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg,rf_<nm>... または brdf_<nm>...
    std::string toCsv(const RadianceFactorTable& table, Quantity quantity);

} // namespace brdf::analysis
