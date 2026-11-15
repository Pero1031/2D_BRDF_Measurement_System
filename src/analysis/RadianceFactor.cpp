/*
* @file src/analysis/RadianceFactor.cpp
* @brief Radiance factor (sample / white plate) and the BRDF derived from it.
*/

#include "brdf/analysis/RadianceFactor.hpp"

#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <sstream>

#include "brdf/core/MathUtils.hpp"

namespace brdf::analysis {

    namespace {

        std::string g(Real v) { return std::format("{:.9g}", v); }

        std::string cell(const std::string& s) {
            if (s.find_first_of(",\"\n") == std::string::npos) return s;
            std::string out = "\"";
            for (char c : s) {
                if (c == '"') out += '"';
                out += c;
            }
            return out + "\"";
        }

    } // namespace

    Result<RadianceFactorTable> computeRadianceFactor(const io::MeasurementTable& sample,
                                                      const io::MeasurementTable& white,
                                                      const AnalysisOptions& options,
                                                      const IrradianceRatioFn& irradiance_ratio) {
        if (!(options.white_reflectance > 0.0)) {
            return fail(ErrorCode::InvalidArgument, "analysis: the white plate reflectance must be positive");
        }
        if (!(options.min_wavelength_nm < options.max_wavelength_nm)) {
            return fail(ErrorCode::InvalidArgument, "analysis: min_wavelength_nm must be smaller than max_wavelength_nm");
        }

        // 波長が一致していること
        if (sample.wavelengths.size() != white.wavelengths.size()
            || (sample.wavelengths - white.wavelengths).cwiseAbs().maxCoeff() > 1e-9) {
            return fail(ErrorCode::InvalidArgument,
                        "analysis: the sample and the white plate were measured at different wavelengths");
        }

        // 出力する波長(範囲で絞り込む)
        std::vector<Eigen::Index> keep;
        for (Eigen::Index i = 0; i < sample.wavelengths.size(); ++i) {
            if (sample.wavelengths(i) >= options.min_wavelength_nm && sample.wavelengths(i) <= options.max_wavelength_nm) {
                keep.push_back(i);
            }
        }
        if (keep.empty()) {
            return fail(ErrorCode::InvalidArgument, "analysis: no measured wavelength is inside the selected range");
        }

        // 反射板の代表値: ポーズごとの平均
        struct Sum { VecX total; int count = 0; };
        std::map<std::string, Sum> white_by_pose;
        for (const io::MeasuredRow& row : white.rows) {
            Sum& s = white_by_pose[row.pose_id];
            if (s.count == 0) s.total = VecX::Zero(white.wavelengths.size());
            s.total += row.spectrum;
            ++s.count;
        }

        RadianceFactorTable out;
        out.wavelengths.resize(static_cast<Eigen::Index>(keep.size()));
        for (std::size_t k = 0; k < keep.size(); ++k) out.wavelengths(static_cast<Eigen::Index>(k)) = sample.wavelengths(keep[k]);

        // 試料のポーズすべてに、反射板の測定があること
        std::string missing;
        {
            std::map<std::string, bool> seen;
            for (const io::MeasuredRow& row : sample.rows) {
                if (!white_by_pose.contains(row.pose_id) && !seen[row.pose_id]) {
                    seen[row.pose_id] = true;
                    missing += (missing.empty() ? "" : ", ") + ("\"" + row.pose_id + "\"");
                }
            }
        }
        if (!missing.empty()) {
            return fail(ErrorCode::InvalidArgument, "analysis: the white plate was not measured at pose(s) " + missing);
        }

        const Real nan = std::numeric_limits<Real>::quiet_NaN();
        const Real brdf_scale = options.white_reflectance * INV_PI;

        for (const io::MeasuredRow& row : sample.rows) {
            const Sum& w = white_by_pose.at(row.pose_id);

            Real correction = 1.0;
            if (irradiance_ratio) {
                auto ratio = irradiance_ratio(row.pose_id, row.sample_mm);
                if (!ratio) return tl::unexpected(ratio.error());
                correction = *ratio;
            }

            RadianceFactorRow r;
            r.pose_id = row.pose_id;
            r.spot_id = row.spot_id;
            r.sample_mm = row.sample_mm;
            r.angles_deg = row.angles_deg;
            r.radiance_factor.resize(static_cast<Eigen::Index>(keep.size()));
            r.brdf.resize(static_cast<Eigen::Index>(keep.size()));

            for (std::size_t k = 0; k < keep.size(); ++k) {
                const Eigen::Index i = keep[k];
                const Real white_radiance = w.total(i) / w.count;
                Real value = nan;
                if (white_radiance > 0.0 && std::isfinite(white_radiance) && correction > 0.0 && std::isfinite(correction)) {
                    value = row.spectrum(i) / white_radiance / correction;
                }
                if (std::isnan(value) || std::isinf(value)) {
                    value = nan;
                    ++out.invalid_values;
                }
                r.radiance_factor(static_cast<Eigen::Index>(k)) = value;
                r.brdf(static_cast<Eigen::Index>(k)) = value * brdf_scale;
            }
            out.rows.push_back(std::move(r));
        }
        return out;
    }

    std::string toCsv(const RadianceFactorTable& table, Quantity quantity) {
        const char* prefix = quantity == Quantity::RadianceFactor ? "rf_" : "brdf_";
        std::ostringstream o;
        o << "pose,spot,x_mm,y_mm,theta_i_deg,phi_i_deg,theta_o_deg,phi_o_deg";
        for (Eigen::Index i = 0; i < table.wavelengths.size(); ++i) o << ',' << prefix << g(table.wavelengths(i));
        o << "\n";

        for (const RadianceFactorRow& r : table.rows) {
            o << cell(r.pose_id) << ',' << r.spot_id << ',' << g(r.sample_mm.x()) << ',' << g(r.sample_mm.y());
            for (Real a : r.angles_deg) o << ',' << g(a);
            const VecX& v = quantity == Quantity::RadianceFactor ? r.radiance_factor : r.brdf;
            for (Eigen::Index i = 0; i < v.size(); ++i) o << ',' << g(v(i));
            o << "\n";
        }
        return o.str();
    }

} // namespace brdf::analysis
