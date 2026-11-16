/*
* @file src/io/TopconRecipeXml.cpp
* @brief TOPCON SDK recipe XML.
*/

#include "brdf/io/TopconRecipeXml.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

namespace brdf::io {

    namespace {

        std::string escape(const std::string& s) {
            std::string out;
            for (const char c : s) {
                switch (c) {
                case '&': out += "&amp;"; break;
                case '<': out += "&lt;"; break;
                case '>': out += "&gt;"; break;
                case '"': out += "&quot;"; break;
                default: out += c;
                }
            }
            return out;
        }

        /// 100 → "100"、37.5 → "37.5"(余計な 0 を付けない)
        std::string number(Real v) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.6g", v);
            return buf;
        }

        int rounded(Real v) { return static_cast<int>(std::lround(v)); }

    } // namespace

    std::string toTopconRecipeXml(const TopconRecipeSettings& s) {
        const bool automatic = !(s.integration_time_ms > 0.0);
        const std::string mode = automatic ? "AUTO" : "MANUAL";
        // AUTO のときの積分時間は、最適化を始めるときの初期値(旧システムのレシピは 100)
        const std::string time = number(automatic ? 100.0 : s.integration_time_ms);

        std::ostringstream o;
        const char* const nl = "\r\n";
        o << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << nl;
        o << "<Recipe version=\"1.1\" datetime=\"" << escape(s.datetime) << "\" comment=\"" << escape(s.comment) << "\">" << nl;
        o << "\t<BaseWhitePoint type=\"D65\" x=\"0.00000\" y=\"0.00000\"/>" << nl;
        o << "\t<MeasurementCondition>" << nl;
        o << "\t\t<DeviceType type=\"" << escape(s.device) << "\"/>" << nl;
        o << "\t\t<BinningType type=\"" << escape(s.binning) << "\"/>" << nl;
        o << "\t\t<LensType type=\"" << escape(s.lens) << "\"/>" << nl;
        o << "\t\t<MeasureDistance mm=\"" << s.measure_distance_mm << "\"/>" << nl;
        o << "\t\t<MeasureTimingType type=\"SERIAL\">" << nl;
        o << "\t\t\t<Serial count=\"1\"/>" << nl;
        o << "\t\t\t<IntervalConditionList>" << nl;
        for (int i = 0; i < 5; ++i) o << "\t\t\t\t<IntervalCondition sec=\"0\" count=\"0\"/>" << nl;
        o << "\t\t\t</IntervalConditionList>" << nl;
        o << "\t\t\t<OptimizeFirstOnly enable=\"FALSE\"/>" << nl;
        o << "\t\t\t<IntervalInterrupt enable=\"FALSE\"/>" << nl;
        o << "\t\t</MeasureTimingType>" << nl;
        o << "\t\t<LayerMode enable=\"FALSE\" count=\"2\"/>" << nl;
        o << "\t\t<MeasurementType type=\"SPECTRAL\"/>" << nl;
        o << "\t\t<FilterCondition>" << nl;

        const auto filter = [&](const char* name, const char* gain) {
            o << "\t\t\t<" << name << " enable=\"TRUE\" integration_time_type=\"" << mode << "\" integration_time=\"" << time
              << "\" gain_type=\"AUTO\" gain=\"" << gain
              << "\" frequency_calc_enable=\"FALSE\" frequency_value=\"50\" averaging_count=\"1\"/>" << nl;
        };
        filter("SpectralFilter", "");
        filter("XFilter", "GAIN_TYPE1");
        filter("YFilter", "GAIN_TYPE1");
        filter("ZFilter", "GAIN_TYPE1");

        o << "\t\t</FilterCondition>" << nl;
        o << "\t\t<SaturationDetectionType type=\"INTERRUPT\"/>" << nl;
        o << "\t\t<TriggerConfig mode_type=\"SOFT\" activation_type=\"RISING\" delay_time=\"0\"/>" << nl;
        o << "\t\t<ScanType type=\"NORMAL\"/>" << nl;
        o << "\t\t<MeasurementWaveLength min=\"" << rounded(s.wavelengths.start) << "\" max=\"" << rounded(s.wavelengths.end)
          << "\" step=\"" << rounded(s.wavelengths.step) << "\"/>" << nl;
        o << "\t\t<MeasurementRange left=\"0\" top=\"0\" right=\"" << s.roi_width << "\" bottom=\"" << s.roi_height << "\"/>" << nl;
        o << "\t\t<OptimizationArea left=\"0\" top=\"0\" right=\"" << s.roi_width << "\" bottom=\"" << s.roi_height << "\"/>" << nl;
        o << "\t\t<ImageRotationType type=\"NONE\"/>" << nl;
        o << "\t</MeasurementCondition>" << nl;
        o << "\t<DiagonalCorrection enable=\"FALSE\"/>" << nl;
        o << "\t<ColorCorrection enable=\"FALSE\"/>" << nl;
        o << "\t<LuminosityCorrection enable=\"FALSE\"/>" << nl;
        o << "\t<AreaCorrection enable=\"FALSE\"/>" << nl;
        o << "\t<WhitePlateData enable=\"FALSE\" filename=\"\" xn=\"0\" yn=\"0\" zn=\"0\" type=\"FILE\"/>" << nl;
        o << "</Recipe>" << nl;
        return o.str();
    }

} // namespace brdf::io
