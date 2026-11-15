/*
* @file src/io/SpotXml.cpp
* @brief SR-5000 spot definition files (SpotDef XML).
*/

#include "brdf/io/SpotXml.hpp"

#include <cctype>
#include <cmath>
#include <format>
#include <sstream>

namespace brdf::io {

    namespace {

        std::string escape(const std::string& s) {
            std::string out;
            for (char c : s) {
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

        // 文字列 s の中で、pos 以降の最初の `name="..."` の値を取り出す。見つからなければ空。
        bool attribute(const std::string& s, std::size_t pos, std::size_t end, const std::string& name, std::string& value) {
            const std::string key = name + "=\"";
            const std::size_t a = s.find(key, pos);
            if (a == std::string::npos || a >= end) return false;
            const std::size_t b = s.find('"', a + key.size());
            if (b == std::string::npos || b >= end) return false;
            value = s.substr(a + key.size(), b - (a + key.size()));
            return true;
        }

    } // namespace

    std::string toSpotXml(const SpotXmlSettings& s, const std::vector<driver::SpotRequest>& spots) {
        const std::string nl = "\r\n";
        std::ostringstream o;
        o << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << nl
          << "<SpotDef version=\"1.0\" type=\"RDM\" datetime=\"" << escape(s.datetime) << "\" comment=\""
          << escape(s.comment) << "\">" << nl
          << "    <Setting>" << nl
          << "        <MeasurementRoi x=\"0\" y=\"0\" width=\"" << s.roi_width << "\" height=\"" << s.roi_height << "\"/>" << nl
          << "        <Binning type=\"" << escape(s.binning) << "\"/>" << nl
          << "        <Trimming x=\"0\" y=\"0\" width=\"0\" height=\"0\"/>" << nl
          << "        <InitSpot shape=\"SQUARE\" width=\"1.00\" height=\"1.00\"/>" << nl
          << "        <Renumbering enable=\"TRUE\"/>" << nl
          << "        <ApplyThreshold enable=\"FALSE\"/>" << nl
          << "    </Setting>" << nl
          << "    <SpotAreaList>" << nl;

        const std::string size = std::format("{:.2f}", s.spot_size);
        for (const driver::SpotRequest& spot : spots) {
            o << "        <AreaData id=\"" << spot.id << "\" enable=\"true\" shape=\"SQUARE\">" << nl
              << "            <AreaCenter x=\"" << std::lround(spot.pixel.x()) << "\" y=\"" << std::lround(spot.pixel.y()) << "\"/>" << nl
              << "            <AreaSize width=\"" << size << "\" height=\"" << size << "\"/>" << nl
              << "            <ThresHold x=\"0.000000\" y=\"0.000000\" z=\"0.000000\"/>" << nl
              << "            <Judgment base=\"FALSE\" xy=\"0\" uv=\"0\" ch=\"0\"/>" << nl
              << "        </AreaData>" << nl;
        }

        o << "    </SpotAreaList>" << nl
          << "</SpotDef>" << nl;
        return o.str();
    }

    Result<std::vector<driver::SpotRequest>> parseSpotXml(std::string_view text, std::string_view origin) {
        const std::string s(text);
        auto err = [&](const std::string& m) {
            return fail(ErrorCode::InvalidArgument, std::string(origin) + ": " + m);
        };
        if (s.find("<SpotDef") == std::string::npos) return err("this is not a SpotDef file (no <SpotDef> element)");

        std::vector<driver::SpotRequest> out;
        std::size_t pos = 0;
        while ((pos = s.find("<AreaData ", pos)) != std::string::npos) {
            const std::size_t end = s.find("</AreaData>", pos);
            if (end == std::string::npos) return err("an <AreaData> element is not closed");

            std::string id, x, y;
            const std::size_t tag_end = s.find('>', pos);
            if (!attribute(s, pos, tag_end, "id", id)) return err("an <AreaData> element has no id");
            const std::size_t centre = s.find("<AreaCenter", pos);
            if (centre == std::string::npos || centre > end) return err("<AreaData id=\"" + id + "\"> has no <AreaCenter>");
            if (!attribute(s, centre, end, "x", x) || !attribute(s, centre, end, "y", y)) {
                return err("<AreaCenter> of spot " + id + " needs x and y");
            }

            driver::SpotRequest spot;
            try {
                spot.id = std::stoi(id);
                spot.pixel = Vec2(std::stod(x), std::stod(y));
            } catch (const std::exception&) {
                return err("spot " + id + " has a value that is not a number");
            }
            out.push_back(spot);
            pos = end;
        }
        return out;
    }

    std::string spotXmlDatetime(std::string_view t) {
        // "YYYYMMDD_HHMMSS"
        if (t.size() != 15 || t[8] != '_') return std::string(t);
        for (std::size_t i = 0; i < t.size(); ++i) {
            if (i != 8 && !std::isdigit(static_cast<unsigned char>(t[i]))) return std::string(t);
        }
        return std::format("{}/{}/{} {}:{}:{}.000", t.substr(0, 4), t.substr(4, 2), t.substr(6, 2),
                           t.substr(9, 2), t.substr(11, 2), t.substr(13, 2));
    }

} // namespace brdf::io
