/*
* @file include/brdf/io/SpotXml.hpp
* @brief SR-5000 / SR-5100 spot definition files (SpotDef XML): which image positions to extract spectra from.
*/

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "brdf/core/Error.hpp"
#include "brdf/driver/ISpectralImager.hpp"

namespace brdf::io {

    /// スポット定義ファイルの設定。
    struct SpotXmlSettings {
        int roi_width = 1376;                       ///< 測定範囲(画像の大きさ)[pixel]
        int roi_height = 1024;
        std::string binning = "BINNING_TYPE1";      ///< 分光放射計のビニング設定(レシピと合わせる)
        Real spot_size = 2.0;                       ///< 各スポットの一辺 [pixel]
        std::string datetime = "2020/01/01 00:00:00.000";
        std::string comment = "Random-Spot";
    };

    /// TOPCON SR-5000 / SR-5100 のソフトに読み込ませるスポット定義(SpotDef XML)を作る。
    /// 旧システムが生成していたファイルと同じ書式(UTF-8、CRLF、4 スペースのインデント、末尾に改行)。
    ///
    /// スポットの中心は整数のピクセルに丸める。旧システムは小数を切り捨てていた(平均 0.5 ピクセルのずれ)が、
    /// ここでは最も近いピクセルに丸める。
    std::string toSpotXml(const SpotXmlSettings& settings, const std::vector<driver::SpotRequest>& spots);

    /// スポット定義ファイルから、スポットの ID と中心を読む(旧システムのファイルも読める)。
    Result<std::vector<driver::SpotRequest>> parseSpotXml(std::string_view text, std::string_view origin = "spot.xml");

    /// "20261116_064647" → "2026/11/16 06:46:47.000"。この形でなければ、そのまま返す。
    std::string spotXmlDatetime(std::string_view timestamp);

} // namespace brdf::io
