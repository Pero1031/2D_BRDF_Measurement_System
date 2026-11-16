/*
* @file include/brdf/io/TopconRecipeXml.hpp
* @brief TOPCON SDK recipe files (recipe XML): the measurement conditions handed to hsLoadRecipe().
*/

#pragma once

#include <string>

#include "brdf/core/Algebra.hpp"
#include "brdf/driver/ISpectralImager.hpp"

namespace brdf::io {

    /// TOPCON SDK のレシピ(測定条件)のうち、新システムが決める項目。
    /// ほかの項目(補正はすべて無効、トリガは SOFT、走査は NORMAL など)は旧システムのレシピと同じ固定値。
    struct TopconRecipeSettings {
        std::string device = "SR5100";               ///< "SR5100" または "SR5000"
        std::string binning = "BINNING_TYPE1";       ///< "BINNING_TYPE1" / "BINNING_TYPE2" / "BINNING_TYPE4"
        std::string lens = "STANDARD";               ///< "WIDE" / "STANDARD" / "TELE" / "MACRO"
        int measure_distance_mm = 1000;              ///< 測定距離(カメラから試料まで)[mm]
        Real integration_time_ms = 0.0;              ///< 積分時間。0 以下のときは AUTO(旧システムと同じ)
        driver::WavelengthRange wavelengths;         ///< 波長範囲 [nm](整数の nm)
        int roi_width = 2448;                        ///< 測定範囲 = 画像全体 [pixel]
        int roi_height = 2048;
        std::string datetime;                        ///< 作成日時(空でよい)
        std::string comment;
    };

    /// 旧システムが使っていた recipe.xml と同じ書式のレシピを作る(UTF-8、CRLF、タブのインデント)。
    std::string toTopconRecipeXml(const TopconRecipeSettings& settings);

} // namespace brdf::io
