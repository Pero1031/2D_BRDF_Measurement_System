/*
* @file include/brdf/app/Analyze.hpp
* @brief Computes the radiance factor and BRDF from a sample run and a white-plate run.
*/

#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "brdf/core/Error.hpp"

namespace brdf::app {

    struct AnalyzeOptions {
        /// 解析する結果フォルダ(result/ の下の名前)。省略すると、種類ごとに最新のもの。
        std::string sample_run;
        std::string white_run;

        /// 照度の補正を、setup.toml の [analysis] irradiance_correction より優先して指定する。
        std::optional<bool> irradiance_correction;

        std::string timestamp;   ///< 出力フォルダ名の日時。空なら現在時刻
    };

    /// 試料の測定と標準拡散反射板の測定から、放射輝度率と BRDF を計算して
    /// result/<日時>_analysis/ に保存する(radiance_factor.csv、brdf.csv、analysis_manifest.toml)。
    ///
    /// 検査: 2 つの測定が、それぞれ sample / white の測定であること、同じポーズ・同じ装置の配置で測られていること
    /// (違えば、何が違うかを示してエラー)。機器は使わない。
    ///
    /// 解析の設定(反射板の反射率、波長の範囲、照度の補正)は、setup.toml の [analysis]。
    Result<std::filesystem::path> analyzeProject(const std::filesystem::path& dir, const AnalyzeOptions& options = {});

} // namespace brdf::app
