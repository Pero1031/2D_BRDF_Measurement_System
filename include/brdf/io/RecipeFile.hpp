/*
* @file include/brdf/io/RecipeFile.hpp
* @brief recipe.toml reading / writing, and import of the previous system's pose CSV.
*/

#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"

namespace brdf::io {

    /// recipe.toml を読む。測定点は [[spots]] の一覧と [spot_grid] のグリッドの両方で書ける。
    /// 検査: ポーズ・測定点が 1 つ以上、ID が重複しない、数値が有限、速度が (0,1]、波長範囲が正しい、など。
    Result<domain::Recipe> parseRecipe(std::string_view text, std::string_view origin = "recipe.toml");

    Result<domain::Recipe> loadRecipe(const std::filesystem::path& path);

    /// コメント付きの TOML に書き出す(測定点は展開した一覧で書く)。
    std::string toToml(const domain::Recipe& recipe);

    /// 旧システムのポーズ CSV(ヘッダ RJ1..RJ6,LJ1..LJ6、角度は度)を読む。
    /// RJ = 右アーム(試料側)、LJ = 左アーム(光源側)。ポーズの ID は 1 からの行番号。
    Result<std::vector<domain::PoseStep>> parseLegacyPoseCsv(std::string_view text,
                                                              std::string_view origin = "poses.csv");

    Result<std::vector<domain::PoseStep>> loadLegacyPoseCsv(const std::filesystem::path& path);

} // namespace brdf::io
