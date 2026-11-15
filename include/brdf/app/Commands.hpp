/*
* @file include/brdf/app/Commands.hpp
* @brief The operations behind the `brdf` command line: init, check, run, angles, import-poses.
*/

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>

#include "brdf/app/Preflight.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/domain/Recipe.hpp"
#include "brdf/io/SetupConfig.hpp"

namespace brdf::app {

    /// プロジェクトフォルダから読み込んだ設定とレシピ。
    struct LoadedProject {
        io::SetupConfig setup;
        domain::Recipe recipe;
    };

    /// setup.toml と recipe.toml を読む。
    Result<LoadedProject> loadProject(const std::filesystem::path& dir);

    /// 現在時刻を "20261116_153000" の形で返す。
    std::string currentTimestamp();

    /// 初期のレシピ(旧システムの最初の 3 ポーズ、3×3 の測定点、確認項目)。init と import-poses が使う。
    domain::Recipe defaultRecipe();

    /// プロジェクトフォルダを作り、setup.toml と recipe.toml のひな型を書く。
    /// 既に setup.toml / recipe.toml があれば、上書きせずエラーにする。
    Result<std::filesystem::path> initProject(const std::filesystem::path& dir);

    /// 実機を動かさずに、全ポーズの可動域・干渉・角度を検査する。
    Result<PreflightReport> checkProject(const std::filesystem::path& dir);

    struct RunOptions {
        std::optional<domain::MeasurementKind> kind;   ///< 指定すると、レシピの種類を上書きする
        bool assume_yes = false;                       ///< 測定前の確認(OK の入力)を省く
        std::string timestamp;                         ///< 結果フォルダ名の日時。空なら現在時刻
    };

    /// 測定を実行して、結果フォルダのパスを返す。
    /// 流れ: 設定の読み込み → 事前検査 → 確認項目 → 装置の組み立て → 測定 → 保存。
    /// 確認項目への OK は in から読む。
    Result<std::filesystem::path> runProject(const std::filesystem::path& dir, const RunOptions& options,
                                             std::istream& in);

    /// 機器を使わず、角度だけを result/<日時>_angles/angles.csv に出力する。
    Result<std::filesystem::path> exportAngles(const std::filesystem::path& dir, const std::string& timestamp = "");

    /// 旧システムのポーズ CSV から recipe.toml を作る。既にあれば force のときだけ上書きする。
    Status importPoses(const std::filesystem::path& legacy_csv, const std::filesystem::path& recipe_out, bool force);

    /// 事前検査の結果をログに出す。
    void logReport(const PreflightReport& report);

} // namespace brdf::app
