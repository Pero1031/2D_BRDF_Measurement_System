/*
* @file include/brdf/app/ProjectLayout.hpp
* @brief Folder layout of a measurement project. Everything is relative to the project folder.
*/

#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "brdf/core/Error.hpp"

namespace brdf::app {

    /// ポーズの名前などを、ファイル名に使える形にする(英数字・- _ . 以外は _ に置き換える)。
    std::string fileSafeName(const std::string& name);

    /// 測定プロジェクトのフォルダ構成。
    ///
    ///   <project>/
    ///     setup.toml         装置の配置
    ///     recipe.toml        測定の手順書
    ///     calibration/       キャリブボードの撮影画像(images/)と、ポーズごとのホモグラフィ(homographies.csv)
    ///     result/<日時>_<種類>/   実行ごとの結果
    ///
    /// プログラムが受け取るパスはプロジェクトフォルダだけ。設定ファイルの中に絶対パスは書かない。
    class ProjectLayout {
    public:
        explicit ProjectLayout(std::filesystem::path root) : root_(std::move(root)) {}

        const std::filesystem::path& root() const { return root_; }
        std::filesystem::path setupFile() const { return root_ / "setup.toml"; }
        std::filesystem::path recipeFile() const { return root_ / "recipe.toml"; }
        std::filesystem::path calibrationDir() const { return root_ / "calibration"; }
        std::filesystem::path calibrationFile() const { return calibrationDir() / "homographies.csv"; }
        std::filesystem::path calibrationImagesDir() const { return calibrationDir() / "images"; }
        std::filesystem::path resultRoot() const { return root_ / "result"; }

        /// フォルダ(calibration / result)を作る。既にあれば何もしない。
        Status createFolders() const;

        /// 実行ごとの結果フォルダを作って、そのパスを返す: result/<timestamp>_<label>
        /// 同じ名前が既にあれば、_2, _3 … を付ける(上書きしない)。
        Result<std::filesystem::path> createRunDir(std::string_view timestamp, std::string_view label) const;

    private:
        std::filesystem::path root_;
    };

} // namespace brdf::app
