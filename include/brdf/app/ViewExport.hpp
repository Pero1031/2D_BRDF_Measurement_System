/*
* @file include/brdf/app/ViewExport.hpp
* @brief Exports the poses of a recipe as a self-contained 3D animation (one HTML file).
*/

#pragma once

#include <filesystem>
#include <string>

#include "brdf/app/Commands.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::app {

    struct ViewOptions {
        /// 1 回の移動(左アームの移動、右アームの移動)を、何コマで補間するか。
        int steps_per_move = 15;
        std::string timestamp;   ///< 出力フォルダ名の日時。空なら現在時刻
    };

    /// レシピの全ポーズを動かしたときの様子を、1 つの HTML(3D アニメーション)にする。
    ///
    /// 動き方は、測定セッションと同じ: 前のポーズから、左アーム → 右アームの順に、関節角を直線的に補間して動かす
    /// (実機の moveJ は関節空間の補間)。最初の位置は、両アームとも (0, 0, 90, 0, 0, 0)。
    /// 全コマの位置は、検査・角度計算と同じ RigGeometry で計算する(HTML 側で運動学を再計算しない)。
    ///
    /// HTML の中身:
    ///  - アーム(リンクを太さのある棒で表した簡易モデル)、光源、試料の板と座標系、カメラとその視野、測定点
    ///  - 再生・コマ送り・ポーズの選択、視点の操作、関節角・左右アームの距離・測定点の角度の表示
    ///  - 事前検査の結果(可動域・干渉)。さらに、**動いている途中**の左右アームの距離も調べ、近づきすぎるポーズを指摘する
    ///
    /// 外部のライブラリは使わないので、ネットワークにつながっていなくても開ける。
    Result<std::string> buildViewHtml(const LoadedProject& project, const ViewOptions& options = {});

    /// result/<日時>_view/view.html に書き出す。
    Result<std::filesystem::path> exportView(const std::filesystem::path& dir, const ViewOptions& options = {});

} // namespace brdf::app
