# 使い方ガイド

`brdf` コマンドの使い方です。シミュレーションでの試しから、実機での測定までを書いています。
英語版は [UserGuide.md](UserGuide.md) です。

> **状態。** 「シミュレーション」と書いた部分は、動作もテストも済んでいます。実機のモード(`mode = "real"`)は、実装して、フェイクを使ったテストは通っていますが、**実機のアームと分光放射計では、まだ一度も試していません**。実機のアームを動かす前に、必ず [SafetyProcedure.md](SafetyProcedure.md) を読んでください。

## 1. 流れを 1 ページで

```
brdf init <dir>                  1. プロジェクトフォルダを作る
   setup.toml, recipe.toml を編集  2. 装置の配置とポーズを書く
brdf check <dir>                 3. 可動域・干渉・角度を検査する(何も動かさない)
brdf view <dir>                  4. アームの動きをブラウザで見る
brdf calibrate <dir>             5. (ボードモード)キャリブボードを撮影し、ポーズごとのホモグラフィを求める
brdf run <dir> --kind white      6. 全ポーズで白色板を測る
brdf run <dir>                   7. 全ポーズで試料を測る
brdf analyze <dir>               8. 放射輝度率と BRDF(= 試料 / 白色板)を求める
```

実機の測定では、手順 5 と 6 が必要です。シミュレーションでは、手順 5 を省けます(シミュレータは自分のカメラを知っているため)。手順 6〜8 は、そのまま実行できます。

## 2. ビルド

```bash
cmake --preset default
cmake --build --preset debug
```

再ビルドを速くしたいときは `tools\build.cmd debug` を使います(Ninja。README を参照)。実行ファイルは `build/Debug/brdf.exe`(Ninja では `build-ninja/Debug/brdf.exe`)です。

実機を使うには、両方の SDK を付けたビルドが必要です。`-DBRDF_WITH_BCAP=ON -DBCAP_SDK_DIR=<...>` と `-DBRDF_WITH_TOPCON=ON -DHS_SDK_DIR=<...>` を指定します(README を参照)。実行時には、`hs_api.dll`(x64)が `PATH` 上にある必要があります。

## 3. シミュレーションで試す

そのまま動かせるプロジェクトが `examples/sim_project` にあります。装置はシミュレーションで、実際には何も動きません。

```bash
build/Debug/brdf.exe check   examples/sim_project            # 実行前の検査
build/Debug/brdf.exe view    examples/sim_project            # 表示された HTML ファイルをブラウザで開く
build/Debug/brdf.exe run     examples/sim_project --yes      # 試料を測る(シミュレーション)
build/Debug/brdf.exe run     examples/sim_project --kind white --yes
build/Debug/brdf.exe analyze examples/sim_project
```

結果は `examples/sim_project/result/` の下に書かれます。

自分のプロジェクトを始めるには、次のようにします。

```bash
build/Debug/brdf.exe init my_measurement
```

説明のコメント付きの `my_measurement/setup.toml` と `recipe.toml` ができます。どちらにも、絶対パスは書かれません。

## 4. プロジェクトフォルダ

```
my_measurement/
  setup.toml      装置の配置とデバイス(あまり変わらない。カメラ位置だけは測定日ごとに変わる)
  recipe.toml     ポーズ、試料上の測定点、撮影条件、チェックリスト
  calibration/
    images/<pose>.png    キャリブボードの画像(brdf calibrate が書く)
    homographies.csv     ポーズごとのホモグラフィと再投影誤差
  result/<日付>_<時刻>_<kind>/
    measurements.csv     (ポーズ, 測定点)ごとに 1 行: 角度 [度]、続けてスペクトル
    run_manifest.toml    いつ、どう測ったか
    setup.toml, recipe.toml   使った設定のコピー
  result/<日付>_<時刻>_analysis/
    radiance_factor.csv, brdf.csv, analysis_manifest.toml
  work/           (実機モードのみ)TOPCON の SDK に渡す一時的なレシピとスポットのファイル
```

### setup.toml

| セクション | 内容 |
|---|---|
| `[devices]` | `mode = "sim"`(シミュレーション)または `"real"`(実機) |
| `[arm]`, `[arm.links]`, `[arm.limits]` | 干渉チェックのアームの半径、リンクの長さ、関節の可動域(既定は VP-6242 の値) |
| `[rig]` | 2 台のアームの土台位置、光源のオフセット、**カメラ位置**、試料面のオフセット |
| `[calibration]`, `[calibration.board]` | `projector`(`"camera-model"` か `"board"`)、ボードの配置、ボードの取り付け向き(`board_u`, `board_v`) |
| `[imager]` | `device = "SR5100"`(既定)または `"SR5000"`、ビニング、レンズ、画像の大きさ |
| `[controllers]` | RC8 コントローラ: IP アドレス(左 `192.168.0.2`、右 `192.168.0.5`)、ポート 5007、タイムアウト、`max_speed`(既定 0.05 = 5%)、`abortable_moves` |
| `[analysis]` | 白色板の反射率、波長の範囲、`irradiance_correction` |
| `[spot_xml]` | スポット定義ファイルのスポットの大きさ |
| `[sim.*]` | シミュレータの設定(`mode = "sim"` のときだけ使う) |

長さは mm、角度は度です。キーの綴りの間違いや値の誤りは、ファイル、セクション、キーを示して報告されます。
**`camera_position` は、測定日ごとに合わせ直してください。** 結果には、そのときの `setup.toml` のコピーが保存されます。

### recipe.toml

- `[recipe]`: 名前、担当者、`kind`(`sample`、`white`、`calib`)、`checklist`(実行前に人が確認する項目)。
- `[capture]`: `integration_time_ms`(0 = 自動。実機)、`arm_speed`(0〜1。`max_speed` を超えない)、波長の範囲と刻み。
- `[[spots]]`: 試料上の測定点。`id`、`x`、`y`(mm、紙の座標系。原点は基準点)。`[spot_grid]`(`center`、`step`、`count`)で格子状にも指定できます。
- `[[poses]]`: `left`(光源側のアーム)と `right`(試料側のアーム)の関節角 J1〜J6(度)。

旧システムのポーズを使うには、`brdf import-poses <csv> --out recipe.toml` を使います(ヘッダーが `RJ1..RJ6,LJ1..LJ6` の CSV。旧システムの 52 ポーズは、すべて読み込めます)。

## 5. コマンド一覧

| コマンド | 内容 |
|---|---|
| `brdf init <dir>` | ひな型付きのプロジェクトフォルダを作る。 |
| `brdf check <dir>` | 可動域、アーム同士の干渉(ポーズの間の移動の途中も)、角度を検査する。何も動かさない。実機を動かす前に、毎回実行する。 |
| `brdf view <dir> [--steps N]` | ポーズの動きを 3D アニメーションにして、`result/<日付>_<時刻>_view/view.html` に出力する。ドラッグで回転、ホイールで拡大縮小、Shift + ドラッグで移動。 |
| `brdf calibrate <dir>` | 全ポーズでキャリブボードを撮影し、円を検出して、ポーズごとのホモグラフィを保存する。 |
| `brdf run <dir> [--kind sample\|white\|calib] [--yes]` | 測定を実行して保存する。チェックリストを表示して、`OK` の入力を待つ。`--yes` で省けるのはチェックリストだけ。 |
| `brdf angles <dir>` | 全ポーズ・全測定点の入射角と観測角だけを出力する。 |
| `brdf analyze <dir> [--sample R] [--white R] [--irradiance-correction]` | 試料と白色板の測定から、放射輝度率と BRDF を計算する(既定は、それぞれの最新)。 |
| `brdf spots <dir>` | キャリブレーションから、TOPCON のソフトに読み込ませるスポット定義ファイル(XML)を、ポーズごとに出力する。`projector = "board"` が必要。 |
| `brdf import-poses <csv> [--out F] [--force]` | 旧システムのポーズ CSV から recipe.toml を作る。 |
| `brdf arm status <dir>` | (実機モード)2 台の現在の関節角を表示する。動かさない。 |
| `brdf arm move <dir> --arm left\|right (--to=J1,..,J6 \| --standard \| --jog N=DEG) [--speed S]` | (実機モード)アームを **1 台だけ** 動かす。最初の動作確認に使う。 |

`brdf --version` でバージョン、`brdf <command> --help` でオプションを表示します。

## 6. 白色板の測定と解析

BRDF は、試料の放射輝度を、同じポーズで測った標準白色板の放射輝度で割って求めます。

1. 試料を右アームに付けて、`brdf run <dir>` を実行する。
2. 白色板をアームに付けて、`brdf run <dir> --kind white` を実行する。白色の測定は、全ポーズで、基準点(0, 0)だけを測る。
3. `brdf analyze <dir>` を実行する。

2 回の測定は、ポーズと装置の配置(カメラ位置など)が同じでなければなりません。`analyze` が確認して、違う点を教えます。
`--irradiance-correction` を付けると、旧システムの系統誤差(光が試料の点ごとに違う強さで当たること)を補正します。付けなければ、旧システムと同じ結果になります。
LED は 380 nm 付近と 770 nm 付近で弱いので、その波長は信頼できません。

## 7. キャリブボードの使い方(実機)

実機では、カメラ画像の中の測定点の位置を、キャリブボードから求めたホモグラフィで決めます。`setup.toml` で次のようにします。

1. 試料を置く場所にボードを取り付ける。円は対称な格子なので、画像だけでは、ボードの向きが分かりません。ボードの右向きと下向きが、試料座標のどの軸に当たるかを、`board_u` と `board_v`(`"+x"`、`"-x"`、`"+y"`、`"-y"`)に書く。ボードは、画像の中で 90° 未満の回転に見えるように置く。
2. `brdf calibrate <dir>` を実行する。報告される再投影誤差を、`max_rms_px` と見比べる。
3. `projector = "board"` にして、`brdf run <dir>` を実行する。

## 8. 実機

実機のモードは、**アーム**(DENSO RC8、b-CAP)と、**分光放射計**(TOPCON SR-5100、`hs_api`)の 2 つからなります。両方の SDK を付けたビルドが必要です。

1. `setup.toml` で `[devices] mode = "real"` にして、`[controllers]`(IP アドレス、ポート)、`[arm.limits]`、`[imager] device` を確認する。
2. [SafetyProcedure.md](SafetyProcedure.md) に従う。アームの届く範囲から物を外し、非常停止に手が届く状態にして、**速度 5%** から始める。段階は、`brdf arm status`、`brdf arm move --jog` での少しずつの移動、標準姿勢、`brdf run` の順。
3. 実機モードの `brdf run` は、装置、**実際の**速度、動かす範囲、止め方を表示して、`OK` の入力を求める(`--yes` でも省けない)。

実機モード(`run`、`calibrate`、`arm`)で、常に働く安全の仕組みは次のとおりです。

- 速度の上限(`max_speed`、既定 5%)。
- 送る前に目標を可動域と照らし合わせ、動かしたあとにエンコーダで到着を確かめる。
- 最初にアームの実際の位置を読み、最初のポーズまでの道のりを検査する。
- ポーズの間の移動は、途中の 20 点で干渉を調べる。ただし、調べるのはアーム同士だけ。カメラ、分光放射計、床、台、試料の板は**考慮していない**。
- 旧システムが使った関節の範囲の外にあるポーズを警告する。
- 失敗したら、両方のアームを止め、モータを OFF にして、接続を閉じる。
- Ctrl+C で、次のポーズへ進む前に止まる。`abortable_moves = true` にすると、移動の途中でもアームを止める(実機では未確認。非常停止の代わりにはならない)。2 回目の Ctrl+C で、プログラムを終了する。

## 9. 困ったとき

| メッセージ / コード | 意味と対処 |
|---|---|
| `JointLimit` | 目標が `[arm.limits]` の外にある。ポーズを変える。 |
| `Collision` | 2 台のアームが近づきすぎる(各アームの周りの 60 mm の半径で判定)。ポーズを変えるか、`brdf view` で確認する。 |
| `Timeout` | 装置が時間内に答えなかった。動作中だったかもしれない。アームは止めて、接続を閉じている。ペンダントで状態を確かめてから、やり直す。 |
| `Communication` | 接続できない、または途切れた。ケーブル、`[controllers]` の IP アドレス、コントローラの電源を確認する。 |
| `Cancelled` | Ctrl+C で中止した。動いているものは残っていない。 |
| `InvalidArgument` ... `nothing was moved` | 動かす前に、要求や答えが断られた(例: `OK` を正確に入力しなかった)。 |
| `NotImplemented` ... `BRDF_WITH_BCAP` / `BRDF_WITH_TOPCON` | `mode = "real"` には、両方の SDK を付けたビルドが必要(2 章)。 |
| `FocusError` | 測定距離が実際の距離と合っていない。ピントとカメラ位置を確認する。 |
| `Saturated` | 測定値が飽和した(明るすぎる)。照明、積分時間、ND フィルタを見直す。 |
| `brdf spots needs ... projector = "board"` | 先にキャリブレーションをする(7 章)。 |
| テストが遅い | `tools\build.cmd debug test` は、重いキャリブボードのテスト 8 件を除く。`test-all` で全件を実行する。 |

## 10. 詳しい資料

- [LegacySystemAnalysis.md](LegacySystemAnalysis.md): 旧システムの解析と、旧システムのデータとの照合
- [DesignPlan.md](DesignPlan.md): 設計と計画
- [SafetyProcedure.md](SafetyProcedure.md): 実機のアームを初めて動かす手順
- [Rc8ArmDesign.md](Rc8ArmDesign.md)、[TopconSdkDesign.md](TopconSdkDesign.md): 実機のドライバと、実機でまだ確かめていないこと
