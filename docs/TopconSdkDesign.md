# TOPCON SDK(hs_api)対応 設計メモ — `ISpectralImager` の実機実装

P6 ① の成果。SDK のヘッダ・C 言語の例・旧システムの制御プログラムを読んで、
`driver::ISpectralImager` の `capture()` / `captureImage()` を SDK の関数呼び出しにどう対応づけるかをまとめる。
**実機はまだ触っていない。** 「未確認」と書いた点は、実機または SDK マニュアルで確かめるまで仮説。

## 0. 調べたもの

| 資料 | 場所 | 状態 |
|---|---|---|
| SDK ヘッダ `hs_api.h`(Ver 3.6.0、2025-01-15) | CI-100 インストーラ(Ver3.6.0)の `SDK\c\include\` | 読了(約 6000 行) |
| インポートライブラリ `hs_api.lib` | 同 `SDK\c\lib\` | あり |
| DLL `hs_api.dll`(x64) | 3.6.0 と同じ版は `D:\bin\` にある。`C:\Program Files\TopconTechnohouse\SR-5000\bin\` のものは 2022-01 の古い版 | 配置のみ確認 |
| C の例 | 同 `SDK\c\examples\`(`manual_measurement`、`load_data_and_output`、`live_view` ほか) | 読了 |
| SDK マニュアル(JPN / ENG の PDF) | 同 `SDK\` | **未読**(AES 暗号化。読むには Python の `cryptography` が要る) |
| 旧システムの制御プログラム | 旧プロジェクトの `test`(`manual_measurement` の改造) | 読了 |

ヘッダのコメントは各関数に引数の説明があるが、**単位や配列の添字の約束は書かれていないものが多い**。
その部分はマニュアルで確認する(§8)。

## 1. SDK の使い方の骨格

SDK の例(旧システムの制御プログラムも同じ)の流れ:

```
hsInitialize(校正データのフォルダ, &config)      … 最初に 1 回。つながっている装置の一覧が config に入る
hsOpenDevice(&config.connected_devices[0], &info) … 接続。info.device_id 以降の呼び出しで使う
hsCreateRecipe(&recipe) / hsLoadRecipe(xml, &recipe)
hsCreateMeasurementData(&recipe, &mid)            … 測定データ(メモリ上)を作る。mid は以後の識別子
hsStartMeasurement(device_id, mid, 1, callback)   … 測定。終わるまで戻らない(ブロッキング)
hsSaveMeasurementData(path, mid)                  … .hsm を保存
hsDestroyMeasurementData(mid) / hsDestroyRecipe
hsCloseDevice(device_id) / hsFinalize()
```

- 返り値はすべて `HsError`。`hsGetErrorString` で文章にできる(256 バイト以上のバッファ)。
- `HsProgressCallback` は `void(__stdcall*)(HsRunningStatus, float)`。**ユーザーデータを渡す引数がない**ので、呼び出し元への橋渡しには静的な変数が要る(装置は 1 台ずつしか使わないので問題ない)。
- 測定データ、レシピ、スポット定義などは、`hsCreate…` で作ったものを **必ず `hsDestroy…` で解放**する。C++ では RAII のラッパで包む。

### 旧システムで実際に起きていたこと

1. 旧の制御プログラムは、`recipe.xml` を読んで測定し、`.hsm` を保存するところまでだけをやる(引数 = レシピと保存先)。
2. スポットの設定(`SpotDef.xml`)の読み込みと、スペクトルの CSV 出力は、**TOPCON の GUI ソフトで人が操作**していた。

SDK には 2 も揃っている(`hsLoadSpotDef`、`hsSetMeasurementStandardSpotDef`、`hsCreateSpotDataList`、`hsGetSpotData`、`hsOutputSpotCsv`。`load_data_and_output` の例が手本)。
新システムでは、これを全部コードにして、人手の工程をなくす。

### 旧のレシピの重要な点

旧のレシピ(`recipe.xml`)は、分光測定(`SPECTRAL`)、380〜780 nm・1 nm ステップ、
**積分時間は `AUTO`**(SDK が測定のたびに最適化する。`HS_STATUS_OPTIMIZING` の段階がある)、測定範囲は画像全体。
→ ピント合わせは SDK にない(レンズは手動)。積分時間は自動に任せるのが旧の運用。

## 2. 重要な発見: `.hsm` が巨大

| 装置 | `.hsm` 1 ポーズの大きさ |
|---|---|
| SR-5000(1376×1024) | 約 2.4 GB |
| SR-5100(2448×2048) | 約 8.7 GB |

13 ポーズで約 113 GB(SR-5100)。旧データがこの大きさなのはこのため。

- **既定では `.hsm` を保存しない**。測定後すぐに指定したスポットのスペクトルだけを取り出して CSV にする(新システムの `measurements.csv`)。
- 生データを残したい人のための設定 `save_raw`(既定 false)は、あとで足してもよい。
- 測定データはメモリ上にも画像全体を持つはず。**64 ビットのプロセス**で、ポーズごとに必ず `hsDestroyMeasurementData` する(メモリの目安は未確認。SR-5100 で 10 GB 前後と見ておく)。

## 3. `ISpectralImager` との対応

### 3.1 `connect()` / `disconnect()`

| 新 | SDK |
|---|---|
| `connect()` | `hsInitialize(calibration_dir, &config)` → `config.num_connected` が 0 なら失敗 → `hsOpenDevice(&config.connected_devices[0], &info)` |
| 接続後の確認 | `info.device_type` が `setup.toml` の `[imager].device` と一致するか。違えば失敗(画像の大きさが違い、スポット座標が狂うため) |
| `disconnect()` | `hsCloseDevice(device_id)` → `hsFinalize()` |
| `isConnected()` | 自分で持つフラグ |

注意:

- `hsInitialize` / `hsFinalize` は**プロセス全体**に効く。`TopconImager` は同時に 1 つだけ(2 つ目の `connect()` はエラーにする)。
- 校正データのフォルダ(旧は `C:\Program Files\TopconTechnohouse\CI-100\Calibration`)は、**`setup.toml` の `[imager].calibration_dir`** で指定できるようにする。初期値は SDK の標準の場所。これは装置ごとの SDK の約束であり、プロジェクトの持ち物ではないので、絶対パスを許す唯一の設定とする。
- 他のソフト(TOPCON の GUI など)が装置を使っていると `HS_ERROR_OTHER_PROCESS` や `CANNOT_CONNECT_TO_DEVICE` になる。メッセージに「他のソフトを閉じる」と添える。

### 3.2 `capture(CaptureRequest)` — 点ごとのスペクトル

```
 1. レシピを作る        (§4: 旧と同じ形式の XML を書き出し → hsCreateRecipe + hsLoadRecipe)
 2. hsCreateMeasurementData(&recipe, &mid)
 3. hsStartMeasurement(device_id, mid, 1, cb)      ← 光学測定。積分時間は AUTO なら内部で最適化
 4. hsGetMeasurementData(mid, &md)                 ← 実際の積分時間・飽和の有無・画像の大きさ
 5. スポット定義を作る  (§5: SpotXml で書き出し → hsLoadSpotDef → hsSetMeasurementStandardSpotDef)
 6. hsCreateSpotDataList(mid, HS_SPOT_DATA_TYPE_STANDARD, cb, &list)
 7. 点ごとに hsGetSpotData → spectral_statistics[波長 − 380].average をスペクトルとして取り出す   (§8 未確認)
 8. hsDestroySpotDataList / hsDestroyMeasurementData / hsDestroyRecipe
```

`CaptureResult` への入れ方:

| `CaptureResult` | 取り出し方 |
|---|---|
| `wavelengths` | 要求の範囲(`CaptureRequest::wavelengths`)。SDK 側の範囲はレシピに書いたもの |
| `spots[i].values` | 点 `id` のスペクトル(順番に頼らず、`spot_id` と `SpotRequest::id` を対応づける) |
| `integration_time_ms` | `md.integration_time[HS_FILTER_TYPE_SPECTRAL]` |
| `measure_distance_mm` | 要求の値(SDK は距離を測らない。レシピに書いた値) |

飽和: `md.is_saturated` が真ならエラーにする(積分時間が AUTO でも起こりうる。`HS_ERROR_SATURATION`、`HS_ERROR_OPTIMIZE_TOO_BRIGHT/TOO_DARK` も同じ扱い)。

### 3.3 `captureImage(ImageRequest)` — キャリブボードの撮影

2 つの方法がある:

| 方法 | 内容 | 長所 | 短所 |
|---|---|---|---|
| **A: ライブ画像**(第一候補) | `hsSetLiveSetting`(binning、積分時間、ROI = 全画面)→ `hsStartCapture` → `hsCaptureLiveImage`(8 ビット RGB)→ `hsStopCapture` → グレーに変換 | 速い。`live_view` の例がそのまま手本。装置の画像の大きさは `hsGetImageSize` | 8 ビットなので、明るすぎると白飛びする。積分時間を手動で決める必要がある |
| B: XYZ 測定の Y | レシピの測定種別を XYZ にして `hsStartMeasurement` → `hsGetMeasurementImage(mid, HS_DATA_TYPE_TRISTIMULUS_Y, &img32f)` | 浮動小数点で白飛びに強い。積分時間は AUTO | 分光測定より速いが、ライブよりは遅い。レシピ XML の XYZ 用の書き方が未確認 |

旧のキャリブ画像(`CalibBoard/N.png`、1376×1024)がどの方法で撮られたかは不明。
**まず A で作る**(コードが少ない)。円の検出は 8 ビットで足りる。必要になれば B を足す。
`ImageRequest::integration_time_ms` が 0 のときは、`TopconImagerConfig::live_integration_ms`(既定 10 ms、live_view の例と同じ)を使う。`hsOptimizeThroughLive` で自動にする案は §8 で確認する。

画像の向き: `HsImage8u3` は左上が原点、行は上から下、RGB の順で 3 バイト/画素(`live_view` の描画コードから)。
`brdf::GrayImage` は同じ向き。

実装(`RealHsApi::captureLiveImage`)では、設定を変えた直後の画像が古いかもしれないので、3 枚撮って最後の 1 枚を使う。グレーへの変換は 3 チャンネルの単純平均(R,G,B の並び順がヘッダの説明と `live_view` の例で食い違っているため、順番によらない方法にした)。

## 4. レシピの作り方

方法は 2 つ:

| 方法 | 内容 |
|---|---|
| (a) 構造体を直接埋める | `hsCreateRecipe` の後、`HsRecipe::measurement` の各項目を代入 |
| **(b) 旧と同じ形式の XML を書き出して `hsLoadRecipe`**(推奨) | 旧の `recipe.xml`(42 行)と同じ形のテンプレートを C++ で書き出す |

(b) を推奨する理由:

- 旧の実績がある形式で、**旧の `recipe.xml` との完全一致をテストにできる**(実機なしで検証できる。`SpotXml` と同じやり方)。
- (a) は `HsRecipe` の既定値(`hsCreateRecipe` が何を入れるか)が分からず、項目も多い(補正の有効/無効が 8 種類など)。
- 書き換える項目は少ない: 装置の種類、ビニング、レンズ、測定距離、積分時間(AUTO / 手動)、波長の範囲、測定範囲 = 画像全体。
  すべて `setup.toml` の `[imager]` と `CaptureRequest` から来る。

補正はすべて無効にする(旧もそう: 対角補正、色補正、輝度補正、エリア補正、白色板補正)。
放射輝度は SDK が装置の校正データで絶対値にする(旧の解析もそれを前提にしていた)。

## 5. スポットの渡し方

`CaptureRequest::spots` は画像上の位置(pixel)。SDK のスポット定義(`HsSpotDef`)は構造が大きいので、**メモリ上で組み立てずに、既にある `io::SpotXml`(旧の XML と同じ形式を書き出せる。旧データで検証済み)で一時ファイルに書き、`hsLoadSpotDef` で読む**。

- 一時ファイルの置き場所は `TopconImagerConfig::work_dir`(呼び出し側が決める。プロジェクトの `result/<日時>/` の下にするのが自然)。ファイル名は `recipe.xml` と `spots.xml`。測定のたびに上書きされるので、あとから見たいときは呼び出し側がコピーする。
- スポットの大きさは `[spot_xml].spot_size`(既存)。
- SDK の標準スポット(`HS_SPOT_DATA_TYPE_STANDARD`)として設定する。旧の GUI も標準スポットの XML を使っていた。

## 6. エラーの対応

`HsError` を `brdf::ErrorCode` に対応づける(`ErrorCode` は足してよい):

| `HsError` | `ErrorCode` | 補足 |
|---|---|---|
| `HS_ERROR_NONE` | (成功) | |
| `NO_CONNECTABLE_DEVICE`、`CANNOT_FIND_CONNECTED_DEVICE`、`CANNOT_CONNECT_TO_DEVICE`、`OTHER_PROCESS` | `DeviceError` | 電源、USB/LAN、他のソフトが使用中でないか |
| `NOT_INITIALIZED`、`SYSTEM_INITIALIZE`、`CALIBRATION_PATH_NOT_FOUND` | `NotConnected` / `InvalidArgument` | 校正データのフォルダを案内 |
| `INVALID_PARAMETER`、`INVALID_IMAGE_SIZE`、`ADJUST_SIZE` | `InvalidArgument` | |
| `SATURATION`、`OPTIMIZE_TOO_BRIGHT`、`OPTIMIZE_TOO_DARK` | `DeviceError`(新しく `Saturated` を足してもよい) | 照明の強さ、積分時間、ND フィルタ |
| `PROCESS_INTERRUPTED`、`PROCESS_TIMEOUT` | `DeviceError` | 中断か時間切れ |
| `CANNOT_LOAD_FILE`、`CANNOT_FIND_FILE`、`INVALID_FILE_FORMAT` | `InvalidArgument` | レシピやスポットの XML が不正 |
| `MEMORY` | `DeviceError` | 画像が大きすぎる(§2)。メモリ不足 |
| その他 | `DeviceError` | `hsGetErrorString` の文章を添える |

エラーメッセージには「どの SDK 関数が、どのコードで失敗したか」を必ず入れる(旧は `std::cout` に出すだけだった)。
**途中で失敗しても、確保したものは必ず解放する**(RAII)。

## 7. 実装の構成

```
include/brdf/driver/topcon/
  HsApi.hpp          …… 使う SDK 関数だけを並べた薄い仮想インターフェース(ITopconApi)。hs_api.h を include しない
  TopconImager.hpp   …… ISpectralImager の実装。ITopconApi を使って §3 の手順を行う
src/driver/topcon/
  RealHsApi.cpp      …… ITopconApi を hs_api.h の関数で実装する。ここだけが hs_api.h を include する
  RecipeXml.cpp      …… レシピ XML の書き出し(旧の recipe.xml と一致をテスト)
  TopconImager.cpp
tests/ test_topcon_imager.cpp  …… フェイクの ITopconApi で、手順・エラー・解放漏れ・飽和を検査
```

- **hs_api.h を include するのは `RealHsApi.cpp` だけ**にする。これで TOPCON の SDK がない PC(開発 PC など)でも、`TopconImager` 本体とそのテストをビルドして動かせる。
- ビルドの切り替えは CMake のオプション `BRDF_WITH_TOPCON`(既定 OFF)。ON のときは、SDK の場所を CMake の変数 `HS_SDK_DIR`(`include/hs_api.h` と `lib/hs_api.lib` の親)で受け取る。**絶対パスはリポジトリに書かない**。
- 実行時には、`hs_api.dll`(x64、3.6.0 と同じ版)が実行ファイルと同じ場所または `PATH` にあること。`brdf check` でこれを調べて案内する。
- `DeviceFactory` の `real` は、`BRDF_WITH_TOPCON` が ON なら `TopconImager` を返し、OFF なら今まで通り `NotImplemented`(「この版は SDK なしでビルドされています」と案内する)。
- 進捗(`HsProgressCallback`)は、`spdlog` のログに出す(`optimizing` → `scanning` → `processing` の段階)。中断 `hsStopMeasurement` は `stop()` 相当で呼べるようにしておく(最初は使わない)。

## 8. 未確認の点(マニュアルまたは実機で確かめる)

| # | 内容 | 確かめ方 | 外れたときの代案 |
|---|---|---|---|
| 1 | `hsGetSpotData` で点を順に取り出す方法(`HsSpotData::spot_id` を入力に使うのか、呼ぶたびに進むのか) | マニュアル、または実機で 2 回呼んで比べる | `hsOutputSpotCsv` で一時 CSV に出して読む(`load_data_and_output` の例が手本。確実だが遅い) |
| 2 | `spectral_statistics[i]` の添字と単位(波長 = 380 + i、分光放射輝度 W/(sr·m²·nm) のはず) | 旧の GUI の CSV と同じ点・同じ測定で比べる | CSV 経由に切り替える |
| 3 | スポットの座標の原点とスポット XML の座標(旧のスポット XML は画素、左上原点のはず) | 既存の旧 XML + 旧 CSV と、新しく取り出した値を比べる | |
| 4 | 積分時間 AUTO のとき、測定ごとに積分時間が変わっても放射輝度が正しいか | 同じ白色板を積分時間を変えて測る | 白色板と試料で積分時間を固定する(手動) |
| 5 | 1 回の測定にかかる時間(SR-5100、380〜780 nm、`NORMAL` 走査) | 実機 | `HIGH_SPEED` 走査との比較 |
| 6 | メモリ使用量(SR-5100 の測定データ) | 実機 | 別プロセスにする、波長刻みを粗くする(例: 5 nm) |
| 7 | レシピ XML の XYZ 測定の書き方(方法 B のため) | `hsSaveRecipe` で GUI の XYZ レシピを書き出して見る | 方法 A だけにする |
| 8 | `[imager].device` と実機の `device_type` の対応(SR5000 = SR-5000H の扱い) | 実機 | |
| 9 | 白飛びの判定(`is_saturated` が AUTO のとき本当に立つか) | 照明を強くして測る | 画素値の最大値で自分で判定する |
| 10 | ライブ画像の積分時間の決め方(`hsOptimizeThroughLive` が使えるか) | 実機 | 固定値(10 ms)+ 設定で変更 |

マニュアルの PDF は暗号化されていて今回は読めなかった。
読む場合は、`pip install cryptography` のあと、`pypdf` で本文を取り出せる(このリポジトリには入れない)。
その結果は、この表の 1・2・3・7・10 に効くはず。

## 9. 実装の状況

② は実装済み(2026-11)。

| 部品 | 場所 |
|---|---|
| SDK の窓口(`ITopconApi`) | `include/brdf/driver/topcon/HsApi.hpp` |
| 実機用(SDK の関数を呼ぶ。`hs_api.h` を include するのはここだけ) | `src/driver/topcon/RealHsApi.cpp`(`BRDF_WITH_TOPCON=ON` のときだけビルド) |
| `ISpectralImager` の実装 | `src/driver/topcon/TopconImager.cpp` |
| レシピ XML | `src/io/TopconRecipeXml.cpp`(旧の `recipe.xml` と一致するテストあり。`tests/data/legacy_recipe.xml`) |
| フェイクでのテスト | `tests/test_topcon_imager.cpp`、`tests/test_topcon_recipe_xml.cpp` |
| 設定 | `[imager] calibration_dir`(省略すると SDK の標準の場所)、`[capture] integration_time_ms = 0` が自動 |

設計からの変更点:

- `ErrorCode::Saturated` を足した(飽和は `DeviceError` と分けた)。
- 積分時間 0 = 自動を、`recipe.toml`、`CaptureRequest`、シミュレータまで通した(旧システムは自動だったため)。
- `hsGetSpotData` の呼び方は仮説のまま(§8 の 1)。`spot_id` に添字を入れて呼び、同じ点が 2 回返ってきたらエラーにする検査を入れた。
- SDK 付きでビルドしたテストの実行ファイルは `hs_api.dll` に依存するので、`hs_api.dll` のある場所を `PATH` に入れないと起動できない。
- SDK 付きのビルドで、`RealHsApi.cpp` のコンパイルと、`hs_api.lib` とのリンクができることは確認した(SDK のある PC)。実機は未接続。

## 10. 次の作業

| 段階 | 内容 | 実機 |
|---|---|---|
| ② | `ITopconApi`、`TopconImager`、レシピ XML の書き出し(旧の `recipe.xml` と一致するテスト)、フェイクでのテスト — **済** | 不要 |
| ③ | `Rc8Arm`(b-CAP。接続はセッションにつき 1 回。関節角は度の double。速度は ExtSpeed)— **済**。[docs/Rc8ArmDesign.md](Rc8ArmDesign.md) | 不要(ビルドまで) |
| ④ | 両方のエラー・タイムアウト・切断のテスト — アームは ③ で済み。TOPCON の SDK のエラーは、`Timeout` と `Communication` を分けて返すようにした | 不要 |
| ⑤ | 実機のアームを低速で 1 回動かす(安全確認から) | **必要** |
| ⑥ | 実機の分光放射計で `capture()`、旧の GUI の CSV と比較(§8 の 1〜3) | **必要** |
| ⑦ | 通しの測定(白色板 → 試料 → `brdf analyze`) | **必要** |

② の受け入れ基準:

- `BRDF_WITH_TOPCON=OFF` の既存のビルドとテストが今まで通り通る(137 件 → 新しいテストを足して 161 件)。
- `BRDF_WITH_TOPCON=ON` で、SDK のある PC でリンクまで通る。
- フェイクの `ITopconApi` を使ったテストで、成功の手順の順序、各段階の失敗、解放漏れがないこと(作ったものと同数を解放)を確認する。
- 生成したレシピ XML が、旧の `recipe.xml` と(日付・コメント・装置の種類以外)一致する。
