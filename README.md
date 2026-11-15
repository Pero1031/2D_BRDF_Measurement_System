# 2D BRDF Measurement

ロボットアーム 2 台と 2D 分光放射計で、BRDF(双方向反射率分布関数)を測定するシステムです。
以前の測定システムを、構造化・汎用化・ドキュメント化して作り直しています。

> **開発中です。** 現在はモックだけで動く段階で、実機は未接続です([現在の状態](#現在の状態))。

## 何を測るシステムか

```
                       2D 分光放射計 (固定)
                              ▲ 撮影
   光源 ───────▶  ┌───────────┐
 (左アーム先端)    │   試料     │ ◀── 右アーム先端が試料を持つ
                   └───────────┘
```

- 左アームが光源を、右アームが試料を動かし、固定された 2D 分光放射計で撮影します。
- 光源とカメラは有限の距離にあるので、試料上の点ごとに入射角と観察角が違います。
  1 回の撮影で、多数の角度の組 (θi, φi, θo, φo) のスペクトルが同時に得られます。
- 入力は**アームの姿勢**で、各点の角度は姿勢から計算します。

旧システムの詳しい解析は [docs/LegacySystemAnalysis.md](docs/LegacySystemAnalysis.md) にあります。

## 現在の状態

| フェーズ | 内容 | 状態 |
|---|---|---|
| P0 | 土台(CMake、テスト、フォルダ構成) | 完了 |
| P1 | 幾何と角度(順運動学、角度計算、旧データとの照合) | 完了 |
| P2 | ドライバ v2 とシミュレータ(`ISpectralImager`、`SimulatedRig`) | 次 |
| P3 | プロジェクトフォルダ、TOML 設定、測定セッション、CLI | 未着手 |
| P4 | キャリブボード、ホモグラフィ、スポット XML | 未着手 |
| P5 | 解析(白色板正規化、BRDF 復元) | 未着手 |
| P6 | 実機ドライバ(DENSO RC8 の b-CAP、TOPCON `hs_api`) | 未着手 |
| P7 | 使い方・セットアップ手順などの文書 | 未着手 |

全体の計画は [docs/DesignPlan.md](docs/DesignPlan.md) です。

### 今できること

- アーム先端の位置・姿勢の計算(順運動学): `geometry::Rc8Kinematics`
- 光源・試料・カメラのワールド座標での位置: `geometry::RigGeometry`
- 試料上の点の入射角・観察角の計算: `optics::computeAngles`
- 旧システムの角度表(約 35 万行)と一致することのテスト
- モックのロボットアームと分光器(`examples/mock_demo.cpp`)

### まだできないこと

実機の制御、測定の自動実行、キャリブボードの処理、BRDF の解析は、まだありません。

## ビルドとテスト

必要なもの:

- Visual Studio 2022(「C++ によるデスクトップ開発」)
- CMake 3.25 以上
- リポジトリ直下の `vcpkg/`(依存ライブラリは `vcpkg.json` で管理)

```bash
cmake --preset default
cmake --build --preset debug
ctest --preset debug
```

- 初回の `cmake --preset default` は、vcpkg が依存ライブラリを用意するので時間がかかります(OpenCV を含みます)。2 回目以降は数秒です。
- Visual Studio で「フォルダーを開く」としてこのフォルダを開いても、同じプリセットで使えます。

モックのデモ:

```bash
build/Debug/brdf_mock_demo.exe
```

旧システムの角度表の全行との照合(通常のテストには含まれません):

```bash
set BRDF_OLD_ANGLE_TABLE_DIR=D:\Data\AngleTable
build\tests\Debug\brdf_tests.exe "[.full]"
```

## フォルダ構成

```
include/brdf/   公開ヘッダ
  core/           型、行列、数学ユーティリティ
  geometry/       関節角、順運動学、装置の配置
  optics/         入射角・観察角の計算
  driver/         ロボット・分光器のインターフェースとモック
  domain/         レシピ(P3 で再設計予定)
src/            実装
tests/          単体テストと、旧システムとの照合テスト(tests/data/)
examples/       モックだけで動くデモ
docs/           解析・設計・計画
tools/          補助スクリプト
```

## 座標系と角度の約束

- ワールド座標: 原点 = 左アームの土台、単位 mm。
- 角度は面の座標系で測ります。θ = 法線(Z 軸)からの角度、φ = X 軸から Y 軸向きの角度。内部の単位はラジアンです。
- 試料の座標系は 2 種類あり、**φ が 90° ずれます**。標準は紙の座標系です。

| 座標系 | 軸の向き | 用途 |
|---|---|---|
| `paperFrame`(標準) | 紙 X = 先端の +Y、紙 Y = 先端の −X、紙 Z = 先端の +Z | 試料点ごとの角度 |
| `tipFrame` | アーム先端の X, Y, Z そのまま | 旧システムの角度表との照合 |

装置のパラメータ(カメラ位置など)は `geometry::RigSetup` に集約しています。
**カメラ位置は測定日ごとに変わる値**なので、実際の測定では設定ファイル(P3 で実装)に記録して、結果と一緒に保存します。

## 旧システムとの違い

- 絶対パスと、ヘッダへの定数の直書きをなくします(プロジェクトフォルダと TOML へ)。
- 計算は表示・入力待ち・`exit()` を持たない純粋関数にします。
- 旧システムの `calcAngle` の φ 反転バグ(θ>90° のとき)は、新システムでは修正済みです。

## 文字コード

ソースは **UTF-8(BOM 付き)・CRLF** で統一しています(`.editorconfig` 参照)。

```bash
python tools/check_encoding.py        # 検査
python tools/check_encoding.py --fix  # 修正
```

## ドキュメント

- [docs/LegacySystemAnalysis.md](docs/LegacySystemAnalysis.md): 旧システムの解析と、旧データとの照合結果
- [docs/DesignPlan.md](docs/DesignPlan.md): 設計と開発計画
- [tests/data/README.md](tests/data/README.md): 照合用データの由来
