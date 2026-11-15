# 2D BRDF Measurement

ロボットアーム 2 台(光源側・試料側)と 2D 分光放射計で BRDF を測定するシステムです。
以前の測定システムを、構造化・汎用化して作り直しています。

- 旧システムの解析: [docs/LegacySystemAnalysis.md](docs/LegacySystemAnalysis.md)
- 設計と開発計画: [docs/DesignPlan.md](docs/DesignPlan.md)

## 現在の状態

| フェーズ | 状態 |
|---|---|
| P0 土台(CMake・テスト) | 完了 |
| P1 幾何と角度(順運動学・角度計算・旧データとの照合) | 完了 |
| P2 以降(ドライバ v2・シミュレータ・CLI …) | 未着手 |

モックだけで動きます。実機(RC8・TOPCON SR-5000H)のドライバはまだありません。

## ビルドとテスト

必要なもの: Visual Studio 2022(C++ デスクトップ開発)、CMake 3.25 以上、
リポジトリ直下の `vcpkg/`(依存ライブラリは `vcpkg.json` で管理)。

```bash
cmake --preset default            # 初回は vcpkg が依存ライブラリを用意する
cmake --build --preset debug
ctest --preset debug
```

Visual Studio で「フォルダーを開く」としてこのフォルダを開いても、同じプリセットで使えます。

モックのデモ:

```bash
build/Debug/brdf_mock_demo.exe
```

旧システムの角度表 全約 35 万行との照合(通常のテストには含まれません):

```bash
set BRDF_OLD_ANGLE_TABLE_DIR=D:\Data\AngleTable
build\tests\Debug\brdf_tests.exe "[.full]"
```

## フォルダ構成

```
include/brdf/   公開ヘッダ     core/ geometry/ optics/ driver/ domain/
src/            実装
tests/          単体テストと、旧システムとの照合テスト(tests/data/)
examples/       モックだけで動くデモ
docs/           解析・設計・計画
tools/          補助スクリプト
```

## 文字コード

ソースは **UTF-8(BOM 付き)・CRLF** で統一しています(`.editorconfig` 参照)。

```bash
python tools/check_encoding.py        # 検査
python tools/check_encoding.py --fix  # 修正
```
