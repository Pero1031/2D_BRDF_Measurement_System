/*
* @file include/brdf/io/SetupConfig.hpp
* @brief setup.toml: the physical layout of the rig and the device mode.
*/

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "brdf/calibration/BoardLayout.hpp"
#include "brdf/calibration/SampleToBoard.hpp"
#include "brdf/core/Error.hpp"
#include "brdf/geometry/RigSetup.hpp"
#include "brdf/sim/MockSpectralImager.hpp"

namespace brdf::io {

    /// 使うデバイス。Sim = シミュレーション、Real = 実機(P6 で対応)。
    enum class DeviceMode { Sim, Real };

    inline const char* toString(DeviceMode mode) { return mode == DeviceMode::Sim ? "sim" : "real"; }

    /// 2D 分光放射計の機種と撮影設定。実機でもシミュレータでも使う(画像の大きさは機種で決まるため)。
    ///
    /// 機種ごとの画像の大きさ(ビニング 1): SR-5100 は 2448×2048、SR-5000 は 1376×1024。
    /// 旧システムのレシピでは、2023-06-20 までが SR5000、2023-06-21 以降が SR5100 だった。
    struct ImagerConfig {
        std::string device = "SR5100";           ///< "SR5100"(最新)または "SR5000"
        std::string binning = "BINNING_TYPE1";   ///< "BINNING_TYPE1" / "BINNING_TYPE2" / "BINNING_TYPE4"(SDK と同じ名前)
        std::string lens = "STANDARD";           ///< "WIDE" / "STANDARD" / "TELE" / "MACRO"

        /// 画像の大きさ [pixel]。省略すると、機種とビニングから決まる。
        std::optional<int> image_width;
        std::optional<int> image_height;

        int binningFactor() const { return binning == "BINNING_TYPE4" ? 4 : (binning == "BINNING_TYPE2" ? 2 : 1); }
        int sensorWidth() const { return device == "SR5000" ? 1376 : 2448; }
        int sensorHeight() const { return device == "SR5000" ? 1024 : 2048; }

        int width() const { return image_width ? *image_width : sensorWidth() / binningFactor(); }
        int height() const { return image_height ? *image_height : sensorHeight() / binningFactor(); }
    };

    /// シミュレーションのカメラ(物理的なカメラモデル)。画像の大きさは ImagerConfig で決まる。
    struct SimCameraConfig {
        /// 注視点での画像の解像度 [pixel/mm]。既定値は、旧システムの 1.81(SR-5000、1376 pixel)を
        /// SR-5100 の画素数(2448 pixel)に換算したもの。同じ視野になる。
        Real pixels_per_mm = 3.22;
        std::optional<Vec3> target;   ///< 省略すると、典型的な測定姿勢での試料位置を向く
    };

    /// シミュレーションで「測られる試料」の BRDF。
    struct SimSampleConfig {
        std::string model = "lambert";   ///< "lambert" または "phong"
        Real peak_nm = 550.0;            ///< 分光反射率の山の中心 [nm]
        Real width_nm = 60.0;            ///< 山の幅 [nm]
        Real peak = 0.8;                 ///< 山の高さ
        Real base = 0.05;                ///< 底上げ
        Real specular = 0.5;             ///< phong: 鏡面成分の強さ
        Real shininess = 50.0;           ///< phong: 鋭さ
    };

    /// 試料面の点を画像上の位置に直す方式と、キャリブボードの設定。
    struct CalibrationConfig {
        /// "camera-model": シミュレータのカメラモデルで直接求める(sim のみ。校正は不要)
        /// "board"       : キャリブボードの撮影から求めたホモグラフィを使う(実機はこちら)
        std::string projector = "camera-model";

        calibration::BoardLayout board = calibration::BoardLayout::legacy2023();

        /// 試料座標の原点(右アーム先端のクロスヘア)が、ボード座標のどこにあるか [mm]。省略すると紙の中心。
        std::optional<Vec2> sample_origin;

        /// ボードの取り付け方。ボードを正面(カメラ側)から見て、右向きの軸 u・下向きの軸 v が、
        /// 試料座標(紙の座標系)のどの軸に当たるか: "+x" "-x" "+y" "-y"。
        /// 既定値(u = +x, v = -y)は、シミュレータのカメラから見た向き(試料の Y 軸が上向き)に合わせてある。
        /// 実機では、ボードの取り付け方に合わせて設定する。
        std::string board_u = "+x";
        std::string board_v = "-y";

        /// ホモグラフィの再投影誤差(rms)の許容値 [pixel]。超えた校正は失敗として扱う。
        Real max_rms_px = 3.0;

        Vec2 sampleOrigin() const {
            return sample_origin ? *sample_origin : Vec2(board.paper_width * 0.5, board.paper_height * 0.5);
        }

        /// 試料座標 → ボード座標。board_u / board_v が正しくなければエラー(設定の読み込み時に検査される)。
        Result<calibration::SampleToBoard> mapping() const {
            return calibration::SampleToBoard::make(board_u, board_v, sampleOrigin());
        }
    };

    /// スポット定義ファイル(SR-5000/5100 のソフトに読み込ませる XML)の設定。
    /// ROI の大きさとビニングは [imager] の値を使う。
    struct SpotXmlConfig {
        Real spot_size = 2.0;                    ///< 各スポットの一辺 [pixel]
    };

    /// シミュレーションだけで使う設定。
    struct SimConfig {
        SimCameraConfig camera;
        sim::MockImagerOptions imager;
        SimSampleConfig sample;
        Real white_reflectance = 0.99;   ///< シミュレータの標準拡散反射板の反射率(白色板の測定で使う)
    };

    /// 解析(brdf analyze)の設定。
    struct AnalysisConfig {
        /// 標準拡散反射板の反射率 ρ(全波長で一定とみなす)。BRDF = 放射輝度率 × ρ / π
        Real white_reflectance = 0.99;

        /// 出力する波長の範囲 [nm]。LED の出力が足りない 380 nm 付近と 770 nm 付近は信頼性が低いとされる。
        Real min_wavelength_nm = 380.0;
        Real max_wavelength_nm = 780.0;

        /// 試料の測定点と反射板の基準点の照度の違い(点光源として cosθi / d²)を補正するか。
        /// false は旧システムと同じ(クロスヘアの 1 点を、全測定点の代表値とする)。
        bool irradiance_correction = false;
    };

    /// setup.toml の内容。旧システムでヘッダに直書きしていた装置の定数をここに集める。
    struct SetupConfig {
        DeviceMode mode = DeviceMode::Sim;
        geometry::RigSetup rig;
        ImagerConfig imager;
        CalibrationConfig calibration;
        SpotXmlConfig spot_xml;
        AnalysisConfig analysis;
        SimConfig sim;
    };

    /// TOML の文字列から読む。origin はエラーメッセージに出すファイル名。
    Result<SetupConfig> parseSetup(std::string_view text, std::string_view origin = "setup.toml");

    /// ファイルから読む。
    Result<SetupConfig> loadSetup(const std::filesystem::path& path);

    /// コメント付きの TOML に書き出す。既定値の SetupConfig を書けば、そのままひな型になる。
    std::string toToml(const SetupConfig& setup);

} // namespace brdf::io
