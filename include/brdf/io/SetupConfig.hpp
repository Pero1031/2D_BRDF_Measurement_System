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

    /// シミュレーションのカメラ。
    struct SimCameraConfig {
        int image_width = 1376;
        int image_height = 1024;
        Real pixels_per_mm = 1.81;
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

    /// スポット定義ファイル(SR-5000 のソフトに読み込ませる XML)の設定。
    struct SpotXmlConfig {
        std::string binning = "BINNING_TYPE1";   ///< 分光放射計のビニング設定(レシピと合わせる)
        Real spot_size = 2.0;                    ///< 各スポットの一辺 [pixel]
    };

    /// setup.toml の内容。旧システムでヘッダに直書きしていた装置の定数をここに集める。
    struct SetupConfig {
        DeviceMode mode = DeviceMode::Sim;
        geometry::RigSetup rig;
        SimCameraConfig camera;
        CalibrationConfig calibration;
        SpotXmlConfig spot_xml;
        sim::MockImagerOptions imager;
        SimSampleConfig sample;
    };

    /// TOML の文字列から読む。origin はエラーメッセージに出すファイル名。
    Result<SetupConfig> parseSetup(std::string_view text, std::string_view origin = "setup.toml");

    /// ファイルから読む。
    Result<SetupConfig> loadSetup(const std::filesystem::path& path);

    /// コメント付きの TOML に書き出す。既定値の SetupConfig を書けば、そのままひな型になる。
    std::string toToml(const SetupConfig& setup);

} // namespace brdf::io
