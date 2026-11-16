/*
* @file include/brdf/domain/Recipe.hpp
* @brief Measurement recipe: which poses to visit, which points to measure, and how to capture.
*/

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "brdf/core/Algebra.hpp"
#include "brdf/driver/ISpectralImager.hpp"
#include "brdf/geometry/Joints.hpp"

namespace brdf::domain {

    /// 測定の種類。旧システムの測定モード(試料 / 白色板 / キャリブボード)に相当する。
    /// 種類によって、結果の保存先と、記録される種類が変わる。
    enum class MeasurementKind { Sample, White, Calib };

    inline const char* toString(MeasurementKind kind) {
        switch (kind) {
        case MeasurementKind::Sample: return "sample";
        case MeasurementKind::White:  return "white";
        case MeasurementKind::Calib:  return "calib";
        }
        return "sample";
    }

    inline std::optional<MeasurementKind> parseKind(std::string_view text) {
        if (text == "sample") return MeasurementKind::Sample;
        if (text == "white")  return MeasurementKind::White;
        if (text == "calib")  return MeasurementKind::Calib;
        return std::nullopt;
    }

    /// 測定で訪れる 1 つのポーズ(左右アームの関節角)。旧システムのポーズ CSV の 1 行に相当する。
    struct PoseStep {
        std::string id;              ///< ポーズの名前(結果ファイルで使う)
        geometry::Joints left;       ///< 左アーム(光源側)の関節角 [rad]
        geometry::Joints right;      ///< 右アーム(試料側)の関節角 [rad]
    };

    /// 測定したい点 1 つ。試料面の座標(紙の座標系, mm)で指定する。
    /// 原点は試料面の基準点(右アーム先端 + 試料面オフセット)、X/Y 軸は紙の座標系の軸。
    struct SpotDefinition {
        int id = 0;
        Vec2 sample_mm = Vec2::Zero();
    };

    /// 撮影条件。
    struct CaptureSettings {
        Real integration_time_ms = 100.0;       ///< 積分時間。0 のときは装置が自動で決める
        Real arm_speed = 0.5;                   ///< アームの速度(0 より大きく 1 以下)
        driver::WavelengthRange wavelengths;    ///< 波長範囲
    };

    /// 測定の手順書。旧システムの「ポーズ CSV + 試料のマスク + レシピ XML のひな型」をまとめたもの。
    struct Recipe {
        std::string name;                       ///< 測定の名前
        std::string operator_name;              ///< 担当者
        std::string description;                ///< メモ
        MeasurementKind kind = MeasurementKind::Sample;

        CaptureSettings capture;
        std::vector<SpotDefinition> spots;      ///< 測定点(1 つ以上)
        std::vector<PoseStep> poses;            ///< ポーズ(1 つ以上)
        std::vector<std::string> checklist;     ///< 測定前に人が確認する項目(旧システムの「準備はできましたか」)
    };

} // namespace brdf::domain
