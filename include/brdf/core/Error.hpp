/*
* @file include/brdf/core/Error.hpp
* @brief Error type and Result alias shared by drivers, simulator and application code.
*/

#pragma once

#include <string>
#include <utility>

#include <tl/expected.hpp>

namespace brdf {

    /// エラーの種類。メッセージの文字列ではなく、この種類で処理を分岐できる。
    enum class ErrorCode {
        NotConnected,       ///< 装置に接続していない
        InvalidArgument,    ///< 引数が不正(速度が範囲外、測定点が空 など)
        OutOfRange,         ///< 値が範囲外(画像の外、可動域の外 など)
        JointLimit,         ///< 関節の可動域を超える指令
        Collision,          ///< アーム同士が干渉する姿勢
        FocusError,         ///< 測定距離が実際の距離と合っていない
        DeviceError,        ///< 装置側のエラー
        Timeout,            ///< 通信が時間内に終わらなかった(装置の状態は不明。動作中だったかもしれない)
        Communication,      ///< 通信ができない・途切れた(接続できない、切断された など)
        Saturated,          ///< 測定値が飽和した(明るすぎる。照明・積分時間・ND フィルタを見直す)
        Cancelled,          ///< 利用者の要求(Ctrl+C など)で中止した
        NotImplemented,     ///< 未実装
    };

    inline const char* toString(ErrorCode code) {
        switch (code) {
        case ErrorCode::NotConnected:    return "NotConnected";
        case ErrorCode::InvalidArgument: return "InvalidArgument";
        case ErrorCode::OutOfRange:      return "OutOfRange";
        case ErrorCode::JointLimit:      return "JointLimit";
        case ErrorCode::Collision:       return "Collision";
        case ErrorCode::FocusError:      return "FocusError";
        case ErrorCode::DeviceError:     return "DeviceError";
        case ErrorCode::Timeout:         return "Timeout";
        case ErrorCode::Communication:   return "Communication";
        case ErrorCode::Saturated:       return "Saturated";
        case ErrorCode::Cancelled:       return "Cancelled";
        case ErrorCode::NotImplemented:  return "NotImplemented";
        }
        return "Unknown";
    }

    struct Error {
        ErrorCode code;
        std::string message;
    };

    /// 成功なら値、失敗なら Error を持つ。例外は使わない。
    template <typename T>
    using Result = tl::expected<T, Error>;

    /// 値を返さない処理の結果。
    using Status = Result<void>;

    /// 失敗を返すためのヘルパ:  return fail(ErrorCode::NotConnected, "...");
    inline tl::unexpected<Error> fail(ErrorCode code, std::string message) {
        return tl::unexpected<Error>(Error{code, std::move(message)});
    }

} // namespace brdf
