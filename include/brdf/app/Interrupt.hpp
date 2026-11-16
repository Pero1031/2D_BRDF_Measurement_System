/*
* @file include/brdf/app/Interrupt.hpp
* @brief Turns Ctrl+C into a CancelToken request while a measurement runs.
*/

#pragma once

#include <csignal>
#include <memory>

#include "brdf/core/Cancel.hpp"

namespace brdf::app {

    /// このオブジェクトがある間、Ctrl+C(SIGINT)を受けると、プロセスを終わらせる代わりに token.request() を呼ぶ。
    /// 装置の後始末(モータ OFF、切断)をしてから終わるため。
    /// **2 回目の Ctrl+C は、既定の動作(プロセスの終了)に戻る**: 中止の処理が進まないときに、止められなくならないように。
    /// 破棄すると、元のハンドラに戻る。プロセスに 1 つだけ作る(入れ子にしない)。
    class ScopedInterruptHandler {
    public:
        explicit ScopedInterruptHandler(std::shared_ptr<CancelToken> token);
        ~ScopedInterruptHandler();

        ScopedInterruptHandler(const ScopedInterruptHandler&) = delete;
        ScopedInterruptHandler& operator=(const ScopedInterruptHandler&) = delete;

    private:
        std::shared_ptr<CancelToken> token_;
        void (*previous_)(int) = SIG_DFL;
    };

} // namespace brdf::app
