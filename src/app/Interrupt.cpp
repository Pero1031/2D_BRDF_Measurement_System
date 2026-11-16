/*
* @file src/app/Interrupt.cpp
* @brief Turns Ctrl+C into a CancelToken request while a measurement runs.
*/

#include "brdf/app/Interrupt.hpp"

#include <atomic>
#include <utility>

namespace brdf::app {

    namespace {

        // シグナルハンドラの中でできるのは、ロックフリーなアトミック変数の読み書きだけ
        std::atomic<CancelToken*> g_token{nullptr};

        void onInterrupt(int) {
            if (CancelToken* t = g_token.load()) t->request();
            // 2 回目の Ctrl+C は、既定の動作(プロセスの終了)にする。Windows では、ハンドラが 1 回ごとに外れるので、
            // どちらにしても明示的に戻しておく
            std::signal(SIGINT, SIG_DFL);
        }

    } // namespace

    ScopedInterruptHandler::ScopedInterruptHandler(std::shared_ptr<CancelToken> token) : token_(std::move(token)) {
        g_token.store(token_.get());
        previous_ = std::signal(SIGINT, onInterrupt);
    }

    ScopedInterruptHandler::~ScopedInterruptHandler() {
        std::signal(SIGINT, previous_ == SIG_ERR ? SIG_DFL : previous_);
        g_token.store(nullptr);
    }

} // namespace brdf::app
