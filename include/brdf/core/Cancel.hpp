/*
* @file include/brdf/core/Cancel.hpp
* @brief A flag that carries a user's request to stop (Ctrl+C) to the code that moves the devices.
*/

#pragma once

#include <atomic>

namespace brdf {

    /// 中止の要求を伝える旗。Ctrl+C のハンドラや別のスレッドが request() で立て、
    /// 装置を動かす側(アームの移動の待ち、測定の姿勢の切り替え)が requested() で見る。
    /// 立てるのは何度でもよく、どのスレッドからでもよい(ロックを使わない)。
    class CancelToken {
    public:
        void request() noexcept { flag_.store(true, std::memory_order_relaxed); }
        bool requested() const noexcept { return flag_.load(std::memory_order_relaxed); }
        void reset() noexcept { flag_.store(false, std::memory_order_relaxed); }

    private:
        std::atomic<bool> flag_{false};
    };

} // namespace brdf
