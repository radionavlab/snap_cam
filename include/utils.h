#pragma once

#include <chrono>

long long CurrentTimeMillis() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}
