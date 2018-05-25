#pragma once

#include <chrono>

long long CurrentTimeMillis() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

void TicToc() {
    static long long previous = 0L;
    static bool tic = false;

    long long now = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    if(tic) {
       tic = false;
       std::cout << "Time Elapsed: " << now - previous << " ms" << std::endl; 
    } else {
        tic = true;
        previous = now;
    }
}
