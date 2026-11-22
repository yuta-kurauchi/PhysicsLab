#pragma once
#include <iostream>
#include <cmath>
#include <array>


struct Vec4 {
    static constexpr std::size_t size = 4; // 要素数
    std::array<float, size> v{};

    // utility
    void display() const {
        std::cout << "{";
        for (int r = 0; r < size; ++r) {
            std::cout << v[r];
            if (r != size - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "} \n";
    }
};