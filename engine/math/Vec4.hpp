#pragma once
#include <iostream>
#include <cmath>
#include <array>
#include "math/Vec3.hpp"


struct Vec4 {
    static constexpr std::size_t size = 4; // 要素数
    std::array<float, size> v{};

    /* コンストラクタ */
    Vec4() = default;
    Vec4(float x, float y, float z, float w) : v{x, y, z, w} {};

    // make vec4
    static Vec4 fromPoint(const Vec3& v) {
        float w = 1.0f;
        return Vec4(v.x, v.y, v.z, w);
    }
    static Vec4 fromDirection(const Vec3& v) {
        float w = 0.0f;
        return Vec4(v.x, v.y, v.z, w);
    }

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