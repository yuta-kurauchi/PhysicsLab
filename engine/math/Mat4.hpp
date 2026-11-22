#pragma once
#include <iostream>
#include <cmath>
#include <array>
#include "math/Vec4.hpp"


struct Mat4 {
    static constexpr std::size_t rows = 4; // 行数
    static constexpr std::size_t cols = 4; // 列数
    // m[列][行]アクセス
    std::array<std::array<float, rows>, cols> m{};

    /* 単位行列 */
    static Mat4 identity() {
        Mat4 res{};
        res.m[0][0] = 1.0f;
        res.m[1][1] = 1.0f;
        res.m[2][2] = 1.0f;
        res.m[3][3] = 1.0f;
        return res;
    }

    /* 基本演算 */
    // 行列積
    Mat4 operator*(const Mat4& other) const {
        Mat4 res{};
        for (int c = 0; c < cols; ++c) {
            for (int r = 0; r < rows; ++r) {
                for (int i = 0; i < 4; ++i) {
                    res.m[c][r] += m[i][r] * other.m[c][i];
                }
            }
        }
        return res;
    }
    // 行列*ベクトル
    Vec4 operator*(const Vec4& v4) const {
        Vec4 res{};
        for (int r = 0; r < rows; ++r) {
            for (int i = 0; i < 4; ++i) {
                res.v[r] += m[i][r] * v4.v[i];
            }
        }
        return res;
    }

    // utility
    void displayColRow() const {
        for (int c = 0; c < cols; ++c) {
            std::cout << "{";
            for (int r = 0; r < rows; ++r) {
                std::cout << m[c][r];
                if (r != rows - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "} \n";
        }
    }
};