#pragma once
#include <iostream>
#include <cmath>
#include <array>
#include "math/Vec3.hpp"
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

    /* Model */
    // 平行移動
    static Mat4 translation(const Vec3& t) {
        Mat4 res = identity();
        res.m[3][0] = t.x;
        res.m[3][1] = t.y;
        res.m[3][2] = t.z;
        return res;
    }
    // 回転(angle: ラジアン)
    static Mat4 rotationZ(float rad) {
        Mat4 res = identity();
        res.m[0][0] = std::cos(rad);
        res.m[0][1] = std::sin(rad);
        res.m[1][0] = -std::sin(rad);
        res.m[1][1] = std::cos(rad);
        return res;
    }
    static Mat4 rotationX(float rad) {
        Mat4 res = identity();
        res.m[1][1] = std::cos(rad);
        res.m[1][2] = std::sin(rad);
        res.m[2][1] = -std::sin(rad);
        res.m[2][2] = std::cos(rad);
        return res;
    }
    static Mat4 rotationY(float rad) {
        Mat4 res = identity();
        res.m[0][0] = std::cos(rad);
        res.m[0][2] = -std::sin(rad);
        res.m[2][0] = std::sin(rad);
        res.m[2][2] = std::cos(rad);
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