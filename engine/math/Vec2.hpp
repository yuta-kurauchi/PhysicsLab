#pragma once
#include <iostream>
#include <cmath>

struct Vec2 {
    float x;
    float y;
    
    /* コンストラクタ */
    Vec2() : x(0.0f), y(0.0f) {};
    Vec2(float x, float y) : x(x), y(y) {};


    /* 演算子のオーバーロード */

    /* 非破壊的 */
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    Vec2 operator/(float scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    /* 破壊的 */
    // 自分への参照を返す
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    Vec2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }


    /* メンバ関数 */
    void Display() const {
        std::cout << "(" << x << ", " << y << ")\n";
    }
    
    float length() const {
        return std::sqrt(x * x + y * y);
    }

    Vec2 normalize() const {
        float len = length();
        if (len == 0.0f) return Vec2(0.0f, 0.0f);
        return Vec2{x / len, y / len};
    }
};