#pragma once
#include <iostream>

struct Vec2 {
    float x;
    float y;
    
    /* コンストラクタ */
    Vec2() : x(0.0f), y(0.0f) {};
    Vec2(float x, float y) : x(x), y(y) {};

    /* メンバ関数 */
    void Display() const {
        std::cout << "(" << x << ", " << y << ")\n";
    }


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
};