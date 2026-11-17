#pragma once
#include <iostream>
#include <cmath>

struct Vec3 {
    float x;
    float y;
    float z;
    
    /* コンストラクタ */
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {};
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {};


    /* 演算子のオーバーロード */

    /* 非破壊的 */
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    Vec3 operator/(float scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    /* 破壊的 */
    // 自分への参照を返す
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    Vec3& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }


    /* メンバ関数 */
    void Display() const {
        std::cout << "(" << x << ", " << y << ", " << z << ")\n";
    }
    
    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3 normalize() const {
        float len = length();
        if (len == 0.0f) return Vec3(0.0f, 0.0f, 0.0f);
        return Vec3{x / len, y / len, z / len};
    }

    float dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    Vec3 cross(const Vec3& v) const {
        return {
            y * v.z - v.y * z,
            z * v.x - v.z * x,
            x * v.y - v.x * y
        };
    }
};