#pragma once
#include <iostream>
#include "math/Vec3.hpp"
#include "math/Mat4.hpp"


struct Camera {
    Vec3 pos{};
    Vec3 forward{};
    Vec3 worldUp{};

    /* コンストラクタ */
    Camera() = default;
    Camera(Vec3 pos_, Vec3 forward_, Vec3 worldUp_) : pos(pos_), forward(forward_), worldUp(worldUp_) {};


    Mat4 getViewMatrix() const {
        Vec3 unitB = -forward.normalize();
        Vec3 unitR = worldUp.cross(unitB).normalize();
        Vec3 unitU = unitB.cross(unitR);

        Mat4 Rotation = Mat4::makeRotationFromAxes(unitR, unitU, unitB);

        Vec4 t = Rotation * (Vec4::fromPoint(-pos));
        Mat4 Translation = Mat4::translation(Vec3(t.v[0], t.v[1],t.v[2]));

        return Translation * Rotation;
    }
};