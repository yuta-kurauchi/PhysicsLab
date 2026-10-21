#pragma once

#include <vector>
#include "math/Vec2.hpp"

/* 色の管理用(後で描画側へ移す) */
struct RGB {
    float r;
    float g;
    float b;
};

/* Obj用の構造体 */

struct Body {
    Vec2 pos{}; // 重心を示す
    Vec2 vel{};
    Vec2 acc{};
    float mass = 0.0f;
    float invMass = 0.0f;
    float angle = 0.0f; // 時計回りが正, rad
    float angularVel = 0.0f; // rad/s
    float inertia = 0.0f;
    float invInertia = 0.0f;
    bool isStatic = false; // 静的オブジェクトを作りたい時だけ、trueにする
};

struct Circle {
    Body body;
    float radius = 0.0f;
};

struct Polygon {
    Body body;
    std::vector<Vec2> localVertices;
    RGB rgb{0.0f, 0.0f, 0.0f};
};

struct AxisInfo {
    Vec2 candidateAxis;
    char whichObj;
    int edgeNum;
};

/* CollisionInfo */
struct CollisionInfo {
    float penetration;
    Vec2 normalAB; // 衝突法線: 物理処理の都合から、ABに統一して管理
    std::vector<Vec2> contactPoints;
};

struct SATCollisionInfo {
    float penetration;
    Vec2 normalAB;
    AxisInfo mtvAxis;
};

struct Contact {
    Vec2 point;
    float JnAccumulator = 0.0f;
    float JtAccumulator = 0.0f;
    float targetVel = 0.0f;
};

struct ContactManifold {
    Body* bodyA;
    Body* bodyB;

    Vec2 normalAB;
    float penetration;

    std::vector<Contact> contacts;

    float getRestitution() const;
    float getMu() const;
};


struct Interval {
    float min;
    float max;
    Interval(float min, float max) : min(min), max(max) {};
};

struct Edge {
    Vec2 p1;
    Vec2 p2;
};