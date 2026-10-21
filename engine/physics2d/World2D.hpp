#pragma once

#include "physics2d/PhysicsTypes.hpp"

class World2D {
public:
    std::vector<Circle> circles;
    std::vector<Polygon> polygons;

    float width = 800.0f;
    float height = 600.0f;

    World2D();
    void step(float dt);

private:
    Body wall;
    std::vector<ContactManifold> manifolds;

    // Collision：World2Dの状態を操作する関数
    /* 壁衝突 */
    void circleWallCollision(Circle& circle);
    void polygonWallCollision(std::vector<Polygon>& polygons);
    /* 物体衝突 */
    void circleCircleCollision();
    void polygonPolygonCollision(std::vector<Polygon>& polygons);
    void circlePolygonCollision(std::vector<Circle>& circles, std::vector<Polygon>& polygons);
};


/* 関数宣言 */


/* 数値積分 */
void updateAcc(Body& body);
void updateVel(Body& body, float dt);
void updatePos(Body& body, float dt);
void updateAngle(Body& body, float dt);


/* Collision */

/* circle */
std::optional<CollisionInfo> detectCircleCircleCollision(const Circle& a, const Circle&b);

/* SAT */
std::optional<SATCollisionInfo> detectPolygonPolygonCollision(const Polygon& a, const Polygon& b);
std::optional<SATCollisionInfo> detectCirclePolygonCollision(const Circle& a, const Polygon& b);

/* 接触点生成 */
std::optional<CollisionInfo> generateCollisionInfo(const Polygon& a, const Polygon& b, const SATCollisionInfo& satCollision);
std::vector<AxisInfo> getCandidateAxesForSAT(std::vector<Vec2>& vertices, char whichObj);
AxisInfo getCandidateAxisForSATFromCircle(const Body& circleBody, const std::vector<Vec2>& vertices);
Interval getProjectedIntervalOnAxis(const std::vector<Vec2>& vertices, const Vec2& axis);
Interval getProjectedIntervalOnAxisFromCircle(const Circle& circle, const Vec2& axis);
AxisInfo detectReferenceAxis(const Vec2& normalRI, const std::vector<AxisInfo>& axes);
AxisInfo detectIncidentAxis(const Vec2& normalRI, const std::vector<AxisInfo>& axes);
Edge getEdge(const std::vector<Vec2>& vertices, int edgeNum);
std::optional<Edge> clipSegmentToLine(Edge points, const Vec2& borderNormal, const Vec2& borderPos);
std::optional<std::vector<Vec2>> decideContactPoint(Edge points, Vec2& borderNormal, Vec2& borderPos);

/* solver */
void resolveCollision(ContactManifold& manifold, Contact& contact);
void resolvePenetration(ContactManifold& manifold);
void applyImpulse(Body& body, const Vec2& impulse, const Vec2& contactArm);

/* 幾何・物理補助 */
float dot(const Vec2& a, const Vec2& b);
float cross(const Vec2& a, const Vec2& b);
std::vector<Vec2> getVerticesPos(const Polygon& polygon);
Vec2 getContactVelocity(const Body& body, const Vec2& contactArm);
Vec2 rotate(const Vec2& v, float angle);
float clamp(const float value, const float min, const float max);

/* 数値チェック */
void numcheck(const Body& body);