#include <iostream>
#include <vector>
#include <cmath>
#include <optional>
#include <random>
#include <limits>
#include <algorithm>
#include "math/Vec2.hpp"
#include "physics2d/PhysicsTypes.hpp"
#include "physics2d/World2D.hpp"



// 処理用
const int solverIterations = 10;

/* 定数 */
const float GRAVITY = 98.0f;
const float WALLRESTITUTION = 0.5f;
const float OBJRESTITUTION = 0.5;
const float WALLKINETICMU = 0.5f;
const float OBJMU = 0.5f;
const float RESTITUTIONTHRESHOLD = 10.0f; // 反発切替は検証済み。値は暫定
const float EPSILON = 2.0f; // Polygon-Wall接触候補の許容差。暫定値
const float PENETRATIONSLOP = 1.0f;
const float PENETRATIONBETA = 1.0f;
const float INF = std::numeric_limits<float>::infinity();


float ContactManifold::getRestitution() const {
    if (bodyA->isStatic || bodyB->isStatic) {
        return WALLRESTITUTION;
    }
    return OBJRESTITUTION;
}

float ContactManifold::getMu() const {
    if (bodyA->isStatic || bodyB->isStatic) {
        return WALLKINETICMU;
    }
    return OBJMU;
}

World2D::World2D() {
    wall = Body{};
    wall.isStatic = true;
}


/* UpdatePhysics */
void World2D::step(float dt) {
    manifolds.clear();
    /* 共通更新 */
    for (Circle& circle : circles) {
        updateAcc(circle.body);
        updateVel(circle.body, dt);
        updatePos(circle.body, dt);
        updateAngle(circle.body, dt);
    }
    for (Polygon& polygon : polygons) {
        updateAcc(polygon.body);
        updateVel(polygon.body, dt);
        updatePos(polygon.body, dt);
        updateAngle(polygon.body, dt);
    }

    /* WallCollision */
    for (Circle& circle : circles) {
        circleWallCollision(circle);
    }
    polygonWallCollision(polygons);

    /* ObjCollision */
    circleCircleCollision();
    polygonPolygonCollision(polygons);
    circlePolygonCollision(circles, polygons);

    // ここにあるの邪魔かも。場所考えるべき
    for (ContactManifold& manifold : manifolds) {
        for (Contact& contact : manifold.contacts) {
            const Vec2 contactPoint = contact.point;
            const Vec2 normal = manifold.normalAB;
            Body& a = *manifold.bodyA;
            Body& b = *manifold.bodyB;

            // 重心から接触点へのベクトル
            Vec2 contactArmA = contactPoint - a.pos;
            Vec2 contactArmB = contactPoint - b.pos;

            Vec2 contactVelA = getContactVelocity(a, contactArmA);
            Vec2 contactVelB = getContactVelocity(b, contactArmB);

            Vec2 relativeVel = contactVelB - contactVelA;
            float vnInit = dot(relativeVel, normal);

            contact.targetVel = 0.0f;

            // 衝突
            if (vnInit < -RESTITUTIONTHRESHOLD) {
                contact.targetVel = -manifold.getRestitution() * vnInit;
            }
        }
    }

    /* Solver */
    for (int i = 0; i < solverIterations; ++i) {
        for (ContactManifold& manifold : manifolds) {
            for (Contact& contact : manifold.contacts) {
                resolveCollision(manifold, contact);
            }
        }
    }

    for (ContactManifold& manifold : manifolds) {
        resolvePenetration(manifold);
    }

    for (const Polygon& poly : polygons) {
        const Body& body = poly.body;
        numcheck(body);
    }
    for (const Circle& cir : circles) {
        const Body& body = cir.body;
        numcheck(body);
    }
}

void numcheck(const Body& body) {
    if (
        std::isfinite(body.pos.x) &&
        std::isfinite(body.pos.y) &&
        std::isfinite(body.vel.x) &&
        std::isfinite(body.vel.y) &&
        std::isfinite(body.acc.x) &&
        std::isfinite(body.acc.y) &&
        std::isfinite(body.angle) &&
        std::isfinite(body.angularVel) &&
        std::isfinite(body.invMass) &&
        std::isfinite(body.invInertia)
    ) {
        return;
    } else {
        std::cout << "NaN or INF があります。\n";
    }
}

/* 加速度・速度・位置・角度更新 */
void updateAcc(Body& body) {
    if (body.isStatic) return;
    body.acc = Vec2(0.0f, 1.0f) * GRAVITY;
}

void updateVel(Body& body, float dt) {
    if (body.isStatic) return;
    body.vel += body.acc * dt;
}

void updatePos(Body& body, float dt) {
    if (body.isStatic) return;
    body.pos += body.vel * dt;
}
void updateAngle(Body& body, float dt) {
    if (body.isStatic) return;
    body.angle += body.angularVel * dt;
}


/* Collision */

/*
WallCollision
Aを壁(静的オブジェクト)としてManifoldsに追加
ここで解く必要はない、Manifoldsに追加さえすれば良い
*/
void World2D::circleWallCollision(Circle& circle) {
    Body& body = circle.body;
    // LeftWall
    if (body.pos.x - circle.radius < 0.0f) {
        float penetration = -(body.pos.x - circle.radius);
        Vec2 normal(1.0f, 0.0f);
        Vec2 contactPoint(body.pos.x - circle.radius, body.pos.y);
        std::vector<Contact> cPoints {Contact{ contactPoint }};
        ContactManifold cwManifold {
            &wall,
            &body,
            normal,
            penetration,
            cPoints
        };
        if (!(cwManifold.bodyA->isStatic && cwManifold.bodyB->isStatic)) {
            manifolds.push_back(cwManifold);
        }
    }
    // RightWall
    if (body.pos.x + circle.radius > width) {
        Vec2 normal(-1.0f, 0.0f);
        float penetration = body.pos.x + circle.radius - width;
        Vec2 contactPoint(body.pos.x + circle.radius, body.pos.y);
        std::vector<Contact> cPoints {Contact{ contactPoint }};
        ContactManifold cwManifold {
            &wall,
            &body,
            normal,
            penetration,
            cPoints
        };
        if (!(cwManifold.bodyA->isStatic && cwManifold.bodyB->isStatic)) {
            manifolds.push_back(cwManifold);
        }
    }
    // TopWall
    if (body.pos.y - circle.radius < 0.0f) {
        Vec2 normal(0.0f, 1.0f);
        float penetration = -(body.pos.y - circle.radius);
        Vec2 contactPoint(body.pos.x, body.pos.y - circle.radius);
        std::vector<Contact> cPoints {Contact{ contactPoint }};
        ContactManifold cwManifold {
            &wall,
            &body,
            normal,
            penetration,
            cPoints
        };
        if (!(cwManifold.bodyA->isStatic && cwManifold.bodyB->isStatic)) {
            manifolds.push_back(cwManifold);
        }
    }
    // BottomWall
    if (body.pos.y + circle.radius > height) {
        Vec2 normal(0.0f, -1.0f);
        float penetration = body.pos.y + circle.radius - height;
        Vec2 contactPoint(body.pos.x, body.pos.y + circle.radius);
        std::vector<Contact> cPoints {Contact{ contactPoint }};
        ContactManifold cwManifold {
            &wall,
            &body,
            normal,
            penetration,
            cPoints
        };
        if (!(cwManifold.bodyA->isStatic && cwManifold.bodyB->isStatic)) {
            manifolds.push_back(cwManifold);
        }
    }
}

void World2D::polygonWallCollision(std::vector<Polygon>& polygons) {
    for (Polygon& polygon : polygons) {
        Body& body = polygon.body;
        std::vector<Vec2> vertices = getVerticesPos(polygon);

        // 一番壁に近い頂点の座標を取得
        float left = INF;
        float right = -INF;
        float top = INF;
        float bottom = -INF;
        for (Vec2& vertex : vertices) {
            float x = vertex.x;
            float y = vertex.y;
            if (x < left) left = x;
            if (x > right) right = x;
            if (y < top) top = y;
            if (y > bottom) bottom = y;
        }

        // ほぼ平行の場合は2点格納
        std::vector<Vec2> cLefts, cRights, cTops, cBottoms;
        for (Vec2& vertex : vertices) {
            float x = vertex.x;
            float y = vertex.y;
            if (abs(left - x) < EPSILON) {
                cLefts.push_back(vertex);
            }
            if (abs(right - x) < EPSILON) {
                cRights.push_back(vertex);
            }
            if (abs(top - y) < EPSILON) {
                cTops.push_back(vertex);
            }
            if (abs(bottom - y) < EPSILON) {
                cBottoms.push_back(vertex);
            }
        }

        if (left < 0.0f) {
            Vec2 normal(1.0f, 0.0f);
            float penetration = -left;
            std::vector<Contact> cPoints;
            for (Vec2& contactPoint : cLefts) {
                cPoints.push_back(Contact{ contactPoint });
            }
            ContactManifold pwManifold {
                &wall,
                &body,
                normal,
                penetration,
                cPoints
            };
            if (!(pwManifold.bodyA->isStatic && pwManifold.bodyB->isStatic)) {
                manifolds.push_back(pwManifold);
            }
        }
        if (right > width) {
            Vec2 normal(-1.0f, 0.0f);
            float penetration = right - width;
            std::vector<Contact> cPoints;
            for (Vec2& contactPoint : cRights) {
                cPoints.push_back(Contact{ contactPoint });
            }
            ContactManifold pwManifold {
                &wall,
                &body,
                normal,
                penetration,
                cPoints
            };
            if (!(pwManifold.bodyA->isStatic && pwManifold.bodyB->isStatic)) {
                manifolds.push_back(pwManifold);
            }
        }
        if (top < 0.0f) {
            Vec2 normal(0.0f, 1.0f);
            float penetration = -top;
            std::vector<Contact> cPoints;
            for (Vec2& contactPoint : cTops) {
                cPoints.push_back(Contact{ contactPoint });
            }
            ContactManifold pwManifold {
                &wall,
                &body,
                normal,
                penetration,
                cPoints
            };
            if (!(pwManifold.bodyA->isStatic && pwManifold.bodyB->isStatic)) {
                manifolds.push_back(pwManifold);
            }
        }
        if (bottom > height) {
            Vec2 normal(0.0f, -1.0f);
            float penetration = bottom - height;
            std::vector<Contact> cPoints;
            for (Vec2& contactPoint : cBottoms) {
                cPoints.push_back(Contact{ contactPoint });
            }
            ContactManifold pwManifold {
                &wall,
                &body,
                normal,
                penetration,
                cPoints
            };
            if (!(pwManifold.bodyA->isStatic && pwManifold.bodyB->isStatic)) {
                manifolds.push_back(pwManifold);
            }
        }
    }
}


void World2D::circleCircleCollision() {
    for (int i = 0; i < circles.size(); ++i) {
        for (int j = i + 1; j < circles.size(); ++j) {
            Circle& a = circles[i];
            Circle& b = circles[j];

            // detect
            const auto collision = detectCircleCircleCollision(a, b);
            // 衝突なし
            if (!collision) continue;

            std::vector<Contact> cPoints;
            for (const Vec2& pos : (*collision).contactPoints) {
                Contact contact{ pos };
                cPoints.push_back(contact);
            }

            ContactManifold ccManifold{
                &a.body,
                &b.body,
                (*collision).normalAB,
                (*collision).penetration,
                cPoints
            };

            if (!(ccManifold.bodyA->isStatic && ccManifold.bodyB->isStatic)) {
                manifolds.push_back(ccManifold);
            }
        }
    }
}

std::optional<CollisionInfo> detectCircleCircleCollision(const Circle& a, const Circle&b) {
    Vec2 subAB = b.body.pos - a.body.pos;
    float centerDistance = subAB.length();
    if (centerDistance < 1e-3f) return std::nullopt; // 中心が一致(一旦これでok)
    float radiusSum = a.radius + b.radius;

    // 衝突(めり込みのみ,接触は後に実装)
    if (centerDistance < radiusSum) {
        float penetration = radiusSum - centerDistance;
        Vec2 normalAB = subAB.normalize();
        Vec2 pointA = a.body.pos + normalAB * a.radius;
        Vec2 pointB = b.body.pos - normalAB * b.radius;
        Vec2 contactPoint = (pointA + pointB) * 0.5f;
        std::vector<Vec2> contactPoints = { contactPoint };
        return CollisionInfo{penetration, normalAB, contactPoints};
    } else {
        return std::nullopt;
    }
}


void World2D::polygonPolygonCollision(std::vector<Polygon>& polygons) {
    for (int i = 0; i < polygons.size(); ++i) {
        for (int j = i + 1; j < polygons.size(); ++j) {
            Polygon& a = polygons[i];
            Polygon& b = polygons[j];

            // detect, sat -> get normal and penetration
            const auto satCollision = detectPolygonPolygonCollision(a, b);
            // 衝突なし
            if (!satCollision) continue;

            // contactPoint取得
            const auto collision = generateCollisionInfo(a, b, *satCollision);

            if (!collision) continue;

            std::vector<Contact> cPoints;
            for (const Vec2& pos : (*collision).contactPoints) {
                Contact contact{ pos };
                cPoints.push_back(contact);
            }

            ContactManifold ppManifold{
                &a.body,
                &b.body,
                (*collision).normalAB,
                (*collision).penetration,
                cPoints
            };

            if (!(ppManifold.bodyA->isStatic && ppManifold.bodyB->isStatic)) {
                manifolds.push_back(ppManifold);
            }
        }
    }
}

void World2D::circlePolygonCollision(std::vector<Circle>& circles, std::vector<Polygon>& polygons) {
    for (int i = 0; i < circles.size(); ++i) {
        for (int j = 0; j < polygons.size(); ++j) {
            Circle& a = circles[i];
            Polygon& b = polygons[j];

            // detect(sat)
            const auto satCollision = detectCirclePolygonCollision(a, b);
            if (!satCollision) continue;

            /* CollisionInfo */
            // 円が埋まっている場合などを考慮した接触点ではないので後に改良する
            Vec2 contactPoint = a.body.pos + satCollision->normalAB * a.radius;
            std::vector<Vec2> contactPoints = { contactPoint };
            CollisionInfo collision{ (*satCollision).penetration, (*satCollision).normalAB, contactPoints };

            std::vector<Contact> cPoints;
            for (const Vec2& pos : collision.contactPoints) {
                Contact contact{ pos };
                cPoints.push_back(contact);
            }

            ContactManifold cpManifold{
                &a.body,
                &b.body,
                collision.normalAB,
                collision.penetration,
                cPoints
            };

            if (!(cpManifold.bodyA->isStatic && cpManifold.bodyB->isStatic)) {
                manifolds.push_back(cpManifold);
            }
        }
    }
}

/* SAT */
std::optional<SATCollisionInfo> detectPolygonPolygonCollision(const Polygon & a, const Polygon & b) {
    float minPenetration = INF;
    Vec2 normalAB;
    AxisInfo mtvAxis;

    std::vector<Vec2> verticesA = getVerticesPos(a);
    std::vector<Vec2> verticesB = getVerticesPos(b);

    std::vector<AxisInfo> axesA = getCandidateAxesForSAT(verticesA, 'a');
    std::vector<AxisInfo> axesB = getCandidateAxesForSAT(verticesB, 'b');
    std::vector<AxisInfo> axes = axesA;
    axes.insert(axes.end(), axesB.begin(), axesB.end());

    for (const AxisInfo& axis : axes) {
        Interval projectA = getProjectedIntervalOnAxis(verticesA, axis.candidateAxis);
        Interval projectB = getProjectedIntervalOnAxis(verticesB, axis.candidateAxis);

        // A固定でBを動かすと考えた時のAを抜け出すための方向ごとの移動量,
        // +: +Axis方向にBを, -: -Axis方向にBを
        float moveBPositive = projectA.max - projectB.min;
        float moveBNegative = projectB.max - projectA.min;
        float minMoveB = std::min(moveBPositive, moveBNegative);

        // 分離軸が存在 -> 衝突なし
        if (minMoveB < 0) return std::nullopt;

        // 一番小さい移動でめり込みを解消できる方向を保存
        if (minMoveB < minPenetration) {
            minPenetration = minMoveB;
            mtvAxis = axis;
            // Bを動かす場合を考えているので、動かす方向がnormalAB
            normalAB = (minMoveB == moveBPositive) ? axis.candidateAxis : axis.candidateAxis * -1;
        }
    }
    return SATCollisionInfo{ minPenetration, normalAB, mtvAxis};
}


std::optional<SATCollisionInfo> detectCirclePolygonCollision(const Circle& a, const Polygon& b) {
    float minPenetration = INF;
    Vec2 normalAB;
    AxisInfo mtvAxis;

    std::vector<Vec2> verticesB = getVerticesPos(b);

    AxisInfo axisA = getCandidateAxisForSATFromCircle(a.body, verticesB);
    std::vector<AxisInfo> axesB = getCandidateAxesForSAT(verticesB, 'b');
    std::vector<AxisInfo> axes = { axisA };
    axes.insert(axes.end(), axesB.begin(), axesB.end());

    for (const AxisInfo& axis : axes) {
        Interval projectA = getProjectedIntervalOnAxisFromCircle(a, axis.candidateAxis);
        Interval projectB = getProjectedIntervalOnAxis(verticesB, axis.candidateAxis);

        // A固定でBを動かすと考えた時のAを抜け出すための方向ごとの移動量,
        // +: +Axis方向にBを, -: -Axis方向にBを
        float moveBPositive = projectA.max - projectB.min;
        float moveBNegative = projectB.max - projectA.min;
        float minMoveB = std::min(moveBPositive, moveBNegative);

        // 分離軸が存在 -> 衝突なし
        if (minMoveB < 0) return std::nullopt;

        // 一番小さい移動でめり込みを解消できる方向を保存
        if (minMoveB < minPenetration) {
            minPenetration = minMoveB;
            mtvAxis = axis;
            // Bを動かす場合を考えているので、動かす方向がnormalAB
            normalAB = (minMoveB == moveBPositive) ? axis.candidateAxis : axis.candidateAxis * -1;
        }
    }
    return SATCollisionInfo{ minPenetration, normalAB, mtvAxis};
}


/* Polygon-Polygon, ContactPoints */
std::optional<CollisionInfo> generateCollisionInfo(const Polygon& a, const Polygon& b, const SATCollisionInfo& satCollision) {
    std::vector<Vec2> verticesA = getVerticesPos(a);
    std::vector<Vec2> verticesB = getVerticesPos(b);

    std::vector<AxisInfo> axesA = getCandidateAxesForSAT(verticesA, 'a');
    std::vector<AxisInfo> axesB = getCandidateAxesForSAT(verticesB, 'b');

    // mtvを作った方をref(衝突された側と考える)にする。
    bool referenceIsA = satCollision.mtvAxis.whichObj == 'a';

    // incはrefと逆の物体を選ぶ
    const Polygon& referenceObj = referenceIsA ? a : b;
    const Polygon& incidentObj = referenceIsA ? b : a;

    std::vector<AxisInfo> referenceAxes = referenceIsA ? axesA : axesB;
    std::vector<AxisInfo> incidentAxes = referenceIsA ? axesB : axesA;

    // 法線RI (接触点を生成用)
    // reference -> incident に向きを統一
    Vec2 normalRI = referenceIsA ?
        satCollision.normalAB : satCollision.normalAB * (-1);

    AxisInfo referenceAxis = detectReferenceAxis(normalRI, referenceAxes);
    AxisInfo incidentAxis = detectIncidentAxis(normalRI, incidentAxes);

    /* contacatPoint */
    Vec2 contactPoint;

    std::vector<Vec2> refVertices = getVerticesPos(referenceObj);
    std::vector<Vec2> incVertices = getVerticesPos(incidentObj);
    Edge refEdge = getEdge(refVertices, referenceAxis.edgeNum);
    Edge incEdge = getEdge(incVertices, incidentAxis.edgeNum);


    Vec2 refEdge12 = refEdge.p2 - refEdge.p1;
    Vec2 refTangent = refEdge12.normalize();
    Vec2 minusrefTangent = refTangent * -1;


    /* p1より右側になるようにincEdgeをclipping */
    auto clipped = clipSegmentToLine(incEdge, minusrefTangent, refEdge.p1);
    // 接触点候補なし
    if (!clipped) return std::nullopt;
    /* p2より左側になるようにincEdgeをclipping */
    clipped = clipSegmentToLine(*clipped, refTangent, refEdge.p2);
    // 接触点候補なし
    if (!clipped) return std::nullopt;
    /* referenceEdgeより内側(obj内部) */
    auto contactPoints = decideContactPoint(*clipped, normalRI, refEdge.p1);
    // 接触点なし
    if (!contactPoints) return std::nullopt;

    return CollisionInfo{ satCollision.penetration, satCollision.normalAB, *contactPoints };
}


/* SATの候補軸を生成
AxisInfo
- candidateAxis
- whichObj: a or b
- edgeNum: 軸を生成した辺の番号(edgeNum = 0, vertices[0] -> vertices[1])
*/
std::vector<AxisInfo> getCandidateAxesForSAT(std::vector<Vec2>& vertices, char whichObj) {
    std::vector<AxisInfo> axes;
    for (int i = 0; i < vertices.size(); ++i) {
        Vec2 p1 = vertices[i];
        int next = (i + 1) % vertices.size();
        Vec2 p2 = vertices[next];
        Vec2 edge = p2 - p1;
        // 頂点が反時計回りなので、外向き
        Vec2 axis = edge.normalize().perpendicular();
        axes.push_back({ axis, whichObj, i});
    }
    return axes;
}

// 円に一番近い頂点から軸を生成
// - edgeNum: 軸を生成した円に一番近い頂点の番号
AxisInfo getCandidateAxisForSATFromCircle(const Body& circleBody, const std::vector<Vec2>& vertices) {
    float minDistance = INF;
    Vec2 axis;
    int count = 0;
    int vertexNum = 0;
    for (const Vec2& vertex : vertices) {
        Vec2 vertexToCenter = circleBody.pos - vertex;
        float distance = vertexToCenter.length();
        if (distance < minDistance) {
            minDistance = distance;
            axis = vertexToCenter.normalize();
            vertexNum = count;
        }
        count++;
    }
    return AxisInfo{ axis, 'b', vertexNum};
}


Interval getProjectedIntervalOnAxis(const std::vector<Vec2>& vertices, const Vec2& axis) {
    Interval interval(INF, -INF);
    for (const Vec2 vertex : vertices) {
        float project = dot(vertex, axis);
        if (project < interval.min) {
            interval.min = project;
        }
        if (project > interval.max) {
            interval.max = project;
        }
    }
    return interval;
}

Interval getProjectedIntervalOnAxisFromCircle(const Circle& circle, const Vec2& axis) {
    float project = dot(circle.body.pos, axis);
    return Interval(project - circle.radius, project + circle.radius);
}

/*
各辺の軸は外向きなので、

Reference Edge
→ normalRI と最も同じ向きの外向き法線
→ dot 最大

Incident Edge
→ normalRI と最も逆向きの外向き法線
→ dot 最小
*/

AxisInfo detectReferenceAxis(const Vec2& normalRI, const std::vector<AxisInfo>& axes) {
    float maxDot = -INF;
    AxisInfo refAxis;

    for (const AxisInfo& axis : axes) {
        float dotValue = dot(normalRI, axis.candidateAxis);

        if (dotValue > maxDot) {
            maxDot = dotValue;
            refAxis = axis;
        }
    }
    return refAxis;
}

AxisInfo detectIncidentAxis(const Vec2& normalRI, const std::vector<AxisInfo>& axes) {
    float minDot = INF;
    AxisInfo refAxis;

    for (const AxisInfo& axis : axes) {
        float dotValue = dot(normalRI, axis.candidateAxis);

        if (dotValue < minDot) {
            minDot = dotValue;
            refAxis = axis;
        }
    }
    return refAxis;
}


Edge getEdge(const std::vector<Vec2>& vertices, int edgeNum) {
    int next = (edgeNum + 1) % vertices.size();
    return { vertices[edgeNum], vertices[next] };
}


/* incEdgeの点を辺上(refEdge)の境界線内に収める (境界線で辺を切るイメージ) */
// points: incEdgeの両端,これをboderPosでclippingする
// borderNormal: 残したい方向と逆方向
std::optional<Edge> clipSegmentToLine(Edge points, const Vec2& borderNormal, const Vec2& borderPos) {
    Vec2 q1 = points.p1;
    Vec2 q2 = points.p2;
    float offset = dot(borderNormal, borderPos);
    float d1 = dot(borderNormal, q1) - offset;
    float d2 = dot(borderNormal, q2) - offset;

    // d <= 0 なら、内側なのでそのまま、
    // d > 0なら、外側なのでsを使ってclipping
    if (d1 <= 0 && d2 <= 0) {
        return Edge{ q1, q2 };
    }
    else if (d1 > 0 && d2 > 0) {
        // 両方外側
        return std::nullopt;
    }

    // points2点の辺を Q(s) = q1 + s(q2 - q1)のようにあらわしている。
    // Q(s) dot borderNormal - offset = 0より sを導出 , 
    // このsにおける点が、 points2点をつなぐ辺の境界線上の点を表すパラメータ
    float s = -d1 / (d2 - d1);
    Vec2 intersection = q1 + (q2 - q1) * s; // 境界線上の点

    if (d1 <= 0 && d2 > 0) {
        // q1 のみ内側 -> q2を境界線でclipping
        q2 = intersection;
        return Edge{ q1, q2 };
    }
    else {
        // q2 のみ内側 -> q1を境界線でclipping
        q1 = intersection;
        return Edge{ q1, q2 };
    }
}


std::optional<std::vector<Vec2>> decideContactPoint(Edge points, Vec2& borderNormal, Vec2& borderPos) {
    std::vector<Vec2> contactPoints;
    Vec2 q1 = points.p1;
    Vec2 q2 = points.p2;
    float offset = dot(borderNormal, borderPos);
    float d1 = dot(borderNormal, q1) - offset;
    float d2 = dot(borderNormal, q2) - offset;

    // d <= 0 -> めり込んでる -> contactPos
    if (d1 <= 0) {
        contactPoints.push_back(q1);
    }
    if (d2 <= 0) {
        contactPoints.push_back(q2);
    }

    // なかった場合
    if (d1 > 0 && d2 > 0) {
        return std::nullopt;
    }
    return contactPoints;
}


/* 共通応答 */
void resolveCollision(ContactManifold& manifold, Contact& contact) {
    const Vec2 contactPoint = contact.point;
    const float oldAccumJnScalar = contact.JnAccumulator;
    const float oldAccumJtScalar = contact.JtAccumulator;
    const Vec2 normal = manifold.normalAB;
    const Vec2 tangent = normal.perpendicular();
    const float targetVel = contact.targetVel;
    Body& a = *manifold.bodyA;
    Body& b = *manifold.bodyB;


    // 重心から接触点へのベクトル
    Vec2 contactArmA = contactPoint - a.pos;
    Vec2 contactArmB = contactPoint - b.pos;

    Vec2 contactVelA = getContactVelocity(a, contactArmA);
    Vec2 contactVelB = getContactVelocity(b, contactArmB);

    Vec2 relativeVel = contactVelB - contactVelA;
    float vn = dot(relativeVel, normal);


    // 有効質量
    float rnA = cross(contactArmA, normal);
    float rnB = cross(contactArmB, normal);
    float effectiveMassNormal =
        a.invMass + b.invMass
        + rnA * rnA * a.invInertia
        + rnB * rnB * b.invInertia;


    // 法線力積
    float deltaJnScalar = 0.0f;
    if (effectiveMassNormal != 0.0f) {
        deltaJnScalar = -(vn - targetVel) / effectiveMassNormal;
    }

    // 累積
    contact.JnAccumulator += deltaJnScalar;

    // 現在のstep内で、それまでに累積した法線力積を打ち消す以上にはしない
    float newAccumJnScalar = std::max(0.0f, contact.JnAccumulator);
    // クランプ後を保持
    contact.JnAccumulator = newAccumJnScalar;

    // 実際に加えるのは差分
    float normalImpulseScalar = newAccumJnScalar - oldAccumJnScalar;


    Vec2 Jn = normal * normalImpulseScalar;

    // 力積適用
    // A: -Jn, B: Jn
    applyImpulse(a, Jn * -1, contactArmA);
    applyImpulse(b, Jn, contactArmB);

    // 反映した後なので再計算
    contactVelA = getContactVelocity(a, contactArmA);
    contactVelB = getContactVelocity(b, contactArmB);
    relativeVel = contactVelB - contactVelA;

    float vt = dot(relativeVel, tangent);

    float rtA = cross(contactArmA, tangent);
    float rtB = cross(contactArmB, tangent);
    float effectiveMassTangent =
        a.invMass + b.invMass
        + rtA * rtA * a.invInertia
        + rtB * rtB * b.invInertia;


    float deltaJtScalar = 0.0f;
    if (effectiveMassTangent != 0.0f) {
        deltaJtScalar = -vt / effectiveMassTangent;
    }

    // 累積
    contact.JtAccumulator += deltaJtScalar;

    // maxFriction = μ accJn
    float maxFrictionImpulseScalar = manifold.getMu() * contact.JnAccumulator;
    float newAccumJtScalar = clamp(contact.JtAccumulator, -maxFrictionImpulseScalar, maxFrictionImpulseScalar);
    // 更新
    contact.JtAccumulator = newAccumJtScalar;

    float frictionImpulseScalar = newAccumJtScalar - oldAccumJtScalar;

    Vec2 Jt = tangent * frictionImpulseScalar;
    // A: -Jt, B: Jt
    applyImpulse(a, Jt * -1, contactArmA);
    applyImpulse(b, Jt, contactArmB);
}

void applyImpulse(Body& body,const Vec2& impulse, const Vec2& contactArm) {
    if (body.isStatic) return;
    body.vel += impulse * body.invMass;
    body.angularVel += cross(contactArm, impulse) * body.invInertia;
}

void resolvePenetration(ContactManifold& manifold) {
    const Vec2 normal = manifold.normalAB;
    const float penetration = manifold.penetration;
    const float correctionValue = PENETRATIONBETA * std::max(penetration - PENETRATIONSLOP, 0.0f);
    Body& a = *manifold.bodyA;
    Body& b = *manifold.bodyB;
    // めり込み補正(質量の逆比で移動)
    float invMassSum = a.invMass + b.invMass;
    if (invMassSum != 0.0f) {
        a.pos -= normal * correctionValue * a.invMass / invMassSum;
        b.pos += normal * correctionValue * b.invMass / invMassSum;
    }
}

/* Utility Function */

float dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

float cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - b.x * a.y;
}

std::vector<Vec2> getVerticesPos(const Polygon& polygon) {
    std::vector<Vec2> worldVertices;
    for (const Vec2& localVertex : polygon.localVertices) {
        Vec2 rotated = rotate(localVertex, polygon.body.angle);
        worldVertices.push_back(polygon.body.pos + rotated);
    }
    return worldVertices;
}

Vec2 getContactVelocity(const Body& body, const Vec2& contactArm) {
    if (body.isStatic) return Vec2();
    // omega cross r: 接線方向と考えてよい(rに垂直)
    Vec2 rotationVel = contactArm.perpendicular() * body.angularVel;
    return body.vel + rotationVel;
}

// 時計回りを正
Vec2 rotate(const Vec2& v, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Vec2(
        v.x * c - v.y * s,
        v.x * s + v.y * c
    );
}

// min <= value <= max
float clamp(const float value, const float min, const float max) {
    return std::max(min, std::min(value, max));
}