#include <iostream>
#include <vector>
#include <cmath>
#include <optional>
#include <random>
#include <limits>
#include <algorithm>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include "math/Vec2.hpp"


/* 色の管理用 */
struct RGB {
    float r;
    float g;
    float b;
};

/* Obj用の構造体 */

struct Body {
    Vec2 pos; // 重心を示す
    Vec2 vel;
    Vec2 acc;
    float mass;
    float invMass;
    float angle; // 時計回りが正, rad
    float angularVel; // rad/s
    float inertia;
    float invInertia;
};

struct Circle {
    Body body;
    float radius;
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
    std::vector<Vec2> contactPoints; // 接触点のワールド座標(いずれベクトルに変更,とりま1点のみ保持)
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


// デバッグ描画用のデータを保持するための構造体
// いずれ色も持たせたい
struct DebugLine {
    Vec2 p1;
    Vec2 p2;
};
std::vector<DebugLine> debugLines;
std::vector<Vec2> debugPoints;

/* プロトタイプ宣言 */

/* UpdatePhysics */
void updatePhysics(float dt);

/* 加速度・速度・位置・角度更新 */
void updateAcc(Body& body);
void updateVel(Body& body, float dt);
void updatePos(Body& body, float dt);
void updateAngle(Body& body, float dt);

/* Collision */
void circleWallCollision(Circle& cirlce);
void polygonWallCollision(std::vector<Polygon>& polygons);
void resolveWallCollision(Body& body, const CollisionInfo& collision);
void circleCircleCollision();
std::optional<CollisionInfo> detectCircleCircleCollision(const Circle& a, const Circle&b);
void polygonPolygonCollision(std::vector<Polygon>& polygons);
void circlePolygonCollision(std::vector<Circle>& circles, std::vector<Polygon>& polygons);
std::optional<SATCollisionInfo> detectPolygonPolygonCollision(const Polygon& a, const Polygon& b);
std::optional<SATCollisionInfo> detectCirclePolygonCollision(const Circle& a, const Polygon& b);
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

/* 応答 */
void resolveCollision(ContactManifold& manifold, Contact& contact);
void resolvePenetration(ContactManifold& manifold);
void applyImpulse(Body& body, const Vec2& impulse, const Vec2& contactArm);

/* Initialize Obj */
void InitializeObj();

/* Render */
void render(SDL_Renderer* renderer);
void drawCircle(SDL_Renderer* renderer, const Circle& circle);
void drawPolygon(SDL_Renderer* renderer, const Polygon& polygon);

/* Utility Func */
float dot(const Vec2& a, const Vec2& b);
float cross(const Vec2& a, const Vec2& b);
std::vector<Vec2> getVerticesPos(const Polygon& polygon);
Vec2 getContactVelocity(const Body& body, const Vec2& contactArm);
Vec2 rotate(const Vec2& v, float angle);
float clamp(const float value, const float min, const float max);


/* 設定 */

// 処理用
const int solverIterations = 10;

// Obj用
int circleNum = 3;
std::vector<Circle> circles(circleNum);
int rectangleNum = 3;
int triangleNum = 2;
std::vector<Polygon> polygons;

// 衝突情報用
std::vector<ContactManifold> manifolds;

/* 定数 */
const double PI = std::acos(-1.0);
const float GRAVITY = 98.0f;
const float WALLRESTITUTION = 0.0f;
const float OBJRESTITUTION = 0.5;
const float WALLKINETICMU = 0.5f;
const float OBJMU = 0.5f;
const float RESTITUTIONTHRESHOLD = 10.0f; // 暫定(要検証)
const float INF = std::numeric_limits<float>::infinity();

// WindowSize
const float W = 800;
const float H = 600;
// WindowMode
const SDL_WindowFlags flags = 0; // 特に機能を持たせる必要ないので0

int main(int argc, char* argv[]) {
    std::cout<< "PhysicsLab has started up\n";
    
    /* SDL処理 */

    /* 初期化 */
    bool isSDLInitialized = SDL_Init(SDL_INIT_VIDEO);

    // SDL初期化失敗
    if (!isSDLInitialized) {
        std::cout << "SDL初期化に失敗\n";
        // エラー内容の表示
        std::cout << SDL_GetError() << "\n";
        // 終了処理
        SDL_Quit();
        return 1;
    }

    /* Window作成 */
    SDL_Window* window = SDL_CreateWindow("PhysicsLab", W, H, flags);

    // window作成失敗(nullptr)
    if (!window) {
        std::cout << "window作成に失敗\n";
        // エラー内容の表示
        std::cout << SDL_GetError() << "\n";
        // 終了処理
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }


    /* Renderer作成 */
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    // renderer生成失敗(nullptr)
    if (!renderer) {
        std::cout << "renderer生成に失敗\n";
        // エラー内容の表示
        std::cout << SDL_GetError() << "\n";
        // 終了処理
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    };

    /* Objの初期化 */
    InitializeObj();


    /* MainLoop */

    bool running = true;
    const float PHYSICSDT = 1 / 60.0f;
    float accumulator = 0.0f;
    Uint64 lastTime = SDL_GetTicksNS();

    while (running) {
        Uint64 currentTime = SDL_GetTicksNS();
        Uint64 elapsedNS = currentTime - lastTime;
        lastTime = currentTime;

        float frameTime = elapsedNS / 1e+9f; // ns -> s

        accumulator += frameTime;


        /* Event */
        SDL_Event event;
        // 溜まっているeventを処理
        while (SDL_PollEvent(&event)) {
            // ウィンドウを閉じるeventの場合
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN) {
                if (event.key.key == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }

        /* UpdatePhysics */
        while (accumulator >= PHYSICSDT) {
            updatePhysics(PHYSICSDT);
            accumulator -= PHYSICSDT;
        }
        
        /* Render */
        render(renderer);
    }

    // 終了処理(windowとSDL)
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "PhysicsLab completed successfully\n";

    return 0;
}



/* UpdatePhysics */
void updatePhysics(float dt) {
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
                contact.targetVel = -OBJRESTITUTION * vnInit;
            }
        }
    }

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
};


/* 加速度・速度・位置・角度更新 */
void updateAcc(Body& body) {
    body.acc = Vec2(0.0f, 1.0f) * GRAVITY;
}

void updateVel(Body& body, float dt) {
    body.vel += body.acc * dt;
}

void updatePos(Body& body, float dt) {
    body.pos += body.vel * dt;
}
void updateAngle(Body& body, float dt) {
    body.angle += body.angularVel * dt;
}


/* Collision */

void circleWallCollision(Circle& circle) {
    Body& body = circle.body;
    // LeftWall
    if (body.pos.x - circle.radius < 0.0f) {
        float penetration = -(body.pos.x - circle.radius);
        Vec2 normal(1.0f, 0.0f);
        Vec2 contactPoint(body.pos.x - circle.radius, body.pos.y);
        std::vector<Vec2> contactPoints = { contactPoint };
        CollisionInfo collision{
            penetration,
            normal,
            contactPoints
        };
        resolveWallCollision(body, collision);
    }
    // RightWall
    if (body.pos.x + circle.radius > W) {
        Vec2 normal(-1.0f, 0.0f);
        float penetration = body.pos.x + circle.radius - W;
        Vec2 contactPoint(body.pos.x + circle.radius, body.pos.y);
        std::vector<Vec2> contactPoints = { contactPoint };
        CollisionInfo collision{
            penetration,
            normal,
            contactPoints
        };
        resolveWallCollision(body, collision);
    }
    // TopWall
    if (body.pos.y - circle.radius < 0.0f) {
        Vec2 normal(0.0f, 1.0f);
        float penetration = -(body.pos.y - circle.radius);
        Vec2 contactPoint(body.pos.x, body.pos.y - circle.radius);
        std::vector<Vec2> contactPoints = { contactPoint };
        CollisionInfo collision{
            penetration,
            normal,
            contactPoints
        };
        resolveWallCollision(body, collision);
    }
    // BottomWall
    if (body.pos.y + circle.radius > H) {
        Vec2 normal(0.0f, -1.0f);
        float penetration = body.pos.y + circle.radius - H;
        Vec2 contactPoint(body.pos.x, body.pos.y + circle.radius);
        std::vector<Vec2> contactPoints = { contactPoint };
        CollisionInfo collision{
            penetration,
            normal,
            contactPoints
        };
        resolveWallCollision(body, collision);
    }
}

void polygonWallCollision(std::vector<Polygon>& polygons) {
    for (Polygon& polygon : polygons) {
        Body& body = polygon.body;
        std::vector<Vec2> vertices = getVerticesPos(polygon);

        // 一番壁に近い頂点の座標を取得
        // 2点接触は考慮していない(後で実装)
        float left = INF;
        float right = -INF;
        float top = INF;
        float bottom = -INF;
        Vec2 cLeft, cRight, cTop, cBottom;
        for (Vec2& vertex : vertices) {
            float x = vertex.x;
            float y = vertex.y;
            if (x < left) {
                left = x;
                cLeft = vertex;
            }
            if (x > right) {
                right = x;
                cRight = vertex;
            }
            if (y < top) {
                top = y;
                cTop = vertex;
            }
            if (y > bottom) {
                bottom = y;
                cBottom = vertex;
            }
        }

        if (left < 0.0f) {
            // 後でcollisioninfo使う形にする。
            // 壁をobjとして扱うようにするときにやる。
            Vec2 normal(1.0f, 0.0f);
            float penetration = -left;
            std::vector<Vec2> contactPoints = { cLeft };
            CollisionInfo collision{
                penetration,
                normal,
                contactPoints
            };
            resolveWallCollision(body, collision);
        }
        if (right > W) {
            Vec2 normal(-1.0f, 0.0f);
            float penetration = right - W;
            std::vector<Vec2> contactPoints = { cRight };
            CollisionInfo collision{
                penetration,
                normal,
                contactPoints
            };
            resolveWallCollision(body, collision);
        }
        if (top < 0.0f) {
            Vec2 normal(0.0f, 1.0f);
            float penetration = -top;
            std::vector<Vec2> contactPoints = { cTop };
            CollisionInfo collision{
                penetration,
                normal,
                contactPoints
            };
            resolveWallCollision(body, collision);
        }
        if (bottom > H) {
            Vec2 normal(0.0f, -1.0f);
            float penetration = bottom - H;
            std::vector<Vec2> contactPoints = { cBottom };
            CollisionInfo collision{
                penetration,
                normal,
                contactPoints
            };
            resolveWallCollision(body, collision);
        }
    }
}

// normalは壁のほう線を使っているので、物体はBだと考える
void resolveWallCollision(Body& body, const CollisionInfo& collision) {
    const float penetration = collision.penetration;
    const Vec2& normal = collision.normalAB;
    const Vec2& tangent = normal.perpendicular();
    const Vec2& contactPoint = collision.contactPoints[0]; // 現状一点のみ応答
    // 重心から接触点へのベクトル
    Vec2 contactArmB = contactPoint - body.pos;

    Vec2 contactVelB = getContactVelocity(body, contactArmB);

    Vec2 relativeVel = contactVelB - Vec2(); // 壁は速度0
    float vn = dot(relativeVel, normal);

    // 近づいている場合は力積作用
    if (vn < 0) {
        // 有効質量
        float rnB = cross(contactArmB, normal);
        float effectiveMassNormal =
            body.invMass
            + rnB * rnB * body.invInertia;

        // 法線力積
        float normalImpulseScalar = -(1 + WALLRESTITUTION) * vn / effectiveMassNormal;
        Vec2 Jn = normal * normalImpulseScalar;

        // 力積適用
        // A: -Jn, B: Jn
        applyImpulse(body, Jn, contactArmB);

        // 反映した後なので再計算
        Vec2 contactVelB = getContactVelocity(body, contactArmB);
        Vec2 relativeVel = contactVelB - Vec2();

        float vt = dot(relativeVel, tangent);

        float rtB = cross(contactArmB, tangent);
        float effectiveMassTangent =
            body.invMass
            + rtB * rtB * body.invInertia;

        float tangentImpulseScalar = -vt / effectiveMassTangent;
        // maxFriction = μ Jn, 接触継続は考慮していない。
        float maxFrictionImpulseScalar = WALLKINETICMU * normalImpulseScalar;
        float frictionImpulseScalar = clamp(tangentImpulseScalar, -maxFrictionImpulseScalar, maxFrictionImpulseScalar);

        Vec2 Jt = tangent * frictionImpulseScalar;
        // A: -Jt, B: Jt
        applyImpulse(body, Jt, contactArmB);
    }
    
    // めり込み補正
    body.pos += normal * penetration;
}

void circleCircleCollision() {
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

            manifolds.push_back(ccManifold);
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


void polygonPolygonCollision(std::vector<Polygon>& polygons) {
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

            manifolds.push_back(ppManifold);
        }
    }
}

void circlePolygonCollision(std::vector<Circle>& circles, std::vector<Polygon>& polygons) {
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

            manifolds.push_back(cpManifold);
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
    float deltaJnScalar = -(vn - targetVel) / effectiveMassNormal;

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

    float deltaJtScalar = -vt / effectiveMassTangent;

    // 累積
    contact.JtAccumulator += deltaJtScalar;

    // maxFriction = μ accJn
    float maxFrictionImpulseScalar = OBJMU * contact.JnAccumulator;
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
    body.vel += impulse * body.invMass;
    body.angularVel += cross(contactArm, impulse) * body.invInertia;
}

void resolvePenetration(ContactManifold& manifold) {
    const Vec2 normal = manifold.normalAB;
    const float penetration = manifold.penetration;
    Body& a = *manifold.bodyA;
    Body& b = *manifold.bodyB;
    // めり込み補正(質量の逆比で移動)
    float invMassSum = a.invMass + b.invMass;
    a.pos -= normal * penetration * a.invMass / invMassSum;
    b.pos += normal * penetration * b.invMass / invMassSum;
}



/* Initialize Obj */
void InitializeObj() {
    polygons.clear();
    polygons.reserve(rectangleNum + triangleNum);

    std::mt19937 gen(std::random_device{}());

    const float minRadius = 10.0f;
    const float maxRadius = 50.0f;
    const float minLength = 30.0f;
    const float maxLength = 80.0f;
    const float minMass = 0.5f;
    const float maxMass = 3.0f;

    // 分布の作成
    std::uniform_real_distribution<float> posX(50.0f, W - 50.0f);
    std::uniform_real_distribution<float> posY(50.0f, H / 2.0f);
    std::uniform_real_distribution<float> velocity(-100.0f, 100.0f);
    std::uniform_real_distribution<float> length(minLength, maxLength);
    std::uniform_real_distribution<float> radius(minRadius, maxRadius);
    std::uniform_real_distribution<float> angle(0.0f, 2.0f * PI); // rad

    
    // circle
    for (Circle& circle : circles) {
        Body& body = circle.body;
        circle.radius = radius(gen);
        // 面積に比例
        body.mass = minMass + (maxMass - minMass) * ((circle.radius * circle.radius) / (maxRadius * maxRadius));
        body.invMass = 1.0f / body.mass;

        body.pos = Vec2(posX(gen), posY(gen));

        body.vel = Vec2(velocity(gen), velocity(gen));
        
        body.angle = angle(gen);
        body.angularVel = 0.0f;

        body.inertia = body.mass * circle.radius * circle.radius * 0.5f; // I_z = m r^2 / 2
        body.invInertia = 1.0f / body.inertia;
    }

    // rectangle
    for (int i = 0; i < rectangleNum; ++i) {
        Polygon rectangle;
        Body& body = rectangle.body;
        float w = length(gen);
        float h = length(gen);
        // 面積に比例
        body.mass = minMass + (maxMass - minMass) * ((w * h) / (maxLength * maxLength));
        body.invMass = 1.0f / body.mass;

        body.pos = Vec2(posX(gen), posY(gen));

        body.vel = Vec2(velocity(gen), velocity(gen));

        // 頂点(反時計回り)
        rectangle.localVertices.resize(4);
        rectangle.localVertices[0] = Vec2(-w / 2.0f, -h / 2.0f);
        rectangle.localVertices[1] = Vec2(-w / 2.0f, h / 2.0f);
        rectangle.localVertices[2] = Vec2(w / 2.0f, h / 2.0f);
        rectangle.localVertices[3] = Vec2(w / 2.0f, -h / 2.0f);

        body.angle = angle(gen);
        body.angularVel = 0.0f;

        body.inertia = body.mass * (w * w + h * h) / 12.0f; // I_z = m (w^2 + h^2) / 12
        body.invInertia = 1.0f / body.inertia;

        // 緑
        rectangle.rgb.g = 1.0f;

        polygons.push_back(rectangle);
    }

    // triangle
    for (int i = 0; i < triangleNum; ++i) {
        Polygon triangle;
        Body& body = triangle.body;
        float w = length(gen);
        float h = length(gen);
        // 面積に比例
        body.mass = minMass + (maxMass - minMass) * ((w * h) / (maxLength * maxLength));
        body.invMass = 1.0f / body.mass;

        body.pos = Vec2(posX(gen), posY(gen));

        body.vel = Vec2(velocity(gen), velocity(gen));

        // 頂点(反時計回り)
        triangle.localVertices.resize(3);
        triangle.localVertices[0] = Vec2(0.0f, -h * 2.0f / 3.0f);
        triangle.localVertices[1] = Vec2(-w / 2.0f, h / 3.0f);
        triangle.localVertices[2] = Vec2(w / 2.0f, h / 3.0f);

        body.angle = angle(gen);
        body.angularVel = 0.0f;

        body.inertia = body.mass * ((w * w / 24.0f) + (h * h / 18.0f)); // I_z = m (w^2 / 24 + h^2 / 18)
        body.invInertia = 1.0f / body.inertia;

        // 青
        triangle.rgb.b = 1.0f;

        polygons.push_back(triangle);
    }
}


/* Render */
void render(SDL_Renderer* renderer) {
    // ScreenClear
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 黒
    SDL_RenderClear(renderer);

    // ObjDraw
    // rect(x, y)はrectの左上の頂点座標
    for (Polygon& polygon : polygons) {
        drawPolygon(renderer, polygon);
    }
    for (Circle& circle : circles) {
        drawCircle(renderer, circle);
    }

    // DebugDraw
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // 青
    for (const DebugLine& line : debugLines) {
        SDL_RenderLine(
            renderer,
            line.p1.x, line.p1.y,
            line.p2.x, line.p2.y
        );
    }
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // 白
    for (const Vec2& point : debugPoints) {
        SDL_FRect rect;
        float point_size = 10.0f; // 点の大きさ

        // 中心が (x, y) になるように座標を調整
        rect.x = point.x - (point_size / 2.0f);
        rect.y = point.y - (point_size / 2.0f);
        rect.w = point_size;
        rect.h = point_size;

        SDL_RenderFillRect(renderer, &rect);
    }

    // Display
    SDL_RenderPresent(renderer);
}


// 円の描画用
void drawCircle(SDL_Renderer* renderer, const Circle& circle) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // 赤
    const int SEGMENTS = 64;

    SDL_FPoint points[SEGMENTS + 1];

    // 描画点の準備
    for (int i = 0; i <= SEGMENTS; ++i) {
        float theta = 2.0f * PI * i / SEGMENTS;

        points[i].x = 
            circle.body.pos.x
            + circle.radius * std::cos(theta);
            
        points[i].y =
            circle.body.pos.y
            + circle.radius * std::sin(theta);
    }

    // renderer更新
    SDL_RenderLines(renderer, points, SEGMENTS + 1);

    // fill
    // SEGMENTS = points.size() - 1
    for (int i = 1; i < SEGMENTS; ++i) {
        int p1 = 0;
        int p2 = i;
        int p3 = i + 1;
        SDL_Vertex vert[3];
        vert[0].position.x = points[p1].x;
        vert[0].position.y = points[p1].y;
        vert[1].position.x = points[p2].x;
        vert[1].position.y = points[p2].y;
        vert[2].position.x = points[p3].x;
        vert[2].position.y = points[p3].y;
        // 赤
        for (int j = 0; j < 3; ++j) {
            vert[j].color.r = 1.0;
            vert[j].color.g = 0.0;
            vert[j].color.b = 0.0;
            vert[j].color.a = 1.0;
        }
        SDL_RenderGeometry(renderer, NULL, vert, 3, NULL, 0);
    }
}

// polygonの描画
void drawPolygon(SDL_Renderer* renderer,const Polygon& polygon) {
    // 辺の色
    SDL_SetRenderDrawColor(renderer, polygon.rgb.r * 255, polygon.rgb.g * 255, polygon.rgb.b * 255, 255);

    std::vector<Vec2> worldVertices = getVerticesPos(polygon);

    std::vector<SDL_FPoint> points;
    for (const Vec2& vertex : worldVertices) {
        points.push_back({ vertex.x, vertex.y });
    }

    // 閉じる
    points.push_back(points.front());

    SDL_RenderLines(renderer, points.data(), points.size());

    // fill
    for (int i = 1; i < worldVertices.size() - 1; ++i) {
        int p1 = 0;
        int p2 = i;
        int p3 = i + 1;
        SDL_Vertex vert[3];
        vert[0].position.x = worldVertices[p1].x;
        vert[0].position.y = worldVertices[p1].y;
        vert[1].position.x = worldVertices[p2].x;
        vert[1].position.y = worldVertices[p2].y;
        vert[2].position.x = worldVertices[p3].x;
        vert[2].position.y = worldVertices[p3].y;
        // tri : 青, rec: 緑
        for (int j = 0; j < 3; ++j) {
            vert[j].color.r = polygon.rgb.r;
            vert[j].color.g = polygon.rgb.g;
            vert[j].color.b = polygon.rgb.b;
            vert[j].color.a = 1.0;
        }
        SDL_RenderGeometry(renderer, NULL, vert, 3, NULL, 0);
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