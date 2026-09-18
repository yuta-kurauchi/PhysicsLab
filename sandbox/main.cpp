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
    float angle; // 時計回りが正
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
    Vec2 contactPoint; // 接触点のワールド座標(いずれベクトルに変更,とりま1点のみ保持)
};

struct SATCollisionInfo {
    float penetration;
    Vec2 normalAB;
    AxisInfo mtvAxis;
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

/* Obj用 */
int circleNum = 10;
std::vector<Circle> circles(circleNum);
int rectangleNum = 10;
int triangleNum = 10;
std::vector<Polygon> polygons;

/* プロトタイプ宣言 */

/* UpdatePhysics */
void updatePhysics(float dt);

/* 加速度・速度・位置更新 */
void updateAcc(Body& body);
void updateVel(Body& body, float dt);
void updatePos(Body& body, float dt);

/* Collision */
void circleWallCollision(Circle& cirlce);
void polygonWallCollision(std::vector<Polygon>& polygons);
void resolveWallCollision(Body& body, const Vec2& normal, const float penetration);
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
void resolveCollision(Body& a, Body& b, const CollisionInfo& collision);
void applyImpulse(Body& a, Body& b, Vec2& Impulse);

/* Initialize Obj */
void InitializeObj();

/* Render */
void render(SDL_Renderer* renderer);
void drawCircle(SDL_Renderer* renderer, const Circle& circle);
void drawPolygon(SDL_Renderer* renderer, const Polygon& polygon);

/* Utility Func */
float dot(const Vec2& a, const Vec2& b);
std::vector<Vec2> getVerticesPos(const Polygon& polygon);
Vec2 rotate(const Vec2& v, float angle);

/* 定数 */
const double PI = std::acos(-1.0);
const float GRAVITY = 98.0f;
const float WALLRESTITUTION = 0.75f;
const float OBJRESTITUTION = 0.75f;
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
    /* 共通更新 */
    for (Circle& circle : circles) {
        updateAcc(circle.body);
        updateVel(circle.body, dt);
        updatePos(circle.body, dt);
    }
    for (Polygon& polygon : polygons) {
        updateAcc(polygon.body);
        updateVel(polygon.body, dt);
        updatePos(polygon.body, dt);
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
};


/* 加速度・速度・位置更新 */
void updateAcc(Body& body) {
    body.acc = Vec2(0.0f, 1.0f) * GRAVITY;
}

void updateVel(Body& body, float dt) {
    body.vel += body.acc * dt;
}

void updatePos(Body& body, float dt) {
    body.pos += body.vel * dt;
}


/* Collision */

void circleWallCollision(Circle& circle) {
    Body& body = circle.body;
    // LeftWall
    if (body.pos.x - circle.radius < 0.0f) {
        Vec2 normal(1.0f, 0.0f);
        float penetration = -(body.pos.x - circle.radius);
        resolveWallCollision(body, normal, penetration);
    }
    // RightWall
    if (body.pos.x + circle.radius > W) {
        Vec2 normal(-1.0f, 0.0f);
        float penetration = body.pos.x + circle.radius - W;
        resolveWallCollision(body, normal, penetration);
    }
    // TopWall
    if (body.pos.y - circle.radius < 0.0f) {
        Vec2 normal(0.0f, 1.0f);
        float penetration = -(body.pos.y - circle.radius);
        resolveWallCollision(body, normal, penetration);
    }
    // BottomWall
    if (body.pos.y + circle.radius > H) {
        Vec2 normal(0.0f, -1.0f);
        float penetration = body.pos.y + circle.radius - H;
        resolveWallCollision(body, normal, penetration);
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
        for (Vec2& vertex : vertices) {
            if (vertex.x < left) {
                left = vertex.x;
            }
            if (vertex.x > right) {
                right = vertex.x;
            }
            if (vertex.y < top) {
                top = vertex.y;
            }
            if (vertex.y > bottom) {
                bottom = vertex.y;
            }
        }

        if (left < 0.0f) {
            // 後でcollisioninfo使う形にする。
            // 壁をobjとして扱うようにするときにやる。
            Vec2 normal(1.0f, 0.0f);
            float penetration = -left;
            resolveWallCollision(body, normal, penetration);
        }
        if (right > W) {
            Vec2 normal(-1.0f, 0.0f);
            float penetration = right - W;
            resolveWallCollision(body, normal, penetration);
        }
        if (top < 0.0f) {
            Vec2 normal(0.0f, 1.0f);
            float penetration = -top;
            resolveWallCollision(body, normal, penetration);
        }
        if (bottom > H) {
            Vec2 normal(0.0f, -1.0f);
            float penetration = bottom - H;
            resolveWallCollision(body, normal, penetration);
        }
    }
}

// normalは壁のほう線を使っているので、物体はBだと考える
void resolveWallCollision(Body& body, const Vec2& normal, const float penetration) {
    // とりあえず並進速度のみ
    float vn = dot(body.vel, normal);

    // 近づいている場合は力積作用
    if (vn < 0) {
        // Jn = -m v_x (1 + e), 壁を質量無限のobjにすれば物体同士と同じようにできる。
        float normalImpulseScalar = -(1 + WALLRESTITUTION) * vn * body.mass;
        Vec2 Jn = normal * normalImpulseScalar;
        // applyImpulse, 物体はBと考えられるので、+Jn
        body.vel += Jn / body.mass;
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

            resolveCollision(a.body, b.body, *collision);
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
        return CollisionInfo{penetration, normalAB, contactPoint};
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

            // contactPoint取得(現状は並進のみなので、回転を実装する際に実装)
            const auto collision = generateCollisionInfo(a, b, *satCollision);

            if (!collision) continue;

            // resolve
            resolveCollision(a.body, b.body, *collision);
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
            CollisionInfo collision{ (*satCollision).penetration, (*satCollision).normalAB, contactPoint };

            // resolve
            resolveCollision(a.body, b.body, collision);
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


/* Polygon-Polygon, ContactPoint */
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

    /* ContactPoint決定 */
    if (contactPoints->size() == 1) {
        contactPoint = (*contactPoints)[0];
    }
    else if (contactPoints->size() == 2) {
        // 二つある場合はとりあえず、中点(本当は2点に力積与えるのが正しい。)
        contactPoint = ((*contactPoints)[0] + (*contactPoints)[1]) * 0.5f;
    }

    return CollisionInfo{ satCollision.penetration, satCollision.normalAB, contactPoint };
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

void resolveCollision(Body& a, Body& b, const CollisionInfo& collision) {
    Vec2 relativeVel = b.vel - a.vel; // とりあえず並進速度のみ
    float vn = dot(relativeVel, collision.normalAB);

    // 近づいている場合は力積作用
    if (vn < 0) {
        float effectiveMassNormal = a.invMass + b.invMass;
        float normalImpulseScalar = -(1 + OBJRESTITUTION) * vn / effectiveMassNormal;
        Vec2 Jn = collision.normalAB * normalImpulseScalar;
        applyImpulse(a, b, Jn);
    }


    // めり込み補正(質量の逆比で移動)
    float invMassSum = a.invMass + b.invMass;
    a.pos -= collision.normalAB * collision.penetration * a.invMass / invMassSum;
    b.pos += collision.normalAB * collision.penetration * b.invMass / invMassSum;
}

void applyImpulse(Body& a, Body& b, Vec2& Impulse) {
    // AB方向の法線で考えているので、
    // Aには -Impulse , Bには Impulse
    a.vel -= Impulse * a.invMass;
    b.vel += Impulse * b.invMass;
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
    std::uniform_real_distribution<float> angle(0.0f, 2.0f * PI);

    
    // circle
    for (Circle& circle : circles) {
        circle.radius = radius(gen);
        // 面積に比例
        circle.body.mass = minMass + (maxMass - minMass) * ((circle.radius * circle.radius) / (maxRadius * maxRadius));
        circle.body.invMass = 1.0f / circle.body.mass;

        circle.body.pos = Vec2(posX(gen), posY(gen));

        circle.body.vel = Vec2(velocity(gen), velocity(gen));
        
    }

    // rectangle
    for (int i = 0; i < rectangleNum; ++i) {
        Polygon rectangle;
        float w = length(gen);
        float h = length(gen);
        // 面積に比例
        rectangle.body.mass = minMass + (maxMass - minMass) * ((w * h) / (maxLength * maxLength));
        rectangle.body.invMass = 1.0f / rectangle.body.mass;

        rectangle.body.pos = Vec2(posX(gen), posY(gen));

        rectangle.body.vel = Vec2(velocity(gen), velocity(gen));

        // 頂点(反時計回り)
        rectangle.localVertices.resize(4);
        rectangle.localVertices[0] = Vec2(-w / 2.0f, -h / 2.0f);
        rectangle.localVertices[1] = Vec2(-w / 2.0f, h / 2.0f);
        rectangle.localVertices[2] = Vec2(w / 2.0f, h / 2.0f);
        rectangle.localVertices[3] = Vec2(w / 2.0f, -h / 2.0f);

        rectangle.body.angle = angle(gen);

        // 緑
        rectangle.rgb.g = 1.0f;

        polygons.push_back(rectangle);
    }

    // triangle
    for (int i = 0; i < triangleNum; ++i) {
        Polygon triangle;
        float w = length(gen);
        float h = length(gen);
        // 面積に比例
        triangle.body.mass = minMass + (maxMass - minMass) * ((w * h) / (maxLength * maxLength));
        triangle.body.invMass = 1.0f / triangle.body.mass;

        triangle.body.pos = Vec2(posX(gen), posY(gen));

        triangle.body.vel = Vec2(velocity(gen), velocity(gen));

        // 頂点(反時計回り)
        triangle.localVertices.resize(3);
        triangle.localVertices[0] = Vec2(0.0f, -h * 2.0f / 3.0f);
        triangle.localVertices[1] = Vec2(-w / 2.0f, h / 3.0f);
        triangle.localVertices[2] = Vec2(w / 2.0f, h / 3.0f);

        triangle.body.angle = angle(gen);

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
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // 赤
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

std::vector<Vec2> getVerticesPos(const Polygon& polygon) {
    std::vector<Vec2> worldVertices;
    for (const Vec2& localVertex : polygon.localVertices) {
        Vec2 rotated = rotate(localVertex, polygon.body.angle);
        worldVertices.push_back(polygon.body.pos + rotated);
    }
    return worldVertices;
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