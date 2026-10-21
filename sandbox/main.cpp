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
#include "physics2d/PhysicsTypes.hpp"
#include "physics2d/World2D.hpp"


/* 構造体 */

enum class SceneType {
    Random,
    SingleBox,
    Stacking
};

// デバッグ描画用のデータを保持するための構造体
// いずれ色も持たせたい
struct DebugLine {
    Vec2 p1;
    Vec2 p2;
};


/* 設定 */

// 初期化設定
constexpr SceneType ACTIVE_SCENE = SceneType::Stacking;
constexpr unsigned int RANDOM_SEED = 12345;
// Obj設定
const int circleNum = 3;
const int rectangleNum = 3;
const int triangleNum = 2;
const int STACKINGNUM = 3;

// WindowSize
const float W = 800;
const float H = 600;
// WindowMode
const SDL_WindowFlags flags = 0; // 特に機能を持たせる必要ないので0

// 定数
const double PI = std::acos(-1.0);

// デバッグ用
std::vector<DebugLine> debugLines;
std::vector<Vec2> debugPoints;


/* プロトタイプ宣言 */

/* Initialize Obj */
void InitializeObj(World2D& world, SceneType scene);
void InitializeRandomScene(World2D& world, unsigned int seed);
void InitializeSingleBoxScene(World2D& world);
void InitializeStackingScene(World2D& world);

/* Render */
void render(SDL_Renderer* renderer, const World2D& world);
void drawCircle(SDL_Renderer* renderer, const Circle& circle);
void drawPolygon(SDL_Renderer* renderer, const Polygon& polygon);

int main(int argc, char* argv[]) {
    std::cout<< "PhysicsLab has started up\n";
    
    World2D world;

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
    InitializeObj(world, ACTIVE_SCENE);


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
            world.step(PHYSICSDT);
            accumulator -= PHYSICSDT;
        }
        
        /* Render */
        render(renderer, world);
    }

    // 終了処理(windowとSDL)
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "PhysicsLab completed successfully\n";

    return 0;
}


/* Initialize Obj */

void InitializeObj(World2D& world, SceneType scene) {
    world.polygons.clear();
    world.circles.clear();

    switch (scene) {
        case SceneType::Random:
            InitializeRandomScene(world, RANDOM_SEED);
            break;
        
        case SceneType::SingleBox:
            InitializeSingleBoxScene(world);
            break;

        case SceneType::Stacking:
            InitializeStackingScene(world);
            break;
    }
}

void InitializeRandomScene(World2D& world, unsigned int seed) {
    world.circles.reserve(circleNum);
    world.polygons.reserve(rectangleNum + triangleNum);

    // seed = std::random_device{}() で今まで通りランダムにできる。
    std::mt19937 gen(seed);

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
    for (int i = 0; i < circleNum; ++i) {
        Circle circle;
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

        world.circles.push_back(circle);
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

        body.inertia = body.mass * (w * w + h * h) / 12.0f; // I_z = m (w^2 + h^2) / 12
        body.invInertia = 1.0f / body.inertia;

        // 緑
        rectangle.rgb.g = 1.0f;

        world.polygons.push_back(rectangle);
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

        world.polygons.push_back(triangle);
    }
}

void InitializeSingleBoxScene(World2D& world) {
    world.polygons.reserve(1);
    Polygon rectangle;
    Body& body = rectangle.body;
    float w = 80.0f;
    float h = 60.0f;
    // 面積に比例
    body.mass = 0.5f + 2.5f * ((w * h) / 6400.0f);
    body.invMass = 1.0f / body.mass;

    body.pos = Vec2(W / 2.0f, H / 2.0f);

    // 頂点(反時計回り)
    rectangle.localVertices.resize(4);
    rectangle.localVertices[0] = Vec2(-w / 2.0f, -h / 2.0f);
    rectangle.localVertices[1] = Vec2(-w / 2.0f, h / 2.0f);
    rectangle.localVertices[2] = Vec2(w / 2.0f, h / 2.0f);
    rectangle.localVertices[3] = Vec2(w / 2.0f, -h / 2.0f);

    body.inertia = body.mass * (w * w + h * h) / 12.0f; // I_z = m (w^2 + h^2) / 12
    body.invInertia = 1.0f / body.inertia;

    // 緑
    rectangle.rgb.g = 1.0f;

    world.polygons.push_back(rectangle);
}

void InitializeStackingScene(World2D& world) {
    const float delta = 20.0f; // 矩形の間隔
    world.polygons.reserve(STACKINGNUM);
    for (int i = 0; i < STACKINGNUM; ++i) {
        Polygon rectangle;
        Body& body = rectangle.body;
        float w = 80.0f;
        float h = 60.0f;
        // 面積に比例
        body.mass = 0.5f + 2.5f * ((w * h) / 6400.0f);
        body.invMass = 1.0f / body.mass;

        body.pos = Vec2(W / 2.0f, H -  (i + 1) * (h + delta));

        // 頂点(反時計回り)
        rectangle.localVertices.resize(4);
        rectangle.localVertices[0] = Vec2(-w / 2.0f, -h / 2.0f);
        rectangle.localVertices[1] = Vec2(-w / 2.0f, h / 2.0f);
        rectangle.localVertices[2] = Vec2(w / 2.0f, h / 2.0f);
        rectangle.localVertices[3] = Vec2(w / 2.0f, -h / 2.0f);

        body.inertia = body.mass * (w * w + h * h) / 12.0f; // I_z = m (w^2 + h^2) / 12
        body.invInertia = 1.0f / body.inertia;

        // 緑
        rectangle.rgb.g = 1.0f;

        world.polygons.push_back(rectangle);
    }
}


/* Render */
void render(SDL_Renderer* renderer, const World2D& world) {
    // ScreenClear
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 黒
    SDL_RenderClear(renderer);

    // ObjDraw
    // rect(x, y)はrectの左上の頂点座標
    for (const Polygon& polygon : world.polygons) {
        drawPolygon(renderer, polygon);
    }
    for (const Circle& circle : world.circles) {
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