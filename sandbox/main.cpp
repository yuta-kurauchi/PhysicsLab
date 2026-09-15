#include <iostream>
#include <vector>
#include <cmath>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include "math/Vec2.hpp"



/* Obj用の構造体 */

struct Body {
    Vec2 pos;
    Vec2 vel;
    Vec2 acc;
    float mass;
    float getInvMass() {
        return 1 / mass;
    }
};

struct Circle {
    Body body;
    float radius;
};

struct Polygon {
    Body body;
    float w;
    float h;
    int vertexNum;  
};


/* Obj用 */
int circleNum = 3;
std::vector<Circle> circles(circleNum);
int rectangleNum = 0;
std::vector<Polygon> rectangles(rectangleNum);

/* プロトタイプ宣言 */

/* UpdatePhysics */
void updatePhysics(float dt);

/* 加速度・速度・位置更新 */
void updateAcc(Body& body);
void updateVel(Body& body, float dt);
void updatePos(Body& body, float dt);

/* Collision */
void circleWallCollision(Circle& cirlce);

/* Initialize Obj */
void InitializeObj();

/* Render */
void render(SDL_Renderer* renderer);
void drawCircle(SDL_Renderer* renderer, const Circle& circle);
void drawRectangle(SDL_Renderer* renderer, const Polygon& rectangle);

/* 定数 */
const float GRAVITY = 98.0f;
const float WALLRESTITUTION = 0.75f;

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
    for (Polygon& rectangle : rectangles) {
        updateAcc(rectangle.body);
        updateVel(rectangle.body, dt);
        updatePos(rectangle.body, dt);
    }

    /* WallCollision */
    for (Circle& circle : circles) {
        circleWallCollision(circle);
    }
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


/* 壁との衝突処理 */
void circleWallCollision(Circle& circle) {
    Body& body = circle.body;
    // LeftWall
    if (body.pos.x - circle.radius < 0.0f) {
        // めり込み解消
        body.pos.x = circle.radius;
        body.vel.x *= -WALLRESTITUTION; // とりま反転のみ
    }
    // RightWall
    if (body.pos.x + circle.radius > W) {
        body.pos.x = W - circle.radius;
        body.vel.x *= -WALLRESTITUTION;
    }
    // TopWall
    if (body.pos.y - circle.radius < 0.0f) {
        body.pos.y = circle.radius;
        body.vel.y *= -WALLRESTITUTION;
    }
    // BottomWall
    if (body.pos.y + circle.radius > H) {
        body.pos.y = H - circle.radius;
        body.vel.y *= -WALLRESTITUTION;
    }
}





/* Initialize Obj */
void InitializeObj() {
    int i = 1;
    for (Circle& circle : circles) {
        // とりあえず均等に配置
        circle.body.pos = Vec2(W * i / (circles.size() + 1), 50.0f);
        circle.radius = 30.0f; // とりま固定値
        i++;
    }
    i = 1;
    for (Polygon& rectangle : rectangles) {
        rectangle.w = 50.0f;
        rectangle.h = 30.0f;
        rectangle.body.pos = Vec2(W * i / (rectangles.size() + 1), 0.0f);
        i++;
    }
}


/* Render */
void render(SDL_Renderer* renderer) {
    // ScreenClear
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 黒
    SDL_RenderClear(renderer);

    // ObjDraw
    // rect(x, y)はrectの左上の頂点座標
    for (Polygon& rectangle : rectangles) {
        drawRectangle(renderer, rectangle);
    }
    for (Circle& circle : circles) {
        drawCircle(renderer, circle);
    }

    // DebugDraw

    // Display
    SDL_RenderPresent(renderer);
}


// 円の描画用
void drawCircle(SDL_Renderer* renderer, const Circle& circle) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // 赤
    const int SEGMENTS = 64;
    const double PI = std::acos(-1.0);

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
}

// 長方形の描画
void drawRectangle(SDL_Renderer* renderer,const Polygon& rectangle) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // 白
    // 重心から左上の頂点座標をレンダー用の座標として作成
    Vec2 renderPos = rectangle.body.pos - Vec2(rectangle.w / 2.0f, rectangle.h / 2.0f); 
    SDL_FRect rect{renderPos.x, renderPos.y, rectangle.w, rectangle.h};
    SDL_RenderFillRect(renderer, &rect);
}