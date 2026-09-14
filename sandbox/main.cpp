#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

// WindowSize
const int W = 800;
const int H = 600;
// WindowMode
const SDL_WindowFlags flags = 0; // 特に機能を持たせる必要ないので0

int main() {
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

    // Loopフラグ
    bool running = true;

    /* Renderer */
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

    /* Debug ScreenSize, PixelSize */
    // // OS上での設定サイズ
    // int windowW, windowH;
    // SDL_GetWindowSize(window, &windowW, &windowH);

    // // 実際の描画ピクセル数
    // int pixelW, pixelH;
    // SDL_GetWindowSizeInPixels(window, &pixelW, &pixelH);

    // // 実際に描画されているサイズ
    // int renderW, renderH;
    // SDL_GetRenderOutputSize(renderer, &renderW, &renderH);

    // std::cout << "Window : " << windowW << " X " << windowH << "\n";
    // std::cout << "Pixels : " << pixelW << " X " << pixelH << "\n";
    // std::cout << "Render : " << renderW << " X " << renderH << "\n";

    /* MainLoop */
    while (running) {

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

        /* Render */

        // ScreenClear
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 黒
        SDL_RenderClear(renderer);

        // ObjDraw
        // rect(x, y)はrectの左上の頂点座標
        float rectW = 50.0f;
        float rectH = 30.0f;
        SDL_FRect rect{W / 2.0f - rectW / 2.0f, H / 2.0f - rectH / 2.0f, rectW, rectH};
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); // 白
        SDL_RenderFillRect(renderer, &rect);

        // DebugDraw

        // Display
        SDL_RenderPresent(renderer);
    }



    // 終了処理(windowとSDL)
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "PhysicsLab completed successfully\n";

    return 0;
}