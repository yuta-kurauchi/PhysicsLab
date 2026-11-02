#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>


/* 設定 */

// WindowSize
const float W = 800.0f;
const float H = 600.0f;

// WindowMode
const SDL_WindowFlags flags = 0; // 特に機能を持たせる必要ないので0
// 提供可能なすべてのシェーダー形式を含めるように。今回はmac,windowsを想定
const SDL_GPUShaderFormat format_flags = SDL_GPU_SHADERFORMAT_SPIRV |
    SDL_GPU_SHADERFORMAT_DXIL |
    SDL_GPU_SHADERFORMAT_MSL |
    SDL_GPU_SHADERFORMAT_METALLIB;
const bool debug_mode = true;
const char* name = nullptr;


int main(int argc, char* argv[]) {
    std::cout << "PhysicsLab3D has started up\n";


    /* SDL処理 */

    /* 初期化 */
    bool isSDLInitialized = SDL_Init(SDL_INIT_VIDEO);
    // 初期化失敗
    if (!isSDLInitialized) {
        SDL_Log("SDLInit failed: %s", SDL_GetError());
        // 終了処理
        SDL_Quit();
        return 1;
    }

    /* Window作成 */
    SDL_Window* window = SDL_CreateWindow("PhysicsLab3D", W, H, flags);
    // window作成失敗
    if (!window) {
        SDL_Log("CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    /* GPU */
    SDL_GPUDevice* device = SDL_CreateGPUDevice(
        format_flags,
        debug_mode,
        name
    );
    // GPU作成失敗
    if (!device) {
        SDL_Log("CreateGPUDevice failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_Log("%s", SDL_GetGPUDeviceDriver(device));

    /* windowとGPUを関連づける */
    if (!SDL_ClaimWindowForGPUDevice(device, window)) {
        SDL_Log("GPUClaimWindow failed: %s", SDL_GetError());
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }


    /* MainLoop */
    bool running = true;
    while (running) {
        /* Event */
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            // ウィンドウを閉じるevent
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN) {
                if (event.key.key == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }
    }

    /* SDL終了処理 */
    SDL_ReleaseWindowFromGPUDevice(device, window);
    SDL_DestroyGPUDevice(device);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "PhyscsiLab3D completed successfully\n";
    return 0;
}