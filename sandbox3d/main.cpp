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


        /* Draw */
        // cmdbuf: gpuへの命令を記録するための箱
        SDL_GPUCommandBuffer* cmdbuf = SDL_AcquireGPUCommandBuffer(device);
        if (!cmdbuf) {
            SDL_Log("AcquireGPUCommandBuffer failed: %s", SDL_GetError());
            running = false;
            break;
        }

        // 描画すべき場所の取得
        SDL_GPUTexture* swapchainTexture = nullptr;
        if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmdbuf, window, &swapchainTexture, nullptr, nullptr)) {
            SDL_Log("WaitAndAcquireGPUSwapchainTexture failed: %s", SDL_GetError());
            SDL_CancelGPUCommandBuffer(cmdbuf);
            running = false;
            break;
        }

        // 描画すべき場所が取得できた場合のみ描画処理を追加
        if (swapchainTexture != nullptr) {
            // 描画設定
            SDL_GPUColorTargetInfo colorTargetInfo{};
            colorTargetInfo.texture = swapchainTexture;
            colorTargetInfo.clear_color = SDL_FColor{ 0.08f, 0.10f, 0.14f, 1.0f };
            colorTargetInfo.load_op = SDL_GPU_LOADOP_CLEAR; // 最初にClearScreen
            colorTargetInfo.store_op = SDL_GPU_STOREOP_STORE;

            // 描画開始
            SDL_GPURenderPass* renderPass = SDL_BeginGPURenderPass(cmdbuf, &colorTargetInfo, 1, nullptr);
            // ここに追加の描画処理を書く
            // 描画終了
            SDL_EndGPURenderPass(renderPass);
        }

        // GPUに送信
        if (!SDL_SubmitGPUCommandBuffer(cmdbuf)) {
            SDL_Log("SubmitGPUCommandBuffer failed: %s", SDL_GetError());
            running = false;
            break;
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