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
// GPUDevice
const bool debug_mode = true;
const char* name = nullptr;
// BuildDirの絶対パス
const char* BasePath = nullptr;


/* プロトタイプ宣言 */
SDL_GPUShader* loadShader(
    SDL_GPUDevice* device,
    const char* filename
);
void InitializeAssetLoader();


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

    InitializeAssetLoader();
    std::cout << "BasePath = " << BasePath << "\n";
    SDL_GPUShader* vertexShader = loadShader(device, "RawTriangle.vert");
    if (!vertexShader) {
        SDL_Log("Failed to create vertex shader!");
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_GPUShader* fragmentShader = loadShader(device, "SolidColor.frag");
    if (!fragmentShader) {
        SDL_Log("Failed to create fragment shader!");
        SDL_ReleaseGPUShader(device, vertexShader);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    std::cout << "Success to create shader\n";

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
    SDL_ReleaseGPUShader(device, vertexShader);
    SDL_ReleaseGPUShader(device, fragmentShader);
    SDL_ReleaseWindowFromGPUDevice(device, window);
    SDL_DestroyGPUDevice(device);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "PhyscsiLab3D completed successfully\n";
    return 0;
}


void InitializeAssetLoader() {
    BasePath = SDL_GetBasePath();
}


SDL_GPUShader* loadShader(
    SDL_GPUDevice* device,
    const char* shaderFilename
) {
    /* ファイル名から、stageを特定 */
    SDL_GPUShaderStage stage;
    if (SDL_strstr(shaderFilename, ".vert")) {
        stage = SDL_GPU_SHADERSTAGE_VERTEX;
    } else if (SDL_strstr(shaderFilename, ".frag")) {
        stage = SDL_GPU_SHADERSTAGE_FRAGMENT;
    } else {
        SDL_Log("Invalid shader stage!");
        return nullptr;
    }

    /* shaderFileへのfullPath作成 */
    char fullPath[256];
    SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(device);
    SDL_GPUShaderFormat format = SDL_GPU_SHADERFORMAT_INVALID;
    const char* entrypoint;

    // backendFormatsに含まれているshaderFormatから作成
    if (backendFormats & SDL_GPU_SHADERFORMAT_MSL) {
        SDL_snprintf(fullPath, sizeof(fullPath), "%sshaders/%s.msl", BasePath, shaderFilename);
        format = SDL_GPU_SHADERFORMAT_MSL;
        entrypoint = "main0";
    } else {
        SDL_Log("Unrecognized backend shader format!");
        return nullptr;
    }

    /* file読み込み */
    size_t codeSize;
    void* code = SDL_LoadFile(fullPath, &codeSize);
    if (!code) {
        SDL_Log("Failed to load shader from disk! %s", fullPath);
        return nullptr;
    }
    std::cout << "Success to load shader from disk! " << fullPath << "\n";
    std::cout << "FileSize is " << codeSize << "\n";

    /* Shader作成 */
    SDL_GPUShaderCreateInfo shaderInfo = {
        codeSize,
        static_cast<Uint8*>(code),
        entrypoint,
        format,
        stage
    };
    SDL_GPUShader* shader = SDL_CreateGPUShader(device, &shaderInfo);
    if (!shader) {
        SDL_Log("Failed to create shader!");
        SDL_free(code);
        return nullptr;
    }


    SDL_free(code);
    return shader;
}
