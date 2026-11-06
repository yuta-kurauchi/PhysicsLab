#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>


/* 構造体 */
struct Vertex {
    float x, y; // Position
    float r, g, b; // Color
};


/* 設定 */

// プログラム正常終了フラグ
bool isSuccessful = true;

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
// 描画設定
bool UsaWireframeMode = false;


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

    /* CreateShaders */
    InitializeAssetLoader();
    SDL_GPUShader* vertexShader = loadShader(device, "PositionColor.vert");
    if (!vertexShader) {
        SDL_Log("Failed to create vertex shader: %s", SDL_GetError());
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_GPUShader* fragmentShader = loadShader(device, "SolidColor.frag");
    if (!fragmentShader) {
        SDL_Log("Failed to create fragment shader: %s", SDL_GetError());
        SDL_ReleaseGPUShader(device, vertexShader);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    /* vertex shader layout */
    SDL_GPUVertexBufferDescription vertexBufferDescription{};
    vertexBufferDescription.slot = 0;
    vertexBufferDescription.pitch = sizeof(Vertex);
    vertexBufferDescription.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vertexBufferDescription.instance_step_rate = 0;

    SDL_GPUVertexAttribute vertexAttribute[2]{};
    vertexAttribute[0].location = 0;
    vertexAttribute[0].buffer_slot = 0;
    vertexAttribute[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    vertexAttribute[0].offset = offsetof(Vertex, x);
    vertexAttribute[1].location = 1;
    vertexAttribute[1].buffer_slot = 0;
    vertexAttribute[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    vertexAttribute[1].offset = offsetof(Vertex, r);

    SDL_GPUVertexInputState vertexInputState{};
    vertexInputState.vertex_buffer_descriptions = &vertexBufferDescription;
    vertexInputState.num_vertex_buffers = 1;
    vertexInputState.vertex_attributes = vertexAttribute;
    vertexInputState.num_vertex_attributes = 2;


    /* Create the pipelines */
    SDL_GPUGraphicsPipelineCreateInfo pipelineCreateInfo{};
    pipelineCreateInfo.target_info.num_color_targets = 1;
    SDL_GPUColorTargetDescription colorTarget{};
    colorTarget.format = SDL_GetGPUSwapchainTextureFormat(device, window);
    pipelineCreateInfo.target_info.color_target_descriptions = &colorTarget;
    pipelineCreateInfo.vertex_input_state = vertexInputState;
    pipelineCreateInfo.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipelineCreateInfo.vertex_shader = vertexShader;
    pipelineCreateInfo.fragment_shader = fragmentShader;

    SDL_GPUGraphicsPipeline* FillPipeline = nullptr;
    pipelineCreateInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    FillPipeline = SDL_CreateGPUGraphicsPipeline(device, &pipelineCreateInfo);
    if (!FillPipeline) {
        SDL_Log("Failed to create fill pipeline: %s", SDL_GetError());
        SDL_ReleaseGPUShader(device, vertexShader);
        SDL_ReleaseGPUShader(device, fragmentShader);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_GPUGraphicsPipeline* LinePipeline = nullptr;
    pipelineCreateInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_LINE;
    LinePipeline = SDL_CreateGPUGraphicsPipeline(device, &pipelineCreateInfo);
    if (!LinePipeline) {
        SDL_Log("Failed to create line pipeline: %s", SDL_GetError());
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseGPUShader(device, vertexShader);
        SDL_ReleaseGPUShader(device, fragmentShader);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    /* Shaderの解放 */
    SDL_ReleaseGPUShader(device, vertexShader);
    SDL_ReleaseGPUShader(device, fragmentShader);


    // 頂点座標の定義
    Vertex vertices[3] = {
        { -0.1f, -0.1f, 1.0f, 0.0f, 0.0f },
        { 0.1f, -0.1f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.1f, 0.0f, 0.0f ,1.0f }
    };

    /* Create the vertex buffer */
    SDL_GPUBufferCreateInfo bufferInfo{
        SDL_GPU_BUFFERUSAGE_VERTEX,
        sizeof(vertices),
        0
    };

    SDL_GPUBuffer* vertexBuffer = nullptr;
    vertexBuffer = SDL_CreateGPUBuffer(device, &bufferInfo);
    if (!vertexBuffer) {
        SDL_Log("Failed to create vertex buffer: %s", SDL_GetError());
        SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }


    /* Create the transfer buffer */
    SDL_GPUTransferBufferCreateInfo transferBufferInfo{
        SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
        sizeof(vertices),
        0
    };
    SDL_GPUTransferBuffer* transferBuffer = nullptr;
    transferBuffer = SDL_CreateGPUTransferBuffer(device, &transferBufferInfo);
    if (!transferBuffer) {
        SDL_Log("Failed to create transfer buffer: %s", SDL_GetError());
        SDL_ReleaseGPUBuffer(device, vertexBuffer);
        SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Acquire transfer buffer memory
    Vertex* transferData = static_cast<Vertex*>(SDL_MapGPUTransferBuffer(
        device,
        transferBuffer,
        false
    ));
    if (!transferData) {
        SDL_Log("Failed to map transfer buffer: %s", SDL_GetError());
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, vertexBuffer);
        SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // copy vertex to transferBuffer
    SDL_memcpy(transferData, vertices, sizeof(vertices));

    SDL_UnmapGPUTransferBuffer(device, transferBuffer);

    
    /* Upload the trasfer data to the vertex buffer */
    SDL_GPUCommandBuffer* uploadCmdBuf = SDL_AcquireGPUCommandBuffer(device);
    if (!uploadCmdBuf) {
        SDL_Log("Failed to upload command buffer: %s", SDL_GetError());
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, vertexBuffer);
        SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(uploadCmdBuf);

    SDL_GPUTransferBufferLocation source{
        transferBuffer,
        0
    };
    SDL_GPUBufferRegion destination{
        vertexBuffer,
        0,
        sizeof(vertices)
    };
    SDL_UploadToGPUBuffer(
        copyPass,
        &source,
        &destination,
        false
    );

    SDL_EndGPUCopyPass(copyPass);
    if (!SDL_SubmitGPUCommandBuffer(uploadCmdBuf)) {
        SDL_Log("Failed to submit GPU command buffer: %s", SDL_GetError());
        SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
        SDL_ReleaseGPUBuffer(device, vertexBuffer);
        SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
        SDL_ReleaseWindowFromGPUDevice(device, window);
        SDL_DestroyGPUDevice(device);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_ReleaseGPUTransferBuffer(device, transferBuffer);

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
            isSuccessful = false;
            break;
        }

        // 描画すべき場所の取得
        SDL_GPUTexture* swapchainTexture = nullptr;
        if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmdbuf, window, &swapchainTexture, nullptr, nullptr)) {
            SDL_Log("WaitAndAcquireGPUSwapchainTexture failed: %s", SDL_GetError());
            SDL_CancelGPUCommandBuffer(cmdbuf);
            running = false;
            isSuccessful = false;
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

            /* RenderPass */
            SDL_GPURenderPass* renderPass = SDL_BeginGPURenderPass(cmdbuf, &colorTargetInfo, 1, nullptr);
            SDL_BindGPUGraphicsPipeline(renderPass, UsaWireframeMode ? LinePipeline : FillPipeline);
            SDL_GPUBufferBinding vertexBufferBinding{};
            vertexBufferBinding.buffer = vertexBuffer;
            vertexBufferBinding.offset = 0;
            SDL_BindGPUVertexBuffers(renderPass, 0, &vertexBufferBinding, 1);
            SDL_DrawGPUPrimitives(renderPass, 3, 1, 0, 0);
            // 描画終了
            SDL_EndGPURenderPass(renderPass);
        }

        // GPUに送信
        if (!SDL_SubmitGPUCommandBuffer(cmdbuf)) {
            SDL_Log("SubmitGPUCommandBuffer failed: %s", SDL_GetError());
            running = false;
            isSuccessful = false;
            break;
        }
    }

    /* SDL終了処理 */
    SDL_ReleaseGPUBuffer(device, vertexBuffer);
    SDL_ReleaseGPUGraphicsPipeline(device, LinePipeline);
    SDL_ReleaseGPUGraphicsPipeline(device, FillPipeline);
    SDL_ReleaseWindowFromGPUDevice(device, window);
    SDL_DestroyGPUDevice(device);
    SDL_DestroyWindow(window);
    SDL_Quit();

    if (isSuccessful) {
        std::cout << "PhyscsiLab3D completed successfully\n";
        return 0;
    } else {
        return 1;
    }
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
