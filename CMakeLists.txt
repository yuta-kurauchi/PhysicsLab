# 最小バージョン要求の設定
cmake_minimum_required(VERSION 3.15)

# プロジェクト名と使用言語の指定
project(PhysicsLab CXX)

# ソースファイルの設定
# set(Y X), XをYに代入
set(SOURCES
    sandbox/main.cpp
)

# 使用するC++の規格の設定(C++17)
# 実行ファイルの設定の前に書かないと反映されない。
set(CMAKE_CXX_STANDARD 17)
# 規格を必ず守らせる。無理だったらエラーになる。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 外部ライブラリ(SDL3)の取得場所の設定
# バージョン固定(3.4.16)
FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.4.16
)

# 取得
FetchContent_MakeAvailable(SDL3)

# 実行ファイルを作成するための設定
add_executable(PhysicsLab ${SOURCES})

# 依存関係(PhysicsLab -> SDL3)をCmakeに伝える
target_link_libraries(PhysicsLab PRIVATE SDL3::SDL3)