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

# 実行ファイルを作成するための設定
add_executable(PhysicsLab ${SOURCES})