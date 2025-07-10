cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(/usr/local/include ./include ${EIGEN3_INCLUDE_DIR})

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES})

message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

# 包含目录（优先使用 target 包含）
target_include_directories(Rasterizer PRIVATE
    ./include
    /opt/homebrew/include
)


# target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)