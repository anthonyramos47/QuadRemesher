cmake_minimum_required(VERSION 3.16)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
project(FieldMesh)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(USE_POLYSCOPE "Use Polyscope for visualization" OFF)

# Fetch dependencies
include(FetchContent)

# libigl
message(STATUS "Fetching libigl...")
set(LIBIGL_GLFW ON CACHE BOOL "" FORCE)
set(LIBIGL_OPENGL ON CACHE BOOL "" FORCE)
set(LIBIGL_IMGUI ON CACHE BOOL "" FORCE)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
)
FetchContent_MakeAvailable(libigl)

# Polyscope
if(USE_POLYSCOPE)
    message(STATUS "Fetching Polyscope...")
    # Use master branch for latest ImGui compatibility
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG master
    )
    # Polyscope uses its own ImGui, so we need to handle the conflict
    set(POLYSCOPE_BACKEND_OPENGL3_GLFW ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(polyscope)
endif()

# Directional
message(STATUS "Fetching Directional...")
FetchContent_Declare(
    directional
    GIT_REPOSITORY https://github.com/avaxman/Directional.git
    GIT_TAG master
)
FetchContent_MakeAvailable(directional)
set(DIRECTIONAL_INCLUDE ${directional_SOURCE_DIR}/include)
set(SADDLEPOINT_INCLUDE ${directional_SOURCE_DIR}/external/SaddlePoint/include)

# CGAL
find_package(CGAL REQUIRED COMPONENTS Core)

# Executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${DIRECTIONAL_INCLUDE}
    ${SADDLEPOINT_INCLUDE}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    igl::core
    igl::opengl
    igl::glfw
    igl::imgui
    CGAL::CGAL
    CGAL::CGAL_Core
)

if(USE_POLYSCOPE)
    target_link_libraries(${PROJECT_NAME} PRIVATE polyscope)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_POLYSCOPE)
endif()

# Copy data folder if exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/data)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

message(STATUS "")
message(STATUS "===========================================")
message(STATUS "  Configuration Summary")
message(STATUS "===========================================")
message(STATUS "  Polyscope:    ${USE_POLYSCOPE}")
message(STATUS "  libigl:       ON")
message(STATUS "  Directional:  ON")
message(STATUS "===========================================")
