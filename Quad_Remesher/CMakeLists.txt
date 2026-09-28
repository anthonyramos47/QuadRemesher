cmake_minimum_required(VERSION 3.16)

# libigl v2.5.0 vendors third-party projects (glad, CoMISo, gmm) that still
# declare cmake_minimum_required(VERSION <3.5). CMake >= 4.0 removed support
# for those, so give every FetchContent sub-project a floor of 3.5.
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version for vendored deps")
endif()

project(QuadRemesherMIQ)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags
if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Apple Silicon (ARM64)")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

# Use FetchContent for dependencies
include(FetchContent)

# Fetch libigl (this will also bring in Eigen and other deps)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
)

# Configure libigl options
option(LIBIGL_COPYLEFT_COMISO "Build with CoMISo" ON)
option(LIBIGL_OPENGL "Build with OpenGL" ON)
option(LIBIGL_GLFW "Build with GLFW" ON)

FetchContent_MakeAvailable(libigl)

# CoMISo's bundled GMM uses deprecated 'register' keyword (removed in C++17)
# and deprecated sprintf. Suppress these as errors.
if(TARGET CoMISo)
    target_compile_options(CoMISo PRIVATE -Wno-register -Wno-deprecated-declarations)
endif()

# ---------------------------------------------------------------------------
# libQEx (robust quad extraction, Ebke et al. 2013) + its OpenMesh dependency.
#
# NOTE ON LICENSING: libQEx is GPLv3. Linking it in makes the resulting
# quadRemesher binary GPLv3 as well. Build with -DUSE_LIBQEX=OFF to opt out;
# the built-in grid extractor is then the only extractor available.
# ---------------------------------------------------------------------------
option(USE_LIBQEX "Build with libQEx quad extraction (GPLv3)" ON)

if(USE_LIBQEX)
    # OpenMesh: build only the static core, no apps/docs/unit tests.
    set(BUILD_APPS OFF CACHE BOOL "" FORCE)
    set(OPENMESH_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(OPENMESH_DOCS OFF CACHE BOOL "" FORCE)
    set(DISABLE_QMAKE_BUILD ON CACHE BOOL "" FORCE)

    FetchContent_Declare(
        openmesh
        GIT_REPOSITORY https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
        GIT_TAG OpenMesh-11.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(openmesh)

    # libQEx ships a 2013-era CMakeLists (cmake_minimum_required 2.6, relies on
    # CMAKE_SOURCE_DIR, calls find_package(OpenMesh), builds demos and doxygen).
    # Download the sources but do NOT add its CMakeLists: SOURCE_SUBDIR points at
    # a directory that contains no CMakeLists.txt, which suppresses the
    # add_subdirectory() that MakeAvailable would otherwise do. We then define
    # our own target against the OpenMesh built above.
    FetchContent_Declare(
        libqex
        GIT_REPOSITORY https://github.com/hcebke/libQEx.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR src
    )
    FetchContent_MakeAvailable(libqex)

    add_library(qex STATIC
        ${libqex_SOURCE_DIR}/interfaces/c/qex.cc
        ${libqex_SOURCE_DIR}/src/MeshExtractor.cc
        ${libqex_SOURCE_DIR}/src/predicates.c
    )
    target_include_directories(qex PUBLIC
        ${libqex_SOURCE_DIR}/interfaces/c
        ${libqex_SOURCE_DIR}/src
    )
    target_link_libraries(qex PUBLIC OpenMeshCore)

    # QEx's exact predicates are only correct when the compiler does not emit
    # x87 extended-precision code. SSE2 is the default on x86-64, but upstream
    # forces it explicitly, so do the same.
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[36]86")
        target_compile_options(qex PRIVATE -msse2 -mfpmath=sse)
    endif()
    target_compile_definitions(qex PUBLIC HAVE_LIBQEX)
    # Upstream 2013 code, compiled against a far newer OpenMesh: quiet its warnings.
    target_compile_options(qex PRIVATE -w)
endif()

# Main executable with MIQ support
add_executable(quadRemesher quadRemesher.cpp)
target_link_libraries(quadRemesher
    igl::core
    igl::opengl
    igl::glfw
    igl_copyleft::comiso
)

if(USE_LIBQEX)
    target_link_libraries(quadRemesher qex)
endif()

# Generates the per-vertex D1/D2 fields that quadRemesher consumes, from the
# mesh's principal curvature directions.
add_executable(principalDirections principalDirections.cpp)
target_link_libraries(principalDirections igl::core)
# GMM headers from CoMISo leak into our compilation unit via header-only includes
target_compile_options(quadRemesher PRIVATE -Wno-register -Wno-deprecated-declarations)

message(STATUS "")
message(STATUS "==========================================")
message(STATUS "QuadRemesher with MIQ (CoMISo)")
message(STATUS "==========================================")
