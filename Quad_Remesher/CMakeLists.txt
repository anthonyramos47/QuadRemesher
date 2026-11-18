cmake_minimum_required(VERSION 3.16)

# libigl v2.5.0 vendors third-party projects (glad, CoMISo, gmm) that still
# declare cmake_minimum_required(VERSION <3.5). CMake >= 4.0 removed support
# for those, so give every FetchContent sub-project a floor of 3.5.
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version for vendored deps")
endif()

# Portable-binary settings; must be set before project().
# Windows: link the MSVC runtime statically so no VC++ redistributable is
# needed. CMP0091 has to be NEW in the fetched sub-projects too, or they would
# keep /MD and fail to link against our /MT objects.
set(CMAKE_POLICY_DEFAULT_CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
# macOS: run on releases older than the build machine.
if(NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "Minimum macOS version")
endif()

project(QuadRemesherMIQ)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags. -march=native makes the binary crash with "illegal
# instruction" on CPUs older than the build machine, so it is opt-in.
option(QUADREMESHER_NATIVE "Optimize for this machine's CPU (binary is not portable)" OFF)
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    if(QUADREMESHER_NATIVE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
    endif()
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

# CoMISo needs BLAS. On Linux prefer the static libblas.a so the shipped binary
# does not depend on the target machine having libblas installed. (macOS uses
# the system Accelerate framework; Windows uses CoMISo's bundled OpenBLAS DLL,
# which is installed next to the executable below.)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT DEFINED BLA_STATIC)
    set(BLA_STATIC ON)
endif()

FetchContent_MakeAvailable(libigl)

# CoMISo's bundled GMM uses deprecated 'register' keyword (removed in C++17)
# and deprecated sprintf. Suppress these as errors.
if(TARGET CoMISo AND NOT MSVC)
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
    # On Linux/macOS OpenMesh always builds a shared OpenMeshCore as well; link
    # the static twin so the binary does not need libOpenMeshCore.so at runtime.
    if(TARGET OpenMeshCoreStatic)
        target_link_libraries(qex PUBLIC OpenMeshCoreStatic)
    else()
        target_link_libraries(qex PUBLIC OpenMeshCore)
    endif()

    # QEx's exact predicates are only correct when the compiler does not emit
    # x87 extended-precision code. SSE2 is the default on x86-64, but upstream
    # forces it explicitly, so do the same.
    # (MSVC targeting x64 always uses SSE2, so it needs nothing here.)
    if(NOT MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[36]86")
        target_compile_options(qex PRIVATE -msse2 -mfpmath=sse)
    endif()
    target_compile_definitions(qex PUBLIC HAVE_LIBQEX)
    # Upstream 2013 code, compiled against a far newer OpenMesh: quiet its warnings.
    if(MSVC)
        target_compile_options(qex PRIVATE /w)
        # OpenMesh's headers refuse to compile on MSVC without M_PI etc.
        target_compile_definitions(qex PUBLIC _USE_MATH_DEFINES NOMINMAX)
    else()
        target_compile_options(qex PRIVATE -w)
    endif()
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
if(MSVC)
    target_compile_options(quadRemesher PRIVATE /bigobj)
    target_compile_definitions(quadRemesher PRIVATE _USE_MATH_DEFINES NOMINMAX)
else()
    target_compile_options(quadRemesher PRIVATE -Wno-register -Wno-deprecated-declarations)
endif()

# Linux: bake in libstdc++/libgcc so the binary does not depend on the build
# machine's compiler version (glibc, libGL and X11 still come from the system).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(quadRemesher PRIVATE -static-libstdc++ -static-libgcc)
    target_link_options(principalDirections PRIVATE -static-libstdc++ -static-libgcc)
endif()

# `cmake --install build --component app --prefix <dir>` produces the release
# bundle. The component keeps out the headers/libs the dependencies install.
install(TARGETS quadRemesher principalDirections RUNTIME DESTINATION bin COMPONENT app)
install(FILES README.md
              ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE
              ${CMAKE_CURRENT_SOURCE_DIR}/../THIRD_PARTY_NOTICES.md
        DESTINATION . COMPONENT app)
if(WIN32)
    # CoMISo links OpenBLAS through an import library; ship its DLLs.
    FetchContent_GetProperties(comiso)
    file(GLOB COMISO_BLAS_DLLS "${comiso_SOURCE_DIR}/ext/OpenBLAS-v0.2.14-Win64-int64/bin/*.dll")
    install(FILES ${COMISO_BLAS_DLLS} DESTINATION bin COMPONENT app)
endif()
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "QuadRemesher")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_COMPONENTS_ALL app)
set(CPACK_ARCHIVE_COMPONENT_INSTALL OFF)
include(CPack)

message(STATUS "")
message(STATUS "==========================================")
message(STATUS "QuadRemesher with MIQ (CoMISo)")
message(STATUS "==========================================")
