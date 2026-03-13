cmake_minimum_required(VERSION 3.21)
project(CrowdSim VERSION 1.0 LANGUAGES CXX)

# ---- C++20 ----
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()
message(STATUS "[CrowdSim] Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
# Fetch WickedEngine
# ============================================================
include(FetchContent)

FetchContent_Declare(
    WickedEngine
    GIT_REPOSITORY https://github.com/turanszkij/WickedEngine.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# Disable editor / tests to keep build fast
set(WICKEDENGINE_BUILD_EDITOR OFF CACHE BOOL "" FORCE)
set(WICKEDENGINE_BUILD_TESTS  OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(WickedEngine)

# ============================================================
# Pick the correct WickedEngine target name
# (older releases export WickedEngine_Windows on Windows)
# ============================================================
if(TARGET WickedEngine_Windows)
    set(WE_TARGET WickedEngine_Windows)
elseif(TARGET WickedEngine)
    set(WE_TARGET WickedEngine)
else()
    message(FATAL_ERROR "Could not find a WickedEngine CMake target. "
        "Check your WickedEngine version or target name.")
endif()
message(STATUS "[CrowdSim] Using WickedEngine target: ${WE_TARGET}")

# ============================================================
# CrowdSim executable
# ============================================================
add_executable(CrowdSim WIN32
    src/main.cpp
    src/CrowdSystem.cpp
    src/CarSystem.cpp
    src/TrafficLightSystem.cpp
)

target_include_directories(CrowdSim PRIVATE
    "${wickedengine_SOURCE_DIR}/WickedEngine"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(CrowdSim PRIVATE ${WE_TARGET})

# ---- MSVC compile flags ----
if(MSVC)
    target_compile_options(CrowdSim PRIVATE
        /O2          # Maximise speed
        /Ob2         # Aggressive inlining
        /Oi          # Intrinsic functions
        /Ot          # Favour fast code
        /arch:AVX2   # AVX2 SIMD
        /MP          # Parallel compilation
        /W3
        /wd4100      # Unreferenced formal parameter
        /wd4201      # Nameless struct/union
        /wd4127      # Conditional expression is constant
    )
    target_compile_definitions(CrowdSim PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        _UNICODE
        UNICODE
        _CRT_SECURE_NO_WARNINGS
    )
endif()

# GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(CrowdSim PRIVATE -O3 -march=native)
endif()

# ============================================================
# Post-build: copy shaders and content to output directory
# ============================================================
set(WE_SHADER_DIR "${wickedengine_SOURCE_DIR}/WickedEngine/shaders")
set(WE_CONTENT_DIR "${wickedengine_SOURCE_DIR}/Content")

add_custom_command(TARGET CrowdSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:CrowdSim>/shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${WE_SHADER_DIR}"
        "$<TARGET_FILE_DIR:CrowdSim>/shaders"
    COMMENT "Copying WickedEngine shaders..."
    VERBATIM
)

if(EXISTS "${WE_CONTENT_DIR}")
    add_custom_command(TARGET CrowdSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${WE_CONTENT_DIR}"
            "$<TARGET_FILE_DIR:CrowdSim>/Content"
        COMMENT "Copying WickedEngine content..."
        VERBATIM
    )
endif()

# If WickedEngine is a shared library, copy the DLL
get_target_property(_WE_TYPE ${WE_TARGET} TYPE)
if(_WE_TYPE STREQUAL "SHARED_LIBRARY")
    add_custom_command(TARGET CrowdSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${WE_TARGET}>
            $<TARGET_FILE_DIR:CrowdSim>
        COMMENT "Copying WickedEngine DLL..."
        VERBATIM
    )
endif()

# Copy custom models (e.g. drzewo.wiscene placed in build/models/ by the user)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/models/drzewo.wiscene")
    add_custom_command(TARGET CrowdSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:CrowdSim>/models"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/build/models/drzewo.wiscene"
            "$<TARGET_FILE_DIR:CrowdSim>/models/drzewo.wiscene"
        COMMENT "Copying drzewo.wiscene..."
        VERBATIM
    )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/models/dom.wiscene")
    add_custom_command(TARGET CrowdSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/build/models/dom.wiscene"
            "$<TARGET_FILE_DIR:CrowdSim>/models/dom.wiscene"
        COMMENT "Copying dom.wiscene..."
        VERBATIM
    )
endif()

set(WE_DXCOMPILER "${wickedengine_SOURCE_DIR}/WickedEngine/dxcompiler.dll")
if(EXISTS "${WE_DXCOMPILER}")
    add_custom_command(TARGET CrowdSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${WE_DXCOMPILER}"
            "$<TARGET_FILE_DIR:CrowdSim>/dxcompiler.dll"
        COMMENT "Copying dxcompiler.dll..."
        VERBATIM
    )
endif()
