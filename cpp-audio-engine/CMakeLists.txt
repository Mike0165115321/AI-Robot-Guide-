cmake_minimum_required(VERSION 3.16)
project(cpp-audio-engine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(USE_GPU "Enable GPU acceleration for Whisper" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Find required packages
find_package(Threads REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Whisper.cpp (will be added as submodule)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/whisper.cpp/CMakeLists.txt)
    set(WHISPER_STANDALONE OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/whisper.cpp)
    set(WHISPER_FOUND TRUE)
else()
    message(STATUS "whisper.cpp not found. Whisper functionality will be disabled.")
    set(WHISPER_FOUND FALSE)
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/audio_processor.cpp
    src/grpc_server.cpp
)

if(WHISPER_FOUND)
    list(APPEND SOURCES src/whisper_service.cpp)
    add_definitions(-DWHISPER_ENABLED)
endif()

# Main executable
add_executable(audio-engine ${SOURCES})

target_link_libraries(audio-engine
    PRIVATE
    Threads::Threads
)

if(WHISPER_FOUND)
    target_link_libraries(audio-engine PRIVATE whisper)
endif()

# Install
install(TARGETS audio-engine DESTINATION bin)

# Print configuration
message(STATUS "")
message(STATUS "=== C++ Audio Engine Configuration ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "GPU Acceleration: ${USE_GPU}")
message(STATUS "Whisper.cpp: ${WHISPER_FOUND}")
message(STATUS "========================================")
