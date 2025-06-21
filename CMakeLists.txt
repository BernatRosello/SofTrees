cmake_minimum_required(VERSION 3.5)
project(SofTrees)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(SofTrees ${SRC_FILES})

target_include_directories(SofTrees PRIVATE ${PROJECT_SOURCE_DIR}/include)

include(FetchContent)

# ================ 
# ==== RAYLIB ==== 
# ================ 
set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET)
if (NOT raylib_FOUND)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  endif()
endif()

# ==================== 
# ==== RAYLIB_CPP ==== 
# ==================== 
message(STATUS "Getting raylib-cpp from Github")
FetchContent_Declare(
  raylib_cpp
  GIT_REPOSITORY https://github.com/RobLoach/raylib-cpp.git
  GIT_TAG v5.5.0
)
FetchContent_MakeAvailable(raylib_cpp)

# ================ 
# ==== BOX 2D ==== 
# ================ 
FetchContent_Declare(
	box2d
	GIT_REPOSITORY https://github.com/erincatto/box2d.git
	GIT_TAG v3.1.1
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(box2d)

# ================= 
# ==== RAYGUI ==== 
# ================= 
# Download raygui.h directly into the raylib source folder
set(RAYGUI_HEADER_URL "https://raw.githubusercontent.com/raysan5/raygui/master/src/raygui.h")
set(RAYGUI_HEADER_DEST "${raylib_SOURCE_DIR}/src/raygui.h")

# Add a custom command to download the file if it doesn't exist
if (NOT EXISTS ${RAYGUI_HEADER_DEST})
    message(STATUS "Downloading raygui.h into raylib source directory")
    file(DOWNLOAD ${RAYGUI_HEADER_URL} ${RAYGUI_HEADER_DEST} SHOW_PROGRESS)
endif()

# Add raylib's src folder to the include directories (so raygui.h is found)
target_include_directories(SofTrees PRIVATE ${raylib_SOURCE_DIR}/src)

# Link all libraries to the project
target_link_libraries(SofTrees PRIVATE raylib raylib_cpp box2d)
