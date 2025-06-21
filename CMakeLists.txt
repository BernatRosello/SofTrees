cmake_minimum_required(VERSION 3.5)
# project name and version
project(SofTrees)

# C++ minimum version  required
set(CMAKE_CXX_STANDARD 17)
# Let the language server know stuff about the project and libraries
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Glob finds all files in a directory, src/ in this case, and assign to the SRC_FILES variable
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
# Creates the executable with the source files
add_executable(SofTrees ${SRC_FILES})
# Sets the include directory
target_include_directories(SofTrees PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Include the command that downloads libraries
include(FetchContent)

# ================ 
# ==== RAYLIB ==== 
# ================ 
set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
  include(FetchContent)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
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

# To use raygui copy the header raygui.h into the raylib dependency source folder (kind of jank, but works)
# https://github.com/raysan5/raygui/blob/master/src/raygui.h

# Link all libraries to the project
target_link_libraries(SofTrees PRIVATE raylib raylib_cpp box2d)
