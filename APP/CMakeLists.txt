cmake_minimum_required(VERSION 3.22)

# Build all application sources under APP/.
# Adding/removing .c files here requires no CMake edits.
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
)

