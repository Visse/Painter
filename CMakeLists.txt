cmake_minimum_required(VERSION 3.10)

add_library(Painter STATIC
   include/Painter2D.h
   src/Painter2D.cpp
)
target_include_directories( Painter PUBLIC include/ )


option (PAINTER_BUILD_TESTS "Build tests for painter" ON)

if (PAINTER_BUILD_TESTS)
    add_subdirectory(tests)
endif (PAINTER_BUILD_TESTS)
