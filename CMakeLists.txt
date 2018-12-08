cmake_minimum_required(VERSION 3.10)

add_library(Painter STATIC
   include/Painter/Fwd.h
   include/Painter/Vec2.h
   include/Painter/Color.h
   include/Painter/Painter2D.h
   src/Painter2D.cpp
)
target_include_directories( Painter PUBLIC include/ )


option (PAINTER_BUILD_TESTS "Build tests for painter" ON)
option (PAINTER_USE_GLM "Use glm for vec2 & color" OFF)

if (PAINTER_USE_GLM)
    target_compile_definitions(Painter PUBLIC -DPAINTER_USE_GLM)
    
    find_path(GLM_INCLUDE_DIR NAMES "glm/glm.hpp" )
    target_include_directories(Painter PUBLIC ${GLM_INCLUDE_DIR})
endif (PAINTER_USE_GLM)

if (PAINTER_BUILD_TESTS)
    add_subdirectory(tests)
endif (PAINTER_BUILD_TESTS)
