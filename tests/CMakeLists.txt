

find_package(SFML COMPONENTS graphics window system REQUIRED)


add_executable(Painter2DTest
    Painter2DTest.cpp
    
    Renderer.h
    Renderer.cpp
)
target_link_libraries(Painter2DTest
    PRIVATE Painter
    PRIVATE ${SFML_LIBRARIES}
)
target_include_directories(Painter2DTest
    PRIVATE ${SFML_INCLUDE_DIR}
)



