file(GLOB SOURCES
    *.c *.cpp
    src/*.c
    port/*.c
)

target_sources(mc_app INTERFACE ${SOURCES})

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
