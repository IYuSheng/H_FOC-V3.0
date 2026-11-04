add_library(mc_board INTERFACE)

target_sources(mc_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_sys.c
)

target_include_directories(mc_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mc_board INTERFACE mc_app)
