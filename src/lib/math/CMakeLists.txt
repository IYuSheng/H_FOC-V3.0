target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_parse.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
