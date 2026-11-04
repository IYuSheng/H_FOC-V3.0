target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_encoder.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
