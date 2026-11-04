target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_current.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_position.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_speed.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_weak.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
