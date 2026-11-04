target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_s_curve_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_trapezoid_planner.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
