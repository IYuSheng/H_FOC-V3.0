target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/identify_inductance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/identify_mechanical.c
    ${CMAKE_CURRENT_SOURCE_DIR}/identify_parameter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/identify_resistance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_frequency_sweep.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
