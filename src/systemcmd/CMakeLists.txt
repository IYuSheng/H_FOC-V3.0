target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/param.c
    ${CMAKE_CURRENT_SOURCE_DIR}/reboot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_cpu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_encoder_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_foc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_identify.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_sweep.c
    ${CMAKE_CURRENT_SOURCE_DIR}/version.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
