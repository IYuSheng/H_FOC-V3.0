target_sources(mc_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/drv_uart.c
)

target_include_directories(mc_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
