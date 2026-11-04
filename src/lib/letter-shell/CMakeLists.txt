file(GLOB SOURCES CONFIGURE_DEPENDS
    *.c
)

target_sources(mc_app INTERFACE ${SOURCES})

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(mc_app INTERFACE
    SHELL_CFG_USER=\"mc_shell_cfg.h\"
)
