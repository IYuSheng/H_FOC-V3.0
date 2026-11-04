add_library(mc_app INTERFACE)

add_subdirectory(drivers)
add_subdirectory(lib)
add_subdirectory(modules)
add_subdirectory(systemcmd)

target_include_directories(mc_app INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/user_main.c
)
