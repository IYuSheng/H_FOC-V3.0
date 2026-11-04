target_sources(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_flux_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_hfi_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mc_hfi_pll.c
)

target_include_directories(mc_app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
