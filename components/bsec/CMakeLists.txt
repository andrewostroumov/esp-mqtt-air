idf_component_register(SRCS
        "bsec_integration.c"
        "bsec_serialized_configurations_iaq.c"
        INCLUDE_DIRS "."
        REQUIRES bme680)

add_library(bseclib STATIC IMPORTED GLOBAL)
set_target_properties(bseclib PROPERTIES IMPORTED_LOCATION "${COMPONENT_DIR}/libalgobsec.a")
target_link_libraries(${COMPONENT_LIB} INTERFACE bseclib)
target_link_libraries(${COMPONENT_TARGET} INTERFACE "-lm" "-lgcc")