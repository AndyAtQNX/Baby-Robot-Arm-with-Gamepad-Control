set(console_bridge_INCLUDE_DIR ${QNX_TARGET}/usr/include)
set(console_bridge_LIBRARIES ${QNX_TARGET}/${CPUVARDIR}/usr/libconsole_bridge.so)
set(console_bridge_LIBRARY ${QNX_TARGET}/${CPUVARDIR}/usr/lib/libconsole_bridge.so)

if(NOT TARGET console_bridge::console_bridge)
        add_library(console_bridge::console_bridge UNKNOWN IMPORTED)
        set_target_properties(console_bridge::console_bridge PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIR}")

        set_property(TARGET console_bridge::console_bridge APPEND PROPERTY
            IMPORTED_LOCATION "${console_bridge_LIBRARY}")
endif()
