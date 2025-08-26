if("$ENV{QNX_HOST}" STREQUAL "")
message(FATAL_ERROR "QNX_HOST environment variable not found. Please set the variable to your host's build tools")
endif()
if("$ENV{QNX_TARGET}" STREQUAL "")
message(FATAL_ERROR "QNX_TARGET environment variable not found. Please set the variable to the qnx target location")
endif()

set(QNX_HOST "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")

message(STATUS "using QNX_HOST ${QNX_HOST}")
message(STATUS "using QNX_TARGET ${QNX_TARGET}")

set(ARCH "$ENV{ARCH}")
set(CPUVAR "$ENV{CPUVAR}")
set(CPUVARDIR "$ENV{CPUVARDIR}")

message(STATUS "using CPUVAR ${CPUVAR}")
message(STATUS "using CPUVARDIR ${CPUVARDIR}")
message(STATUS "using ARCH ${ARCH}")

message(STATUS "using CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}")

set(QNX TRUE)
set(CMAKE_SYSTEM_NAME QNX)

set(CMAKE_C_COMPILER ${QNX_HOST}/usr/bin/qcc)
set(CMAKE_CXX_COMPILER ${QNX_HOST}/usr/bin/q++)

set(CMAKE_SYSTEM_PROCESSOR "${ARCH}")

set(CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES} ${QNX_TARGET}/usr/include)

set(EXTRA_CMAKE_C_FLAGS "-Wl,-rpath-link,${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR}/usr/lib:${CMAKE_INSTALL_PREFIX}/lib -D_QNX_SOURCE ")
set(EXTRA_CMAKE_CXX_FLAGS "${EXTRA_CMAKE_C_FLAGS} -std=c++17")
set(EXTRA_CMAKE_LINKER_FLAGS "-Wl,--build-id=md5")

# needs a cpu + variant
set(CMAKE_C_FLAGS "-Vgcc_nto${CPUVAR} ${EXTRA_CMAKE_C_FLAGS}" CACHE STRING "c_flags")
set(CMAKE_CXX_FLAGS "-Vgcc_nto${CPUVAR} ${EXTRA_CMAKE_CXX_FLAGS}" CACHE STRING "cxx_flags")

# needs only cpu, ARCH=(CPU only)
set(CMAKE_AR "${QNX_HOST}/usr/bin/nto${ARCH}-ar${HOST_EXECUTABLE_SUFFIX}" CACHE PATH "archiver")
set(CMAKE_RANLIB "${QNX_HOST}/usr/bin/nto${ARCH}-ranlib${HOST_EXECUTABLE_SUFFIX}" CACHE PATH "ranlib")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_CMAKE_LINKER_FLAGS}" CACHE STRING "exe_linker_flags")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${EXTRA_CMAKE_LINKER_FLAGS}" CACHE STRING "so_linker_flags")

set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)

# the variable below has to be set according to the output of running sysconfig.get_config_var('SOABI') on the target
# this will allow python extension files to be found.
set(PYTHON_SOABI cpython-38)
find_package(PythonInterp 3 REQUIRED)
set(PYTHON_INCLUDE_DIR ${QNX_TARGET}/usr/include/${CPUVARDIR}/python3.8;${QNX_TARGET}/${CPUVARDIR}/usr/include/python3.8;${QNX_TARGET}/usr/include/python3.8;${NUMPY_HEADERS})
set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR})
set(PYTHON_LIBRARY ${QNX_TARGET}/${CPUVARDIR}/usr/lib/libpython3.8.so)
set(PYTHON_LIBRARIES ${PYTHON_LIBRARY})
set(PYTHONLIBS_FOUND)
set(PYTHON_MODULE_EXTENSION .cpython-38.so)
set(PYTHON_IS_DEBUG FALSE)

set(CMAKE_FIND_ROOT_PATH "${CMAKE_INSTALL_PREFIX};${QNX_TARGET};${QNX_TARGET}/${CPUVARDIR};${ROS2_HOST_INSTALLATION_PATH}")

set(CMAKE_SKIP_RPATH TRUE CACHE BOOL "If set, runtime paths are not added when using shared libraries.")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

