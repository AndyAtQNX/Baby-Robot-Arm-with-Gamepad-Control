#!/bin/bash
 
set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
 
# --- Environment Sanity Check ---
if [ ! -d "$QNX_TARGET" ]; then
    echo "QNX_TARGET is not set. Exiting..."
    exit 1
fi
 
echo "--- QNX Environment Variables ---"
printenv | grep "QNX"
echo "-------------------------------"
 
# --- Main Build Loop (for aarch64 only) ---
for arch in aarch64; do
 
    # Set architecture-specific variables
    if [ "$arch" == "aarch64" ]; then
        CPUVARDIR=aarch64le
        CPUVAR=aarch64le
    else
        echo "Invalid architecture. This script is configured for aarch64 only. Exiting..."
        exit 1
    fi
 
    echo "CPU set to $CPUVAR"
    echo "CPUVARDIR set to $CPUVARDIR"
 
    # Export variables so child processes (like colcon/cmake) can see them
    export CPUVARDIR=${CPUVARDIR}
    export CPUVAR=${CPUVAR}
    export ARCH=${arch}
 
    # --- ROS2 Installation Discovery ---
    if [ -f "$QNX_TARGET/$CPUVARDIR/opt/ros/humble/local_setup.bash" ]; then
	    ROS2_HOST_INSTALLATION_PATH=$QNX_TARGET/$CPUVARDIR/opt/ros/humble
	    CMAKE_MODULE_PATH="$PWD/platform/modules"
        echo "Found ROS2 Installation in $ROS2_HOST_INSTALLATION_PATH"
    elif [ -f "$HOME/ros2_humble/install/$CPUVARDIR/local_setup.bash" ]; then
	      ROS2_HOST_INSTALLATION_PATH=$HOME/ros2_humble/install/$CPUVARDIR
	      CMAKE_MODULE_PATH=" "
	      echo "ROS2 found in $ROS2_HOST_INSTALLATION_PATH"
    else
	      echo "Failed to find ROS2 in expected locations, please set ROS2_HOST_INSTALLATION_PATH"
	      continue
    fi
 
    # Source the ROS2 environment for cross-compilation
    . $ROS2_HOST_INSTALLATION_PATH/local_setup.bash
    
    echo "--- ROS Environment Variables ---"
    printenv | grep "ROS"
    echo "-------------------------------"
 
    # --- Build Execution ---
    colcon build --merge-install --cmake-force-configure \
    --build-base=$PWD/build/$CPUVARDIR \
    --event-handlers console_direct+ \
    --install-base=$PWD/install/$CPUVARDIR \
    --cmake-args \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DCMAKE_TOOLCHAIN_FILE="$PWD/platform/qnx.nto.toolchain.cmake" \
        -DBUILD_TESTING:BOOL="OFF" \
        -DCMAKE_BUILD_TYPE="Release" \
        -DCMAKE_MODULE_PATH=$CMAKE_MODULE_PATH \
        -DROS2_HOST_INSTALLATION_PATH=$ROS2_HOST_INSTALLATION_PATH \
        -DROS_EXTERNAL_DEPS_INSTALL=${QNX_TARGET}/${CPUVARDIR}/opt/ros/humble \
        -Wno-dev --no-warn-unused-cli \
    --packages-select  joy_teleop_hiddi arm_controller
 
    rc=$?
    if [ $rc -eq 0 ]; then
        echo "$arch Success"
    else
        echo "$arch Error: $rc"
        exit $rc
    fi
 
    echo " "
 
done
 