cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS2)
    endif()
else()
    message(WARNING "ROS_DISTRO not set, assuming non-ROS mode")
    add_compile_definitions(USE_CMAKE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        find_package(ament_cmake REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(robot_state_publisher REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(gazebo_ros REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(robot_msgs REQUIRED)
        find_package(robot_joint_controller REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(gazebo_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        ament_package()
    endif()
    add_compile_options(${GAZEBO_CXX_FLAGS})
    find_package(gazebo REQUIRED)
endif()

# Inference Runtime

set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")

# ONNX Runtime Setup
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Detect Jetson platform
set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_EXT "lib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash download_inference_runtime.sh onnx")
endif()

# LibTorch Setup
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")
set(LIBTORCH_DIR "${INFERENCE_RUNTIME_DIR}/libtorch")
set(RL_SAR_ENABLE_TORCH ON CACHE BOOL "Enable LibTorch inference runtime")

if(RL_SAR_ENABLE_TORCH AND EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
    set(USE_TORCH "ON")
    add_compile_definitions(USE_TORCH)

    message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")

    # For Jetson with old CMake: set common Jetson CUDA architectures to avoid CUDA17 requirement
    if(CMAKE_VERSION VERSION_LESS "3.18")
        set(TORCH_CUDA_ARCH_LIST "5.3;6.2;7.2;8.7" CACHE STRING "CUDA architectures")
        set(ENV{TORCH_CUDA_ARCH_LIST} "5.3;6.2;7.2;8.7")
        message(STATUS "CMake ${CMAKE_VERSION} detected - using common Jetson CUDA architectures")
    endif()

    set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
    find_package(Torch QUIET)

    if(Torch_FOUND)
        message(STATUS "LibTorch version: ${Torch_VERSION}")

        # Filter out any NOTFOUND entries from TORCH_LIBRARIES (e.g. CUDA_nvrtc_LIBRARY
        # is absent on Jetson since only runtime libs are installed, not the full toolkit stub)
        set(_torch_libs_filtered "")
        foreach(_lib ${TORCH_LIBRARIES})
            if(NOT "${_lib}" MATCHES "NOTFOUND")
                list(APPEND _torch_libs_filtered "${_lib}")
            else()
                message(STATUS "Skipping missing optional Torch library: ${_lib}")
            endif()
        endforeach()
        set(TORCH_LIBRARIES ${_torch_libs_filtered})

        # CUDA::nvrtc may not resolve on Jetson (runtime-only install without dev stubs).
        # Create an INTERFACE stub so link steps that reference it don't fail.
        if(NOT TARGET CUDA::nvrtc)
            add_library(CUDA::nvrtc INTERFACE IMPORTED)
            find_library(_nvrtc_lib nvrtc
                HINTS /usr/local/cuda/targets/aarch64-linux/lib
                      /usr/local/cuda-12.6/targets/aarch64-linux/lib
                      /usr/local/cuda/lib64)
            if(_nvrtc_lib)
                set_target_properties(CUDA::nvrtc PROPERTIES
                    INTERFACE_LINK_LIBRARIES "${_nvrtc_lib}")
                message(STATUS "Found libnvrtc: ${_nvrtc_lib}")
            else()
                message(STATUS "libnvrtc not found - CUDA::nvrtc stub created (optional)")
            endif()
        endif()

        message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
    else()
        set(USE_TORCH "OFF")
        message(WARNING "LibTorch found but find_package(Torch) failed")
    endif()
else()
    if(RL_SAR_ENABLE_TORCH)
        message(WARNING "LibTorch not found. Please run: bash download_inference_runtime.sh libtorch")
    else()
        message(STATUS "LibTorch disabled by RL_SAR_ENABLE_TORCH=OFF")
    endif()
endif()

# RPATH Configuration for inference libraries
if(USE_TORCH OR USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${LIBTORCH_DIR}/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
    endif()
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PREBUILT_LIB_PATHS}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()

    message(STATUS "Configured RPATH:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
    link_directories(/usr/local/lib)
    include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    # Find OpenMP (required by LibTorch on macOS)
    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        message(STATUS "Found OpenMP library: ${OMP_LIB}")
        set(OPENMP_LIBRARIES ${OMP_LIB})
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${OMP_LIB_DIR}")
        message(STATUS "Added OpenMP to RPATH: ${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    set(ARCH_DIR "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_sdk2
if(SYSTEM_TYPE STREQUAL "Linux")
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
    add_subdirectory(library/thirdparty/robot_sdk/unitree/unitree_sdk2)
endif()

include_directories(
    include
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    fsm_robot
    # Qmini serial-port motor SDK
    # SerialPort.h lives here; errorClass.h is at serialPort/include/errorClass.h
    library/thirdparty/robot_sdk/qmini/include/serialPort
    # SerialPort.h uses #include "serialPort/include/errorClass.h", so parent needed too
    library/thirdparty/robot_sdk/qmini/include
    library/thirdparty/robot_sdk/qmini/include/unitreeMotor
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
    if(SYSTEM_TYPE STREQUAL "Darwin" AND OPENMP_LIBRARIES)
        target_link_libraries(inference_runtime PUBLIC ${OPENMP_LIBRARIES})
    endif()
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS inference_runtime DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for motion loader
add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

# library for rl sdk
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    motion_loader
    inference_runtime
    Python3::Python
    Python3::Module
    TBB::tbb
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl sim (ROS2 only)
if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_sim
            joint_state_broadcaster
            robot_state_publisher
            rclcpp
            gazebo_ros
            std_msgs
            robot_msgs
            robot_joint_controller
            rclpy
            gazebo_msgs
            std_srvs
            geometry_msgs
        )
        install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl real go2 (also supports go2w via wheel mode)
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_go2 src/rl_real_go2.cpp)
    target_link_libraries(rl_real_go2
        unitree_sdk2
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    # Add unitree_sdk2 library path to RPATH
    set_target_properties(rl_real_go2 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_go2
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_go2 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# executable for rl real g1
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_g1 src/rl_real_g1.cpp)
    target_link_libraries(rl_real_g1
        unitree_sdk2
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    # Add unitree_sdk2 library path to RPATH
    set_target_properties(rl_real_g1 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/${ARCH_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_g1
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_g1 DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()
endif()

# MJX Hyper Go 7303 / ET6 two-channel RC car deployment.
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_mjx_et6 src/rl_real_mjx_et6.cpp)
    target_link_libraries(rl_real_mjx_et6
        inference_runtime
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    set_target_properties(rl_real_mjx_et6 PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    )
endif()

# executable for rl real qmini
if(SYSTEM_TYPE STREQUAL "Linux")
    # Qmini serial-port motor SDK (aarch64 only)
    set(QMINI_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/qmini/lib/${ARCH_DIR})
    find_library(UNITREE_MOTOR_SDK UnitreeMotorSDK_Arm64
        HINTS ${QMINI_SDK_LIB_DIR}
        NO_DEFAULT_PATH
    )
    if(NOT UNITREE_MOTOR_SDK)
        # fallback: try without suffix
        find_library(UNITREE_MOTOR_SDK UnitreeMotorSDK
            HINTS ${QMINI_SDK_LIB_DIR}
            NO_DEFAULT_PATH
        )
    endif()
    if(UNITREE_MOTOR_SDK)
        message(STATUS "Found Qmini UnitreeMotorSDK: ${UNITREE_MOTOR_SDK}")
    else()
        message(WARNING "Qmini UnitreeMotorSDK not found in ${QMINI_SDK_LIB_DIR} (expected aarch64 target), skipping rl_real_qmini")
    endif()

    if(NOT UNITREE_MOTOR_SDK)
        # Skip rl_real_qmini — SDK library is only available for aarch64 (Jetson/ARM target)
    else()
    add_executable(rl_real_qmini src/rl_real_qmini.cpp)
    target_link_libraries(rl_real_qmini
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        ${UNITREE_MOTOR_SDK}
    )
    set_target_properties(rl_real_qmini PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${QMINI_SDK_LIB_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            ament_target_dependencies(rl_real_qmini
                rclcpp
                geometry_msgs
            )
            install(TARGETS rl_real_qmini DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()

    # Calibration tool — no ROS dependency, runs standalone
    add_executable(rl_calib_qmini src/rl_calib_qmini.cpp)
    target_link_libraries(rl_calib_qmini
        ${YAML_CPP_LIBRARIES}
        ${UNITREE_MOTOR_SDK}
    )
    set_target_properties(rl_calib_qmini PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${QMINI_SDK_LIB_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            install(TARGETS rl_calib_qmini DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()

    # Mirror tool (Jetson side) — no ROS dependency, sends UDP
    add_executable(rl_mirror_qmini src/rl_mirror_qmini.cpp)
    target_link_libraries(rl_mirror_qmini
        ${YAML_CPP_LIBRARIES}
        ${UNITREE_MOTOR_SDK}
    )
    set_target_properties(rl_mirror_qmini PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${QMINI_SDK_LIB_DIR}"
    )
    if(NOT USE_CMAKE)
        if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
            install(TARGETS rl_mirror_qmini DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()

    endif() # UNITREE_MOTOR_SDK found
endif() # SYSTEM_TYPE Linux

# Mirror tool (PC/sim side) — ROS2 node, receives UDP and drives Gazebo
if(NOT USE_CMAKE)
    add_executable(rl_mirror_sim src/rl_mirror_sim.cpp)
    target_link_libraries(rl_mirror_sim
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_mirror_sim
            rclcpp
            std_msgs
            robot_msgs
            robot_joint_controller
            std_srvs
            geometry_msgs
        )
        install(TARGETS rl_mirror_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for actuator net
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds rviz ${PROJECT_ROOT_DIR}/policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available")
    endif()
    if(IS_JETSON)
        message(STATUS "LibTorch: Jetson device detected")
        if(JETPACK_VERSION)
            message(STATUS "JetPack version: ${JETPACK_VERSION}")
        endif()
    endif()
endif()
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "========================================")
