#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# ========================
# Configuration
# ========================

# ========================
# Build Functions
# ========================

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_SCRIPT="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ -f "$DOWNLOAD_SCRIPT" ]; then
        print_info "Checking inference libraries..."
        bash "$DOWNLOAD_SCRIPT" || {
            print_error "Failed to setup inference libraries"
            exit 1
        }
        print_success "Inference runtime setup completed!"
    else
        print_warning "Download script not found: $DOWNLOAD_SCRIPT"
    fi
}

setup_robot_descriptions() {
    print_header "[Setting up Robot Descriptions]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_ROBOT_DESC_SCRIPT="${SCRIPT_DIR}/scripts/download_robot_descriptions.sh"

    if [ -f "$DOWNLOAD_ROBOT_DESC_SCRIPT" ]; then
        print_info "Checking robot description files..."
        bash "$DOWNLOAD_ROBOT_DESC_SCRIPT" || {
            print_error "Failed to setup robot descriptions"
            exit 1
        }
        print_success "Robot descriptions setup completed!"
    else
        print_warning "Robot descriptions download script not found: $DOWNLOAD_ROBOT_DESC_SCRIPT"
    fi
}

run_cmake_build() {
    print_header "[Running CMake Build]"
    print_warning "NOTE: CMake build is for hardware deployment only, not for simulation."
    print_separator

    # Detect CUDA toolkit root for Jetson (installed via apt as cuda-12-6 or similar)
    local cuda_root=""
    for dir in /usr/local/cuda /usr/local/cuda-12.6 /usr/local/cuda-12; do
        if [ -d "${dir}/bin" ]; then
            cuda_root="$dir"
            break
        fi
    done

    local cmake_cuda_args=""
    if [ -n "$cuda_root" ]; then
        print_info "Found CUDA toolkit at: ${cuda_root}"
        # Add nvcc to PATH so CMake find_package(CUDA) can locate it
        export PATH="${cuda_root}/bin${PATH:+:$PATH}"
        cmake_cuda_args="-DCUDA_TOOLKIT_ROOT_DIR=${cuda_root} -DCMAKE_CUDA_COMPILER=${cuda_root}/bin/nvcc"
    fi

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON ${cmake_cuda_args}
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "CMake build completed!"
}

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    # Clean existing symlinks
    clean_existing_symlinks "${packages[@]}"

    # Detect incompatible artifacts
    detect_incompatible_build_artifacts

    # Create appropriate symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        create_symlinks_for_all_packages
    else
        create_symlinks_for_specific_packages "${packages[@]}"
    fi

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        print_header "[Using colcon build]"
        print_info "Building all packages..."
        colcon build --merge-install --symlink-install
    else
        print_header "[Using colcon build]"
        print_info "Building specific packages: $package_list"
        colcon build --merge-install --symlink-install --packages-select $package_list
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    local packages=("$@")

    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    if [ ${#packages[@]} -eq 0 ]; then
        echo "  - All package.xml symlinks in directory src/"
    else
        echo "  - Package.xml symlinks for: ${packages[*]}"
    fi
    echo "  - directory build/"
    echo "  - directory cmake_build/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"

    # Ask for confirmation
    if [ ${#packages[@]} -eq 0 ]; then
        if ! ask_confirmation "Are you sure you want to clean ALL symlinks and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    else
        if ! ask_confirmation "Are you sure you want to clean symlinks for specified packages and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    fi

    # Remove package.xml symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all symlinks"
    else
        print_info "Removing symlinks for specific packages..."
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ]; then
                if [ -L "$package_dir/package.xml" ]; then
                    rm -f "$package_dir/package.xml"
                    print_success "Removed symlink from $package_name"
                else
                    print_warning "No symlink found for $package_name"
                fi
            else
                print_error "Package '$package_name' not found in src directory"
            fi
        done
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ cmake_build/ install/ log/ logs/

    print_success "Clean completed!"
}

clean_existing_symlinks() {
    local packages=("$@")

    print_header "[Cleaning Existing Symlinks]"

    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all existing package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all existing symlinks"
    else
        print_info "Removing existing symlinks for specified packages..."
        removed_packages=()
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ] && [ -L "$package_dir/package.xml" ]; then
                rm -f "$package_dir/package.xml"
                removed_packages+=("$package_name")
            fi
        done

        if [ ${#removed_packages[@]} -gt 0 ]; then
            print_success "Removed existing symlinks from: ${removed_packages[*]}"
        else
            print_warning "No existing symlinks found"
        fi
    fi
}

# ========================
# ROS Specific Functions
# ========================

detect_incompatible_build_artifacts() {
    print_header "[Checking for Incompatible Build Artifacts]"

    local needs_cleanup=false

    # Check for stale build artifacts
    if [ -d "devel" ] || [ -d ".catkin_tools" ]; then
        print_warning "Found stale build artifacts (devel/ or .catkin_tools/). Cleaning workspace..."
        needs_cleanup=true
    fi

    if [ "$needs_cleanup" = true ]; then
        clean_workspace
    else
        print_success "No incompatible build artifacts found"
    fi
}

create_symlinks_for_package() {
    local package_dir="$1"
    local package_name=$(basename "$package_dir")

    if [ -d "$package_dir" ]; then
        if [ -f "$package_dir/package.ros2.xml" ]; then
            [ -e "$package_dir/package.xml" ] && rm -f "$package_dir/package.xml"

            if [[ "$ROS_DISTRO" == "foxy" || "$ROS_DISTRO" == "humble" ]]; then
                ln -s package.ros2.xml "$package_dir/package.xml"
                return 0
            else
                print_error "Unknown ROS version: $ROS_DISTRO"
                return 1
            fi
        fi
    fi
    return 1
}

create_symlinks_for_all_packages() {
    print_header "[Creating Symlinks for All Packages]"

    created_packages=()
    while IFS= read -r -d '' package_dir; do
        package_dir=$(dirname "$package_dir")
        package_name=$(basename "$package_dir")
        if create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done < <(find src -name "package.ros2.xml" -print0)

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    else
        print_warning "No packages with dual ROS support found"
    fi
}

create_symlinks_for_specific_packages() {
    local packages=("$@")

    print_header "[Creating Symlinks for Specific Packages]"
    print_info "Packages to process: ${packages[*]}"

    created_packages=()
    for package_name in "${packages[@]}"; do
        package_dir=$(find src -name "$package_name" -type d | head -n 1)
        if [ -n "$package_dir" ] && create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    fi
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove symlinks and build artifacts)"
    echo -e "  -m, --cmake      Build using CMake (for hardware deployment only)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all symlinks and build artifacts"
    echo -e "  $0 --clean package1   # Clean specific package and build artifacts"
    echo -e "  $0 -m                 # Build with CMake for hardware deployment"
}

main() {
    local packages=()
    local clean_mode=false
    local cmake_mode=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle CMake build mode
    if [ "$cmake_mode" = true ]; then
        setup_inference_runtime
        run_cmake_build
        exit 0
    fi

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    # Handle ROS build
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash first."
        print_info "For hardware deployment, use the --cmake option instead."
        exit 1
    fi

    setup_inference_runtime
    setup_robot_descriptions
    run_ros_build "${packages[@]}"
}

main "$@"