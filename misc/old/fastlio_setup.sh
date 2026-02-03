#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
TARGET_ROS_DISTRO="humble"
SKIP_DEPS=0
SKIP_BUILD=0
FASTLIO_REMOTE_URL="https://github.com/Ericsii/FAST_LIO_ROS2.git"
LIVOX_REMOTE_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
LIVOX_SDK_REMOTE_URL="https://github.com/Livox-SDK/Livox-SDK2.git"
FORCE_LIVOX_SOURCE=0
SKIP_LIVOX_SDK=0
SKIP_LIVOX_DRIVER=0

usage() {
    cat <<EOF
Usage: ${SCRIPT_NAME} [options]

Options:
  --ros-distro <name>  Target ROS 2 distribution (default: humble)
  --skip-deps          Skip apt/rosdep dependency installation
  --skip-build         Skip the colcon build step
  --skip-livox-sdk     Skip cloning/building Livox-SDK2
  --skip-livox-driver  Skip installing Livox ROS Driver 2
  --livox-source       Force cloning livox_ros_driver2 into the workspace instead of apt
  -h, --help           Show this help and exit
EOF
}

log() {
    printf '[fastlio_setup] %s\n' "$*"
}

fail() {
    printf '[fastlio_setup][error] %s\n' "$*" >&2
    exit 1
}

source_preserving_u() {
    local target="$1"
    local had_u=0

    if [[ $- == *u* ]]; then
        had_u=1
        set +u
    fi

    # shellcheck source=/dev/null
    source "${target}"

    if (( had_u )); then
        set -u
    fi
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        fail "Required command '$1' is missing. Install it and rerun."
    fi
}

ensure_packages() {
    local -a missing=()
    for pkg in "$@"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing+=("$pkg")
        fi
    done

    if ((${#missing[@]})); then
        log "Installing missing packages: ${missing[*]}"
        sudo apt-get update
        sudo apt-get install -y "${missing[@]}"
    else
        log "Requested apt packages already installed."
    fi
}

ensure_rosdep_init() {
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        log "rosdep not initialized on this machine. Initializing..."
        sudo rosdep init
    fi
    rosdep update
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --ros-distro)
                [[ $# -ge 2 ]] || fail "--ros-distro requires an argument"
                TARGET_ROS_DISTRO="$2"
                shift 2
                ;;
            --skip-deps)
                SKIP_DEPS=1
                shift
                ;;
            --skip-build)
                SKIP_BUILD=1
                shift
                ;;
            --skip-livox-sdk)
                SKIP_LIVOX_SDK=1
                shift
                ;;
            --skip-livox-driver)
                SKIP_LIVOX_DRIVER=1
                shift
                ;;
            --livox-source)
                FORCE_LIVOX_SOURCE=1
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                fail "Unknown argument: $1"
                ;;
        esac
    done
}

run_colcon_build() {
    local workspace_dir="$1"
    log "Building FAST-LIO in ${workspace_dir}"
    (
        cd "${workspace_dir}"
        colcon build --symlink-install --packages-select fast_lio
    )
}

clean_fastlio_build_artifacts() {
    local build_pkg_dir="${WORKSPACE_DIR}/build/fast_lio"
    local install_pkg_dir="${WORKSPACE_DIR}/install/fast_lio"
    local log_pkg_dir="${WORKSPACE_DIR}/log/latest_build/fast_lio"

    for path in "${build_pkg_dir}" "${install_pkg_dir}" "${log_pkg_dir}"; do
        if [[ -e "${path}" ]]; then
            log "Removing stale FAST-LIO artifact: ${path}"
            rm -rf "${path}"
        fi
    done
}

ensure_cstdint_include() {
    local file="$1"

    if grep -Eq '^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$' "${file}"; then
        return
    fi

    local tmp_file="${file}.tmp.$$"
    awk '
        BEGIN { have=0; added=0; inblock=0 }
        {
            if ($0 ~ /^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$/) { have=1 }
            if ($0 ~ /^[[:space:]]*#include[[:space:]]*[<"].*[>"][[:space:]]*$/) {
                inblock=1
                print
                next
            }
            if (inblock && !have && !added) {
                print "#include <cstdint>"
                added=1
            }
            inblock=0
            print
        }
    ' "${file}" > "${tmp_file}"
    mv "${tmp_file}" "${file}"
}

install_livox_sdk2() {
    if (( SKIP_LIVOX_SDK == 1 )); then
        log "Skipping Livox-SDK2 installation as requested."
        return
    fi

    if [[ ! -d "${LIVOX_SDK_DIR}" ]]; then
        log "Cloning Livox-SDK2 into ${LIVOX_SDK_DIR}"
        git clone "${LIVOX_SDK_REMOTE_URL}" "${LIVOX_SDK_DIR}"
    else
        log "Livox-SDK2 already present at ${LIVOX_SDK_DIR}"
    fi

    local define_h="${LIVOX_SDK_DIR}/sdk_core/comm/define.h"
    local file_manager_h="${LIVOX_SDK_DIR}/sdk_core/logger_handler/file_manager.h"

    if [[ -f "${define_h}" ]]; then
        ensure_cstdint_include "${define_h}"
    else
        log "Warning: ${define_h} not found while patching includes."
    fi

    if [[ -f "${file_manager_h}" ]]; then
        ensure_cstdint_include "${file_manager_h}"
    else
        log "Warning: ${file_manager_h} not found while patching includes."
    fi

    local build_dir="${LIVOX_SDK_DIR}/build"
    mkdir -p "${build_dir}"
    log "Building Livox-SDK2 (cmake .. && make -j)"
    (
        cd "${build_dir}"
        cmake ..
        local jobs
        jobs="$(nproc 2>/dev/null || echo 4)"
        make -j"${jobs}"
        sudo make install
    )
    log "Livox-SDK2 installed successfully."
}

ensure_livox_ros_driver2() {
    if (( SKIP_LIVOX_DRIVER == 1 )); then
        log "Skipping Livox ROS Driver 2 installation as requested."
        return
    fi

    local apt_pkg="ros-${TARGET_ROS_DISTRO}-livox-ros-driver2"
    local using_source=0

    if (( FORCE_LIVOX_SOURCE == 0 )); then
        if dpkg -s "${apt_pkg}" >/dev/null 2>&1; then
            log "Detected ${apt_pkg}; livox_ros_driver2 already provided via apt."
            return
        fi

        if ! apt-cache policy "${apt_pkg}" 2>/dev/null | grep -q "Candidate: (none)"; then
            log "Installing ${apt_pkg} via apt so livox_ros_driver2 is discoverable."
            if sudo apt-get install -y "${apt_pkg}"; then
                return
            fi
            log "Failed to install ${apt_pkg}. Falling back to source clone."
        else
            log "No apt candidate for ${apt_pkg}. Will clone livox_ros_driver2 from source."
        fi
    else
        log "User requested source-based livox_ros_driver2 installation."
    fi

    using_source=1

    if [[ -d "${LIVOX_DIR}" ]]; then
        log "livox_ros_driver2 already present at ${LIVOX_DIR}"
    else
        log "Cloning livox_ros_driver2 from ${LIVOX_REMOTE_URL}"
        git clone "${LIVOX_REMOTE_URL}" "${LIVOX_DIR}"
    fi

    if (( using_source == 1 )); then
        log "Installing livox_ros_driver2 dependencies via rosdep."
        rosdep install --from-paths "${LIVOX_DIR}" --ignore-src --rosdistro "${TARGET_ROS_DISTRO}" -r -y
        log "Building livox_ros_driver2 via build.sh ${TARGET_ROS_DISTRO}"
        (
            cd "${LIVOX_DIR}"
            bash ./build.sh "${TARGET_ROS_DISTRO}"
        )
        log "livox_ros_driver2 built successfully."
    fi
}

parse_args "$@"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
FAST_LIO_DIR="${SRC_DIR}/FAST_LIO_ROS2"
LIVOX_DIR="${SRC_DIR}/livox_ros_driver2"
LIVOX_SDK_DIR="${SRC_DIR}/Livox-SDK2"
ROS_SETUP_FILE="/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
FAST_LIO_IGNORE_FILE="${FAST_LIO_DIR}/COLCON_IGNORE"

log "Detected workspace root: ${WORKSPACE_DIR}"

if [[ ! -d "${SRC_DIR}" ]]; then
    fail "Unable to locate workspace src folder (looked at ${SRC_DIR})."
fi

if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
    fail "ROS 2 '${TARGET_ROS_DISTRO}' is not installed (missing ${ROS_SETUP_FILE}). Install it and rerun."
fi

source_preserving_u "${ROS_SETUP_FILE}"

if [[ "${ROS_DISTRO:-}" != "${TARGET_ROS_DISTRO}" ]]; then
    log "Warning: sourced ROS_DISTRO='${ROS_DISTRO:-unknown}' differs from requested '${TARGET_ROS_DISTRO}'."
fi

require_command git
require_command colcon
require_command rosdep

if [[ ! -d "${FAST_LIO_DIR}" ]]; then
    log "FAST_LIO_ROS2 not found inside ${SRC_DIR}. Cloning repository..."
    git clone "${FASTLIO_REMOTE_URL}" "${FAST_LIO_DIR}"
elif [[ ! -d "${FAST_LIO_DIR}/.git" ]]; then
    fail "Existing directory ${FAST_LIO_DIR} is not a git checkout. Remove/move it and rerun."
else
    log "FAST_LIO_ROS2 already exists at ${FAST_LIO_DIR}; keeping current state."
fi

if (( SKIP_DEPS == 0 )); then
    log "Ensuring base dependencies are installed..."
    ensure_packages build-essential python3-rosdep python3-colcon-common-extensions cmake
    ensure_rosdep_init
    install_livox_sdk2
    ensure_livox_ros_driver2
    log "Installing package-specific dependencies via rosdep..."
    rosdep install --from-paths "${FAST_LIO_DIR}" --ignore-src --rosdistro "${TARGET_ROS_DISTRO}" -r -y
else
    log "Skipping dependency installation as requested."
fi

if (( SKIP_BUILD == 0 )); then
    if [[ -f "${FAST_LIO_IGNORE_FILE}" ]]; then
        log "Removing ${FAST_LIO_IGNORE_FILE} so FAST-LIO is not ignored by colcon."
        rm -f "${FAST_LIO_IGNORE_FILE}"
    fi
    clean_fastlio_build_artifacts
    run_colcon_build "${WORKSPACE_DIR}"
else
    log "Skipping colcon build as requested."
fi

log "FAST-LIO setup finished. Source ${WORKSPACE_DIR}/install/setup.bash before running launch files."
