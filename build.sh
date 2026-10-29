#!/usr/bin/env bash
#
# build.sh - build picocalc-text-starter from the command line
#
# Uses the SDK and tools that the Raspberry Pi Pico VS Code extension
# installs in ~/.pico-sdk, at the versions CMakeLists.txt names, so a build
# here matches a build in VS Code. Set PICO_SDK_PATH (and have the toolchain,
# cmake and ninja on your PATH) to use your own instead.
#
# Usage: ./build.sh [-b board] [-t type] [-c] [-h]
#
#   -b board  board to build for: pico, pico_w, pico2 or pico2_w
#             (default: the one CMakeLists.txt sets, or the last one built)
#   -t type   CMake build type: Debug, Release, RelWithDebInfo or MinSizeRel
#   -c        clean: delete the build directory first
#   -h        show this help
#
# The firmware ends up in build/picocalc-text-starter.uf2.

set -euo pipefail

cd "$(dirname "$0")"

PROJECT=picocalc-text-starter
BUILD_DIR=build
PICO_HOME="${HOME}/.pico-sdk"

board=""
build_type=""
clean=0

usage()
{
    sed -n '/^# Usage:/,/^# The firmware/p' "$0" | sed 's/^# \{0,1\}//'
}

while getopts "b:t:ch" opt; do
    case "$opt" in
    b) board="$OPTARG" ;;
    t) build_type="$OPTARG" ;;
    c) clean=1 ;;
    h) usage; exit 0 ;;
    *) usage >&2; exit 1 ;;
    esac
done

# The versions the VS Code extension uses, from CMakeLists.txt
cmake_setting()
{
    sed -n "s/^set($1 \(.*\))$/\1/p" CMakeLists.txt
}
sdk_version=$(cmake_setting sdkVersion)
toolchain_version=$(cmake_setting toolchainVersion)
picotool_version=$(cmake_setting picotoolVersion)

# Put the extension's tools on the PATH when they are there
add_to_path()
{
    if [ -d "$1" ]; then
        PATH="$1:$PATH"
    fi
}
add_to_path "${PICO_HOME}/toolchain/${toolchain_version}/bin"
add_to_path "${PICO_HOME}/picotool/${picotool_version}/picotool"
add_to_path "$(ls -d "${PICO_HOME}"/ninja/v* 2>/dev/null | sort -V | tail -1)"
add_to_path "$(ls -d "${PICO_HOME}"/cmake/v3.* 2>/dev/null | sort -V | tail -1)/bin"
export PATH

if [ -z "${PICO_SDK_PATH:-}" ]; then
    export PICO_SDK_PATH="${PICO_HOME}/sdk/${sdk_version}"
fi
if [ -z "${PICO_TOOLCHAIN_PATH:-}" ] && [ -d "${PICO_HOME}/toolchain/${toolchain_version}" ]; then
    export PICO_TOOLCHAIN_PATH="${PICO_HOME}/toolchain/${toolchain_version}"
fi

# Check everything is there before starting
missing=0
if [ ! -f "${PICO_SDK_PATH}/pico_sdk_init.cmake" ]; then
    echo "error: Pico SDK ${sdk_version} not found at ${PICO_SDK_PATH}" >&2
    missing=1
fi
for tool in cmake ninja arm-none-eabi-gcc; do
    if ! command -v "$tool" >/dev/null; then
        echo "error: ${tool} not found" >&2
        missing=1
    fi
done
if [ "$missing" -ne 0 ]; then
    echo "Install the Raspberry Pi Pico extension for VS Code and open this project once" >&2
    echo "to download the SDK and tools, or set PICO_SDK_PATH and PATH yourself." >&2
    exit 1
fi

# A board or build type that differs from the last build needs a clean
# build directory, as the SDK can't change them in place
cached()
{
    sed -n "s/^$1:[A-Z]*=//p" "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null
}
if [ -n "$board" ] && [ -n "$(cached PICO_BOARD)" ] && [ "$board" != "$(cached PICO_BOARD)" ]; then
    echo "Board changed from $(cached PICO_BOARD) to ${board}, starting a clean build"
    clean=1
fi
if [ -n "$build_type" ] && [ -n "$(cached CMAKE_BUILD_TYPE)" ] && [ "$build_type" != "$(cached CMAKE_BUILD_TYPE)" ]; then
    echo "Build type changed from $(cached CMAKE_BUILD_TYPE) to ${build_type}, starting a clean build"
    clean=1
fi

if [ "$clean" -eq 1 ]; then
    rm -rf "$BUILD_DIR"
fi

# Configure when there is no build directory yet, or something new was asked for
if [ ! -f "${BUILD_DIR}/build.ninja" ] || [ -n "$board" ] || [ -n "$build_type" ]; then
    cmake_args=(-S . -B "$BUILD_DIR" -G Ninja)
    [ -n "$board" ] && cmake_args+=("-DPICO_BOARD=${board}")
    [ -n "$build_type" ] && cmake_args+=("-DCMAKE_BUILD_TYPE=${build_type}")
    cmake "${cmake_args[@]}"
fi

ninja -C "$BUILD_DIR"

echo
echo "Built ${BUILD_DIR}/${PROJECT}.uf2 for $(cached PICO_BOARD)"
