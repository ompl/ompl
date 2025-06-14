#!/usr/bin/env bash

set -eux

# Dependency versions.
boost_version="1.87.0"

# Collect some information about the build target.
build_os="$(uname)"
python_version=$(python3 -c 'import sys; v=sys.version_info; print(f"{v.major}.{v.minor}")')

install_boost() {
    b2_args=("$@")

    curl -L "https://archives.boost.io/release/${boost_version}/source/boost_${boost_version//./_}.tar.bz2" | tar xj
    pushd "boost_${boost_version//./_}"

    ./bootstrap.sh
    sudo ./b2 "${b2_args[@]}" \
        --with-serialization \
        --with-filesystem \
        --with-system \
        --with-program_options \
        install

    popd
}

# Work inside a temporary directory.
cd "$(mktemp -d -t 'ompl-wheels.XXX')"

if [ "${build_os}" == "Darwin" ]; then
    # On MacOS, we may be cross-compiling for a different architecture. Detect
    # that here.
    build_arch="${OMPL_BUILD_ARCH:-x86_64}"

    # Make sure we install the target Python version from brew instead of
    # depending on the system version.
    brew install --overwrite "python@${python_version}"

    if [ "${build_arch}" == "x86_64" ]; then
        install_boost architecture=x86 address-model=64 cxxflags="-arch x86_64"
    elif [ "${build_arch}" == "arm64" ]; then
        install_boost architecture=arm address-model=64 cxxflags="-arch arm64"
    fi
fi
