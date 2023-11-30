#!/usr/bin/env bash

set -eux

# Dependency versions.
castxml_version="0.6.2" # version specifier for Linux only
boost_version="1.83.0"

# Collect some information about the build target.
build_os="$(uname)"
python_version=$(python3 -c 'import sys; v=sys.version_info; print(f"{v.major}.{v.minor}")')

install_boost() {
    b2_args=("$@")

    curl -L "https://boostorg.jfrog.io/artifactory/main/release/${boost_version}/source/boost_${boost_version//./_}.tar.bz2" | tar xj
    pushd "boost_${boost_version//./_}"

    # Tell boost-python the exact Python install to use, since we may have
    # multiple on the host system.
    python_include_path=$(python3 -c "from sysconfig import get_paths as gp; print(gp()['include'])")
    echo "using python : ${python_version} : : ${python_include_path} ;" > "$HOME/user-config.jam"

    ./bootstrap.sh
    sudo ./b2 "${b2_args[@]}" \
        --with-serialization \
        --with-filesystem \
        --with-system \
        --with-program_options \
        --with-python \
        install

    popd
}

install_castxml() {
    curl -L "https://github.com/CastXML/CastXML/archive/refs/tags/v${castxml_version}.tar.gz" | tar xz

    pushd "CastXML-${castxml_version}"
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .
    make install
    popd
}


# Work inside a temporary directory.
cd "$(mktemp -d -t 'ompl-wheels.XXX')"

if [ "${build_os}" == "Linux" ]; then
    # Install CastXML dependency from source, since the manylinux container
    # doesn't have a prebuilt version in the repos.
    install_castxml

    # Install the latest Boost, because it has to be linked to the exact version of
    # Python for which we are building the wheel.
    install_boost
elif [ "${build_os}" == "Darwin" ]; then
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
