#!/usr/bin/env bash

set -eux

build_os="$(uname)"
arch="$(uname -m)"

# Dependency versions
boost_version="1.87.0"

install_boost() {
    curl -L "https://archives.boost.io/release/${boost_version}/source/boost_${boost_version//./_}.tar.bz2" | tar xj
    pushd "boost_${boost_version//./_}"
    
    ./bootstrap.sh
    ./b2 install \
        --with-serialization \
        --with-program_options \
        --prefix=/usr/local

    popd
    rm -rf "boost_${boost_version//./_}"
}

if [ "${build_os}" == "Linux" ]; then
    # Install base dependencies
    yum -y install \
        sudo \
        eigen3-devel \
        llvm-devel-18.* \
        clang-devel-18.* \
        bzip2 \
        wget \
        openssl-devel

    # Install Boost from source (yum versions are often too old)
    install_boost

    if [ "${arch}" == "aarch64" ]; then
        yum -y install gcc gcc-c++ libstdc++-devel
    fi

elif [ "${build_os}" == "Darwin" ]; then
    echo "Detected macOS. Installing dependencies..."
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        eigen \
        boost \
        yaml-cpp \
        llvm@18
fi
