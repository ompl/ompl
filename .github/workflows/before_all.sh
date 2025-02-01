#!/usr/bin/env bash

set -eux

build_os="$(uname)"

if [ "${build_os}" == "Linux" ]; then
    # Instalar dependencias con yum
    yum -y install \
        sudo \
        eigen3 \
        llvm-devel \
        clang-devel \
        gcc-c++ \
        cmake \
        pkgconfig \
        boost-devel \
        eigen3-devel \
        ode-devel \
        wget \
        yaml-cpp-devel \
        python3-devel \
        python3-pip \
        boost-python3-devel \
        python3-numpy

    # manylinux ships with a pypy installation. Make it available on the $PATH
    # so the OMPL build process picks it up and can make use of it during the
    # Python binding generation stage.
    ln -s /opt/python/pp310-pypy310_pp73/bin/pypy /usr/bin
elif [ "${build_os}" == "Darwin" ]; then
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install \
        cmake \
        pkg-config \
        boost \
        eigen \
        ode \
        wget \
        yaml-cpp \
        pypy3 \
        llvm@18 \
        boost-python3 \
        freeglut \
        assimp \
        libccd
fi
