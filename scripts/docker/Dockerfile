FROM ubuntu:noble AS builder
# avoid interactive configuration dialog from tzdata, which gets pulled in
# as a dependency
ENV DEBIAN_FRONTEND=noninteractive
ENV CXX=clang++
RUN apt-get update && \
    apt-get install -y \
        castxml \
        clang \
        cmake \
        git \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-test-dev \
        libeigen3-dev \
        libexpat1 \
        libflann-dev \
        libtriangle-dev \
        libyaml-cpp-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        pypy3 \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    pip3 install --break-system-packages pygccxml pyplusplus
COPY . /ompl
WORKDIR /ompl
RUN git submodule update --init --recursive && \
    cmake \
        -G Ninja \
        -B build \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DVAMP_PORTABLE_BUILD=ON \
        -DCMAKE_INSTALL_PREFIX=/usr && \
    cmake --build build -t update_bindings && \
    cmake --build build && \
    cmake --install build && \
    cd tests/cmake_export && \
    cmake -B build -DCMAKE_INSTALL_PREFIX=../../install && \
    cmake --build build

FROM ubuntu:noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libeigen3-dev \
        libflann-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-numpy \
        python3-pip \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev

COPY --from=builder /usr /usr
RUN useradd -ms /bin/bash ompl
USER ompl
WORKDIR /home/ompl
