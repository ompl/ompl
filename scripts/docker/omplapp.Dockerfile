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
        freeglut3-dev \
        git \
        libassimp-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libccd-dev \
        libeigen3-dev \
        libexpat1 \
        libfcl-dev \
        libtriangle-dev \
        libyaml-cpp-dev \
        ninja-build \
        pkg-config \
        python3-celery \
        python3-dev \
        python3-flask \
        python3-numpy \
        python3-opengl \
        python3-pip \
        python3-pyqt5.qtopengl \
        pypy3 \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    pip3 install --break-system-packages pygccxml pyplusplus PyOpenGL-accelerate
COPY . /omplapp
WORKDIR /omplapp
RUN cmake \
        -G Ninja \
        -B build \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -G Ninja \
        /omplapp && \
    cmake --build build -t update_bindings && \
    cmake --build build && \
    cmake --install build

FROM ubuntu:noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        clang \
        cmake \
        freeglut3-dev \
        git \
        libassimp-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libccd-dev \
        libeigen3-dev \
        libfcl-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python-celery-common \
        python3-celery \
        python3-dev \
        python3-flask \
        python3-numpy \
        python3-opengl \
        python3-pip \
        python3-pyqt5.qtopengl \
        uwsgi-plugin-python3 \
        wget && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    # install PyOpenGL
    pip install --break-system-packages PyOpenGL-accelerate

COPY --from=builder /usr /usr
RUN useradd -ms /bin/bash ompl
USER ompl
WORKDIR /home/ompl
