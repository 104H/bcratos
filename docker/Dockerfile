FROM "mcr.microsoft.com/devcontainers/base:jammy"

WORKDIR /bcratos

#####
# Install software for Franka Emika Arm
# https://frankaemika.github.io/docs/installation_linux.html#building-from-source
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" \
    && apt-get -y install \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    libfmt-dev \
    libserial-dev \
    python3-pip \
    libboost-log-dev \
    lsb-release \
    curl

# install pinnochio
RUN mkdir -p /etc/apt/keyrings
RUN curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" \
    && apt-get -y install robotpkg-pinocchio

# install libfranka from source
RUN git clone --recursive https://github.com/frankaemika/libfranka \
    && cd libfranka \
    && git checkout 0.15.0 \
    && git submodule update \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF .. \
    && cmake --build . \
    && cpack -G DEB \
    && dpkg -i libfranka*.deb

RUN git clone https://github.com/fpagliughi/sockpp.git \
    && cd sockpp \
    && git checkout 15d8371 \
    && cmake -Bbuild . \
    && cmake --build build/ --target install \
    && cp build/libsockpp.so* /usr/lib/

RUN pip3 install pre-commit

ADD grabobject grabobject/

RUN useradd -ms /bin/bash developer
USER developer
