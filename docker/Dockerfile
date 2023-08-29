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
    libeigen3-dev

RUN git clone --recursive https://github.com/frankaemika/libfranka \
    && cd libfranka \
    && git checkout 0.10.0 \
    && git submodule update \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && cmake --build . \
    && cpack -G DEB \
    && dpkg -i libfranka*.deb

ADD grabobject grabobject/
# RUN cd grabobject \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make

RUN useradd -ms /bin/bash developer
USER developer