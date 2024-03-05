FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG TOOLS_PATH=/opt/gcc-arm-none-eabi
ARG ARM_VERSION=13.2.rel1
ARG ARM_HOST_ARCH=x86_64
ARG CONTAINER_USER=stm
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GCC_TOOLCHAIN_URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_VERSION}/binrel/arm-gnu-toolchain-${ARM_VERSION}-${ARM_HOST_ARCH}-arm-none-eabi.tar.xz"

RUN apt-get update && \
    apt-get install -y -q \
    git \
    cmake \
    ninja-build \
    openocd \
    stlink-tools \
    xz-utils \
    bzip2 \
    screen \
    curl

# Get ARM Toolchain
RUN mkdir ${TOOLS_PATH} \
       && curl -Lo gcc-arm-none-eabi.tar.xz ${GCC_TOOLCHAIN_URL} \
       && tar xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C ${TOOLS_PATH} \
       && rm gcc-arm-none-eabi.tar.xz
RUN rm ${TOOLS_PATH}/*.txt
RUN rm -rf ${TOOLS_PATH}/share/doc


# Add Toolchain to PATH
ENV PATH="$PATH:${TOOLS_PATH}/bin"

RUN groupadd --gid $USER_GID $CONTAINER_USER \
    && adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" ${CONTAINER_USER} \
    && usermod -a -G plugdev $CONTAINER_USER \
    && usermod -a -G dialout $CONTAINER_USER

USER ${CONTAINER_USER}
ENV USER=${CONTAINER_USER}
WORKDIR /home/${CONTAINER_USER}

RUN sed -i 's/@\\h//g' ~/.bashrc
# RUN sed -i 's/\\w\\/\\W\\/g' ~/.bashrc

CMD ["/bin/bash", "-c"]