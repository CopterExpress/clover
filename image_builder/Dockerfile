# for alpine use apk

FROM debian

ENV DEBIAN_FRONTEND 'noninteractive'
ENV LANG 'C.UTF-8'
ENV LC_ALL 'C.UTF-8'

RUN apt-get update -qq > /dev/null
RUN apt-get install -y --no-install-recommends -qq jq unzip wget parted apt-utils git ca-certificates > /dev/null
RUN apt-get clean

COPY ./image_config.sh /builder/image_config.sh
COPY ./autobuild.sh /builder/autobuild.sh
COPY ./autosizer.sh /builder/autosizer.sh
COPY ./qemu-arm-resin /builder/qemu-arm-resin

VOLUME image/ /builder/image

WORKDIR /builder
CMD ./autobuild.sh
