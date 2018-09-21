# for alpine use apk

FROM debian

ENV DEBIAN_FRONTEND 'noninteractive'
ENV LANG 'C.UTF-8'
ENV LC_ALL 'C.UTF-8'

RUN apt-get update -qq > /dev/null
RUN apt-get install -y --no-install-recommends -qq bc jq unzip wget parted apt-utils git ca-certificates gawk > /dev/null
RUN apt-get clean

COPY ./*.sh /builder/
COPY ./qemu-arm-resin /builder/qemu-arm-resin

WORKDIR /builder
CMD ./build.sh
