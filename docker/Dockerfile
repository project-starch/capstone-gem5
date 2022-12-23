FROM ubuntu:22.04

ENV TZ=Asia/Singapore

RUN apt-get update
RUN apt-get install -y gcc g++ scons mold wget make git tar
RUN mkdir -p /toolchain
ADD setup-toolchain.sh /toolchain
RUN cd /toolchain && bash setup-toolchain.sh
RUN rm /toolchain/setup-toolchain.sh

RUN apt-get install -y apt-utils m4 python-is-python3
RUN apt-get install -y python-dev-is-python3

ADD entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

