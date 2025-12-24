#!/bin/bash
set -e  # 에러 발생 시 즉시 스크립트 중단

apt update
apt install -y libpoco-dev libeigen3-dev libfmt-dev


sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio

sudo apt-get remove "*libfranka*"

LIBFRANKA_VERSION="0.15.0"
git config --global http.sslVerify false
git clone --recurse-submodules https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout ${LIBFRANKA_VERSION}
git submodule update

mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make
cpack -G DEB
sudo dpkg -i libfranka*.deb

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openrobots/lib" >> ~/.bashrc