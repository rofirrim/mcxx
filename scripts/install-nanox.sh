#!/bin/bash
set -ex

wget http://rofi.roger-ferrer.org/apt/mirror/pm.bsc.es/nanox-0.14a.tar.gz
tar xf nanox-0.14a.tar.gz
cd nanox-0.14a
./configure --prefix=$HOME/nanox-install CC=gcc-6 CPP=gcc-6 CCAS=gcc-6 CXX=g++-6
make -j$(nproc)
make install
