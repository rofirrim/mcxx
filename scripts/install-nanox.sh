#!/bin/bash
set -ex

## This script presumes that ccache is in the PATH

wget http://rofi.roger-ferrer.org/apt/mirror/pm.bsc.es/nanox-0.14a.tar.gz
tar xf nanox-0.14a.tar.gz
cd nanox-0.14a
./configure --prefix=$HOME/nanox-install CC="ccache gcc-6" CXX="ccache g++-6" --disable-debug --disable-instrumentation
make -j$(nproc)
make install
