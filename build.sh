#!/bin/sh
set -e
build_type="Ninja"

rm -rf build_release
mkdir -p build_release
cd build_release
cmake -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Release ..
ninja
cd ..
