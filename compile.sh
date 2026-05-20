#!/bin/bash

mkdir build

# build exec for cpp

cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release
make -C build -j4


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j

#build the map reduction test
cmake -S . -B build
cmake --build build --target map_reduction_test -j4