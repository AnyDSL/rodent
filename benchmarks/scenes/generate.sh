#!/bin/bash
ARTY=/space/perard/sources/arty/build/src/arty
cd sponza
export ARTY_SCENE_SIZE=4000
export ARTY_AO_FACTOR=0.8
export ARTY_AO_OFFSET=0.1
${ARTY} sponza.yml -a 4 -s 75 -o sponza-ao.png
${ARTY} sponza.yml -a 1 -s 10 -o sponza-pt.png
cd ..
cd crown
export ARTY_SCENE_SIZE=200
export ARTY_AO_FACTOR=0.9
export ARTY_AO_OFFSET=0.0
${ARTY} crown.yml -a 4 -s 75 -o crown-ao.png
${ARTY} crown.yml -a 1 -s 10 -o crown-pt.png
cd ..
cd san-miguel
export ARTY_SCENE_SIZE=50
export ARTY_AO_FACTOR=0.9
export ARTY_AO_OFFSET=0.0
${ARTY} san-miguel.yml -a 4 -s 75 -o san-miguel-ao.png
${ARTY} san-miguel.yml -a 1 -s 10 -o san-miguel-pt.png
cd ..
cd powerplant
export ARTY_SCENE_SIZE=200000
export ARTY_AO_FACTOR=0.9
export ARTY_AO_OFFSET=0.0
${ARTY} powerplant.yml -a 4 -s 75 -o powerplant-ao.png
${ARTY} powerplant.yml -a 1 -s 10 -o powerplant-pt.png
cd ..
