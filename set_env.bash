#!/bin/bash
export KINETIC_LIB_DIR=$(pwd)/extern/lib
export KINETIC_INCLUDE_DIR=$(pwd)/extern/include
export LD_LIBRARY_PATH=$KINETIC_LIB_DIR:$(pwd)/install/lib:/usr/local/lib:$LD_LIBRARY_PATH
#for f in (ls ${KINETIC_LIB_DIR})
#do
# if [ -d ${KINETIC_LIB_DIR}/$f ];then
#   export LD_LIBRARY_PATH=${KINETIC_LIB_DIR}/$f:$LD_LIBRARY_PATH
# fi
#done
export LD_LIBRARY_PATH=$KINETIC_LIB_DIR:$KINETIC_LIB_DIR/boost:$KINETIC_LIB_DIR/urdfdom:$KINETIC_LIB_DIR/tinyxml2:$KINETIC_LIB_DIR/assimp:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$KINETIC_LIB_DIR/console_bridge:$KINETIC_LIB_DIR/fcl:$KINETIC_LIB_DIR/icu:$KINETIC_LIB_DIR/log4cxx:$KINETIC_LIB_DIR/poco:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$KINETIC_LIB_DIR/python:$KINETIC_LIB_DIR/minizip:$KINETIC_LIB_DIR/apr:$KINETIC_LIB_DIR/minizip:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$KINETIC_LIB_DIR/ccd:$LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=$KINETIC_LIB_DIR/curl:$KINETIC_LIB_DIR/ssl:$KINETIC_LIB_DIR/crypto:$LD_LIBRARY_PATH
#conda install --yes --file requirements.txt