#!/bin/bash
if [ -d lib64-${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION} ]
  then
    echo 'Xerces is built for' ${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}
  else
    echo 'Building Xerces for' ${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}
    cd src/xercesc
    make clean
    ./runConfigure -plinux -cgcc -x${GRAINS_CPP_COMPILER} -b64 -minmem -nsocket -tnative -rpthread
    make
    cd ../../
    cp -r lib lib64-${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}
    cd lib64-${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}
    rm libxerces-c.so libxerces-c.so.28 libxerces-depdom.so libxerces-depdom.so.28
    ln -s libxerces-c.so.28.0 libxerces-c.so.28
    ln -s libxerces-c.so.28 libxerces-c.so
    ln -s libxerces-depdom.so.28.0 libxerces-depdom.so.28          
    ln -s libxerces-depdom.so.28 libxerces-depdom.so
    cd ../
    rm -rf lib
fi
