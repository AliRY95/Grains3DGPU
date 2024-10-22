# Definition
# Grains
export GRAINS_HOME=${HOME}/Desktop/Work/Codes/GrainsGPU
export GRAINS_ROOT=${GRAINS_HOME}/Grains
# End Grains


# CPU
export GRAINS_CPP_COMPILER=g++
export GRAINS_CPP_COMPILER_DIST="GNU"
export GRAINS_CPP_COMPILER_VERSION="13.2.0"
# End CPU


# GPU
export GRAINS_GPU_COMPILER=nvcc
export GRAINS_GPU_COMPILER_DIST="CUDA"
export GRAINS_GPU_COMPILER_VERSION="12.6.0"
export GRAINS_GPU_COMPILER_ROOT=/usr/local/cuda-12.6
export GRAINS_GPU_COMPILER_INCDIR="${GRAINS_GPU_COMPILER_ROOT}/include"
export GRAINS_GPU_COMPILER_BINDIR="${GRAINS_GPU_COMPILER_ROOT}/bin"
export GRAINS_GPU_COMPILER_LIBDIR="${GRAINS_GPU_COMPILER_ROOT}/lib64"
# End GPU


# Xerces
export GRAINS_XERCES_ROOT=${GRAINS_HOME}/XERCES-2.8.0
export GRAINS_XERCES_INCDIR="${GRAINS_XERCES_ROOT}/include"
export GRAINS_XERCES_BINDIR="${GRAINS_XERCES_ROOT}/bin"
export GRAINS_XERCES_LIBDIR="${GRAINS_XERCES_ROOT}/lib64-${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}"
# End Xerces


# Full extension
export GRAINS_FULL_EXT=${GRAINS_CPP_COMPILER_DIST}-${GRAINS_CPP_COMPILER_VERSION}-${GRAINS_GPU_COMPILER_DIST}-${GRAINS_GPU_COMPILER_VERSION}
# End Full extension


# Display
echo -e '\033[31mGRAINS_HOME\033[0m =' $GRAINS_HOME
echo -e '\033[31mGRAINS_CPP_COMPILER\033[0m =' $GRAINS_CPP_COMPILER
echo -e '\033[31mGRAINS_CPP_COMPILER_DIST\033[0m =' $GRAINS_CPP_COMPILER_DIST
echo -e '\033[31mGRAINS_CPP_COMPILER_VERSION\033[0m =' $GRAINS_CPP_COMPILER_VERSION
echo -e '\033[31mGRAINS_GPU_COMPILER\033[0m =' $GRAINS_GPU_COMPILER
echo -e '\033[31mGRAINS_GPU_COMPILER_DIST\033[0m =' $GRAINS_GPU_COMPILER_DIST
echo -e '\033[31mGRAINS_GPU_COMPILER_VERSION\033[0m =' $GRAINS_GPU_COMPILER_VERSION
echo -e '\033[31mGRAINS_GPU_COMPILER_ROOT\033[0m =' $GRAINS_GPU_COMPILER_ROOT
echo -e '\033[31mGRAINS_FULL_EXT\033[0m =' $GRAINS_FULL_EXT
echo -e '\033[31mXERCES_ROOT\033[0m =' $GRAINS_XERCES_ROOT


# Compilers
export GRAINS_CPP_COMPILER_FLAGS="-m64 -O3 -fPIC -std=c++20 \
    -Wno-ctor-dtor-privacy \
    -Wall -Wextra -Wconversion -Wshadow -Wpedantic -Wwrite-strings \
    -fmax-errors=8 \
    -g"
export GRAINS_CPP_LINKER_FLAGS="${GRAINS_CPP_COMPILER_FLAGS} -shared"
###########
export GRAINS_GPU_COMPILER="${GRAINS_GPU_COMPILER_BINDIR}/${GRAINS_GPU_COMPILER}"
export GRAINS_GPU_LINKER="${GRAINS_GPU_COMPILER_BINDIR}/${GRAINS_GPU_COMPILER}"
export GRAINS_GPU_COMPILER_FLAGS="-t=8 -x cu -m64 -O3 -dlto -dc \
    -std=c++20 -arch=sm_75 -lineinfo \
    -cudart static -cudadevrt static \
    -maxrregcount=128 -use_fast_math -extra-device-vectorization -restrict \
    -Xcompiler "-rdynamic,-fPIC,-fopenmp" \
    -g -diag-suppress 554"
export GRAINS_GPU_LINKER_FLAGS="-O3 -dlto -arch=sm_75 -lineinfo -lcudart \
    -maxrregcount=128 -use_fast_math -extra-device-vectorization -restrict \
    -lcudart -lcudadevrt \
    -lgomp \
    -g"
###########
export GRAINS_XERCES_FLAGS="-L${GRAINS_XERCES_LIBDIR} -lxerces-c -lxerces-depdom"
###########
export GRAINS_Z_LIB="/usr/lib64"
export GRAINS_Z_FLAGS="-L${GRAINS_Z_LIB} -lz"
# End Flags


# LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GRAINS_XERCES_LIBDIR}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GRAINS_ROOT}/lib
# End LD_LIBRARY_PATH

# Compatibilty for Xerces
source $GRAINS_HOME/Env/grains_xerces.env.sh