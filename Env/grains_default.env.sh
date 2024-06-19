# Definitions
export GRAINS_ROOT=${GRAINS_HOME}/Grains
# End Definitions


# Full extension
export GRAINS_FULL_EXT=${GRAINS_CUDA_DISTRIB}-${GRAINS_CUDA_VERSION}-${GRAINS_SERCOMPIL_ENV}-${GRAINS_SERCOMPIL_VERSION}
# End Full extension


# LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${XERCESC_ROOT}/lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GRAINS_ROOT}/lib
# End LD_LIBRARY_PATH


# Display
echo -e '\033[31mGRAINS_HOME\033[0m =' $GRAINS_HOME
echo -e '\033[31mGRAINS_CPP_COMPILER\033[0m =' $GRAINS_CPP_COMPILER
echo -e '\033[31mGRAINS_CPP_COMPILER_DIST\033[0m =' $GRAINS_CPP_COMPILER_DIST
echo -e '\033[31mGRAINS_CPP_COMPILER_VERSION\033[0m =' $GRAINS_CPP_COMPILER_VERSION
echo -e '\033[31mGRAINS_GPU_COMPILER\033[0m =' $GRAINS_GPU_COMPILER
echo -e '\033[31mGRAINS_GPU_COMPILER_DIST\033[0m =' $GRAINS_GPU_COMPILER_DISTRIB
echo -e '\033[31mGRAINS_GPU_COMPILER_VERSION\033[0m =' $GRAINS_GPU_COMPILER_VERSION
echo -e '\033[31mGRAINS_GPU_COMPILER_ROOT\033[0m =' $GRAINS_GPU_COMPILER_ROOT
echo -e '\033[31mGRAINS_FULL_EXT\033[0m =' $GRAINS_FULL_EXT
echo -e '\033[31mXERCESCROOT & XERCESC_ROOT\033[0m =' $XERCESCROOT '&' $XERCESC_ROOT


# General compilation flags - not displayed
#if [ $GRAINS_CPP = "g++" ]
if [ $GRAINS_SERCOMPIL_ENV = "GNU" ] 
then
  GCCFLAGS="-pedantic -W -Wno-long-long -Wno-ctor-dtor-privacy -Wno-unused-parameter -D_GLIBCXX_USE_CXX11_ABI=0 -std=c++11 "
fi
export GRAINS_GENCCFLAGS="-m${GRAINS_BITS_DEFAULT} ${GRAINS_COMPIL_OPT} -fPIC -Wall -Wwrite-strings -Wconversion -Wshadow -Wno-deprecated -Wno-comment ${GCCFLAGS}"
export GRAINS_MPICCFLAGS="-DMPICH_IGNORE_CXX_SEEK -DMPICH_SKIP_MPICXX -DOMPI_IGNORE_CXX_SEEK -DOMPI_SKIP_MPICXX"


# System include directories to generate dependencies 
GRAINS_SYSINC="${GRAINS_MPI_INCDIR} ${XERCESCROOT}/include"
# Next line depends on the operating system. Not mandatory to change, 
# unless you do not want makedepend to return 1000s of harmless warnings 
GRAINS_SYSINC="${GRAINS_SYSINC} /usr/include/c++/4.8.5  /usr/lib/gcc/x86_64-redhat-linux/4.8.5/include /usr/include/c++/4.8.2/x86_64-redhat-linux"
export GRAINS_SYSINC
