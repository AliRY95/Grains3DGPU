#!/bin/bash
export GRAINS_BITS_DEFAULT=64
export GRAINS_SERCOMPIL_ENV=${GRAINS_CPP_COMPILER_DIST}
export GRAINS_SERCOMPIL_VERSION=${GRAINS_CPP_COMPILER_VERSION}
export GRAINS_C=gcc
export GRAINS_CPP=${GRAINS_CPP_COMPILER}
export XERCESCROOT=${GRAINS_XERCES_ROOT}
export XERCESC_ROOT=${XERCESCROOT}
# We basically rename some variables here. The bash comes from the previous
# version of Grains, and we do this to make it compatible with XERSESC which is
# configured for the previous version of Grains
