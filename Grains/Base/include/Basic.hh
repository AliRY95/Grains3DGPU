#ifndef _BASIC_HH_
#define _BASIC_HH_

#include <float.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <new>
#include <chrono>
#include <string>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>


// =============================================================================
/** @brief Various constants and type definitions.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
/** @name Enumerations */
//@{    
/** @brief Space dimensions */
enum Direction 
{
    X, // x direction
    Y, // y direction
    Z, // z direction
    W, // scalar component of quaternions
    NONE // no direction
};
//@}


/** @name Constant macros */
//@{
#define long_string    255
#define DEGS_PER_RAD   57.29577951308232286465
#define RADS_PER_DEG   0.01745329251994329547
#define PI             3.14159265358979323846
#define TWO_PI         6.28318530717958623200
#define LOWEPS         1.e-6
#define EPSILON1       1.0e-10
#define EPSILON2       1.0e-15
#define HIGHEPS        1.0e-20
#define TIMEFORMAT     10
#define POSITIONFORMAT 16
//@}


/** @name Compiler macros */
//@{
#ifdef __NVCC__
    #define __HOST__          __host__
    #define __DEVICE__        __device__
    #define __HOSTDEVICE__    __host__ __device__
    #define INLINE            __forceinline__
    #define __RESTRICT__      __restrict__
#else
    #define __HOST__          
    #define __DEVICE__        __device__
    #define __HOSTDEVICE__
    #define INLINE            inline
    #define __RESTRICT__      restrict
#endif
//@}


/** @name CUDA error handling */
//@{
// Macro for outputting CUDA errors
#define cudaErrCheck( ans ) { cudaAssert( ( ans ), __FILE__, __LINE__ ); }

/** @brief Returns CUDA error */
static INLINE void cudaAssert( cudaError_t code, 
                               const char *file,
                               int line,
                               bool abort = false )
{
   if ( code != cudaSuccess )
   {
      fprintf( stderr, 
               "GPUassert: %s %s %d\n", 
               cudaGetErrorString( code ), 
               file, 
               line );
      if ( abort )
        exit( code );
   }
}
//@}


/** @name Structs */
//@{    
/** @brief Struct for 3D usngined integers, mostly used in LinkedCell */
// struct uint3
// {
//   unsigned int x; // x component
//   unsigned int y; // y component
//   unsigned int z; // z component
// };
//@}


#endif