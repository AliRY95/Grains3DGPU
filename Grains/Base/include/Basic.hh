#ifndef _BASIC_HH_
#define _BASIC_HH_

#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1
#include <bits/stdc++.h>
#include <float.h>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include <istream>
#include <list>
#include <cmath>
#include <limits>
#include <new>
#include <chrono>
#include <string>
#include <ios>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>


// =============================================================================
/** @brief Various constants and type definitions.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
/** @name Compiler macros */
//@{
#ifdef __NVCC__
    #define __HOST__          __host__
    #define __DEVICE__        __device__
    #define __HOSTDEVICE__    __host__ __device__
    #define __GLOBAL__        __global__
    #define INLINE            __forceinline__
    #define __RESTRICT__      __restrict__
#else
    #define __HOST__          
    #define __DEVICE__        __device__
    #define __HOSTDEVICE__
    #define __GLOBAL__        
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


/** @name Constant expressions at compile-time */
//@{
/** \brief PI value */
template <class T> constexpr T PI = T( 3.1415926535897932385L );
/** \brief 2*PI value */
template <class T> constexpr T TWO_PI = T( 6.28318530717958623200L );
/** \brief Degree per radian value */
template <class T> constexpr T DEGS_PER_RAD = T( 57.29577951308232286465L );
/** \brief Radian per degree value */
template <class T> constexpr T RADS_PER_DEG = T( 0.01745329251994329547L );
/** \brief High (Machine) epsilon value */
template <class T> constexpr T HIGHEPS = T( 1.e-15 );
template <> constexpr float HIGHEPS<float> = float( 1.e-08 );
/** \brief Epsilon value */
template <class T> constexpr T EPS = T( 1.e-10 );
template <> constexpr float EPS<float> = float( 1.e-05 );
/** \brief Low epsilon value */
template <class T> constexpr T LOWEPS = T( 1.e-05 );
template <> constexpr float LOWEPS<float> = float( 1.e-03 );
// /** \brief Function to mimic tab in stderr */
// constexpr std::string shiftString( int n )
// {
//     std::string out = "";
//     for ( int i = 0; i < n; ++i )
//         out += " ";
//     return ( out );
// }
#define shiftString0   "" 
#define shiftString1   " "
#define shiftString2   "  "
#define shiftString3   "   "
#define shiftString6   "      "
#define shiftString9   "         "
#define shiftString12  "            "
#define shiftString15  "               "
#define zeroVector3T   Vector3<T>( T( 0 ), T( 0 ), T( 0 ) )
#define zeroVector3D   Vector3D( 0., 0., 0. )
#define zeroVector3F   Vector3F( 0.f, 0.f, 0.f )
#define noContact      ContactInfo<T>( zeroVector3T, zeroVector3T, T( 0 ) )
//@}


#endif