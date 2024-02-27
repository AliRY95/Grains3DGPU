#ifndef _BASIC_HH_
#define _BASIC_HH_

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


// ============================================================================
/** @brief Various constants and type definitions.

    @author A.Yazdani - 2023 - Construction */
// ============================================================================
// Macro for outputting CUDA errors
#define cudaErrCheck( ans ) { cudaAssert( ( ans ), __FILE__, __LINE__ ); }

// Macros
#define EPSILON1 1.0e-10
#define EPSILON2 1.0e-15
#define EPSILON3 1.0e-20

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


// /**@name Constants */
// //@{
// const int long_string = 255; /**< long string size */
// const double DEGS_PER_RAD = 57.29577951308232286465; /**< degree per 
//   radian */
// const double RADS_PER_DEG =  0.01745329251994329547; /**< radian per 
//   degree */
// const double PI = 3.14159265358979323846; /**< pi number */
// const double TWO_PI = 6.28318530717958623200; /**< 2 times pi number */
// const double LOWEPS = 1.0e-6; /**< very low precision approximation 
//   constant */
// const double EPSILON = 1.0e-10; /**< low precision approximation constant */
// const double EPSILON2 = 1.0e-15; /**< high precision approximation 
//   constant */
// const double EPSILON3 = 1.0e-20; /**< very high precision approximation 
//   constant */
// const int TIMEFORMAT = 10; /**< number of significant digits after the
//   decimal point to write time in high precision format */
// const int POSITIONFORMAT = 16; /**< number of significant digits after the
//   decimal point to write component position in high precision format */
// //@}


/** @name CUDA Basic methods */
//@{
/** @brief Returns whether a real number is approximately 0 with respect to
EPSILON1 
@param x the real number */
__host__ __device__ inline bool eqz( double x )
{ 
  return ( fabs(x) <= EPSILON1 );
}

// /** @brief Returns the minimum of 2 real numbers defined as double
// @param x 1st real number 
// @param y 2nd real number */    
// __host__ __device__ inline double min( double x, double y ) 
// { 
//   return ( x > y ? y : x );
// }

// /** @brief Returns the maximum of 2 real numbers defined as double
// @param x 1st real number 
// @param y 2nd real number */      
// __host__ __device__ inline double max( double x, double y ) 
// { 
//   return ( x < y ? y : x );
// }

/** @brief Sests the minimum of 2 real numbers defined as double to these 2
numbers
@param x 1st real number 
@param y 2nd real number */     
__host__ __device__ inline void set_min( double& x, double y )
{ 
  if (x > y) x = y;
}

/** @brief Sests the maximum of 2 real numbers defined as double to these 2
numbers
@param x 1st real number 
@param y 2nd real number */ 
__host__ __device__ inline void set_max( double& x, double y )
{ 
  if (x < y) x = y; 
}

// /** @brief Returns an angle in radians given an angle in degrees
// @param x angle in degrees */
// __host__ __device__ inline double rads( double x ) 
// { 
//   return ( x * RADS_PER_DEG ); 
// }

// /** @brief Returns an angle in degrees given an angle in radians
// @param x angle in radians */
// __host__ __device__ inline double degs( double x ) 
// { 
//   return ( x * DEGS_PER_RAD );
// }

/** @brief Returns CUDA error */
inline void cudaAssert( cudaError_t code, 
                        const char *file,
                        int line,
                        bool abort = false )
{
   if ( code != cudaSuccess )
   {
      fprintf( stderr, 
               "GPUassert: %s %s %d\n", cudaGetErrorString( code ), file, line );
      if ( abort )
        exit( code );
   }
}
//@}


#endif