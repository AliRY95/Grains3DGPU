#ifndef _KERNELS_CUH_
#define _KERNELS_CUH_

#include "Vector3.cu"
#include "Convex.cu"


// -----------------------------------------------------------------------------
namespace GrainsCPU
{
// ...
template <typename T>
void collisionDetectionGJKwithAABB( Convex<T> const* a,
                                    Vector3<T> const* cen,
                                    bool* result,
                                    int const N )
{
    for ( int i = 0; i < N; i++ )
        // for ( int j = i - 32; j < i + 32 && j > 0 && j < N; j++ ) // or start from j = i?
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            // result[i] = intersectAABB( a, a, cen[i], cen[j] );
            if ( intersectAABB( a, a, cen[i], cen[j] ) )
                intersectGJK( a, a, cen[i], cen[j] );
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
void collisionDetectionGJK( Convex<T> const* a,
                            Vector3<T> const* cen,
                            bool* result,
                            int const N )
{
    for ( int i = 0; i < N; i++ )
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            result[i] = intersectGJK( a, a, cen[i], cen[j] );
            // result[N * i + j] = intersect( a, a, cen[i], cen[j] );
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
void collisionDetectionSpherical( T const r1,
                                  Vector3<T> const* cen,
                                  bool* result,
                                  int const N )
{
    for ( int i = 0; i < N; i++ )
        for ( int j = 0; j < N; j++ )
            result[i] = ( ( cen[i] - cen[j] ).Norm() <= 2. * r1 );
            // result[N * i + j] = ( ( cen[i] - cen[j] ).Norm() <= 2. * r1 );
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
void collisionDetectionAABB( Vector3<T> const* box,
                             Vector3<T> const* cen,
                             bool* result,
                             int const N )
{
    for( int i = 0; i < N; i++ )
    {
        for( int j = 0; j < N; j++ )
        {
            if ( fabs( cen[i][X] - cen[j][X] ) > ( box[0][X] + box[0][X] ) )
                result[i] = false;
                // result[N * i + j] = false;
            else if ( fabs( cen[i][Y] - cen[j][Y] ) > ( box[0][Y] + box[0][Y] ) )
                result[i] = false;
                // result[N * i + j] = false;
            else if ( fabs( cen[i][Z] - cen[j][Z] ) > ( box[0][Z] + box[0][Z] ) )
                result[i] = false;
                // result[N * i + j] = false;
            else // We have an overlap
                result[i] = true;
                // result[N * i + j] = true;
        }
    }
};
} // GrainsCPU namespace end




// -----------------------------------------------------------------------------
// ...
namespace GrainsGPU
{
// ...
template <typename T>
__global__ void collisionDetectionGJKwithAABB( Convex<T> const* __restrict__ a,
                                               Vector3<T> const* __restrict__ cen,
                                               bool* result,
                                               int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;
    
    Vector3<T> primaryParticleCen = cen[tid];
    Vector3<T> secondaryParticleCen;
    for ( int j = 0; j < N; j++ )
    // for ( int j = tid - 32; j < tid + 32 && j > 0 && j < N; j++ )
    {
        secondaryParticleCen = cen[j];
        if ( intersectAABB( a, a, primaryParticleCen, secondaryParticleCen ) )
            intersectGJK( a, a, primaryParticleCen, secondaryParticleCen );
    }
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
__global__ void collisionDetectionGJK( Convex<T> const* a,
                                       Vector3<T> const* cen,
                                       bool* result,
                                       int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    for ( int j = 0; j < N; j++ )
        result[tid] = intersectGJK( a, a, cen[tid], cen[j] );
        // result[N * tid + j] = intersect( a, a, cen[tid], cen[j] );
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
__global__ void collisionDetectionSpherical( T const r1,
                                             Vector3<T> const* cen,
                                             bool* result,
                                             int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    for ( int j = 0; j < N; j++ )
        result[tid] = ( ( cen[tid] - cen[j] ).Norm() <= 2. * r1 );
        // result[N * tid + j] = ( ( cen[tid] - cen[j] ).Norm() <= 2. * r1 );
};




// -----------------------------------------------------------------------------
// ...
template <typename T>
__global__ void collisionDetectionAABB( Vector3<T> const* box,
                                        Vector3<T> const* cen,
                                        bool* result,
                                        int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;
    
    for ( int j = 0; j < N; j++ )
    {
        if ( fabs( cen[tid][X] - cen[j][X] ) > ( box[0][X] + box[0][X] ) )
            result[tid] = false;
            // result[N * tid + j] = false;
        else if ( fabs( cen[tid][Y] - cen[j][Y] ) > ( box[0][Y] + box[0][Y] ) )
            result[tid] = false;
            // result[N * tid + j] = false;
        else if ( fabs( cen[tid][Z] - cen[j][Z] ) > ( box[0][Z] + box[0][Z] ) )
            result[tid] = false;
            // result[N * tid + j] = false;
        else // We have an overlap
            result[tid] = true;
            // result[N * tid + j] = true;
    }
};
} // GrainsGPU namespace end




// -----------------------------------------------------------------------------
template <typename T>
__global__ void setupConvex( T x, T y, T z, Convex<T>* convex )
{
    // convex->setExtent( x, y, z );
    *convex = *(new Convex( x, y, z ));
    // convex( x, y, z);
};


#endif
