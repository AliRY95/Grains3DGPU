#ifndef _KERNELS_CU_
#define _KERNELS_CU_

#include "Vector3.hh"
#include "Convex.hh"
#include "Box.hh"

// -----------------------------------------------------------------------------
namespace GrainsCPU
{
// ...
void collisionDetectionGJK( Convex const* a,
                            Vec3d const* cen,
                            bool* result,
                            int const N )
{
    for ( int i = 0; i < N; i++ )
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            result[i] = intersectGJK( a, a, cen[i], cen[j] );
};
} // GrainsCPU namespace end




// -----------------------------------------------------------------------------
// ...
namespace GrainsGPU
{
__global__ void collisionDetectionGJK( Convex** a,
                                       Vec3d const* cen,
                                       bool* result,
                                       int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    // Vec3d const& primaryParticleCen = cen[tid];
    for ( int j = 0; j < N; j++ )
        result[tid] = intersectGJK( *a, *a, cen[tid], cen[j] );
};
} // GrainsGPU namespace end




// -----------------------------------------------------------------------------
__global__ void setupConvex( double x, double y, double z, Convex** d_convex )
{
    // convex->setExtent( x, y, z );
    (*d_convex) = (new Box( x, y, z ));
    // convex( x, y, z);
};

#endif
