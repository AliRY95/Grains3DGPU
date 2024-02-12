#ifndef _KERNELS_CU_
#define _KERNELS_CU_

#include "Transform3.hh"
#include "Convex.hh"
#include "Box.hh"

// -----------------------------------------------------------------------------
namespace GrainsCPU
{
// ...
void collisionDetectionGJK( Convex const* a,
                            Transform3d const* tr3d,
                            bool* result,
                            int const N )
{
    for ( int i = 0; i < N; i++ )
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            result[i] = intersectGJK( *a, *a, tr3d[i], tr3d[j] );
};
} // GrainsCPU namespace end




// -----------------------------------------------------------------------------
// ...
namespace GrainsGPU
{
__global__ void collisionDetectionGJK( Convex** a,
                                       Transform3d const* tr3d,
                                       bool* result,
                                       int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    Convex const& AA = **a;
    Transform3d const& primaryParticleTr = tr3d[tid];
    // extern __shared__ Vec3d secondaryParticleCen[32];
    // for ( int j = 0; j < 32; j++ )
    //     secondaryParticleCen = cen[tid];
    __syncthreads();
    for ( int j = 0; j < N; j++ )
        result[tid] = intersectGJK( AA, AA, primaryParticleTr, tr3d[j] );
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
