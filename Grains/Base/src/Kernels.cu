#ifndef _KERNELS_CU_
#define _KERNELS_CU_

#include "Sphere.hh"
#include "Box.hh"
#include "Superquadric.hh"
#include "Convex.hh"
#include "Transform3.hh"
#include "RigidBody.hh"


namespace GrainsCPU {
// -----------------------------------------------------------------------------
// ...
void collisionDetectionGJK( RigidBody const* const* rb,
                            Transform3d const* tr3d,
                            bool* result,
                            int const N )
{
    RigidBody const& AA = **rb;
    for ( int i = 0; i < N; i++ )
    {
        Transform3d trA = tr3d[i];
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            result[i] = intersectRigidBodies( AA, AA, trA, tr3d[j] );
    }
    // RigidBody const& AA = **rb;
    // for ( int i = 0; i < N; i++ )
    // {
    //     Transform3d trA = tr3d[i];
    //     Transform3d trB2A;
    //     for ( int j = 0; j < N; j++ ) // or start from j = i?
    //     {
    //         trB2A = tr3d[j];
    //         trB2A.relativeToTransform( trA );
    //         result[i] = intersectRigidBodies( AA, AA, trB2A );
    //     }
    // }
};

// -----------------------------------------------------------------------------
// ...
void setupRigidBody( ConvexType cType,
                     double a,
                     double b,
                     double c,
                     double ct,
                     RigidBody** rb )
{
    Convex* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere( a );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box( a, b, c );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric( a, b, c, 2., 3. );
        *rb = new RigidBody( cvx, ct );
    }
};
} // GrainsCPU namespace end




// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
namespace GrainsGPU
{
// ...
__global__ void collisionDetectionGJK( RigidBody const* const* a,
                                       Transform3d const* tr3d,
                                       bool* result,
                                       int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    RigidBody const& AA = **a;
    Transform3d trA = tr3d[tid];
    for ( int j = 0; j < N; j++ )
        result[tid] = intersectRigidBodies( AA, AA, trA, tr3d[j] );
    // int bid = gridDim.x * gridDim.y * blockIdx.z + 
    //           blockIdx.y * gridDim.x + 
    //           blockIdx.x;
    // int tid = bid * blockDim.x + threadIdx.x;

    // RigidBody const& AA = **a;
    // Transform3d trA = tr3d[tid];
    // Transform3d trB2A;
    // for ( int j = 0; j < N; j++ )
    // {
    //     trB2A = tr3d[j];
    //     trB2A.relativeToTransform( trA );
    //     result[tid] = intersectRigidBodies( AA, AA, trB2A );
    //     // result[tid] = intersectRigidBodies( AA, AA, trA, tr3d[j] );
    // }
};




// -----------------------------------------------------------------------------
// ...
__global__ void setupRigidBody( ConvexType cType,
                                double a,
                                double b,
                                double c,
                                double ct,
                                RigidBody** rb )
{
    Convex* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere( a );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box( a, b, c );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric( a, b, c, 2., 3. );
        *rb = new RigidBody( cvx, ct );
    }
};
} // GrainsGPU namespace end


#endif
