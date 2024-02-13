#ifndef _KERNELS_CU_
#define _KERNELS_CU_


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
    for ( int i = 0; i < N; i++ )
        for ( int j = 0; j < N; j++ ) // or start from j = i?
            result[i] = intersectRigidBodies( **rb, **rb, tr3d[i], tr3d[j] );
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
    // if ( cType == BOX )
    //     m_convex = new Box( a, b, c );
    // else ( cType == SUPERQUADRIC )
    //     m_convex = new Superquadric( a, b, c, 2., 3. );
    // m_Volume = m_convex->computeVolume();
    // m_convex->computeInertia( m_inertia, m_inertia_1 );
    // m_boundingVolume = m_convex->computeAABB();
    // m_circumscribedRadius = m_convex->computeCircumscribedRadius();
    Convex* cvx;
    if ( cType == BOX )
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
    Transform3d const& primaryParticleTr = tr3d[tid];
    // extern __shared__ Vec3d secondaryParticleCen[32];
    // for ( int j = 0; j < 32; j++ )
    //     secondaryParticleCen = cen[tid];
    // __syncthreads();
    for ( int j = 0; j < N; j++ )
        result[tid] = intersectRigidBodies( AA, AA, primaryParticleTr, tr3d[j] );
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
    if ( cType == BOX )
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
