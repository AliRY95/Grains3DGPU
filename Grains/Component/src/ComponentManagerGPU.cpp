#include <curand.h>

#include "ComponentManager.hh"
#include "ComponentManagerGPU.hh"

#include "GrainsParameters.hh"


#define numComponents (GrainsParameters::m_numComponents)
// -----------------------------------------------------------------------------
// Constructor with the number of particles randomly positioned in the 
// computational domain
// TODO: Use curand to generate random arrays on device
ComponentManagerGPU::ComponentManagerGPU()
{
    // Allocating memory on host
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsId,
                              numComponents * numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_rigidBodyId,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentId,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsCount,
                              numComponents * sizeof( Transform3d ) ) );

        // // Randomize array on host
        // for( int i = 0; i < numComponents; i++ )
        // {
        //     double aX = angle( generator );
        //     double aY = angle( generator );
        //     double aZ = angle( generator );
        //     tr[i].setBasis( aX, aY, aZ );
        //     tr[i].setOrigin( Vec3d( location( generator ),
        //                             location( generator ),
        //                             location( generator ) ) );
        // }
}




// -----------------------------------------------------------------------------
// Constructor with the number of particles randomly positioned in the 
// computational domain
ComponentManagerGPU::ComponentManagerGPU( ComponentManager const& cm )
{
    // Allocating memory on host
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsId,
                              numComponents * numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_rigidBodyId,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentId,
                              numComponents * sizeof( Transform3d ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsCount,
                              numComponents * sizeof( Transform3d ) ) );

    // Copying the arrays from host to device
    setTransform( cm.getTransform() );
    setNeighborsId( cm.getNeighborsId() );
    setRigidBodyId( cm.getRigidBodyId() );
    setComponentId( cm.getComponentId() );
    setNeighborsCount( cm.getNeighborsCount() );
}




// -----------------------------------------------------------------------------
// Destructor
ComponentManagerGPU::~ComponentManagerGPU()
{
    cudaFree( m_transform );
    cudaFree( m_neighborsId );
    cudaFree( m_rigidBodyId );
    cudaFree( m_componentId );
    cudaFree( m_neighborsCount );
}




// -----------------------------------------------------------------------------
// Gets components transformation
Transform3d* ComponentManagerGPU::getTransform() const
{
    Transform3d* h_transform = new Transform3d[numComponents];
    cudaErrCheck( cudaMemcpy( h_transform,
                              m_transform,
                              numComponents * sizeof( Transform3d ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_transform );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor Id
unsigned int* ComponentManagerGPU::getNeighborsId() const
{
    unsigned int* h_neighborsId = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_neighborsId,
                              m_neighborsId,
                              numComponents * numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the array of components rigid body Id
unsigned int* ComponentManagerGPU::getRigidBodyId() const
{
    unsigned int* h_rigidBodyId = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_rigidBodyId,
                              m_rigidBodyId,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
unsigned int* ComponentManagerGPU::getComponentId() const
{
    unsigned int* h_componentId = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_componentId,
                              m_componentId,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_componentId );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor count
unsigned int* ComponentManagerGPU::getNeighborsCount() const
{
    unsigned int* h_neighborsCount = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_neighborsCount,
                              m_neighborsCount,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_neighborsCount );
}




// -----------------------------------------------------------------------------
// Sets components transformation
void ComponentManagerGPU::setTransform( Transform3d const* tr )
{
    cudaErrCheck( cudaMemcpy( m_transform,
                              tr,
                              numComponents * sizeof( Transform3d ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
void ComponentManagerGPU::setNeighborsId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_neighborsId,
                              id,
                              numComponents * numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
void ComponentManagerGPU::setRigidBodyId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_rigidBodyId,
                              id,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
void ComponentManagerGPU::setComponentId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_componentId,
                              id,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor count
void ComponentManagerGPU::setNeighborsCount( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_neighborsCount,
                              id,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




/* -----------------------------------------------------------------------------
--------------------------------------------------------------------------------
----------------------------------------------------------------------------- */
// Collision Detection Kernel
__global__ void collisionDetectionGJK( RigidBody const* const* a,
                                       Transform3d const* tr3d,
                                       bool* result,
                                       int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;

    const RigidBody& AA = **a;
    const Transform3d& trA = tr3d[tid];
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
// Detects collision between particles
void ComponentManagerGPU::detectCollision( RigidBody const* const* rb, 
                                           bool* result )
{
    dim3 dimBlock( 256, 1, 1 );
    dim3 dimGrid( numComponents / 256, 1, 1 );
    collisionDetectionGJK<<< dimGrid, dimBlock >>> ( rb,
                                                     m_transform,
                                                     result,
                                                     numComponents );
}




/* -----------------------------------------------------------------------------
--------------------------------------------------------------------------------
----------------------------------------------------------------------------- */


#undef numComponents