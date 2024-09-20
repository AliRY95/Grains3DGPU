// #include <curand.h>
#include <cooperative_groups.h>
#include "thrust/device_ptr.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

#include "ComponentManagerGPU.hh"
#include "ComponentManagerGPU_Kernels.hh"
#include "LinkedCellGPUWrapper.hh"


// -----------------------------------------------------------------------------
// Constructor with the number of particles, number of obstacles, and number of
// cells. We only allocate memory in the constructor. Filling out the data
// members must be done separately.
template <typename T>
ComponentManagerGPU<T>::ComponentManagerGPU( unsigned int nParticles,
                                             unsigned int nObstacles,
                                             unsigned int nCells )
: m_nParticles( nParticles )
, m_nObstacles( nObstacles )
, m_nCells( nCells )
{
    // Allocating memory on host
    cudaErrCheck( cudaMalloc( (void**)&m_rigidBodyId,
                              m_nParticles * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              m_nParticles * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_velocity,
                              m_nParticles * sizeof( Kinematics<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_torce,
                              m_nParticles * sizeof( Torce<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentId,
                              m_nParticles * sizeof( int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentCellHash,
                              m_nParticles * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashStart,
                              ( m_nCells + 1 ) * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashEnd,
                              ( m_nCells + 1 ) * sizeof( unsigned int ) ) );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManagerGPU<T>::~ComponentManagerGPU()
{
    cudaFree( m_rigidBodyId );
    cudaFree( m_transform );
    cudaFree( m_velocity );
    cudaFree( m_torce );
    cudaFree( m_componentId );
    cudaFree( m_componentCellHash );
    cudaFree( m_cellHashStart );
    cudaFree( m_cellHashEnd );
    // cudaFree( m_neighborsCount );
    // cudaFree( m_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the number of particles in manager
template <typename T>
unsigned int ComponentManagerGPU<T>::getNumberOfParticles() const
{
    return( m_nParticles );
}




// -----------------------------------------------------------------------------
// Gets the number of obstacles in manager
template <typename T>
unsigned int ComponentManagerGPU<T>::getNumberOfObstacles() const
{
    return( m_nObstacles );
}




// -----------------------------------------------------------------------------
// Gets the number of cells in manager
template <typename T>
unsigned int ComponentManagerGPU<T>::getNumberOfCells() const
{
    return( m_nCells );
}




// -----------------------------------------------------------------------------
// Gets the array of components rigid body Id
template <typename T>
std::vector<unsigned int> ComponentManagerGPU<T>::getRigidBodyId() const
{
    std::vector<unsigned int> h_rigidBodyId( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_rigidBodyId.data(),
                              m_rigidBodyId,
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets components transformation
template <typename T>
std::vector<Transform3<T>> ComponentManagerGPU<T>::getTransform() const
{
    std::vector<Transform3<T>> h_transform( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_transform.data(),
                              m_transform,
                              m_nParticles * sizeof( Transform3<T> ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_transform );
}




// -----------------------------------------------------------------------------
// Gets components velocities
template <typename T>
std::vector<Kinematics<T>> ComponentManagerGPU<T>::getVelocity() const
{
    std::vector<Kinematics<T>> h_velocity( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_velocity.data(),
                              m_velocity,
                              m_nParticles * sizeof( Kinematics<T> ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_velocity );
}




// -----------------------------------------------------------------------------
// Gets components torce
template <typename T>
std::vector<Torce<T>> ComponentManagerGPU<T>::getTorce() const
{
    std::vector<Torce<T>> h_torce( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_torce.data(),
                              m_torce,
                              m_nParticles * sizeof( Torce<T> ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_torce );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
template <typename T>
std::vector<int> ComponentManagerGPU<T>::getComponentId() const
{
    std::vector<int> h_componentId( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_componentId.data(),
                              m_componentId,
                              m_nParticles * sizeof( int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_componentId );
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
template <typename T>
void ComponentManagerGPU<T>::setRigidBodyId( std::vector<unsigned int> const& id )
{
    cudaErrCheck( cudaMemcpy( m_rigidBodyId,
                              id.data(),
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets components transformation
template <typename T>
void ComponentManagerGPU<T>::setTransform( std::vector<Transform3<T>> const& t )
{
    cudaErrCheck( cudaMemcpy( m_transform,
                              t.data(),
                              m_nParticles * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets components velocity
template <typename T>
void ComponentManagerGPU<T>::setVelocity( std::vector<Kinematics<T>> const& v )
{
    cudaErrCheck( cudaMemcpy( m_velocity,
                              v.data(),
                              m_nParticles * sizeof( Kinematics<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets components torce
template <typename T>
void ComponentManagerGPU<T>::setTorce( std::vector<Torce<T>> const& t )
{
    cudaErrCheck( cudaMemcpy( m_torce,
                              t.data(),
                              m_nParticles * sizeof( Torce<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
template <typename T>
void ComponentManagerGPU<T>::setComponentId( std::vector<int> const& id )
{
    cudaErrCheck( cudaMemcpy( m_componentId,
                              id.data(),
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Updates links between components and linked cell
template <typename T>
void ComponentManagerGPU<T>::updateLinks( LinkedCell<T> const* const* LC )
{
    // Kernel launch parameters
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;
    unsigned int sMemSize = sizeof( unsigned int ) * ( numThreads + 1 );

    // Zeroing out the arrays
    zeroOutArray_kernel<<< numBlocks, numThreads >>>( m_cellHashStart,
                                                      m_nCells + 1 );
    zeroOutArray_kernel<<< numBlocks, numThreads >>>( m_cellHashEnd,
                                                      m_nCells + 1 );

    // First - finding the cell hash for each particle
    computeLinearLinkedCellHashGPU_kernel<<< numBlocks, numThreads >>>
                                                        ( LC,
                                                          m_transform, 
                                                          m_nParticles,
                                                          m_componentCellHash );
    
    // Second - sorting the particle ids according to the cell hash
    thrust::sort_by_key (
      thrust::device_ptr<unsigned int>( m_componentCellHash ),
      thrust::device_ptr<unsigned int>( m_componentCellHash + m_nParticles ),
      thrust::device_ptr<int>( m_componentId ) );
    
    // Third - reseting the cellStart array and finding the start location of 
    // each hash
    sortComponentsAndFindCellStart_kernel<<< numBlocks, numThreads, sMemSize >>> 
                                                        ( m_componentCellHash,
                                                          m_nParticles,
                                                          m_cellHashStart,
                                                          m_cellHashEnd );
}




// -----------------------------------------------------------------------------
// Detects collision between particles
// TODO: thread safety flag and MaxOccupancy
template <typename T>
void ComponentManagerGPU<T>::detectCollisionAndComputeContactForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB, 
                                        ContactForceModel<T> const* const* CF,
                                        int* result )
{
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;
    
    updateLinks( LC );
    detectCollisionAndComputeContactForces_kernel<<< numBlocks, numThreads >>> 
                                                          ( LC,
                                                            RB,
                                                            CF,
                                                            m_rigidBodyId,
                                                            m_transform,
                                                            m_velocity,
                                                            m_torce,
                                                            m_componentId,
                                                            m_componentCellHash,
                                                            m_cellHashStart,
                                                            m_cellHashEnd,
                                                            m_nParticles,
                                                            GrainsParameters<T>::m_gravity[X],
                                                            GrainsParameters<T>::m_gravity[Y],
                                                            GrainsParameters<T>::m_gravity[Z],
                                                            result );
}




// -----------------------------------------------------------------------------
// Moves components in the simulation
template <typename T>
void ComponentManagerGPU<T>::moveComponents( TimeIntegrator<T> const* const* TI,
                                             RigidBody<T, T> const* const* RB )
{
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;

    moveComponents_kernel<<< numBlocks, numThreads >>> (
                                        RB,
                                        TI,
                                        m_rigidBodyId,
                                        m_transform,
                                        m_velocity,
                                        m_torce,
                                        m_nParticles );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerGPU<float>;
template class ComponentManagerGPU<double>;


#undef numCells
#undef numComponents