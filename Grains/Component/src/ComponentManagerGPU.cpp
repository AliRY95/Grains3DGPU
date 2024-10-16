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
    cudaErrCheck( cudaMalloc( (void**)&m_obstacleRigidBodyId,
                              m_nObstacles * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              m_nParticles * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_obstacleTransform,
                              m_nObstacles * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_velocity,
                              m_nParticles * sizeof( Kinematics<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_torce,
                              m_nParticles * sizeof( Torce<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_particleId,
                              m_nParticles * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_particleCellHash,
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
    cudaFree( m_obstacleRigidBodyId );
    cudaFree( m_transform );
    cudaFree( m_obstacleTransform );
    cudaFree( m_velocity );
    cudaFree( m_torce );
    cudaFree( m_particleId );
    cudaFree( m_particleCellHash );
    cudaFree( m_cellHashStart );
    cudaFree( m_cellHashEnd );
    // cudaFree( m_neighborsCount );
    // cudaFree( m_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the array of particles rigid body Ids
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
// Gets the array of obstacles rigid body Ids
template <typename T>
std::vector<unsigned int> ComponentManagerGPU<T>::getRigidBodyIdObstacles() 
                                                                        const
{
    std::vector<unsigned int> h_obstaclesRigidBodyId( m_nObstacles );
    cudaErrCheck( cudaMemcpy( h_obstaclesRigidBodyId.data(),
                              m_obstacleRigidBodyId,
                              m_nObstacles * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_obstaclesRigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets particles transformations
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
// Gets obstacles transformations
template <typename T>
std::vector<Transform3<T>> ComponentManagerGPU<T>::getTransformObstacles() const
{
    std::vector<Transform3<T>> h_obstacleTransform( m_nObstacles );
    cudaErrCheck( cudaMemcpy( h_obstacleTransform.data(),
                              m_obstacleTransform,
                              m_nObstacles * sizeof( Transform3<T> ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_obstacleTransform );
}




// -----------------------------------------------------------------------------
// Gets particles velocities
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
// Gets particles torces
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
// Gets the array of particles Ids
template <typename T>
std::vector<unsigned int> ComponentManagerGPU<T>::getParticleId() const
{
    std::vector<unsigned int> h_particleId( m_nParticles );
    cudaErrCheck( cudaMemcpy( h_particleId.data(),
                              m_particleId,
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_particleId );
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
// Sets the array of particles rigid body Ids
template <typename T>
void ComponentManagerGPU<T>::setRigidBodyId( 
                                        std::vector<unsigned int> const& id )
{
    cudaErrCheck( cudaMemcpy( m_rigidBodyId,
                              id.data(),
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of obstacles rigid body Ids
template <typename T>
void ComponentManagerGPU<T>::setRigidBodyIdObstacles( 
                                        std::vector<unsigned int> const& id )
{
    cudaErrCheck( cudaMemcpy( m_obstacleRigidBodyId,
                              id.data(),
                              m_nObstacles * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets particles transformations
template <typename T>
void ComponentManagerGPU<T>::setTransform( std::vector<Transform3<T>> const& t )
{
    cudaErrCheck( cudaMemcpy( m_transform,
                              t.data(),
                              m_nParticles * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets obstacles transformations
template <typename T>
void ComponentManagerGPU<T>::setTransformObstacles( 
                                        std::vector<Transform3<T>> const& t )
{
    cudaErrCheck( cudaMemcpy( m_obstacleTransform,
                              t.data(),
                              m_nObstacles * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets particles velocities
template <typename T>
void ComponentManagerGPU<T>::setVelocity( std::vector<Kinematics<T>> const& v )
{
    cudaErrCheck( cudaMemcpy( m_velocity,
                              v.data(),
                              m_nParticles * sizeof( Kinematics<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets particles torces
template <typename T>
void ComponentManagerGPU<T>::setTorce( std::vector<Torce<T>> const& t )
{
    cudaErrCheck( cudaMemcpy( m_torce,
                              t.data(),
                              m_nParticles * sizeof( Torce<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of particles Ids
template <typename T>
void ComponentManagerGPU<T>::setParticleId( std::vector<unsigned int> const& id )
{
    cudaErrCheck( cudaMemcpy( m_particleId,
                              id.data(),
                              m_nParticles * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Updates links between particles and linked cell
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
                                                          m_particleCellHash );
    
    // Second - sorting the particle ids according to the cell hash
    thrust::sort_by_key (
      thrust::device_ptr<unsigned int>( m_particleCellHash ),
      thrust::device_ptr<unsigned int>( m_particleCellHash + m_nParticles ),
      thrust::device_ptr<unsigned int>( m_particleId ) );
    
    // Third - reseting the cellStart array and finding the start location of 
    // each hash
    sortComponentsAndFindCellStart_kernel<<< numBlocks, numThreads, sMemSize >>> 
                                                        ( m_particleCellHash,
                                                          m_nParticles,
                                                          m_cellHashStart,
                                                          m_cellHashEnd );
}




// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and obstacles
template <typename T>
void ComponentManagerGPU<T>::detectCollisionAndComputeContactForcesObstacles( 
                                    RigidBody<T, T> const* const* particleRB,
                                    RigidBody<T, T> const* const* obstacleRB,
                                    ContactForceModel<T> const* const* CF )
{
    // Launch parameters
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;

    // Invoke the kernel
    detectCollisionAndComputeContactForcesObstacles_kernel
                        <<< numBlocks, numThreads >>> ( particleRB,
                                                        obstacleRB,
                                                        CF,
                                                        m_rigidBodyId,
                                                        m_transform,
                                                        m_velocity,
                                                        m_torce,
                                                        m_obstacleRigidBodyId,
                                                        m_obstacleTransform,
                                                        m_nParticles,
                                                        m_nObstacles );
}




// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and particles
template <typename T>
void ComponentManagerGPU<T>::detectCollisionAndComputeContactForcesParticles( 
                                    RigidBody<T, T> const* const* particleRB,
                                    LinkedCell<T> const* const* LC,
                                    ContactForceModel<T> const* const* CF,
                                    int* result )
{
    // Launch parameters
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;

    // Invoke the kernel
    detectCollisionAndComputeContactForcesParticles_kernel
                            <<< numBlocks, numThreads >>> ( particleRB,
                                                            LC,
                                                            CF,
                                                            m_rigidBodyId,
                                                            m_transform,
                                                            m_velocity,
                                                            m_torce,
                                                            m_particleId,
                                                            m_particleCellHash,
                                                            m_cellHashStart,
                                                            m_cellHashEnd,
                                                            m_nParticles,
                                                            result );
}




// -----------------------------------------------------------------------------
// Detects collision and computes forces between all components
template <typename T>
void ComponentManagerGPU<T>::detectCollisionAndComputeContactForces( 
                                    RigidBody<T, T> const* const* particleRB, 
                                    RigidBody<T, T> const* const* obstacleRB, 
                                    LinkedCell<T> const* const* LC,
                                    ContactForceModel<T> const* const* CF,
                                    int* result )
{
     // Updates links between components and linked cell
    updateLinks( LC );
    
    // Particle-particle interactions
    detectCollisionAndComputeContactForcesParticles( particleRB,
                                                     LC,
                                                     CF,
                                                     result );
    
    // Particle-obstacle interactions
    detectCollisionAndComputeContactForcesObstacles( particleRB,
                                                     obstacleRB,
                                                     CF );
}




// -----------------------------------------------------------------------------
// Adds external forces such as gravity
template <typename T>
void ComponentManagerGPU<T>::addExternalForces( 
                                    RigidBody<T, T> const* const* particleRB, 
                                    Vector3<T> const& g )
{
    // Launch parameters
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;

    // since g is a host-side vector, we need to break it into three components
    // to be able to pass it to the kernel
    T const gX = g[X], gY = g[Y], gZ = g[Z];
    
    // Invoke the kernel
    addExternalForces_kernel<<< numBlocks, numThreads >>> ( particleRB,
                                                            m_rigidBodyId,
                                                            m_torce,
                                                            gX, gY, gZ,
                                                            m_nParticles );
}




// -----------------------------------------------------------------------------
// Updates the position and velocities of particles
template <typename T>
void ComponentManagerGPU<T>::moveParticles( RigidBody<T, T> const* const* RB,
                                            TimeIntegrator<T> const* const* TI )
{
    // Launch parameters
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( m_nParticles + numThreads - 1 ) / numThreads;

    // Invoke the kernel
    moveParticles_kernel<<< numBlocks, numThreads >>> ( RB,
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