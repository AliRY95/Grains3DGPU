#include <curand.h>
#include <cooperative_groups.h>
#include "thrust/device_ptr.h"
#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"

// #include "ComponentManager.hh"
#include "ComponentManagerGPU.hh"
#include "ComponentManagerGPU_Kernels.hh"
#include "GrainsParameters.hh"
#include "LinkedCell_Kernels.hh"
#include "LinkedCell.hh"


#define numComponents (GrainsParameters<T>::m_numComponents)
#define numCells (GrainsParameters<T>::m_numCells)
// -----------------------------------------------------------------------------
// Constructor with the number of particles randomly positioned in the 
// computational domain
// TODO: Use curand to generate random arrays on device
template <typename T>
ComponentManagerGPU<T>::ComponentManagerGPU()
{
    // Allocating memory on host
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              numComponents * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsId,
                              numComponents * numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_rigidBodyId,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentId,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentCellHash,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsCount,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashStart,
                              ( numCells + 1 ) * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashEnd,
                              ( numCells + 1 ) * sizeof( unsigned int ) ) );

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
template <typename T>
ComponentManagerGPU<T>::ComponentManagerGPU( ComponentManagerCPU<T> const& cm )
{
    // Allocating memory on host
    cudaErrCheck( cudaMalloc( (void**)&m_transform,
                              numComponents * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsId,
                              numComponents * numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_rigidBodyId,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentId,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_componentCellHash,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_neighborsCount,
                              numComponents * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashStart,
                              ( numCells + 1 ) * sizeof( unsigned int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_cellHashEnd,
                              ( numCells + 1 ) * sizeof( unsigned int ) ) );


    // Copying the arrays from host to device
    setTransform( cm.getTransform().data() );
    // setNeighborsId( cm.getNeighborsId().data() );
    setRigidBodyId( cm.getRigidBodyId().data() );
    setComponentId( cm.getComponentId().data() );
    // setNeighborsCount( cm.getNeighborsCount().data() );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManagerGPU<T>::~ComponentManagerGPU()
{
    cudaFree( m_transform );
    cudaFree( m_neighborsId );
    cudaFree( m_rigidBodyId );
    cudaFree( m_componentId );
    cudaFree( m_componentCellHash );
    cudaFree( m_neighborsCount );
    cudaFree( m_cellHashStart );
    cudaFree( m_cellHashEnd );
}




// -----------------------------------------------------------------------------
// Gets components transformation
template <typename T>
Transform3<T>* ComponentManagerGPU<T>::getTransform() const
{
    Transform3<T>* h_transform = new Transform3<T>[numComponents];
    cudaErrCheck( cudaMemcpy( h_transform,
                              m_transform,
                              numComponents * sizeof( Transform3<T> ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_transform );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor Id
template <typename T>
unsigned int* ComponentManagerGPU<T>::getNeighborsId() const
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
template <typename T>
unsigned int* ComponentManagerGPU<T>::getRigidBodyId() const
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
template <typename T>
unsigned int* ComponentManagerGPU<T>::getComponentId() const
{
    unsigned int* h_componentId = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_componentId,
                              m_componentId,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_componentId );
}




// -----------------------------------------------------------------------------
// Gets the array of components cell hash
template <typename T>
unsigned int* ComponentManagerGPU<T>::getComponentCellHash() const
{
    unsigned int* h_componentCellHash = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_componentCellHash,
                              m_componentCellHash,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_componentCellHash );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor count
template <typename T>
unsigned int* ComponentManagerGPU<T>::getNeighborsCount() const
{
    unsigned int* h_neighborsCount = new unsigned int[numComponents];
    cudaErrCheck( cudaMemcpy( h_neighborsCount,
                              m_neighborsCount,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_neighborsCount );
}




// -----------------------------------------------------------------------------
// Gets the array of cells hash start
template <typename T>
unsigned int* ComponentManagerGPU<T>::getCellHashStart() const
{
    unsigned int* h_cellHashStart = new unsigned int[numCells + 1];
    cudaErrCheck( cudaMemcpy( h_cellHashStart,
                              m_cellHashStart,
                              ( numCells + 1 ) * sizeof( unsigned int ), 
                              cudaMemcpyDeviceToHost ) );
    return( h_cellHashStart );
}




// -----------------------------------------------------------------------------
// Sets components transformation
template <typename T>
void ComponentManagerGPU<T>::setTransform( Transform3<T> const* tr )
{
    cudaErrCheck( cudaMemcpy( m_transform,
                              tr,
                              numComponents * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
template <typename T>
void ComponentManagerGPU<T>::setNeighborsId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_neighborsId,
                              id,
                              numComponents * numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
template <typename T>
void ComponentManagerGPU<T>::setRigidBodyId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_rigidBodyId,
                              id,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
template <typename T>
void ComponentManagerGPU<T>::setComponentId( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_componentId,
                              id,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components cell hash
template <typename T>
void ComponentManagerGPU<T>::setComponentCellHash( unsigned int const* hash )
{
    cudaErrCheck( cudaMemcpy( m_componentCellHash,
                              hash,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor count
template <typename T>
void ComponentManagerGPU<T>::setNeighborsCount( unsigned int const* count )
{
    cudaErrCheck( cudaMemcpy( m_neighborsCount,
                              count,
                              numComponents * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Sets the array of cells hash start
template <typename T>
void ComponentManagerGPU<T>::setCellHashStart( unsigned int const* id )
{
    cudaErrCheck( cudaMemcpy( m_cellHashStart,
                              id,
                              ( numCells + 1 ) * sizeof( unsigned int ), 
                              cudaMemcpyHostToDevice ) );
}




// -----------------------------------------------------------------------------
// Updates the linked cell list according to the linked cell provided
template <typename T>
void ComponentManagerGPU<T>::updateLinkedCellList( LinkedCell<T> const* const* LC )
{
    // First - finding the cell hash for each particle
    // LC->computeLinearLinkedCellHashGPU( m_transform, 
    //                                        numComponents,
    //                                        m_componentCellHash );
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( numComponents + numThreads - 1 ) / numThreads;
    unsigned int sMemSize = sizeof( unsigned int ) * ( numThreads + 1 );
    computeLinearLinkedCellHashGPU_kernel<<< numBlocks, numThreads >>>
                                                        ( LC,
                                                          m_transform, 
                                                          numComponents,
                                                          m_componentCellHash );
    
    // Second - sorting the particle ids according to the cell hash
    thrust::sort_by_key (
      thrust::device_ptr<unsigned int>( m_componentCellHash ),
      thrust::device_ptr<unsigned int>( m_componentCellHash + numComponents ),
      thrust::device_ptr<unsigned int>( m_componentId ) );
    
    // Third - reseting the cellStart array anf finding the start location of 
    // each hash
    // TODO: RESET m_cellHashStartEnd
    // unsigned int numThreads = 256;
    // unsigned int numBlocks = ( numComponents + numThreads - 1 ) / numThreads;
    sortComponentsAndFindCellStart<<< numBlocks, numThreads, sMemSize >>> 
                                                        ( m_componentCellHash,
                                                          numComponents,
                                                          m_cellHashStart,
                                                          m_cellHashEnd );
}




// -----------------------------------------------------------------------------
// Detects collision between particles
// TODO: thread safety flag and MaxOccupancy
template <typename T>
void ComponentManagerGPU<T>::detectCollision( LinkedCell<T> const* const* LC,
                                              RigidBody<T, T> const* const* rb, 
                                              int* result )
{
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( numComponents + numThreads - 1 ) / numThreads;
    // collisionDetectionN2<<< numBlocks, numThreads >>> ( rb,
    //                                                     m_transform,
    //                                                     numComponents,
    //                                                     result );
    
    
    // updateLinkedCellList( LC );
    zeroOutArray<<< numBlocks, numThreads >>>( m_cellHashStart,
                                               numCells + 1 );
    zeroOutArray<<< numBlocks, numThreads >>>( m_cellHashEnd,
                                               numCells + 1 );
    unsigned int sMemSize = sizeof( unsigned int ) * ( numThreads + 1 );
    // LC->computeLinearLinkedCellHashGPU( m_transform,
    //                                     numComponents,
    //                                     m_componentCellHash );
    computeLinearLinkedCellHashGPU_kernel<<< numBlocks, numThreads >>>
                                                        ( LC,
                                                          m_transform, 
                                                          numComponents,
                                                          m_componentCellHash );
    
    // Second - sorting the particle ids according to the cell hash
    thrust::sort_by_key (
      thrust::device_ptr<unsigned int>( m_componentCellHash ),
      thrust::device_ptr<unsigned int>( m_componentCellHash + numComponents ),
      thrust::device_ptr<unsigned int>( m_componentId ) );
    
    // Third - reseting the cellStart array and finding the start location of 
    // each hash
    // TODO: RESET m_cellHashStartEnd
    // unsigned int numThreads = 256;
    // unsigned int numBlocks = ( numComponents + numThreads - 1 ) / numThreads;
    sortComponentsAndFindCellStart<<< numBlocks, numThreads, sMemSize >>> 
                                                        ( m_componentCellHash,
                                                          numComponents,
                                                          m_cellHashStart,
                                                          m_cellHashEnd );


    collisionDetectionLinkedCell<<< numBlocks, numThreads >>> 
                                                          ( LC,
                                                            m_componentId,
                                                            m_componentCellHash,
                                                            m_cellHashStart,
                                                            m_cellHashEnd,
                                                            rb,
                                                            m_transform,
                                                            numComponents,
                                                            result );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerGPU<float>;
template class ComponentManagerGPU<double>;


#undef numCells
#undef numComponents