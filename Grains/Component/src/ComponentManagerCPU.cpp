#include <random>

#include "ComponentManager.hh"
#include "ComponentManagerCPU.hh"

#include "GrainsParameters.hh"



#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"



#define numComponents (GrainsParameters::m_numComponents)
#define numCells (GrainsParameters::m_numCells)
// -----------------------------------------------------------------------------
// Constructor with the number of particles randomly positioned in the 
// computational domain
// TODO: Cases with multiple RigidBodies
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU()
{
    // Randomly initializing transforms
    std::default_random_engine generator;
    std::uniform_real_distribution<T> location( T( 0 ), T( 1 ) );
    std::uniform_real_distribution<T> angle( T( 0 ), T( 2 * M_PI ) );

    // Allocating memory on host
    m_transform = new Transform3<T>[numComponents];
    m_neighborsId = new unsigned int[numComponents * numComponents];
    m_rigidBodyId = new unsigned int[numComponents];
    m_componentId = new unsigned int[numComponents];
    m_componentCellHash = new unsigned int[numComponents];
    m_neighborsCount = new unsigned int[numComponents];
    m_cellHashStart = new unsigned int[numCells + 1];
    m_cellHashEnd = new unsigned int[numCells + 1];
    // Initialzing the arrays
    for( int i = 0; i < numComponents; i++ )
    {
        // m_transform
        T aX = angle( generator );
        T aY = angle( generator );
        T aZ = angle( generator );
        m_transform[i].setBasis( aX, aY, aZ );
        m_transform[i].setOrigin( Vector3( location( generator ),
                                           location( generator ),
                                           location( generator ) ) );
        
        // m_rigidBodyId
        m_rigidBodyId[i] = 0;

        // m_compId
        m_componentId[i] = i;
    }
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManagerCPU<T>::~ComponentManagerCPU()
{
    delete[] m_transform;
    delete[] m_neighborsId;
    delete[] m_rigidBodyId;
    delete[] m_componentId;
    delete[] m_componentCellHash;
    delete[] m_neighborsCount;
    delete[] m_cellHashStart;
    delete[] m_cellHashEnd;
}





// -----------------------------------------------------------------------------
// Gets components transformation
template <typename T>
Transform3<T>* ComponentManagerCPU<T>::getTransform() const
{
    return( m_transform );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor Id
template <typename T>
unsigned int* ComponentManagerCPU<T>::getNeighborsId() const
{
    return( m_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the array of components rigid body Id
template <typename T>
unsigned int* ComponentManagerCPU<T>::getRigidBodyId() const
{
    return( m_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
template <typename T>
unsigned int* ComponentManagerCPU<T>::getComponentId() const
{
    return( m_componentId );
}




// -----------------------------------------------------------------------------
// Gets the array of components cell hash
template <typename T>
unsigned int* ComponentManagerCPU<T>::getComponentCellHash() const
{
    return( m_componentCellHash );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor count
template <typename T>
unsigned int* ComponentManagerCPU<T>::getNeighborsCount() const
{
    return( m_neighborsCount );
}




// -----------------------------------------------------------------------------
// Gets the array of cells hash start
template <typename T>
unsigned int* ComponentManagerCPU<T>::getCellHashStart() const
{
    return( m_cellHashStart );
}




// -----------------------------------------------------------------------------
// Sets components transformation
template <typename T>
void ComponentManagerCPU<T>::setTransform( Transform3<T> const* tr )
{
    for ( int i = 0; i < numComponents; i++ )
        m_transform[i] = tr[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
template <typename T>
void ComponentManagerCPU<T>::setNeighborsId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_neighborsId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
template <typename T>
void ComponentManagerCPU<T>::setRigidBodyId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_rigidBodyId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
template <typename T>
void ComponentManagerCPU<T>::setComponentId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_componentId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components cell hash
template <typename T>
void ComponentManagerCPU<T>::setComponentCellHash( unsigned int const* hash )
{
     for ( int i = 0; i < numComponents; i++ )
        m_componentCellHash[i] = hash[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor count
template <typename T>
void ComponentManagerCPU<T>::setNeighborsCount( unsigned int const* count )
{
     for ( int i = 0; i < numComponents; i++ )
        m_neighborsCount[i] = count[i];
}




// -----------------------------------------------------------------------------
// Sets the array of cells hash start
template <typename T>
void ComponentManagerCPU<T>::setCellHashStart( unsigned int const* id )
{
     for ( int i = 0; i < numCells + 1; i++ )
        m_cellHashStart[i] = id[i];
}




// -----------------------------------------------------------------------------
// Detects collision between particles
template <typename T>
void ComponentManagerCPU<T>::detectCollision( LinkedCell<T> const* const* LC,
                                              RigidBody<T> const* const* rb, 
                                              int* result )
{
    // for ( int i = 0; i < numComponents; i++ )
    // {
    //     const RigidBody& AA = *( rb[m_rigidBodyId[i]] );
    //     const Transform3d& trA = m_transform[i];
    //     for ( int j = 0; j < numComponents; j++ ) // or start from j = i?
    //         result[i] += intersectRigidBodies( AA, AA, trA, m_transform[j] );
    // }

    
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
    

    (*LC)->computeLinearLinkedCellHashCPU( m_transform,
                                           numComponents,
                                           m_componentCellHash );

    thrust::sort_by_key ( m_componentCellHash,
                          m_componentCellHash + numComponents,
                          m_componentId );

    for ( int i = 0; i < numCells + 1; i++ )
    {
        m_cellHashStart[ i ] = 0;
        m_cellHashEnd[ i ] = 0;
    }

    for ( int i = 0; i < numComponents; i++ )
    {
        unsigned int hash = m_componentCellHash[ i ];
        if ( i == 0 || hash != m_componentCellHash[ i - 1 ] )
            m_cellHashStart[ hash ] = i;
        if ( i > 0 )
            m_cellHashEnd[ m_componentCellHash[ i - 1 ] ] = i;
        if ( i == numComponents - 1 )
            m_cellHashEnd[ hash ] = i + 1;
    }

    for ( int pId = 0; pId < numComponents; pId++ )
    {
        unsigned int const compId = m_componentId[ pId ];
        unsigned int const cellHash = m_componentCellHash[ pId ];
        RigidBody<T> const& rigidBodyA = **rb; // TODO: FIX to *( a[ m_rigidBodyId[ compId ] ] )?
        Transform3<T> const& transformA = m_transform[ compId ];
        for ( int k = -1; k < 2; k++ )
        {
            for ( int j = -1; j < 2; j++ ) 
            {
                for ( int i = -1; i < 2; i++ ) 
                {
                    int neighboringCellHash =
                    (*LC)->computeNeighboringCellLinearHash( cellHash, i, j, k );
                    int startId = m_cellHashStart[ neighboringCellHash ];
                    int endId = m_cellHashEnd[ neighboringCellHash ];
                    for ( int id = startId; id < endId; id++ )
                    {
                        // TODO:
                        // RigidBody const& rigidBodyB = 8( a[ m_rigidBodyId[ compId ] ] ); ???
                        int secondaryId = m_componentId[ id ];
                        Transform3<T> const& transformB = m_transform[ secondaryId ];
                        // result[compId] += intersectRigidBodies( rigidBodyA,
                        //                                     rigidBodyA,
                        //                                     transformA, 
                        //                                     transformB );
                        ContactInfo<T> ci = closestPointsRigidBodies( rigidBodyA,
                                                                    rigidBodyA,
                                                                    transformA, 
                                                                    transformB );
                        // std::cout << ci.getOverlapDistance() << "\n";
                        result[compId] += ( ci.getOverlapDistance() < 0. );
                    }
                }
            }
        }
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
// template class ComponentManagerCPU<float>;
template class ComponentManagerCPU<double>;


#undef numCells
#undef numComponents

