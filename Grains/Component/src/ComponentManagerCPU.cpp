#include <random>

#include "ComponentManagerCPU.hh"
#include "ComponentManager.hh"
#include "GrainsParameters.hh"
#include "LinkedCell.hh"
#include "CollisionDetection.hh"
#include "VectorMath.hh"


#include <algorithm>
#include <execution>
#include <omp.h>



#include "thrust/for_each.h"
#include "thrust/iterator/zip_iterator.h"
#include "thrust/sort.h"


#define numComponents (GrainsParameters<T>::m_numComponents)
#define numCells (GrainsParameters<T>::m_numCells)
/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Sorts one vector based on another vector
static INLINE void sortByKey( std::vector<unsigned int>& data, 
                              std::vector<unsigned int>& key )
{
    // Create a vector of indices
    std::size_t N = data.size();
    std::vector<std::size_t> indices( N );
    for ( std::size_t i = 0; i < indices.size(); ++i )
        indices[i] = i;

    // Sort the indices based on the key vector
    std::sort( indices.begin(), 
               indices.end(), 
               [&key]( std::size_t i1, std::size_t i2 ) 
               { return key[i1] < key[i2]; } );

    // Reorder the key and data vectors based on the sorted indices
    std::vector<unsigned int> sortedData( N );
    std::vector<unsigned int> sortedKey( N );
    for ( std::size_t i = 0; i < N; ++i )
    {
        sortedKey[i] = key[ indices[i] ];
        sortedData[i] = data[ indices[i] ];
    }
    key = sortedKey;
    data = sortedData;
}





/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Constructor with the number of particles randomly positioned in the 
// computational domain
// TODO: Cases with multiple RigidBodies
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU()
{
    // Randomly initializing transforms
    std::default_random_engine generator;
    std::uniform_real_distribution<T> locationX( T( 0 ), 
                                        GrainsParameters<T>::m_dimension[X] );
    std::uniform_real_distribution<T> locationY( T( 0 ), 
                                        GrainsParameters<T>::m_dimension[Y] );
    std::uniform_real_distribution<T> locationZ( T( 0 ), 
                                        GrainsParameters<T>::m_dimension[Z] );
    std::uniform_real_distribution<T> angle( T( 0 ), T( 2 * M_PI ) );

    // Initialzing the vectors
    Transform3<T> tr;
    for( int i = 0; i < numComponents; i++ )
    {
        // m_transform
        tr.setBasis( angle( generator ), 
                     angle( generator ), 
                     angle( generator ) );
        tr.setOrigin( GrainsParameters<T>::m_origin + 
                      Vector3<T>( locationX( generator ),
                                  locationY( generator ),
                                  locationZ( generator ) ) );
        m_transform.push_back( tr );
        
        // m_rigidBodyId
        m_rigidBodyId.push_back( 0 );

        // m_componentCellHash
        m_componentCellHash.push_back( 0 );

        // m_compId
        m_componentId.push_back( i );
    }


    for ( int i = 0; i < numCells + 1; i++ )
    {
        m_cellHashStart.push_back( 0 );
        m_cellHashEnd.push_back( 0 );
    }
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManagerCPU<T>::~ComponentManagerCPU()
{}





// -----------------------------------------------------------------------------
// Gets components transformation
template <typename T>
std::vector<Transform3<T>> ComponentManagerCPU<T>::getTransform() const
{
    return( m_transform );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor Id
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getNeighborsId() const
{
    return( m_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the array of components rigid body Id
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getRigidBodyId() const
{
    return( m_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getComponentId() const
{
    return( m_componentId );
}




// -----------------------------------------------------------------------------
// Gets the array of components cell hash
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getComponentCellHash() const
{
    return( m_componentCellHash );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor count
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getNeighborsCount() const
{
    return( m_neighborsCount );
}




// -----------------------------------------------------------------------------
// Gets the array of cells hash start
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getCellHashStart() const
{
    return( m_cellHashStart );
}




// -----------------------------------------------------------------------------
// Sets components transformation
template <typename T>
void ComponentManagerCPU<T>::setTransform( std::vector<Transform3<T>> const& tr )
{
    m_transform = tr;
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
template <typename T>
void ComponentManagerCPU<T>::setNeighborsId( std::vector<unsigned int> const& id )
{
    m_neighborsId = id;
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
template <typename T>
void ComponentManagerCPU<T>::setRigidBodyId( std::vector<unsigned int> const& id )
{
    m_rigidBodyId = id;
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
template <typename T>
void ComponentManagerCPU<T>::setComponentId( std::vector<unsigned int> const& id )
{
    m_componentId = id;
}




// -----------------------------------------------------------------------------
// Sets the array of components cell hash
template <typename T>
void ComponentManagerCPU<T>::setComponentCellHash( std::vector<unsigned int> const& hash )
{
    m_componentCellHash = hash;
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor count
template <typename T>
void ComponentManagerCPU<T>::setNeighborsCount( std::vector<unsigned int> const& count )
{
    m_neighborsCount = count;
}




// -----------------------------------------------------------------------------
// Sets the array of cells hash start
template <typename T>
void ComponentManagerCPU<T>::setCellHashStart( std::vector<unsigned int> const& id )
{
    m_cellHashStart = id;
}




// -----------------------------------------------------------------------------
// Detects collision between particles
template <typename T>
void ComponentManagerCPU<T>::detectCollision( LinkedCell<T> const* const* LC,
                                              RigidBody<T, T> const* const* rb, 
                                              int* result )
{
    // // for ( int i = 0; i < numComponents; i++ )
    // // {
    // //     const RigidBody& AA = *( rb[m_rigidBodyId[i]] );
    // //     const Transform3d& trA = m_transform[i];
    // //     for ( int j = 0; j < numComponents; j++ ) // or start from j = i?
    // //         result[i] += intersectRigidBodies( AA, AA, trA, m_transform[j] );
    // // }

    
    // // RigidBody const& AA = **rb;
    // // for ( int i = 0; i < N; i++ )
    // // {
    // //     Transform3d trA = tr3d[i];
    // //     Transform3d trB2A;
    // //     for ( int j = 0; j < N; j++ ) // or start from j = i?
    // //     {
    // //         trB2A = tr3d[j];
    // //         trB2A.relativeToTransform( trA );
    // //         result[i] = intersectRigidBodies( AA, AA, trB2A );
    // //     }
    // // }

    (*LC)->computeLinearLinkedCellHashCPU( m_transform,
                                       numComponents,
                                       m_componentCellHash );

    sortByKey( m_componentId, m_componentCellHash );

    std::fill( m_cellHashStart.begin(), m_cellHashStart.end(), 0 );
    std::fill( m_cellHashEnd.begin(), m_cellHashEnd.end(), 0 );
    
    for ( int i = 0; i < numComponents; i++ )
    {
        unsigned int hash = m_componentCellHash.at( i );
        if ( i == 0 )
            m_cellHashStart.at( hash ) = i;
        if ( i != 0 && hash != m_componentCellHash.at( i - 1 ) )
            m_cellHashStart.at( hash ) = i;
        if ( i > 0 )
            m_cellHashEnd.at( m_componentCellHash.at( i - 1 ) ) = i;
        if ( i == numComponents - 1 )
            m_cellHashEnd.at( hash ) = i + 1;
    }
    
    // #pragma omp parallel for
    for ( int pId = 0; pId < numComponents; pId++ )
    {
        unsigned int const compId = m_componentId[ pId ];
        unsigned int const cellHash = m_componentCellHash[ pId ];
        RigidBody<T, T> const& rigidBodyA = **rb; // TODO: FIX to *( a[ m_rigidBodyId[ compId ] ] )?
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
                        // To skip the self-collision
                        if ( secondaryId == compId )
                            continue;
                        Transform3<T> const& transformB = m_transform[ secondaryId ];
                        // result[compId] += intersectRigidBodies( rigidBodyA,
                        //                                     rigidBodyA,
                        //                                     transformA, 
                        //                                     transformB );
                        ContactInfo<T> ci = closestPointsRigidBodies( rigidBodyA,
                                                                    rigidBodyA,
                                                                    transformA, 
                                                                    transformB );
                        // if( ci.getOverlapDistance() < T( 0 ) )
                        //     cout << compId << " " << secondaryId << " " <<
                        //     ci.getOverlapDistance() << endl;
                        result[compId] += ( ci.getOverlapDistance() < T( 0 ) );
                    }
                }
            }
        }
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerCPU<float>;
template class ComponentManagerCPU<double>;


#undef numCells
#undef numComponents

