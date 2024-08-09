#include <random>
#include <algorithm>
#include <execution>
#include <omp.h>

#include "ComponentManagerCPU.hh"
#include "ComponentManager.hh"
#include "GrainsParameters.hh"
#include "LinkedCell.hh"
#include "CollisionDetection.hh"
#include "VectorMath.hh"
#include "FirstOrderExplicit.hh"
#include "Quaternion.hh"
#include "QuaternionMath.hh"


#define numComponents (GrainsParameters<T>::m_numComponents)
#define numCells (GrainsParameters<T>::m_numCells)
/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Sorts both vectors data and key based on the key values
static INLINE void sortByKey( std::vector<int>& data, 
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
    std::vector<int> sortedData( N );
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
// Default constructor
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU()
{}




// -----------------------------------------------------------------------------
// Constructor with the number of particles, randomly positioned in the 
// computational domain
// TODO: Cases with multiple RigidBodies
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU( unsigned int nParticles,
                                             unsigned int nObstacles,
                                             unsigned int nCells )
: m_nParticles( nParticles )
, m_nObstacles( nObstacles )
, m_nCells( nCells )
{
    for( int i = 0; i < m_nParticles; i++ )
    {
        m_rigidBodyId.push_back( 0 );
        m_transform.push_back( Transform3<T>() );
        m_kinematics.push_back( Kinematics<T>() );
        m_torce.push_back( Torce<T>() );
        m_componentId.push_back( i );
        m_componentCellHash.push_back( 0 );
    }

    // Initialzing the vectors for obstacles


    // Initialzing the vectors for cells
    // The size of these vectors is one bigger that nCells because we reserve
    // cellID = 0.
    for ( int i = 0; i < m_nCells + 1; i++ )
    {
        m_cellHashStart.push_back( 0 );
        m_cellHashEnd.push_back( 0 );
    }
}




// -----------------------------------------------------------------------------
// Constructor with the number of particles, randomly positioned in the 
// computational domain
// TODO: Cases with multiple RigidBodies
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU( 
                                    std::vector<unsigned int> numEachRigidBody,
                                    unsigned int nObstacles,
                                    unsigned int nCells )
: m_nParticles( numEachRigidBody.back() )
, m_nObstacles( nObstacles )
, m_nCells( nCells )
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

    // Initialzing the vectors for particles
    unsigned int rb_counter = 0;
    Transform3<T> tr;
    for( int i = 0; i < m_nParticles; i++ )
    {
        // m_rigidBodyId
        if ( i == numEachRigidBody[ rb_counter ] )
            ++rb_counter;
        m_rigidBodyId.push_back( rb_counter );

        // m_transform
        tr.setBasis( angle( generator ), 
                     angle( generator ), 
                     angle( generator ) );
        tr.setOrigin( GrainsParameters<T>::m_origin + 
                      Vector3<T>( locationX( generator ),
                                  locationY( generator ),
                                  locationZ( generator ) ) );
        m_transform.push_back( tr );
        
        m_kinematics.push_back( Kinematics<T>() );

        m_torce.push_back( Torce<T>() );

        // m_compId
        m_componentId.push_back( i );

        // m_componentCellHash
        m_componentCellHash.push_back( 0 );
    }

    // Initialzing the vectors for obstacles


    // Initialzing the vectors for cells
    // The size of these vectors is one bigger that nCells because we reserve
    // cellID = 0.
    for ( int i = 0; i < m_nCells + 1; i++ )
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
// Gets the number of particles in manager
template <typename T>
unsigned int ComponentManagerCPU<T>::getNumberOfParticles() const
{
    return( m_nParticles );
}




// -----------------------------------------------------------------------------
// Gets the number of obstacles in manager
template <typename T>
unsigned int ComponentManagerCPU<T>::getNumberOfObstacles() const
{
    return( m_nObstacles );
}




// -----------------------------------------------------------------------------
// Gets the number of cells in manager
template <typename T>
unsigned int ComponentManagerCPU<T>::getNumberOfCells() const
{
    return( m_nCells );
}




// -----------------------------------------------------------------------------
// Gets components rigid body Id
template <typename T>
std::vector<unsigned int> ComponentManagerCPU<T>::getRigidBodyId() const
{
    return( m_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets components transformation
template <typename T>
std::vector<Transform3<T>> ComponentManagerCPU<T>::getTransform() const
{
    return( m_transform );
}




// -----------------------------------------------------------------------------
// Gets components kinematics
template <typename T>
std::vector<Kinematics<T>> ComponentManagerCPU<T>::getKinematics() const
{
    return( m_kinematics );
}




// -----------------------------------------------------------------------------
// Gets components torce
template <typename T>
std::vector<Torce<T>> ComponentManagerCPU<T>::getTorce() const
{
    return( m_torce );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
template <typename T>
std::vector<int> ComponentManagerCPU<T>::getComponentId() const
{
    return( m_componentId );
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
template <typename T>
void ComponentManagerCPU<T>::setRigidBodyId( std::vector<unsigned int> const& id )
{
    m_rigidBodyId = id;
}




// -----------------------------------------------------------------------------
// Sets components transformation
template <typename T>
void ComponentManagerCPU<T>::setTransform( std::vector<Transform3<T>> const& t )
{
    m_transform = t;
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
template <typename T>
void ComponentManagerCPU<T>::setKinematics( std::vector<Kinematics<T>> const& k )
{
    m_kinematics = k;
}




// -----------------------------------------------------------------------------
// Sets the array of components cell hash
template <typename T>
void ComponentManagerCPU<T>::setTorce( std::vector<Torce<T>> const& t )
{
    m_torce = t;
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
template <typename T>
void ComponentManagerCPU<T>::setComponentId( std::vector<int> const& id )
{
    m_componentId = id;
}




// -----------------------------------------------------------------------------
// Updates linked cell information in manager
template <typename T>
void ComponentManagerCPU<T>::updateLinks( LinkedCell<T> const* const* LC )
{
    // Updating m_componentCellHash according to the linkedCell. That is, 
    // assigning a hash value to each particle based on the cell it belongs to.
    (*LC)->computeLinearLinkedCellHashCPU( m_transform,
                                           m_nParticles,
                                           m_componentCellHash );

    // Sorting the hash values and componentIds according to the hash values
    sortByKey( m_componentId, m_componentCellHash );

    // Reseting start and end of each cell hash value
    std::fill( m_cellHashStart.begin(), m_cellHashStart.end(), 0 );
    std::fill( m_cellHashEnd.begin(), m_cellHashEnd.end(), 0 );


    // Finding the start and end of each cell hash value
    for ( int i = 0; i < m_nParticles; i++ )
    {
        unsigned int hash = m_componentCellHash[ i ];
        if ( i == 0 )
            m_cellHashStart[ hash ] = i;
        if ( i != 0 && hash != m_componentCellHash[ i - 1 ] )
            m_cellHashStart.at( hash ) = i;
        if ( i > 0 )
            m_cellHashEnd[ m_componentCellHash[ i - 1 ] ] = i;
        if ( i == numComponents - 1 )
            m_cellHashEnd[ hash ] = i + 1;
    }
}




// -----------------------------------------------------------------------------
// Detects collision between particles
template <typename T>
void ComponentManagerCPU<T>::detectCollision( LinkedCell<T> const* const* LC,
                                              RigidBody<T, T> const* const* RB,
                                              HODCContactForceModel<T> const* const* CF,
                                              int* result )
{
    // updating links between components and linked cell
    updateLinks( LC );
    
    // #pragma omp parallel for
    for ( int pId = 0; pId < m_nParticles; pId++ )
    {
        // Parameters of the primary particle
        unsigned int const compId = m_componentId[ pId ];
        unsigned int const cellHash = m_componentCellHash[ pId ];
        RigidBody<T, T> const& rbA = *( RB[ m_rigidBodyId[ compId ] ] );
        Transform3<T> const& trA = m_transform[ compId ];
        // Torce<T>& m_torce[ compId ];
        for ( int k = -1; k < 2; k++ ) {
        for ( int j = -1; j < 2; j++ ) { 
        for ( int i = -1; i < 2; i++ ) {
            int neighboringCellHash =
                (*LC)->computeNeighboringCellLinearHash( cellHash, i, j, k );
            int startId = m_cellHashStart[ neighboringCellHash ];
            int endId = m_cellHashEnd[ neighboringCellHash ];
            for ( int id = startId; id < endId; id++ )
            {           
                unsigned int const secondaryId = m_componentId[ id ];
                // To skip self-collision
                if ( secondaryId == compId )
                    continue;
                RigidBody<T, T> const& rbB = 
                                        *( RB[ m_rigidBodyId[ secondaryId ] ] );
                Transform3<T> const& trB = m_transform[ secondaryId ];
                // result[compId] += intersectRigidBodies( rigidBodyA,
                //                                     rigidBodyA,
                //                                     transformA, 
                //                                     transformB );
                ContactInfo<T> ci = closestPointsRigidBodies( rbA,
                                                              rbB,
                                                              trA, 
                                                              trB );
                if ( ci.getOverlapDistance() < T( 0 ) )
                    (*CF)->computeForces( ci, 
                                          zeroVector3T,
                                          zeroVector3T,
                                          rbA.getMass(),
                                          rbB.getMass(),
                                          m_torce[ compId ] );
                result[compId] += ( ci.getOverlapDistance() < T( 0 ) );
            }
        } } }
    }
}




// -----------------------------------------------------------------------------
// Detects collision between particles
template <typename T>
void ComponentManagerCPU<T>::moveParticles( FirstOrderExplicit<T> const* const* TI,
                                            RigidBody<T, T> const* const* RB )
{
    // #pragma omp parallel for
    for ( int pId = 0; pId < m_nParticles; pId++ )
    {
        // Parameters of the particle
        RigidBody<T, T> const& rb = *( RB[ m_rigidBodyId[ pId ] ] );
        Transform3<T>& tr = m_transform[ pId ];
        // ---------------------------------------------------------------------
        // Computing the accelerations
        Vector3<T> transAcc( m_torce[ pId ].getForce() / rb.getMass() );
        // Angular momentum
        // Inverse inertia tensor
        T invInertia[6];
        rb.getInertia_1( invInertia );
        Vector3<T> angAcc, angAccTemp;
        // Quaternion and eotation quaternion conjugate
        Quaternion<T> qRot;
        qRot.setQuaternion( tr.getBasis() );
        Quaternion<T> qRotCon = qRot.conjugate(); 
        // Write torque in body-fixed coordinates system
        angAcc = qRotCon.multToVector3( m_torce[ pId ].getTorque() * qRot );
        // Compute I^-1.(T + I.w ^ w) in body-fixed coordinates system 
        angAccTemp[0] = invInertia[0] * angAcc[0] + 
                        invInertia[1] * angAcc[1] + 
                        invInertia[2] * angAcc[2];
        angAccTemp[1] = invInertia[1] * angAcc[0] +
                        invInertia[3] * angAcc[1] +
                        invInertia[4] * angAcc[2];
        angAccTemp[2] = invInertia[2] * angAcc[0] + 
                        invInertia[4] * angAcc[1] +
                        invInertia[5] * angAcc[2];
  
        // Write I^-1.(T + I.w ^ w) in space-fixed coordinates system
        angAcc = qRot.multToVector3( angAccTemp * qRotCon );
        // ---------------------------------------------------------------------


        // Time integration
        Vector3<T> transMove, avgAngVel;
        (*TI)->Move( transAcc, angAcc, m_kinematics[ pId ],
                     transMove, avgAngVel );
        
        // Translational motion
        m_transform[ pId ].composeLeftByTranslation( transMove );
        
        // Angular motion 
        T nOmega = norm( avgAngVel );
        if ( nOmega > LOWEPS ) 
        {
            T c = cos( nOmega * 0.001 / 2. );
            T s = sin( nOmega * 0.001 / 2. );
            Vector3<T> t;
            t = ( s / nOmega ) * avgAngVel;
            qRotCon.setQuaternion( t, c );
        } 
        else 
            qRotCon.setQuaternion( T( 0 ), T( 0 ), T( 0 ), T( 1 ) );
        
        qRot = qRotCon * qRot;
        m_transform[ pId ].composeLeftByRotation( T( 0.5 ) * ( m_kinematics[ pId ].getAngularVelocity() * qRot ) );

        
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerCPU<float>;
template class ComponentManagerCPU<double>;


#undef numCells
#undef numComponents

