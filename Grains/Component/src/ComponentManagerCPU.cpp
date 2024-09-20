#include <random>
#include <algorithm>
#include <execution>
#include <omp.h>

#include "ComponentManagerCPU.hh"
#include "Quaternion.hh"
#include "VectorMath.hh"
#include "QuaternionMath.hh"


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
// Constructor with the number of particles, number of obstacles, and number of
// cells with all other data members at zero.
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU( unsigned int nParticles,
                                             unsigned int nObstacles,
                                             unsigned int nCells )
: m_nParticles( nParticles )
, m_nObstacles( nObstacles )
, m_nCells( nCells )
{
    // // Initializing the vectors for obstacles
    // for( int i = 1; i < m_nObstacles + 1; i++ )
    // {
    //     m_rigidBodyId.push_back( 0 );
    //     m_transform.push_back( Transform3<T>() );
    //     m_velocity.push_back( Kinematics<T>() );
    //     m_torce.push_back( Torce<T>() );
    //     m_componentId.push_back( -i );
    //     m_componentCellHash.push_back( 0 );
    //     // m_isActive.push_back( 0 );
    // }

    // Initializing the vectors for particles
    for( int i = 0; i < m_nParticles; i++ )
    {
        m_rigidBodyId.push_back( 0 );
        m_transform.push_back( Transform3<T>() );
        m_velocity.push_back( Kinematics<T>() );
        m_torce.push_back( Torce<T>() );
        m_componentId.push_back( i );
        m_componentCellHash.push_back( 0 );
        // m_isActive.push_back( 0 );
    }

    // Initializing the vectors for cells
    // The size of these vectors is one bigger that nCells because we reserve
    // cellID = 0.
    for ( int i = 0; i < m_nCells + 1; i++ )
    {
        m_cellHashStart.push_back( 0 );
        m_cellHashEnd.push_back( 0 );
    }
}




// -----------------------------------------------------------------------------
// Constructor with the number of each rigid body, number of obstacles, and 
// number of cells. Components are randomly positioned.
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
    std::uniform_real_distribution<T> angle( T( 0 ), TWO_PI<T> );

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
        
        m_velocity.push_back( Kinematics<T>() );

        m_torce.push_back( Torce<T>() );

        // m_compId
        m_componentId.push_back( i );

        // m_componentCellHash
        m_componentCellHash.push_back( 0 );
    }



    m_transform[0].setOrigin( Vector3<T>( -0.25, -0.25, 0.5 ) );
    // m_transform[1].setOrigin( Vector3<T>( 0.25, 0.25, 0.5 ) );
    m_velocity[0].setTranslationalComponent( Vector3<T>( 1e-1, 1e-1, 0 ) );
    // m_velocity[1].setTranslationalComponent( Vector3<T>( -1e-1, -1e-1, 0 ) );

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
// Gets components velocities
template <typename T>
std::vector<Kinematics<T>> ComponentManagerCPU<T>::getVelocity() const
{
    return( m_velocity );
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
// Sets components velocity
template <typename T>
void ComponentManagerCPU<T>::setVelocity( std::vector<Kinematics<T>> const& v )
{
    m_velocity = v;
}




// -----------------------------------------------------------------------------
// Sets components torce
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
// Updates links between components and linked cell
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
        if ( i == m_nParticles - 1 )
            m_cellHashEnd[ hash ] = i + 1;
    }
}




// -----------------------------------------------------------------------------
// Detects collision between particles
template <typename T>
void ComponentManagerCPU<T>::detectCollisionAndComputeContactForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB,
                                        ContactForceModel<T> const* const* CF,
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
        T massA = rbA.getMass();
        unsigned int matA = rbA.getMaterial();
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
                {
                    // CF ID given materialIDs
                    unsigned int contactForceID = 
                    ContactForceModelBuilderFactory<T>::computeHash( 
                                                            matA, 
                                                            rbB.getMaterial() );
                    // velocities of the particles
                    Kinematics<T> v1( m_velocity[ compId ] );
                    Kinematics<T> v2( m_velocity[ secondaryId ] );
                    // geometric point of contact
                    Vector3<T> contactPt( ci.getContactPoint() );
                    // relative velocity at contact point
                    Vector3<T> relVel( v1.kinematicsAtPoint( contactPt ) -
                                       v2.kinematicsAtPoint( contactPt ) );
                    // relative angular velocity
                    Vector3<T> relAngVel( v1.getAngularComponent() - 
                                          v2.getAngularComponent() );
                    CF[contactForceID]->computeForces( ci, 
                                                       relVel,
                                                       relAngVel,
                                                       massA,
                                                       rbB.getMass(),
                                                       m_torce[ compId ] );
                }
                result[compId] += ( ci.getOverlapDistance() < T( 0 ) );
            }
        } } }
        // Adding the gravitational force to the torce
        m_torce[compId].addForce( massA * GrainsParameters<T>::m_gravity );
    }
}




// -----------------------------------------------------------------------------
// Moves components in the simulation
template <typename T>
void ComponentManagerCPU<T>::moveComponents( TimeIntegrator<T> const* const* TI,
                                             RigidBody<T, T> const* const* RB )
{
    // #pragma omp parallel for
    for ( int pId = 0; pId < m_nParticles; pId++ )
    {
        // Rigid body
        RigidBody<T, T> const* rb = RB[ m_rigidBodyId[ pId ] ];

        // First, we compute quaternion of orientation
        Quaternion<T> qRot( m_transform[ pId ].getBasis() );
        // Next, we compute accelerations and reset torces
        Kinematics<T> const& acceleration = rb->computeAcceleration( 
                                                                m_torce[ pId ], 
                                                                qRot );
        m_torce[ pId ].reset();
        // Finally, we move particles using the given time integration
        Vector3<T> transMotion;
        Quaternion<T> rotMotion;
        (*TI)->Move( acceleration, 
                     m_velocity[ pId ],
                     transMotion, 
                     rotMotion );
        // and update the transformation of the component
        m_transform[ pId ].updateTransform( transMotion, rotMotion );
        // TODO
        // qRot = qRotChange * qRot;
        // qRotChange = T( 0.5 ) * ( m_velocity[ pId ].getAngularComponent() * qRot );
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerCPU<float>;
template class ComponentManagerCPU<double>;


#undef numCells
#undef numComponents

