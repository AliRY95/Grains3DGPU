#include <random>

#include "ComponentManager.hh"
#include "ComponentManagerCPU.hh"

#include "GrainsParameters.hh"


#define numComponents (GrainsParameters::m_numComponents)
// -----------------------------------------------------------------------------
// Constructor with the number of particles randomly positioned in the 
// computational domain
// TODO: Cases with multiple RigidBodies
ComponentManagerCPU::ComponentManagerCPU()
{
    // Randomly initializing transforms
    std::default_random_engine generator;
    std::uniform_real_distribution<double> location( 0., 1. );
    std::uniform_real_distribution<double> angle( 0., 2. * M_PI );

    // Allocating memory on host
    m_transform = new Transform3d[numComponents];
    m_neighborsId = new unsigned int[numComponents * numComponents];
    m_rigidBodyId = new unsigned int[numComponents];
    m_componentId = new unsigned int[numComponents];
    m_neighborsCount = new unsigned int[numComponents];
    // Initialzing the arrays
    for( int i = 0; i < numComponents; i++ )
    {
        // m_transform
        double aX = angle( generator );
        double aY = angle( generator );
        double aZ = angle( generator );
        m_transform[i].setBasis( aX, aY, aZ );
        m_transform[i].setOrigin( Vec3d( location( generator ),
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
ComponentManagerCPU::~ComponentManagerCPU()
{
    delete[] m_transform;
    delete[] m_neighborsId;
    delete[] m_rigidBodyId;
    delete[] m_componentId;
    delete[] m_neighborsCount;
}





// -----------------------------------------------------------------------------
// Gets components transformation
Transform3d* ComponentManagerCPU::getTransform() const
{
    return( m_transform );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor Id
unsigned int* ComponentManagerCPU::getNeighborsId() const
{
    return( m_neighborsId );
}




// -----------------------------------------------------------------------------
// Gets the array of components rigid body Id
unsigned int* ComponentManagerCPU::getRigidBodyId() const
{
    return( m_rigidBodyId );
}




// -----------------------------------------------------------------------------
// Gets the array of component Ids
unsigned int* ComponentManagerCPU::getComponentId() const
{
    return( m_componentId );
}




// -----------------------------------------------------------------------------
// Gets the array of components neighbor count
unsigned int* ComponentManagerCPU::getNeighborsCount() const
{
    return( m_neighborsCount );
}




// -----------------------------------------------------------------------------
// Sets components transformation
void ComponentManagerCPU::setTransform( Transform3d const* tr )
{
    for ( int i = 0; i < numComponents; i++ )
        m_transform[i] = tr[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor Id
void ComponentManagerCPU::setNeighborsId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_neighborsId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components rigid body Id
void ComponentManagerCPU::setRigidBodyId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_rigidBodyId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of component Ids
void ComponentManagerCPU::setComponentId( unsigned int const* id )
{
    for ( int i = 0; i < numComponents; i++ )
        m_componentId[i] = id[i];
}




// -----------------------------------------------------------------------------
// Sets the array of components neighbor count
void ComponentManagerCPU::setNeighborsCount( unsigned int const* id )
{
     for ( int i = 0; i < numComponents; i++ )
        m_neighborsCount[i] = id[i];
}




// -----------------------------------------------------------------------------
// Detects collision between particles
void ComponentManagerCPU::detectCollision( RigidBody const* const* rb, 
                                           bool* result )
{
    for ( int i = 0; i < numComponents; i++ )
    {
        const RigidBody& AA = *( rb[m_rigidBodyId[i]] );
        const Transform3d& trA = m_transform[i];
        for ( int j = 0; j < numComponents; j++ ) // or start from j = i?
            result[i] = intersectRigidBodies( AA, AA, trA, m_transform[j] );
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
}


#undef numComponents