#include "ComponentManager.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
ComponentManager<T>::ComponentManager()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManager<T>::~ComponentManager()
{}




// -----------------------------------------------------------------------------
// Copies data from another ComponentManager object
// It is useful for copying data from host->device or device->host
template <typename T>
void ComponentManager<T>::copy( ComponentManager<T> const* cm )
{
    setRigidBodyId( cm->getRigidBodyId() );
    setRigidBodyIdObstacles( cm->getRigidBodyIdObstacles() );
    setTransform( cm->getTransform() );
    setTransformObstacles( cm->getTransformObstacles() );
    setVelocity( cm->getVelocity() );
    setTorce( cm->getTorce() );
    setParticleId( cm->getParticleId() );
}




// -----------------------------------------------------------------------------
// Initializes the RigidBody IDs and transformations for obstacles
template <typename T>
void ComponentManager<T>::initializeObstacles( 
                            std::vector<unsigned int> numEachUniqueObstacles,
                            std::vector<Transform3<T>> initTr )
{
    std::cout << "Cannot initialize obstacles directly on the device. "
              << "Try initializing on host first, and copy to device. "
              << "Aborting Grains!"
              << std::endl;
    exit( 1 );
}




// -----------------------------------------------------------------------------
// Initializes the RigidBody IDs and transformations for particles
template <typename T>
void ComponentManager<T>::initializeParticles( 
                            std::vector<unsigned int> numEachUniqueParticles,
                            std::vector<Transform3<T>> initTr )
{
    std::cout << "Cannot initialize particles directly on the device. "
              << "Try initializing on host first, and copy to device. "
              << "Aborting Grains!"
              << std::endl;
    exit( 1 );
}




// -----------------------------------------------------------------------------
// Inserts particles according to a given insertion policy
template <typename T>
void ComponentManager<T>::insertParticles( Insertion<T>* ins )
{
    std::cout << "Cannot insert particles directly on the device. "
              << "Try inserting on host first, and copy to device. "
              << "Aborting Grains!"
              << std::endl;
    exit( 1 );
}

    
    
    
// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManager<float>;
template class ComponentManager<double>;