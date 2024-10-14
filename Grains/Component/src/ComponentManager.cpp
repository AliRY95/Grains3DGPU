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
template <typename T>
void ComponentManager<T>::copy( ComponentManager<T> const* cm )
{
    setRigidBodyId( cm->getRigidBodyId() );
    setTransform( cm->getTransform() );
    setVelocity( cm->getVelocity() );
    setTorce( cm->getTorce() );
    setComponentId( cm->getComponentId() );
}




// -----------------------------------------------------------------------------
// Initializes the RigidBody IDs and transformations for the simulation
template <typename T>
void ComponentManager<T>::initialize( std::vector<unsigned int> numEachRigidBody,
                                      std::vector<Transform3<T>> initTr )
{
    std::cout << "Cannot initialize directly on the device. "
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