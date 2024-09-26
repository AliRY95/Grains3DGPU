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
// Inserts particles according to a given insertion policy
template <typename T>
void ComponentManager<T>::insertParticles( Insertion<T> const* ins )
{
    std::cout << "Cannot insert particles directly on the device."
              << " Aborting Grains!"
              << std::endl;
    exit( 1 );
}

    
    
    
// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManager<float>;
template class ComponentManager<double>;