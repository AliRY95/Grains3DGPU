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
// Explicit instantiation
template class ComponentManager<float>;
template class ComponentManager<double>;