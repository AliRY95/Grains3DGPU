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
// Explicit instantiation
template class ComponentManager<float>;
template class ComponentManager<double>;