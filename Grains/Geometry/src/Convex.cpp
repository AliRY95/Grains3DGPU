#include "Convex.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Convex<T>::Convex()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Convex<T>::~Convex()
{}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Convex<float>;
template class Convex<double>;