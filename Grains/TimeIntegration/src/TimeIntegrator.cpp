#include "TimeIntegrator.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::TimeIntegrator()
{}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::TimeIntegrator( TimeIntegrator<T> const& ti )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::~TimeIntegrator()
{}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class TimeIntegrator<float>;
template class TimeIntegrator<double>;