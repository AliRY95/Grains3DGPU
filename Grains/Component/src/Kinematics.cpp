#include "Kinematics.hh"
#include "Vector3.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Kinematics<T>::Kinematics()
: m_translational( zeroVector3T )
, m_angular( zeroVector3T )
{}




// -----------------------------------------------------------------------------
// Constructor with two vectors as input parameters
template <typename T>
__HOSTDEVICE__
Kinematics<T>::Kinematics( Vector3<T> const& translational, 
                           Vector3<T> const& angular )
: m_translational( translational )
, m_angular( angular )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Kinematics<T>::~Kinematics()
{}




// -----------------------------------------------------------------------------
// Gets the translational component of the kinematics
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::getTranslationalComponent() const
{
    return ( m_translational );
}




// -----------------------------------------------------------------------------
// Gets the angular component of the kinematics
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::getAngularComponent() const
{
    return ( m_angular );
}




// -----------------------------------------------------------------------------
// Sets the translational component of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::setTranslationalComponent( Vector3<T> const& translational )
{
    m_translational = translational;
}




// -----------------------------------------------------------------------------
// Sets the angular component of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::setAngularComponent( Vector3<T> const& angular )
{
    m_angular = angular;
}




// -----------------------------------------------------------------------------
// Adds a vector to the translational component of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::addToTranslationalComponent( 
                                            Vector3<T> const& translational )
{
    m_translational += translational;
}




// -----------------------------------------------------------------------------
// Adds a vector to the angular component of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::addToAngularComponent( Vector3<T> const& omega )
{
    m_angular += omega;
}




// -----------------------------------------------------------------------------
// Returns the total velocity U + om x R given R 
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::kinematicsAtPoint( Vector3<T> const& R ) const
{
    // return ( m_translational + ( m_angular ^ R ) );
    return ( m_translational );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Kinematics<float>;
template class Kinematics<double>;