#include "Kinematics.hh"
#include "Vector3.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Kinematics<T>::Kinematics()
{}




// -----------------------------------------------------------------------------
// Constructor with a u and a omega as input parameters
template <typename T>
__HOSTDEVICE__
Kinematics<T>::Kinematics( Vector3<T> const& u, 
                           Vector3<T> const& omega )
: m_translationalVelocity( u )
, m_angularVelocity( omega )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Kinematics<T>::~Kinematics()
{}




// -----------------------------------------------------------------------------
// Gets translational velocity of the kinematics
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::getTranslationalVelocity() const
{
    return ( m_translationalVelocity );
}




// -----------------------------------------------------------------------------
// Gets angular velocity of the kinematics
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::getAngularVelocity() const
{
    return ( m_angularVelocity );
}




// -----------------------------------------------------------------------------
// Sets the translational velocity of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::setTranslationalVelocity( Vector3<T> const& u )
{
    m_translationalVelocity = u;
}




// -----------------------------------------------------------------------------
// Sets the angular velocity of the kinematics
template <typename T>
__HOSTDEVICE__
void Kinematics<T>::setAngularVelocity( Vector3<T> const& omega )
{
    m_angularVelocity = omega;
}




// -----------------------------------------------------------------------------
// Returns the total velocity U + om x R given R 
template <typename T>
__HOSTDEVICE__
Vector3<T> Kinematics<T>::Velocity( Vector3<T> const& R ) const
{
    return ( m_translationalVelocity + ( m_angularVelocity ^ R ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Kinematics<float>;
template class Kinematics<double>;