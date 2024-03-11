#include "Vector3.hh"
#include "Kinematics.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__
Kinematics::Kinematics()
{}




// -----------------------------------------------------------------------------
// Constructor with a u and a omega as input parameters
template <typename T>
__host__ __device__
Kinematics::Kinematics( Vector3<T> const& u, Vector3<T> const& omega )
: m_translationalVelocity( u )
, m_angularVelocity( omega )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__
Kinematics::~Kinematics()
{}




// -----------------------------------------------------------------------------
// Gets translational velocity of the kinematics
template <typename T>
__host__ __device__
Vector3<T> Kinematics::getTranslationalVelocity() const
{
    return ( m_translationalVelocity );
}




// -----------------------------------------------------------------------------
// Gets angular velocity of the kinematics
template <typename T>
__host__ __device__
Vector3<T> Kinematics::getAngularVelocity() const
{
    return ( m_angularVelocity );
}




// -----------------------------------------------------------------------------
// Sets the translational velocity of the kinematics
template <typename T>
__host__ __device__
void Kinematics::setTranslationalVelocity( Vector3<T> const& u )
{
    m_translationalVelocity = u;
}




// -----------------------------------------------------------------------------
// Sets the angular velocity of the kinematics
template <typename T>
__host__ __device__
void Kinematics::setForce( Vector3<T> const& omega )
{
    m_angularVelocity = omega;
}




// -----------------------------------------------------------------------------
// Returns the total velocity U + om x R given R 
template <typename T>
__host__ __device__
Vector3<T> Kinematics::Velocity( Vector3<T> const& R )
{
    return ( m_translationalVelocity + ( m_angularVelocity ^ R ) );
}


