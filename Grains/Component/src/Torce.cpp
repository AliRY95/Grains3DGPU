#include "Vector3.hh"
#include "Torce.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__
Torce::Torce()
{}




// -----------------------------------------------------------------------------
// Constructor with a torque and a force as input parameters
template <typename T>
__host__ __device__
Torce::Torce( Vector3<T> const& t, Vector3<T> const& f )
: m_torque( t )
, m_force( f )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__
Torce::~Torce()
{}




// -----------------------------------------------------------------------------
// Gets the total torque of the torce
template <typename T>
__host__ __device__
Vector3<T> Torce::getTorque() const
{
    return ( m_torque );
}




// -----------------------------------------------------------------------------
// Gets the total force of the torce
template <typename T>
__host__ __device__
Vector3<T> Torce::getForce() const
{
    return ( m_force );
}




// -----------------------------------------------------------------------------
// Sets the total torque of the torce
template <typename T>
__host__ __device__
void Torce::setTorque( Vector3<T> const& t )
{
    m_torque = t;
}




// -----------------------------------------------------------------------------
// Sets the total force of the torce
template <typename T>
__host__ __device__
void Torce::setForce( Vector3<T> const& f )
{
    m_force = f;
}




// -----------------------------------------------------------------------------
// Adds a force to the torce
template <typename T>
__host__ __device__
void Torce::addTorque( Vector3<T> const& t )
{
    m_torque += t;
}




// -----------------------------------------------------------------------------
// Adds a force to the torce
template <typename T>
__host__ __device__
void Torce::addForce( Vector3<T> const& f )
{
    m_force += f;
}




