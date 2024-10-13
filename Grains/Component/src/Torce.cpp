#include "Torce.hh"
#include "Vector3.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Torce<T>::Torce()
: m_torque( zeroVector3T )
, m_force( zeroVector3T )
{}




// -----------------------------------------------------------------------------
// Constructor with a torque and a force as input parameters
template <typename T>
__HOSTDEVICE__
Torce<T>::Torce( Vector3<T> const& t, 
                 Vector3<T> const& f )
: m_torque( t )
, m_force( f )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Torce<T>::~Torce()
{}




// -----------------------------------------------------------------------------
// Gets the total torque of the torce
template <typename T>
__HOSTDEVICE__
Vector3<T> Torce<T>::getTorque() const
{
    return ( m_torque );
}




// -----------------------------------------------------------------------------
// Gets the total force of the torce
template <typename T>
__HOSTDEVICE__
Vector3<T> Torce<T>::getForce() const
{
    return ( m_force );
}




// -----------------------------------------------------------------------------
// Sets the total torque of the torce
template <typename T>
__HOSTDEVICE__
void Torce<T>::setTorque( Vector3<T> const& t )
{
    m_torque = t;
}




// -----------------------------------------------------------------------------
// Sets the total force of the torce
template <typename T>
__HOSTDEVICE__
void Torce<T>::setForce( Vector3<T> const& f )
{
    m_force = f;
}




// -----------------------------------------------------------------------------
// Resets the torce
template <typename T>
__HOSTDEVICE__
void Torce<T>::reset()
{
    m_torque.reset();
    m_force.reset();
}




// -----------------------------------------------------------------------------
// Adds a force to the torce
template <typename T>
__HOSTDEVICE__
void Torce<T>::addTorque( Vector3<T> const& t )
{
    m_torque += t;
}




// -----------------------------------------------------------------------------
// Adds a force to the torce
template <typename T>
__HOSTDEVICE__
void Torce<T>::addForce( Vector3<T> const& f )
{
    m_force += f;
}




// -----------------------------------------------------------------------------
// Adds a force to the torce with accounting for the additional torque
template <typename T>
__HOSTDEVICE__
void Torce<T>::addForce( Vector3<T> const& f,
                         Vector3<T> const& p )
{
    m_force += f;
    m_torque += ( p ^ f );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
std::ostream& operator << ( std::ostream& fileOut, 
                            Torce<T> const& t )
{
    fileOut << t.getTorque() << std::endl << t.getForce();
    return ( fileOut );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
std::istream& operator >> ( std::istream& fileIn, 
                            Torce<T>& t )
{
    Vector3<T> vec;
    fileIn >> vec;
    t.setTorque( vec );
    fileIn >> vec;
    t.setForce( vec );
    return ( fileIn );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Torce<float>;
template class Torce<double>;

#define X( T ) \
template std::ostream& operator << <T>( std::ostream& fileOut,                 \
                                        Torce<T> const& t );              \
                                                                               \
template std::istream& operator >> <T>( std::istream& fileIn,                  \
                                        Torce<T>& t );
X( float )
X( double )
#undef X