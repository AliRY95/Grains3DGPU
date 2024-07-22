#include "ContactInfo.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__ 
ContactInfo<T>::ContactInfo()
{}




// -----------------------------------------------------------------------------
// Constructor with min and max points along with extent of each cell
template <typename T>
__HOSTDEVICE__ 
ContactInfo<T>::ContactInfo( Vector3<T> const& pt, 
                             Vector3<T> const& vec, 
                             T overlap )
: m_contactPoint( pt )
, m_contactVector( vec )
, m_overlapDistance( overlap )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ 
ContactInfo<T>::~ContactInfo()
{}




// -----------------------------------------------------------------------------
// Gets the contact point
template <typename T>
__HOSTDEVICE__ 
Vector3<T> ContactInfo<T>::getContactPoint() const
{
    return ( m_contactPoint );
}




// -----------------------------------------------------------------------------
// Gets the contact vector
template <typename T>
__HOSTDEVICE__ 
Vector3<T> ContactInfo<T>::getContactVector() const
{
    return ( m_contactVector );
}




// -----------------------------------------------------------------------------
// Gets the overlap distance
template <typename T>
__HOSTDEVICE__ 
T ContactInfo<T>::getOverlapDistance() const
{
    return ( m_overlapDistance );
}




// -----------------------------------------------------------------------------
// Sets the contact point
template <typename T>
__HOSTDEVICE__ 
void ContactInfo<T>::setContactPoint( Vector3<T> const& p )
{
    m_contactPoint = p;
}




// -----------------------------------------------------------------------------
// Sets the contact vector
template <typename T>
__HOSTDEVICE__ 
void ContactInfo<T>::setContactVector( Vector3<T> const& v )
{
    m_contactVector = v;
}




// -----------------------------------------------------------------------------
// Sets the overlap distance
template <typename T>
__HOSTDEVICE__ 
void ContactInfo<T>::setOverlapDistance( T d )
{
    m_overlapDistance = d;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ContactInfo<float>;
template class ContactInfo<double>;