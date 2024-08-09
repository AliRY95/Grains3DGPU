#include "RigidBody.hh"
#include "ConvexBuilderFactory.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody()
: m_convex( NULL )
, m_boundingBox( NULL )
{}




// -----------------------------------------------------------------------------
// Constructor with a convex and the crust thickness
// TODO: inertia
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody( Convex<T>* convex, 
                            T ct,
                            T density )
: m_convex( convex )
, m_crustThickness( ct )
{
    m_volume = m_convex->computeVolume();
    m_mass = density * m_volume;
    m_convex->computeInertia( m_inertia, m_inertia_1 );
    // We cast type T to U just in case they are different.
    // It happens only at the start when the rigid body is created.
    m_boundingBox = new 
                    BoundingBox( Vector3<U>( m_convex->computeBoundingBox() ) );
    m_circumscribedRadius = U( m_convex->computeCircumscribedRadius() );
}




// -----------------------------------------------------------------------------
// Constructor with an XML input
// TODO: inertia
template <typename T, typename U>
__HOST__
RigidBody<T, U>::RigidBody( DOMNode* root )
{
    // Convex
    DOMNode* shape = ReaderXML::getNode( root, "Convex" );
    m_convex = ConvexBuilderFactory<T>::create( shape );
    m_crustThickness = 
                T( ReaderXML::getNodeAttr_Double( shape, "CrustThickness" ) );
    m_volume = m_convex->computeVolume();
    T density = T( ReaderXML::getNodeAttr_Double( root, "Density" ) );
    m_mass = density * m_volume;
    m_convex->computeInertia( m_inertia, m_inertia_1 );
    // We cast type T to U just in case they are different.
    // It happens only at the start when the rigid body is created.
    m_boundingBox = new 
                    BoundingBox( Vector3<U>( m_convex->computeBoundingBox() ) );
    m_circumscribedRadius = U( m_convex->computeCircumscribedRadius() );
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody( RigidBody<T, U> const& rb )
: m_convex( NULL )
, m_crustThickness( rb.m_crustThickness )
, m_volume( rb.m_volume )
, m_mass( rb.m_mass )
, m_boundingBox( NULL )
, m_circumscribedRadius( rb.m_circumscribedRadius )
{
    if ( rb.m_convex )
        m_convex = rb.m_convex->clone();
    if ( rb.m_boundingBox )
        m_boundingBox = rb.m_boundingBox->clone();
    for ( int i = 0; i < 6; ++i )
    {
        m_inertia[i] = rb.m_inertia[i];
        m_inertia_1[i] = rb.m_inertia_1[i];
    }
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::~RigidBody()
{
    delete m_convex;
    delete m_boundingBox;
}




// -----------------------------------------------------------------------------
// Returns the rigid body's convex
template <typename T, typename U>
__HOSTDEVICE__
Convex<T>* RigidBody<T, U>::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's crust thickness
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getMass() const
{
    return ( m_mass );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
void RigidBody<T, U>::getInertia( T (&inertia)[6] ) const
{
    for ( int i = 0; i < 6; ++i )
        inertia[i] = m_inertia[i];
}




// -----------------------------------------------------------------------------
// Returns the inverse of rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
void RigidBody<T, U>::getInertia_1( T (&inertia_1)[6] ) const
{
    for ( int i = 0; i < 6; ++i )
        inertia_1[i] = m_inertia_1[i];
}




// -----------------------------------------------------------------------------
// Returns the rigid body's bounding box
template <typename T, typename U>
__HOSTDEVICE__
BoundingBox<U>* RigidBody<T, U>::getBoundingBox() const
{
    return ( m_boundingBox );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's circumscribed radius
template <typename T, typename U>
__HOSTDEVICE__
U RigidBody<T, U>::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




// -----------------------------------------------------------------------------
// // Computes the acceleration of the rigid body given a torce
// template <typename T, typename U>
// __HOSTDEVICE__
// Kinematics<T> RigidBody<T, U>::computeAcceleration( Torce<T> const& t ) const
// {
//     Kinematics<T> acceleration;

//     // Translational momentum
//     acceleration.setTranslationalVelocity( t.getForce() / m_mass );


//     // Rotation quaternion conjugate
//     Quaternion<T> pConjugate = m_QuaternionRotation.Conjugate(); 

//     // Write torque in body-fixed coordinates system
//     work = pConjugate.multToVector3( *( t.getTorque() ), 
//                                      m_QuaternionRotation );

//     // Compute I^-1.(T + I.w ^ w) in body-fixed coordinates system 
//     vTmp[0] = m_inertia_1[0] * work[0] + 
//               m_inertia_1[1] * work[1] + 
//               m_inertia_1[2] * work[2];
//     vTmp[1] = m_inertia_1[1] * work[0] +
//               m_inertia_1[3] * work[1] +
//               m_inertia_1[4] * work[2];
//     vTmp[2] = m_inertia_1[2] * work[0] + 
//               m_inertia_1[4] * work[1] +
//               m_inertia_1[5] * work[2];
  
//     // Write I^-1.(T + I.w ^ w) in space-fixed coordinates system
//     work = m_QuaternionRotation.multToVector3( vTmp, pConjugue );

//     // Compute m_dOmegadt
//     acceleration.setAngularVelocity( work );

//     return( acceleration );
// }




// -----------------------------------------------------------------------------
// Explicit instantiation
template class RigidBody<float, float>;
template class RigidBody<double, float>;
template class RigidBody<double, double>;