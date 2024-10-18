#include "RigidBody.hh"
#include "ConvexBuilderFactory.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"
#include "QuaternionMath.hh"


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
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody( Convex<T>* convex, 
                            T ct,
                            unsigned int material,
                            T density )
: m_convex( convex )
, m_crustThickness( ct )
, m_material( material )
{
    // Scaling vector
    Vector3<T> boundingBox = m_convex->computeBoundingBox();
    m_scaling[X] = ( boundingBox[X] - m_crustThickness ) / boundingBox[X];
    m_scaling[Y] = ( boundingBox[Y] - m_crustThickness ) / boundingBox[Y];
    m_scaling[Z] = ( boundingBox[Z] - m_crustThickness ) / boundingBox[Z];
    // Volume and mass
    m_volume = m_convex->computeVolume();
    m_mass = density * m_volume;
    // Considering the density for tensor of inertia
    if ( density == 0 )
    {
        for ( int i = 0; i < 6; i++ )
        {
            m_inertia[i] = 0;
            m_inertia_1[i] = 0; 
        }
    }
    else
    {
        // Storing inertia and inverse of it
        m_convex->computeInertia( m_inertia, m_inertia_1 );
        for ( int i = 0; i < 6; i++ )
        {
            m_inertia[i] *= density;
            m_inertia_1[i] /= density; 
        }
    }
    // Last, bounding volume and circumscribed radius
    // We cast type T to U just in case they are different.
    // It happens only at the start when the rigid body is created.
    m_boundingBox = new BoundingBox( Vector3<U>( boundingBox ) );
    m_circumscribedRadius = U( m_convex->computeCircumscribedRadius() );
}




// -----------------------------------------------------------------------------
// Constructor with an XML input
template <typename T, typename U>
__HOST__
RigidBody<T, U>::RigidBody( DOMNode* root )
{
    // Convex
    DOMNode* shape = ReaderXML::getNode( root, "Convex" );
    m_convex = ConvexBuilderFactory<T>::create( shape );
    // Crust thickenss
    m_crustThickness = 
                T( ReaderXML::getNodeAttr_Double( shape, "CrustThickness" ) );
    // Scaling vector
    Vector3<T> boundingBox = m_convex->computeBoundingBox();
    m_scaling[X] = ( boundingBox[X] - m_crustThickness ) / boundingBox[X];
    m_scaling[Y] = ( boundingBox[Y] - m_crustThickness ) / boundingBox[Y];
    m_scaling[Z] = ( boundingBox[Z] - m_crustThickness ) / boundingBox[Z];
    // Material
    std::string material = ReaderXML::getNodeAttr_String( root, "Material" );
    // checking if the material name is already defined.
    // If yes, we access the ID and store it for the rigid body.
    // If it is not, we add the material to the map.
    if ( GrainsParameters<T>::m_materialMap.count( material ) )
         m_material = GrainsParameters<T>::m_materialMap[ material ];
    else
    {
        // Getting the ID of the last material added to the map. 
        // This is basically the same as the size of the map.
        unsigned int id = GrainsParameters<T>::m_materialMap.size();
        GrainsParameters<T>::m_materialMap.emplace( material, id );
    }
    // Volume and mass
    m_volume = m_convex->computeVolume();
    T density = T( ReaderXML::getNodeAttr_Double( root, "Density" ) );
    m_mass = density * m_volume;
    // Considering the density for tensor of inertia
    if ( density == 0 )
    {
        for ( int i = 0; i < 6; i++ )
        {
            m_inertia[i] = 0;
            m_inertia_1[i] = 0; 
        }
    }
    else
    {
        // Storing inertia and inverse of it
        m_convex->computeInertia( m_inertia, m_inertia_1 );
        for ( int i = 0; i < 6; i++ )
        {
            m_inertia[i] *= density;
            m_inertia_1[i] /= density; 
        }
    }
    // Last, bounding volume and circumscribed radius
    // We cast type T to U just in case they are different.
    // It happens only at the start when the rigid body is created.
    m_boundingBox = new BoundingBox( Vector3<U>( boundingBox ) );
    m_circumscribedRadius = U( m_convex->computeCircumscribedRadius() );
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody( RigidBody<T, U> const& rb )
: m_convex( NULL )
, m_crustThickness( rb.m_crustThickness )
, m_scaling( rb.m_scaling )
, m_material( rb.m_material )
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
// Gets the rigid body's convex
template <typename T, typename U>
__HOSTDEVICE__
Convex<T>* RigidBody<T, U>::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's crust thickness
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Gets the scaling vector related to crust thickness
template <typename T, typename U>
__HOSTDEVICE__
Vector3<T> RigidBody<T, U>::getScalingVector() const
{
    return ( m_scaling );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's material ID
template <typename T, typename U>
__HOSTDEVICE__
unsigned int RigidBody<T, U>::getMaterial() const
{
    return ( m_material );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's volume
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's volume
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getMass() const
{
    return ( m_mass );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
void RigidBody<T, U>::getInertia( T (&inertia)[6] ) const
{
    for ( int i = 0; i < 6; ++i )
        inertia[i] = m_inertia[i];
}




// -----------------------------------------------------------------------------
// Gets the inverse of rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
void RigidBody<T, U>::getInertia_1( T (&inertia_1)[6] ) const
{
    for ( int i = 0; i < 6; ++i )
        inertia_1[i] = m_inertia_1[i];
}




// -----------------------------------------------------------------------------
// Gets the rigid body's bounding box
template <typename T, typename U>
__HOSTDEVICE__
BoundingBox<U>* RigidBody<T, U>::getBoundingBox() const
{
    return ( m_boundingBox );
}




// -----------------------------------------------------------------------------
// Gets the rigid body's circumscribed radius
template <typename T, typename U>
__HOSTDEVICE__
U RigidBody<T, U>::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




// -----------------------------------------------------------------------------
// Computes the acceleration of the rigid body given a torce and angular 
// velocity in the body-fixed coordinate system -- In the body-fixed coordinate 
// system, the moment of inertia tensor is assumed to be diagonal.
template <typename T, typename U>
__HOSTDEVICE__
Kinematics<T> RigidBody<T, U>::computeMomentum( Vector3<T> const& omega,
                                                Torce<T> const& t ) const
{
    // Translational momentum
    Vector3<T> transMomentum( t.getForce() / m_mass );

    // Angular momentum
    // Torque
    Vector3<T> angMomentum( t.getTorque() );
    // Compute T + (I.w) ^ w in the body-fixed coordinates system     
    angMomentum[0] += ( m_inertia[3] - m_inertia[5] ) * omega[Y] * omega[Z];
    angMomentum[1] += ( m_inertia[5] - m_inertia[0] ) * omega[X] * omega[Z];
    angMomentum[2] += ( m_inertia[0] - m_inertia[3] ) * omega[X] * omega[Y];
    // Compute I^-1.(T + w ^ (I.w)) in the body-fixed coordinates system     
    angMomentum[0] *= m_inertia_1[0];
    angMomentum[1] *= m_inertia_1[3];
    angMomentum[2] *= m_inertia_1[5];

    return( Kinematics<T>( transMomentum, angMomentum ) );
}




// -----------------------------------------------------------------------------
// Computes the acceleration of the rigid body given the angular velocity and a
// torce in the space-fixed coordinate system
template <typename T, typename U>
__HOSTDEVICE__
Kinematics<T> RigidBody<T, U>::computeMomentum( Vector3<T> const& omega,
                                                Torce<T> const& t,
                                                Quaternion<T> const& q ) const
{
    // Translational momentum
    Vector3<T> transMomentum( t.getForce() / m_mass );

    // Angular momentum
    // Quaternion and rotation quaternion conjugate
    Quaternion<T> qCon( q.conjugate() );
    // Write omega in the body-fixed coordinates system
    Vector3<T> angVelocity( qCon.multToVector3( omega * q ) );
    // Write torque in the body-fixed coordinates system
    Vector3<T> angMomentum( qCon.multToVector3( t.getTorque() * q ) );

    // Compute I.w in the body-fixed coordinates system
    Vector3<T> angMomentumTemp;
    angMomentumTemp[0] = m_inertia[0] * angVelocity[0] + 
                         m_inertia[1] * angVelocity[1] + 
                         m_inertia[2] * angVelocity[2];
    angMomentumTemp[1] = m_inertia[1] * angVelocity[0] +
                         m_inertia[3] * angVelocity[1] +
                         m_inertia[4] * angVelocity[2];
    angMomentumTemp[2] = m_inertia[2] * angVelocity[0] + 
                         m_inertia[4] * angVelocity[1] +
                         m_inertia[5] * angVelocity[2];

    // Compute T + I.w ^ w in the body-fixed coordinates system 
    angMomentum += angMomentumTemp ^ angVelocity;
    
    // Compute I^-1.(T + I.w ^ w) in body-fixed coordinates system 
    angMomentumTemp[0] = m_inertia_1[0] * angMomentum[0] + 
                         m_inertia_1[1] * angMomentum[1] + 
                         m_inertia_1[2] * angMomentum[2];
    angMomentumTemp[1] = m_inertia_1[1] * angMomentum[0] +
                         m_inertia_1[3] * angMomentum[1] +
                         m_inertia_1[4] * angMomentum[2];
    angMomentumTemp[2] = m_inertia_1[2] * angMomentum[0] + 
                         m_inertia_1[4] * angMomentum[1] +
                         m_inertia_1[5] * angMomentum[2];
    // Write I^-1.(T + I.w ^ w) in space-fixed coordinates system
    angMomentum = q.multToVector3( angMomentumTemp * qCon );

    return( Kinematics<T>( transMomentum, angMomentum ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class RigidBody<float, float>;
template class RigidBody<double, float>;
template class RigidBody<double, double>;