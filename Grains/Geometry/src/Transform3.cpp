#include "Transform3.hh"
#include "MatrixMath.hh"


// -----------------------------------------------------------------------------
// Default constructor. Origin is def and matrix is identity
template <typename T>
__HOSTDEVICE__
Transform3<T>::Transform3( T def )
{
    m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                      T( 0 ), T( 1 ), T( 0 ), 
                      T( 0 ), T( 0 ), T( 1 ) );
    m_origin.setValue( def, def, def );
}




// -----------------------------------------------------------------------------
// Constructor with origin coordinates as inputs and matrix is identity
template <typename T>
__HOSTDEVICE__
Transform3<T>::Transform3( T x, T y, T z )
{
    m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                      T( 0 ), T( 1 ), T( 0 ), 
                      T( 0 ), T( 0 ), T( 1 ) );
    m_origin.setValue( x, y, z );
}




// -----------------------------------------------------------------------------
// Constructor with a 1D array of 12 values as inputs containing the
// rotation matrix coefficients following by the origin coordinates
template <typename T>
__HOSTDEVICE__
Transform3<T>::Transform3( T const t[12] ) 
{ 
    setValue( t );    
}




// -----------------------------------------------------------------------------
// Constructor using an XML node
template <typename T>
__HOST__
Transform3<T>::Transform3( DOMNode* root )
{
    // Initialization of the 1D array containing all the transformation
    // coefficients ordered as Mxx, Mxy, Mxz, Myx, Myy, Myz, Mzx, Mzy, Mzz,
    // Ox, Oy, Oz  
    T t[12] = { 1, 0, 0, 
                0, 1, 0, 
                0, 0, 1, 
                0, 0, 0 };

    // Origin
    DOMNode* pos = ReaderXML::getNode( root, "Centre" );
    if ( pos ) 
    {
        T x = T( ReaderXML::getNodeAttr_Double( pos, "X" ) );
        T y = T( ReaderXML::getNodeAttr_Double( pos, "Y" ) );
        T z = T( ReaderXML::getNodeAttr_Double( pos, "Z" ) );
        setOrigin( Vector3<T>( x, y, z ) );
    }

    // Rotation matrix
    // We only allow rotation matrices, stretching/scaling is prohibited
    DOMNode* angPos = ReaderXML::getNode( root, "AngularPosition" ); 
    string type = ReaderXML::getNodeAttr_String( angPos, "Type" );
    if ( type == "Matrix" )
    {
        Matrix3<T> mat;
        std::string values = ReaderXML::getNodeValue_String( angPos );
        std::istringstream inValues( values.c_str() ); 
        inValues >> mat;
        setBasis( mat );
        // TODO:
        // Check that the matrix is a rotation matrix
        // if ( !m_basis.isRotation() )
        // {
        //     cout << "A matrix in one of the AngularPosition XML nodes is"
        //         << " not a rotation matrix !!!" << endl;
        //     exit(1);
        // }
    }
    else if ( type == "Angles" )
    {
        // read in degree
        T aX = T( ReaderXML::getNodeAttr_Double( angPos, "aX" ) );
        T aY = T( ReaderXML::getNodeAttr_Double( angPos, "aY" ) );
        T aZ = T( ReaderXML::getNodeAttr_Double( angPos, "aZ" ) );
        // change to Radian
        setBasis( RADS_PER_DEG<T> * aX, 
                  RADS_PER_DEG<T> * aY,
                  RADS_PER_DEG<T> * aZ );
    }
    else
    {
        // set basis to zero rotation
        setBasis( T( 0 ), T( 0 ), T( 0 ) );
    }
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
Transform3<T>::Transform3( Transform3<T> const& t )
{
    m_basis = t.m_basis;
    m_origin = t.m_origin;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Transform3<T>::~Transform3()
{}




// -----------------------------------------------------------------------------
// Gets the orientation of the transformation
template <typename T>
__HOSTDEVICE__
Matrix3<T> Transform3<T>::getBasis() const
{
    return ( m_basis );
}




// -----------------------------------------------------------------------------
// Gets the origin of the transformation
template <typename T>
__HOSTDEVICE__
Vector3<T> Transform3<T>::getOrigin() const
{
    return ( m_origin );
}




// -----------------------------------------------------------------------------
// Sets the transformation with an 1D array of 12 values as inputs 
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setValue( T const t[12] ) 
{
    m_basis.setValue( t[0], t[1], t[2], 
                      t[3], t[4], t[5], 
                      t[6], t[7], t[8] );
    m_origin.setValue( t[9], t[10], t[11] );
}




// -----------------------------------------------------------------------------
// Sets the matrix part of the transformation
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setBasis( Matrix3<T> const& m )
{
    m_basis = m;
}




// -----------------------------------------------------------------------------
// Sets the matrix part of the transformation with specified rotations around 
// each principal axis
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setBasis( T aX, 
                              T aY,
                              T aZ )
{
    m_basis = Matrix3<T>( cos(aZ)*cos(aY),
                          cos(aZ)*sin(aY)*sin(aX) - sin(aZ)*cos(aX),
                          cos(aZ)*sin(aY)*cos(aX) + sin(aZ)*sin(aX),
                          sin(aZ)*cos(aY),
                          sin(aZ)*sin(aY)*sin(aX) + cos(aZ)*cos(aX),
                          sin(aZ)*sin(aY)*cos(aX) - cos(aZ)*sin(aX),
                          -sin(aY),
                          cos(aY)*sin(aX),
                          cos(aY)*cos(aX) );
}




// -----------------------------------------------------------------------------
// Sets the origin of the transformation
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setOrigin( Vector3<T> const& v ) 
{
    m_origin = v;
}




// -----------------------------------------------------------------------------
// Sets the transformation to the identity
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setIdentity() 
{
    m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                      T( 0 ), T( 1 ), T( 0 ), 
                      T( 0 ), T( 0 ), T( 1 ) );
    m_origin.setValue( T( 0 ), T( 0 ), T( 0 ) );
}




// -----------------------------------------------------------------------------
// Set the transformation to the inverse of another transformation t
template <typename T>
__HOSTDEVICE__
void Transform3<T>::setToInverseTransform( Transform3<T> const& t ) 
{
    m_basis = t.m_basis.inverse();
    m_origin.setValue( - m_basis[X] * t.m_origin, 
                       - m_basis[Y] * t.m_origin, 
                       - m_basis[Z] * t.m_origin );
}




// -----------------------------------------------------------------------------
// Composition of affine transformations: this = t2 o t1 (t1 first
// followed by t2)
template <typename T>
__HOSTDEVICE__ 
void Transform3<T>::setToTransformsComposition( Transform3<T> const& t1, 
                                                Transform3<T> const& t2 ) 
{  
    m_basis = t2.m_basis * t1.m_basis;
    m_origin = t2( t1.m_origin );
}




// -----------------------------------------------------------------------------
// Composition with a scaling transformation: this = this o scaling
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeWithScaling( Vector3<T> const& v )
{
    T x = v[X], y = v[Y], z = v[Z];
    m_basis[X][X] *= x;
    m_basis[X][Y] *= y;
    m_basis[X][Z] *= z;
    m_basis[Y][X] *= x;
    m_basis[Y][Y] *= y;
    m_basis[Y][Z] *= z;
    m_basis[Z][X] *= x;
    m_basis[Z][Y] *= y;
    m_basis[Z][Z] *= z;
} 




// -----------------------------------------------------------------------------
// Composition on the left by a rotation described by a transform:
// this = rot o this (this first followed by rot)
// This composition leaves the origin unchanged but does not check that rot 
// is indeed a rotation 
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeLeftByRotation( Transform3<T> const& t ) 
{
    m_basis = t.m_basis * m_basis;
}




// -----------------------------------------------------------------------------
// Composition on the left by a rotation described by a quaternion: 
// this = rot( quaternion ) o this ( this first followed by rot( quaternion ) )
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeLeftByRotation( Quaternion<T> const& q ) 
{
    Matrix3<T> const& mm = m_basis; 
    T qx = q[X], qy = q[Y], qz = q[Z], qw = q[W];
    T px, py, pz, pw;
  
    // We compute below the matrix product M_rot(q).basis where
    // M_rot(q) is the rotation matrix corresponding to the quaternion q
    for ( int i = 0; i < 3; ++i )
    {
        px = qy * mm[Z][i] - qz * mm[Y][i] + qw * mm[X][i];
        py = qz * mm[X][i] - qx * mm[Z][i] + qw * mm[Y][i];    
        pz = qx * mm[Y][i] - qy * mm[X][i] + qw * mm[Z][i]; 
        pw = - qx * mm[X][i] - qy * mm[Y][i] - qz * mm[Z][i];
        m_basis[X][i] = qy * pz - qz * py - pw * qx + qw * px;
        m_basis[Y][i] = qz * px - qx * pz - pw * qy + qw * py;          
        m_basis[Z][i] = qx * py - qy * px - pw * qz + qw * pz;    
    }
}




// -----------------------------------------------------------------------------
// Composition on the left by a translation:
// this = trans(vector) o this (this first followed by trans(vector))
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeLeftByTranslation( Vector3<T> const& v )
{ 
    m_origin += v;
}




// -----------------------------------------------------------------------------
// Composition on the left by another affine transformation: 
// this = t o this (this first followed by t)
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeLeftByTransform( Transform3<T> const& t ) 
{
    m_origin = t.m_origin + t.m_basis * m_origin;
    m_basis = t.m_basis * m_basis;
}




// -----------------------------------------------------------------------------
// Composition on the right by another affine transformation: 
// this = this o t (t first followed by this)
template <typename T>
__HOSTDEVICE__
void Transform3<T>::composeRightByTransform( Transform3<T> const& t ) 
{
    m_origin += m_basis * t.m_origin;
    m_basis *= t.m_basis;
}




// -----------------------------------------------------------------------------
// Relative transformation with respect to t
template <typename T>
__HOSTDEVICE__
void Transform3<T>::relativeToTransform( Transform3<T> const& t ) 
{
    Matrix3<T> const inverseRotation = ( t.m_basis ).transpose();
    m_basis = inverseRotation * m_basis;
    m_origin = inverseRotation * ( m_origin - t.m_origin );
}




// -----------------------------------------------------------------------------
// Updates the transformation with a displacement and a rotation
template <typename T>
__HOSTDEVICE__
void Transform3<T>::updateTransform( Vector3<T> const& transMotion,
                                     Quaternion<T> const& rotMotion ) 
{
    this->composeLeftByTranslation( transMotion );
    this->composeLeftByRotation( rotMotion );
}




// -----------------------------------------------------------------------------
// Returns the result of applying the transformation to the input vector
template <typename T>
__HOSTDEVICE__
Vector3<T> Transform3<T>::operator () ( Vector3<T> const& v ) const 
{
    return ( Vector3<T>( m_basis[X] * v + m_origin[X], 
                         m_basis[Y] * v + m_origin[Y], 
                         m_basis[Z] * v + m_origin[Z] ) );
}




// -----------------------------------------------------------------------------
// Equal operator to another Transform object
template <typename T>
__HOSTDEVICE__
Transform3<T>& Transform3<T>::operator = ( Transform3<T> const& t )
{
    if ( &t != this )
    {  
        m_basis = t.m_basis;
        m_origin = t.m_origin;
    }
    return ( *this );
}




// -----------------------------------------------------------------------------
// Conversion operator to float
template <>
__HOSTDEVICE__
Transform3<double>::operator Transform3<float> () const
{
    Matrix3<double> const m = m_basis;
    Vector3<double> const v = m_origin;
    float const t[12] = { (float) m[X][X], (float) m[X][Y], (float) m[X][Z],
                          (float) m[Y][X], (float) m[Y][Y], (float) m[Y][Z],
                          (float) m[Z][X], (float) m[Z][Y], (float) m[Z][Z],
                          (float) v[X]   , (float) v[Y]   , (float) v[X]    };
    return ( Transform3<float> ( t ) );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
std::ostream& operator << ( std::ostream& fileOut, 
                            Transform3<T> const& t )
{
    fileOut << "Position: " << std::endl;
    fileOut << t.getOrigin() << std::endl;
    fileOut << "Orientation: " << std::endl;
    fileOut << t.getBasis();
    return ( fileOut );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
std::istream& operator >> ( std::istream& fileIn, 
                            Transform3<T>& t )
{
    Vector3<T> v;
    Matrix3<T> m;

    fileIn >> v;
    fileIn >> m;

    t.setOrigin( v );
    t.setBasis( m );
    
    return ( fileIn );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Transform3<float>;
template class Transform3<double>;

#define X( T ) \
template std::ostream& operator << <T>( std::ostream& fileOut,                 \
                                        Transform3<T> const& t );              \
                                                                               \
template std::istream& operator >> <T>( std::istream& fileIn,                  \
                                        Transform3<T>& t );
X( float )
X( double )
#undef X