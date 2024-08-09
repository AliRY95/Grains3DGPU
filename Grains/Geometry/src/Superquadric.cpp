#include "Superquadric.hh"
#include "MiscMath.hh"


// multiple of 4
#define visuNodeNbOnPer 16 


// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
template <typename T>
__HOSTDEVICE__
Superquadric<T>::Superquadric( T a,
                               T b,
                               T c, 
                               T n1, 
                               T n2 )
: m_a( a )
, m_b( b )
, m_c( c )
, m_n1( n1 )
, m_n2( n2 )
{}




// -----------------------------------------------------------------------------
// Constructor with an input stream
template <typename T>
__HOST__
Superquadric<T>::Superquadric( std::istream& fileIn )
{
    readConvex( fileIn );
}




// -----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
template <typename T>
__HOST__
Superquadric<T>::Superquadric( DOMNode* root )
{
    m_a  = T( ReaderXML::getNodeAttr_Double( root, "a" ) );
    m_b  = T( ReaderXML::getNodeAttr_Double( root, "b" ) );
    m_c  = T( ReaderXML::getNodeAttr_Double( root, "c" ) );
    m_n1 = T( ReaderXML::getNodeAttr_Double( root, "n1" ) );
    m_n2 = T( ReaderXML::getNodeAttr_Double( root, "n2" ) );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Superquadric<T>::~Superquadric()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Superquadric<T>::getConvexType() const
{
    return ( SUPERQUADRIC );
}




// -----------------------------------------------------------------------------
// Returns the extent in a Vector3 format
template <typename T>
__HOSTDEVICE__
Vector3<T> Superquadric<T>::getExtent() const
{
    return ( Vector3<T>( m_a, m_b, m_c ) );
}




// -----------------------------------------------------------------------------
// Returns the exponents (blockiness) in a Vector3 format with Z = 0
template <typename T>
__HOSTDEVICE__
Vector3<T> Superquadric<T>::getExponent() const
{
    return ( Vector3<T>( m_n1, m_n2, T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// Returns a clone of the superquadric
template <typename T>
__HOSTDEVICE__
Convex<T>* Superquadric<T>::clone() const
{
    return ( new Superquadric<T>( m_a, m_b, m_c, m_n1, m_n2 ) );
}




// -----------------------------------------------------------------------------
// Returns the volume of the Superquadric
template <typename T>
__HOSTDEVICE__
T Superquadric<T>::computeVolume() const
{
#ifdef __CUDA_ARCH__
    T const C = T( 2 ) / T( 3 ) * m_a * m_b * m_c;
    T const eps1 = T( 2 ) / m_n1;
    T const eps2 = T( 2 ) / m_n2;

    return ( C * eps1 * eps2 * grainsBeta( eps1, T( 0.5 ) * eps1 ) * 
                               grainsBeta( T( 0.5 ) * eps2, T( 0.5 ) * eps2 ) );
#else
    T const C = T( 2 ) / T( 3 ) * m_a * m_b * m_c;
    T const eps1 = T( 2 ) / m_n1;
    T const eps2 = T( 2 ) / m_n2;

    return ( C * eps1 * eps2 * std::beta( eps1, T( 0.5 ) * eps1 ) * 
                               std::beta( T( 0.5 ) * eps2, T( 0.5 ) * eps2 ) );
#endif                            
}




// -----------------------------------------------------------------------------
// Returns the volume of the Superquadric - specialized for floats
template <>
__HOSTDEVICE__
float Superquadric<float>::computeVolume() const
{
#ifdef __CUDA_ARCH__
    float const C = 2.f / 3.f * m_a * m_b * m_c;
    float const eps1 = 2.f / m_n1;
    float const eps2 = 2.f / m_n2;

    return ( C * eps1 * eps2 * grainsBetaf( eps1, 0.5f * eps1 ) * 
                               grainsBetaf( 0.5f * eps2, 0.5f * eps2 ) );
#else
    float const C = 2.f / 3.f * m_a * m_b * m_c;
    float const eps1 = 2.f / m_n1;
    float const eps2 = 2.f / m_n2;

    return ( C * eps1 * eps2 * std::betaf( eps1, 0.5f * eps1 ) * 
                               std::betaf( 0.5f * eps2, 0.5f * eps2 ) );
#endif                            
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
void Superquadric<T>::computeInertia( T (&inertia)[6], 
                                      T (&inertia_1)[6] ) const
{
#ifdef __CUDA_ARCH__
    T const eps1 = T( 2 ) / m_n1;
    T const eps2 = T( 2 ) / m_n2;
    T const C = T( 0.4 ) * m_a * m_b * m_c * eps1 * eps2 ;

    T const prod1 = grainsBeta( T( 1.5 ) * eps2, T( 0.5 ) * eps2 ) * 
                    grainsBeta( T( 2 ) * eps1, T( 0.5 ) * eps1 );
    T const prod2 = m_c * m_c * 
                    grainsBeta( T( 0.5 ) * eps2, T( 0.5 ) * eps2 ) * 
                    grainsBeta( T( 1.5 ) * eps1, eps1 );

    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[0] = C * ( m_b * m_b * prod1 + prod2 );
    inertia[3] = C * ( m_a * m_a * prod1  + prod2 );
    inertia[5] = C * ( m_a * m_a + m_b * m_b ) * prod1;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
    inertia_1[5] = T( 1 ) / inertia[5];
#else
    T const eps1 = T( 2 ) / m_n1;
    T const eps2 = T( 2 ) / m_n2;
    T const C = T( 0.4 ) * m_a * m_b * m_c * eps1 * eps2 ;

    T const prod1 = std::beta( T( 1.5 ) * eps2, T( 0.5 ) * eps2 ) * 
                    std::beta( T( 2 ) * eps1, T( 0.5 ) * eps1 );
    T const prod2 = m_c * m_c * 
                    std::beta( T( 0.5 ) * eps2, T( 0.5 ) * eps2 ) * 
                    std::beta( T( 1.5 ) * eps1, eps1 );

    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[0] = C * ( m_b * m_b * prod1 + prod2 );
    inertia[3] = C * ( m_a * m_a * prod1  + prod2 );
    inertia[5] = C * ( m_a * m_a + m_b * m_b ) * prod1;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
    inertia_1[5] = T( 1 ) / inertia[5];
#endif
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor -
// specialized for floats
template <>
__HOSTDEVICE__
void Superquadric<float>::computeInertia( float (&inertia)[6], 
                                          float (&inertia_1)[6] ) const
{
#ifdef __CUDA_ARCH__
    float const eps1 = 2.f / m_n1;
    float const eps2 = 2.f / m_n2;
    float const C = 0.4f * m_a * m_b * m_c * eps1 * eps2 ;

    float const prod1 = grainsBetaf( 1.5f * eps2, 0.5f * eps2 ) * 
                        grainsBetaf( 2.f * eps1, 0.5f * eps1 );
    float const prod2 = m_c * m_c * 
                        grainsBetaf( 0.5f * eps2, 0.5f * eps2 ) * 
                        grainsBetaf( 1.5f * eps1, eps1 );

    inertia[1] = inertia[2] = inertia[4] = 0.f;
    inertia[0] = C * ( m_b * m_b * prod1 + prod2 );
    inertia[3] = C * ( m_a * m_a * prod1  + prod2 );
    inertia[5] = C * ( m_a * m_a + m_b * m_b ) * prod1;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = 0.f;
    inertia_1[0] = 1.f / inertia[0];
    inertia_1[3] = 1.f / inertia[3];
    inertia_1[5] = 1.f / inertia[5];
#else
    float const eps1 = 2.f / m_n1;
    float const eps2 = 2.f / m_n2;
    float const C = 0.4f * m_a * m_b * m_c * eps1 * eps2 ;

    float const prod1 = std::betaf( 1.5f * eps2, 0.5f * eps2 ) * 
                        std::betaf( 2.f * eps1, 0.5f * eps1 );
    float const prod2 = m_c * m_c * 
                        std::betaf( 0.5f * eps2, 0.5f * eps2 ) * 
                        std::betaf( 1.5f * eps1, eps1 );

    inertia[1] = inertia[2] = inertia[4] = 0.f;
    inertia[0] = C * ( m_b * m_b * prod1 + prod2 );
    inertia[3] = C * ( m_a * m_a * prod1  + prod2 );
    inertia[5] = C * ( m_a * m_a + m_b * m_b ) * prod1;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = 0.f;
    inertia_1[0] = 1.f / inertia[0];
    inertia_1[3] = 1.f / inertia[3];
    inertia_1[5] = 1.f / inertia[5];
#endif
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Superquadric
template <typename T>
__HOSTDEVICE__
T Superquadric<T>::computeCircumscribedRadius() const
{
    if ( ( m_n1 == T( 2 ) ) && ( m_n2 == T( 2 ) ) )
        return ( max( m_a, max( m_b, m_c ) ) );
    else if ( m_n1 == T( 2 ) )
    {
        T const alpha = pow( m_b / m_a, T( 2 ) / ( m_n2 - T( 2 ) ) );
        T const xt = T( 1 ) / pow( T( 1 ) + pow( alpha, m_n2 ), T( 1 ) / m_n2 );
        return ( max( m_c, 
        sqrt( m_a * xt * m_a * xt + alpha * m_b * xt * alpha * m_b * xt ) ) );
    }
    else if ( m_n2 == T( 2 ) )
    {
        T const m = max( m_a, m_b );
        T const beta = pow( m_c * m_c / ( m * m ), T( 1 ) / ( m_n1 - T( 2 ) ) );
        T const xt = T( 1 ) / pow( T( 1 ) + pow( beta, m_n1 ), T( 1 ) / m_n1 );
        return ( sqrt( m * xt * m * xt + beta * m_c * xt * beta * m_c * xt ) );
    }
    else
    {
        T const alpha = pow( m_b / m_a, T( 2 ) / ( m_n2 - T( 2 ) ) );
        T const gamma = pow( T( 1 ) + pow( alpha, m_n2 ), 
                             m_n1 / m_n2 - T( 1 ) );
        T const beta = pow( gamma * m_c * m_c / ( m_a * m_a ), 
                            T( 1 ) / (m_n1 - T( 1 ) ) );
        T const xt = T( 1 ) / 
                     pow( pow( T( 1 ) + pow( alpha, m_n2 ), m_n1 / m_n2 ) +
                          pow( beta, m_n1 ), T( 1 ) / m_n1 );

        return ( sqrt( m_a * xt * m_a * xt + 
                       alpha * m_b * xt * alpha * m_b * xt +
                       beta * m_c * xt * beta * m_c * xt ) );
    }
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Superquadric - specialized for floats
template <>
__HOSTDEVICE__
float Superquadric<float>::computeCircumscribedRadius() const
{
    if ( ( m_n1 == 2.f ) && ( m_n2 == 2.f ) )
        return ( max( m_a, max( m_b, m_c ) ) );
    else if ( m_n1 == 2.f )
    {
        float const alpha = pow( m_b / m_a, 2.f / ( m_n2 - 2.f ) );
        float const xt = 1.f / powf( 1.f + powf( alpha, m_n2 ), 1.f / m_n2 );
        return ( max( m_c, 
        sqrtf( m_a * xt * m_a * xt + alpha * m_b * xt * alpha * m_b * xt ) ) );
    }
    else if ( m_n2 == 2.f )
    {
        float const m = max( m_a, m_b );
        float const beta = powf( m_c * m_c / ( m * m ), 1.f / ( m_n1 - 2.f ) );
        float const xt = 1.f / powf( 1.f + powf( beta, m_n1 ), 1.f / m_n1 );
        return ( sqrtf( m * xt * m * xt + beta * m_c * xt * beta * m_c * xt ) );
    }
    else
    {
        float const alpha = powf( m_b / m_a, 2.f / ( m_n2 - 2.f ) );
        float const gamma = powf( 1.f + powf( alpha, m_n2 ), 
                                  m_n1 / m_n2 - 1.f );
        float const beta = powf( gamma * m_c * m_c / ( m_a * m_a ), 
                                 1.f / (m_n1 - 2.f ) );
        float const xt = 1.f / 
                         powf( powf( 1.f + powf( alpha, m_n2 ), m_n1 / m_n2 )
                         + powf( beta, m_n1 ), 1.f / m_n1 );

        return ( sqrtf( m_a * xt * m_a * xt + 
                        alpha * m_b * xt * alpha * m_b * xt +
                        beta * m_c * xt * beta * m_c * xt ) );
    }
}




// -----------------------------------------------------------------------------
// Returns the bounding box to Superquadric
template <typename T>
__HOSTDEVICE__
Vector3<T> Superquadric<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_a, 
                         m_b, 
                         m_c ) );
}




// -----------------------------------------------------------------------------
// Superquadric support function, returns the support point P, i.e. the point on
// the surface of the Superquadric that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Superquadric<T>::support( Vector3<T> const& v ) const
{
    T const abvx = fabs( v[X] );
    T const abvy = fabs( v[Y] );
    T const abvz = fabs( v[Z] );
    T const signx = sgn( v[X] );
    T const signy = sgn( v[Y] );
    T const signz = sgn( v[Z] );

    Vector3<T> sup;

    if ( abvx == T( 0 ) )
    {
        if ( abvy == T( 0 ) )
            return ( Vector3<T>( T( 0 ), T( 0 ), signz * m_c ) );
        else
        {
            T const alpha = pow( m_c / m_b * abvz / abvy, 
                                     T( 1 ) / ( m_n1 - T( 1 ) ) );
            T const yt = T( 1 ) / pow( T( 1 ) + 
                                        pow( alpha, m_n1 ), T( 1 ) / m_n1);
            return ( Vector3<T>( T( 0 ), 
                                 signy * m_b * yt, 
                                 signz * alpha * m_c * yt ) );
        }
    }
    else
    {
        T const alpha = pow( m_b / m_a * abvy / abvx, 
                                T( 1 ) / ( m_n2 - T( 1 ) ) );
        T const temp = T( 1 ) + pow( alpha, m_n2 );
        T const gamma = pow( temp, 
                            ( m_n1 - m_n2 ) / ( m_n2 * ( m_n1 - T( 1 ) ) ) );
        T const beta = gamma * pow( m_c / m_a * abvz / abvx, 
                                    T( 1 ) / ( m_n1 - T( 1 ) ) );
        T const xt = T( 1 ) / pow( pow( temp, m_n1 / m_n2 ) +
                                    pow( beta, m_n1 ) , T( 1 ) / m_n1 );
        return ( Vector3<T>( signx * m_a * xt, 
                             signy * alpha * m_b * xt, 
                             signz * beta * m_c * xt ) );
    }
}




// -----------------------------------------------------------------------------
// Superquadric support function, returns the support point P, i.e. the point on
// the surface of the Superquadric that satisfies max(P.v) - specialized for
// floats
template <>
__HOSTDEVICE__
Vector3<float> Superquadric<float>::support( Vector3<float> const& v ) const
{
    float const abvx = fabsf( v[X] );
    float const abvy = fabsf( v[Y] );
    float const abvz = fabsf( v[Z] );
    float const signx = sgn( v[X] );
    float const signy = sgn( v[Y] );
    float const signz = sgn( v[Z] );

    Vec3F sup;

    if ( abvx == 0.f )
    {
        if ( abvy == 0.f )
            return ( Vector3<float>( 0.f, 0.f, signz * m_c ) );
        else
        {
            float const alpha = powf( m_c / m_b * abvz / abvy, 
                                                    1. / ( m_n1 - 1. ) );
            float const yt = 1.f / powf( 1.f + powf( alpha, m_n1 ), 
                                            1.f / m_n1);
            return ( Vector3<float>( 0.f, 
                                     signy * m_b * yt, 
                                     signz * alpha * m_c * yt ) );
        }
    }
    else
    {
        float const alpha = powf( m_b / m_a * abvy / abvx, 
                                    1.f / ( m_n2 - 1.f ) );
        float const temp = 1.f + powf( alpha, m_n2 );
        float const gamma = powf( temp, 
                            ( m_n1 - m_n2 ) / ( m_n2 * ( m_n1 - 1.f ) ) );
        float const beta = gamma * powf( m_c / m_a * abvz / abvx, 
                                            1.f / ( m_n1 - 1.f ) );
        float const xt = 1.f / powf( powf( temp, m_n1 / m_n2 ) +
                                        powf( beta, m_n1 ) , 1.f / m_n1 );
        return ( Vector3<float>( signx * m_a * xt, 
                                 signy * alpha * m_b * xt, 
                                 signz * beta * m_c * xt ) );
    }
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
void Superquadric<T>::readConvex( std::istream& fileIn )
{
    std::cout << "Program Error :\n" 
              << "Superquadric::readConvex is not implemented.\n";
    exit( 3 );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
void Superquadric<T>::writeConvex( std::ostream& fileOut ) const
{
    fileOut << "Superquadric with dimensions " << m_a << ", " 
                                               << m_b << ", "
                                               << m_c << ", and exponents " 
                                               << m_n1 << ", "
                                               << m_n2 << ".\n";
}




// -----------------------------------------------------------------------------
// Returns the number of points to write the superquadric in a Paraview format
template <typename T>
__HOST__
int Superquadric<T>::numberOfPoints_PARAVIEW() const
{
    return ( visuNodeNbOnPer * ( visuNodeNbOnPer - 1 ) + 3 );
}




// -----------------------------------------------------------------------------
// Returns the number of elementary polytopes to write the superquadric in a 
// Paraview format
template <typename T>
__HOST__
int Superquadric<T>::numberOfCells_PARAVIEW() const
{
    return ( visuNodeNbOnPer * visuNodeNbOnPer );
}




// -----------------------------------------------------------------------------
// Returns a list of points describing the superquadric in a Paraview format
template <typename T>
__HOST__
std::list<Vector3<T>> Superquadric<T>::writePoints_PARAVIEW( 
                                           Transform3<T> const& transform,
                                           Vector3<T> const* translation ) const
{
    T const eps1 = T( 2 ) / m_n1 ;
    T const eps2 = T( 2 ) / m_n2 ;
    std::list<Vector3<T>> ParaviewPoints;
    T const dtheta = T( PI ) / visuNodeNbOnPer;
    T const dphi = T( 2 ) * T( PI ) / visuNodeNbOnPer;
    Vector3<T> pp, pptrans;

    pp[X] = pp[Y] = T( 0 );
    // Gravity center
    pp[Z] = T( 0 );
    pptrans = transform( pp );
    if ( translation )
      pptrans += *translation;
    ParaviewPoints.push_back( pptrans );

    // Top point
    pp[Z] = m_c;
    pptrans = transform( pp );
    if ( translation )
      pptrans += *translation;
    ParaviewPoints.push_back( pptrans );

    // Bottom point
    pp[Z] = -m_c;
    pptrans = transform( pp ) ;
    if ( translation )
      pptrans += *translation;
    ParaviewPoints.push_back( pptrans );

    // Regular points on the surface
    T cost, sint, costeps1, sinteps1, cosp, sinp;

    for ( int i = 1; i < visuNodeNbOnPer; i++ )
    {
      cost = cos( i * dtheta );
      sint = sin( i * dtheta );

      if ( cost == T( 0 ) )
        costeps1 = T( 0 );
      else if ( cost < T( 0 ) )
        costeps1 = -pow( -cost, eps1 );
      else
        costeps1 = pow( cost, eps1 );

      // Theta is always strictly between 0 and pi so sint is strictly positive
      sinteps1 = pow( sint, eps1 );

      for ( int j = 0; j < visuNodeNbOnPer; j++ )
      {
        cosp = cos( j * dphi );
        sinp = sin( j * dphi );
        if ( cosp == T( 0 ) )
          pp[X] = T( 0 );
        else if ( cosp < T( 0 ) )
          pp[X] = -m_a * sinteps1 * pow( -cosp, eps2 );
        else
          pp[X] = m_a * sinteps1 * pow( cosp, eps2 );

        if ( sinp == T( 0 ) )
          pp[Y] = T( 0 );
        else if ( sinp < T( 0 ) )
          pp[Y] = -m_b * sinteps1 * pow( -sinp, eps2 );
        else
          pp[Y] = m_b * sinteps1 * pow( sinp, eps2 );

        pp[Z] = m_c * costeps1 ;

        pptrans = transform( pp ) ;
        if ( translation )
          pptrans += *translation;
        ParaviewPoints.push_back( pptrans );
      }
    }

    return ( ParaviewPoints );
}




// -----------------------------------------------------------------------------
// Writes the connectivity of the superquadric in a Paraview format
template <typename T>
__HOST__
void Superquadric<T>::writeConnection_PARAVIEW( std::list<int>& connectivity,
    	                                        std::list<int>& offsets, 
                                                std::list<int>& cellstype, 
                                                int& firstpoint_globalnumber,
                                                int& last_offset ) const
{
    // Top cells: tetrahedron
    for ( int j = 0; j < visuNodeNbOnPer - 1; j++ )
    {
        // Center
        connectivity.push_back( firstpoint_globalnumber );
        // Top point
        connectivity.push_back( firstpoint_globalnumber + 1 );
        connectivity.push_back( firstpoint_globalnumber + 3 + j );
        connectivity.push_back( firstpoint_globalnumber + 3 + j + 1);
        last_offset += 4;
        offsets.push_back( last_offset );
        cellstype.push_back( 10 );
    }
    // Center
    connectivity.push_back( firstpoint_globalnumber );
    // Top point
    connectivity.push_back( firstpoint_globalnumber + 1 );
    // Last point of the first latitude (i=1)
    connectivity.push_back( firstpoint_globalnumber + 3 + visuNodeNbOnPer - 1 );
    // First point of the first latitude (i=1)
    connectivity.push_back( firstpoint_globalnumber + 3 );
    last_offset += 4;
    offsets.push_back( last_offset );
    cellstype.push_back( 10 );

    // Regular cells: Pyramid
    int pointact = 3; // Current point (we will browse the grid)

    for ( int i = 1 ; i < visuNodeNbOnPer - 1; ++i )
    {
        for ( int j = 0; j < visuNodeNbOnPer - 1; j++ )
        {
            // Center
            connectivity.push_back( firstpoint_globalnumber );
            // Current point
            connectivity.push_back( firstpoint_globalnumber + pointact );
            // same latitude, +1 longitude
            connectivity.push_back( firstpoint_globalnumber + pointact + 1 ) ;
            // +1 latitude, +1 longitude
            connectivity.push_back( firstpoint_globalnumber + pointact
                                                        + visuNodeNbOnPer + 1 );
            // +1 latitude, same longitude
            connectivity.push_back( firstpoint_globalnumber + pointact
                                                            + visuNodeNbOnPer );
            last_offset += 5;
            offsets.push_back( last_offset );
            cellstype.push_back( 14 );
            pointact++;
        }
        // Center
        connectivity.push_back( firstpoint_globalnumber );
        // Current point (last of its latitude)
        connectivity.push_back( firstpoint_globalnumber + pointact );
        // First point (j=0) on the same latitude as pointact
        connectivity.push_back( firstpoint_globalnumber + pointact
                                                    - ( visuNodeNbOnPer - 1 ) );
        // First point (j=0) on the latitude under that of pointact
        connectivity.push_back( firstpoint_globalnumber + pointact + 1 );
        // +1 latitude, same longitude
        connectivity.push_back( firstpoint_globalnumber + pointact
                                                            + visuNodeNbOnPer );
        last_offset += 5;
        offsets.push_back( last_offset );
        cellstype.push_back( 14 );
        pointact++;
    }

    // Bottom cells: tetrahedron
    const int Firstptlastlat = 3 + visuNodeNbOnPer * (visuNodeNbOnPer - 2);

    for ( int j = 0; j < visuNodeNbOnPer - 1; j++ )
    {
        connectivity.push_back( firstpoint_globalnumber ); // Center
        connectivity.push_back( firstpoint_globalnumber + 2 ); // Bottom point
        connectivity.push_back( firstpoint_globalnumber + Firstptlastlat + j );
        connectivity.push_back( firstpoint_globalnumber + Firstptlastlat 
                                                                    + j + 1 );
        last_offset += 4;
        offsets.push_back( last_offset );
        cellstype.push_back( 10 );
    }
    connectivity.push_back( firstpoint_globalnumber ); // Center
    connectivity.push_back( firstpoint_globalnumber + 2 ); // Bottom point
    // Last point last latitude
    connectivity.push_back( firstpoint_globalnumber + Firstptlastlat
                                                        + visuNodeNbOnPer - 1 );
    // First point last latitude
    connectivity.push_back( firstpoint_globalnumber + Firstptlastlat );
    last_offset += 4;
    offsets.push_back( last_offset );
    cellstype.push_back( 10 );

    firstpoint_globalnumber += visuNodeNbOnPer * ( visuNodeNbOnPer - 1 ) + 3;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Superquadric<float>;
template class Superquadric<double>;