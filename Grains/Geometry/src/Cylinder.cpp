#include "Cylinder.hh"


// multiple of 4
#define visuNodeNbOnPer 32


// -----------------------------------------------------------------------------
// Constructor with radius and height as input parameters
template <typename T>
__HOSTDEVICE__
Cylinder<T>::Cylinder( T r,
                       T h )
: m_radius( r )
, m_halfHeight( h / T( 2 ) )
{}




// -----------------------------------------------------------------------------
// Constructor with an input stream
template <typename T>
__HOST__
Cylinder<T>::Cylinder( std::istream& fileIn )
{
    readConvex( fileIn );
}




// -----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
template <typename T>
__HOST__
Cylinder<T>::Cylinder( DOMNode* root )
{
    m_radius  = T( ReaderXML::getNodeAttr_Double( root, "Radius" ) );
    m_halfHeight  = T( ReaderXML::getNodeAttr_Double( root, "Height" ) )
                                                                 / T( 2 );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Cylinder<T>::~Cylinder()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Cylinder<T>::getConvexType() const
{
    return ( CYLINDER );
}




// -----------------------------------------------------------------------------
// Returns the radius
template <typename T>
__HOSTDEVICE__
T Cylinder<T>::getRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Returns the height
template <typename T>
__HOSTDEVICE__
T Cylinder<T>::getHeight() const
{
    return ( T( 2 ) * m_halfHeight );
}




// -----------------------------------------------------------------------------
// Returns a clone of the cylinder
template <typename T>
__HOSTDEVICE__
Convex<T>* Cylinder<T>::clone() const
{
    return( new Cylinder<T>( m_radius, T( 2 ) * m_halfHeight ) );
}




// -----------------------------------------------------------------------------
// Returns the volume of the cylinder
template <typename T>
__HOSTDEVICE__
T Cylinder<T>::computeVolume() const
{
    return ( TWO_PI<T> * m_halfHeight * m_radius * m_radius );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
void Cylinder<T>::computeInertia( T (&inertia)[6], 
                                  T (&inertia_1)[6] ) const
{
    T r2 = m_radius * m_radius;
    T c = T( .5 ) * PI<T> * m_halfHeight * r2;
    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[0] = inertia[5] = 
                    c * ( T( 4 ) * m_halfHeight * m_halfHeight / T( 3 ) + r2 );
    inertia[3] = T( 2 ) * c * r2;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[5] = inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the cylinder
template <typename T>
__HOSTDEVICE__
T Cylinder<T>::computeCircumscribedRadius() const
{
    
    return ( sqrt( m_radius * m_radius + m_halfHeight * m_halfHeight ) );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to cylinder
template <typename T>
__HOSTDEVICE__
Vector3<T> Cylinder<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_radius, 
                         m_halfHeight, 
                         m_radius ) );
}




// -----------------------------------------------------------------------------
// Cylinder support function, returns the support point P, i.e. the point on
// the surface of the Cylinder that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Cylinder<T>::support( Vector3<T> const& v ) const
{
    T s = sqrt( v[X] * v[X] + v[Z] * v[Z] );
    if ( s > EPS<T> )
    {
        T d = m_radius / s;
        return ( Vector3<T>( v[X] * d,
                             v[Y] < T( 0 ) ? - m_halfHeight : m_halfHeight,
                             v[Z] * d ) );
    }
    else
      return ( Vector3<T>( T( 0 ), 
                           v[Y] < T( 0 ) ? - m_halfHeight : m_halfHeight, 
                           T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
void Cylinder<T>::readConvex( std::istream& fileIn )
{
    fileIn >> m_radius >> m_halfHeight;
    m_halfHeight /= T( 2 );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
void Cylinder<T>::writeConvex( std::ostream& fileOut ) const
{
    fileOut << "Cylinder with radius " << m_radius
            << ", and height " << T( 2 ) * m_halfHeight << ".\n";
}




// -----------------------------------------------------------------------------
// Returns the number of points to write the cylinder in a Paraview format
template <typename T>
__HOST__
int Cylinder<T>::numberOfPoints_PARAVIEW() const
{
    return ( 2 * visuNodeNbOnPer + 2 );
}




// -----------------------------------------------------------------------------
// Returns the number of elementary polytopes to write the cylinder in a 
// Paraview format
template <typename T>
__HOST__
int Cylinder<T>::numberOfCells_PARAVIEW() const
{
    return ( visuNodeNbOnPer );
}




// -----------------------------------------------------------------------------
// Returns a list of points describing the cylinder in a Paraview format
template <typename T>
__HOST__
std::list<Vector3<T>> Cylinder<T>::writePoints_PARAVIEW( 
                                           Transform3<T> const& transform,
                                           Vector3<T> const* translation ) const
{
    list<Vector3<T>> ParaviewPoints;
    Vector3<T> pp, p;
    T dtheta = TWO_PI<T> / visuNodeNbOnPer;

    // Lower disk rim
    p[Y] = - m_halfHeight;
    for ( int i = 0; i < visuNodeNbOnPer; ++i )
    {
        p[X] = m_radius * cos ( i * dtheta );
        p[Z] = m_radius * sin ( i * dtheta );
        pp = transform( p );
        if ( translation ) 
            pp += *translation;
        ParaviewPoints.push_back( pp );
    }

    // Upper disk rim
    p[Y] = m_halfHeight;
    for ( int i = 0; i < visuNodeNbOnPer; ++i )
    {
        p[X] = m_radius * cos ( i * dtheta );
        p[Z] = m_radius * sin ( i * dtheta );
        pp = transform( p );
        if ( translation )
            pp += *translation;
        ParaviewPoints.push_back( pp );
    }

    // Lower disk center
    p[X] = T( 0 );
    p[Y] = - m_halfHeight;
    p[Z] = T( 0 );
    pp = transform( p );
    if ( translation ) 
        pp += *translation;
    ParaviewPoints.push_back( pp );

    // Upper disk center
    p[Y] = m_halfHeight;
    pp = transform( p );
    if ( translation ) 
        pp += *translation;
    ParaviewPoints.push_back( pp );

    return ( ParaviewPoints );
}




// -----------------------------------------------------------------------------
// Writes the connectivity of the cylinder in a Paraview format
template <typename T>
__HOST__
void Cylinder<T>::writeConnection_PARAVIEW( std::list<int>& connectivity,
                                            std::list<int>& offsets, 
                                            std::list<int>& cellstype, 
                                            int& firstpoint_globalnumber,
                                            int& last_offset ) const
{
    for ( int i = 0; i < visuNodeNbOnPer - 1; ++i )
    {
        connectivity.push_back( firstpoint_globalnumber + i );
        connectivity.push_back( firstpoint_globalnumber + i + 1 );
        connectivity.push_back( firstpoint_globalnumber + 2 * visuNodeNbOnPer );
        connectivity.push_back( firstpoint_globalnumber + i + visuNodeNbOnPer);
        connectivity.push_back( firstpoint_globalnumber + i + visuNodeNbOnPer
            + 1 );
        connectivity.push_back( firstpoint_globalnumber + 2 * visuNodeNbOnPer
            + 1 );
        last_offset += 6;
        offsets.push_back( last_offset );
        cellstype.push_back( 13 );
    }
    connectivity.push_back( firstpoint_globalnumber + visuNodeNbOnPer - 1 );
    connectivity.push_back( firstpoint_globalnumber );
    connectivity.push_back( firstpoint_globalnumber + 2 * visuNodeNbOnPer );
    connectivity.push_back( firstpoint_globalnumber + 2 * visuNodeNbOnPer - 1 );
    connectivity.push_back( firstpoint_globalnumber + visuNodeNbOnPer );
    connectivity.push_back( firstpoint_globalnumber + 2 * visuNodeNbOnPer + 1 );
    last_offset += 6;
    offsets.push_back( last_offset );
    cellstype.push_back( 13 );

    firstpoint_globalnumber += 2 * visuNodeNbOnPer + 2;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Cylinder<float>;
template class Cylinder<double>;

#undef visuNodeNbOnPer