#include "Box.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
template <typename T>
__HOSTDEVICE__ 
Box<T>::Box( T x, 
             T y,
             T z )
: m_extent( Vector3<T>( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Constructor with an input stream
template <typename T>
__HOST__
Box<T>::Box( std::istream& fileIn )
{
    readConvex( fileIn );
}




// -----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
template <typename T>
__HOST__
Box<T>::Box( DOMNode* root )
{
    m_extent[X] = T( ReaderXML::getNodeAttr_Double( root, "LX" ) ) / T( 2 );
    m_extent[Y] = T( ReaderXML::getNodeAttr_Double( root, "LY" ) ) / T( 2 );
    m_extent[Z] = T( ReaderXML::getNodeAttr_Double( root, "LZ" ) ) / T( 2 );
}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
template <typename T>
__HOSTDEVICE__
Box<T>::Box( Vector3<T> const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Box<T>::~Box()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Box<T>::getConvexType() const
{
    return ( BOX );
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::getExtent() const
{
    return ( m_extent );
}




// -----------------------------------------------------------------------------
// Sets values of the edge length
template <typename T>
__HOSTDEVICE__
void Box<T>::setExtent( T x, T y, T z )
{
    m_extent = Vector3<T>( x, y, z );
}




// -----------------------------------------------------------------------------
// Returns a clone of the box
template <typename T>
__HOSTDEVICE__
Convex<T>* Box<T>::clone() const
{
    return( new Box<T>( T( 2 ) * m_extent[X],
                        T( 2 ) * m_extent[Y],
                        T( 2 ) * m_extent[Z] ) );
}




// -----------------------------------------------------------------------------
// Returns the volume of the box
template <typename T>
__HOSTDEVICE__
T Box<T>::computeVolume() const
{
    return ( T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z] );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
void Box<T>::computeInertia( T (&inertia)[6], 
                             T (&inertia_1)[6] ) const
{
    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[0] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[Z] * m_extent[Z] ) / T( 3 );
    inertia[3] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[X] * m_extent[X] + m_extent[Z] * m_extent[Z] ) / T( 3 );
    inertia[5] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[X] * m_extent[X] ) / T( 3 );

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
    inertia_1[5] = T( 1 ) / inertia[5];
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the box
template <typename T>
__HOSTDEVICE__
T Box<T>::computeCircumscribedRadius() const
{
    return ( m_extent.norm() );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to box
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_extent[X], 
                         m_extent[Y], 
                         m_extent[Z] ) );
}




// -----------------------------------------------------------------------------
// Box support function, returns the support point P, i.e. the point on the
// surface of the box that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::support( Vector3<T> const& v ) const
{
    return ( Vector3<T>( v[X] < T( 0 ) ? -m_extent[X] : m_extent[X],
                         v[Y] < T( 0 ) ? -m_extent[Y] : m_extent[Y],
                         v[Z] < T( 0 ) ? -m_extent[Z] : m_extent[Z] ) );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
void Box<T>::readConvex( std::istream& fileIn )
{
    fileIn >> m_extent;
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
void Box<T>::writeConvex( std::ostream& fileOut ) const
{
    fileOut << "Box with dimensions " << m_extent << ".\n";
}




// -----------------------------------------------------------------------------
// Returns the number of points to write the box in a Paraview format
template <typename T>
__HOST__
int Box<T>::numberOfPoints_PARAVIEW() const
{
    return ( 8 );
}




// -----------------------------------------------------------------------------
// Returns the number of elementary polytopes to write the box in a Paraview 
// format
template <typename T>
__HOST__
int Box<T>::numberOfCells_PARAVIEW() const
{
    return ( 1 );
}




// -----------------------------------------------------------------------------
// Returns a list of points describing the box in a Paraview format
template <typename T>
__HOST__
std::list<Vector3<T>> Box<T>::writePoints_PARAVIEW( 
                                        Transform3<T> const& transform,
                                        Vector3<T> const* translation ) const
{
    std::list<Vector3<T>> ParaviewPoints;
    Vector3<T> p;
    p.setValue( - m_extent[X], - m_extent[Y], - m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_extent[X], - m_extent[Y], - m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_extent[X], - m_extent[Y], m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( - m_extent[X], - m_extent[Y], m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( - m_extent[X], m_extent[Y], - m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_extent[X], m_extent[Y], - m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_extent[X], m_extent[Y], m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( - m_extent[X], m_extent[Y], m_extent[Z] );
    ParaviewPoints.push_back( transform( p ) );
    return ( ParaviewPoints );
}




// -----------------------------------------------------------------------------
// Writes the connectivity of the box in a Paraview format
template <typename T>
__HOST__
void Box<T>::writeConnection_PARAVIEW( std::list<int>& connectivity,
    	                               std::list<int>& offsets, 
                                       std::list<int>& cellstype, 
                                       int& firstpoint_globalnumber,
                                       int& last_offset ) const
{
    int count = firstpoint_globalnumber;
    for ( int i = 0; i < 8; ++i )
    {
        connectivity.push_back( count );
        ++count;
    }
    last_offset += 8;
    offsets.push_back( last_offset );
    cellstype.push_back( 12 );
    firstpoint_globalnumber += 8;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Box<float>;
template class Box<double>;