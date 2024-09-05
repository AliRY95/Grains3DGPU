#include "Rectangle.hh"


// -----------------------------------------------------------------------------
// Constructor with the dimensions of the rectangle
template <typename T>
__HOSTDEVICE__ 
Rectangle<T>::Rectangle( T x, 
                         T y )
: m_LX( x / T( 2 ) )
, m_LY( y / T( 2 ) )
{}




// -----------------------------------------------------------------------------
// Constructor with an input stream
template <typename T>
__HOST__
Rectangle<T>::Rectangle( std::istream& fileIn )
{
    readConvex( fileIn );
}




// -----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
template <typename T>
__HOST__
Rectangle<T>::Rectangle( DOMNode* root )
{
    m_LX = T( ReaderXML::getNodeAttr_Double( root, "LX" ) ) / T( 2 );
    m_LY = T( ReaderXML::getNodeAttr_Double( root, "LY" ) ) / T( 2 );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Rectangle<T>::~Rectangle()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Rectangle<T>::getConvexType() const
{
    return ( RECTANGLE );
}




// -----------------------------------------------------------------------------
// Returns the edge lengths in a Vector3 format with Z = 0
template <typename T>
__HOSTDEVICE__
Vector3<T> Rectangle<T>::getExtent() const
{
    return ( Vector3<T>( m_LX, m_LY, T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// Sets values of the edge length
template <typename T>
__HOSTDEVICE__
void Rectangle<T>::setExtent( T x, 
                              T y )
{
    m_LX = x / T( 2 );
    m_LY = y / T( 2 );
}




// -----------------------------------------------------------------------------
// Returns a clone of the rectangle
template <typename T>
__HOSTDEVICE__
Convex<T>* Rectangle<T>::clone() const
{
    return( new Rectangle<T>( T( 2 ) * m_LX, T( 2 ) * m_LY ) );
}




// -----------------------------------------------------------------------------
// Returns the volume (area) of the rectangle
template <typename T>
__HOSTDEVICE__
T Rectangle<T>::computeVolume() const
{
    return ( T( 4 ) * m_LX * m_LY );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
void Rectangle<T>::computeInertia( T (&inertia)[6], 
                                   T (&inertia_1)[6] ) const
{
    // Active 2D plane is XY -> rotation around Z
    inertia[1] = inertia[2] = inertia[4]= T( 0 );
    inertia[0] = T( 4 ) / T( 3 ) * m_LX * m_LY * m_LY * m_LY;
    inertia[3] = T( 4 ) / T( 3 ) * m_LX * m_LX * m_LX * m_LY;
    inertia[5] = T( 2 ) / T( 3 ) * m_LX * m_LY * ( m_LX * m_LX + m_LY * m_LY );

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
    inertia_1[5] = T( 1 ) / inertia[5];
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the rectangle
template <typename T>
__HOSTDEVICE__
T Rectangle<T>::computeCircumscribedRadius() const
{
    return ( sqrt( m_LX * m_LX + m_LY * m_LY ) );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to the rectangle
template <typename T>
__HOSTDEVICE__
Vector3<T> Rectangle<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_LX, 
                         m_LY, 
                         T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// Box support function, returns the support point P, i.e. the point on the
// surface of the box that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Rectangle<T>::support( Vector3<T> const& v ) const
{
    return ( Vector3<T>( v[X] < T( 0 ) ? -m_LX : m_LX,
                         v[Y] < T( 0 ) ? -m_LY : m_LY,
                         T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
void Rectangle<T>::readConvex( std::istream& fileIn )
{
    fileIn >> m_LX >> m_LY;
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
void Rectangle<T>::writeConvex( std::ostream& fileOut ) const
{
    fileOut << "Rectangle with dimensions " << T( 2 ) * m_LX
            << ", and " << T( 2 ) * m_LY << ".\n";
}




// -----------------------------------------------------------------------------
// Returns the number of points to write the rectangle in a Paraview format
template <typename T>
__HOST__
int Rectangle<T>::numberOfPoints_PARAVIEW() const
{
    return ( 4 );
}




// -----------------------------------------------------------------------------
// Returns the number of elementary polytopes to write the rectangle in a 
// Paraview format
template <typename T>
__HOST__
int Rectangle<T>::numberOfCells_PARAVIEW() const
{
    return ( 1 );
}




// -----------------------------------------------------------------------------
// Returns a list of points describing the rectangle in a Paraview format
template <typename T>
__HOST__
std::list<Vector3<T>> Rectangle<T>::writePoints_PARAVIEW( 
                                        Transform3<T> const& transform,
                                        Vector3<T> const* translation ) const
{
    std::list<Vector3<T>> ParaviewPoints;
    Vector3<T> p;
    p.setValue( - m_LX, - m_LY, T( 0 ) );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_LX, - m_LY, T( 0 ) );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( - m_LX, m_LY, T( 0 ) );
    ParaviewPoints.push_back( transform( p ) );
    p.setValue( m_LX, m_LY, T( 0 ) );
    ParaviewPoints.push_back( transform( p ) );
    return ( ParaviewPoints );
}




// -----------------------------------------------------------------------------
// Writes the connectivity of the box in a Paraview format
template <typename T>
__HOST__
void Rectangle<T>::writeConnection_PARAVIEW( std::list<int>& connectivity,
    	                                     std::list<int>& offsets, 
                                             std::list<int>& cellstype, 
                                             int& firstpoint_globalnumber,
                                             int& last_offset ) const
{
    int count = firstpoint_globalnumber;
    for ( int i = 0; i < 4; ++i )
    {
        connectivity.push_back( count );
        ++count;
    }
    last_offset += 4;
    offsets.push_back( last_offset );
    cellstype.push_back( 8 );

    firstpoint_globalnumber += 4;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Rectangle<float>;
template class Rectangle<double>;