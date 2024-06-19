#include "VectorMath.hh"
#include "Convex.hh"
#include "Sphere.hh"


#define visuNodeNbPerQar 8 


// -----------------------------------------------------------------------------
// Constructor with radius
template <typename T>
__HOSTDEVICE__ 
Sphere<T>::Sphere( T r )
: m_radius( r )
{}




// -----------------------------------------------------------------------------
// Constructor with an input stream
template <typename T>
__HOST__
Sphere<T>::Sphere( std::istream& fileIn )
{
    readConvex( fileIn );
}




// -----------------------------------------------------------------------------
// Constructor with an XML node as an input parameter
template <typename T>
__HOST__
Sphere<T>::Sphere( DOMNode* root )
{
    m_radius = T( ReaderXML::getNodeAttr_Double( root, "Radius" ) );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Sphere<T>::~Sphere()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Sphere<T>::getConvexType() const
{
    return ( SPHERE );
}




// -----------------------------------------------------------------------------
// Returns the radius
template <typename T>
__HOSTDEVICE__
T Sphere<T>::getRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Sets the radius
template <typename T>
__HOSTDEVICE__
void Sphere<T>::setRadius( T r )
{
    m_radius = r;
}




// -----------------------------------------------------------------------------
// Returns the volume of the Sphere
template <typename T>
__HOSTDEVICE__
T Sphere<T>::computeVolume() const
{
    return ( T( 4 ) * M_PI * m_radius * m_radius * m_radius / T ( 3 ) );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
bool Sphere<T>::computeInertia( T* inertia, 
                                T* inertia_1 ) const
{
    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[5] = inertia[3] = inertia[0] = T( 8 ) * M_PI / T( 15 ) * 
                                                        pow( m_radius, T( 5 ));

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[5] = inertia_1[3] = inertia_1[0] = T( 1 ) / inertia[0];

    return ( true );
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Sphere
template <typename T>
__HOSTDEVICE__
T Sphere<T>::computeCircumscribedRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to Sphere
template <typename T>
__HOSTDEVICE__
Vector3<T> Sphere<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_radius, 
                         m_radius, 
                         m_radius ) );
}




// -----------------------------------------------------------------------------
// Sphere support function, returns the support point P, i.e. the point on the
// surface of the Sphere that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Sphere<T>::support( Vector3<T> const& v ) const
{
    return ( m_radius * v.normalized() );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
void Sphere<T>::readConvex( std::istream& fileIn )
{
    fileIn >> m_radius;
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
void Sphere<T>::writeConvex( std::ostream& fileOut ) const
{
    fileOut << "Sphere with radius " << m_radius << ".\n";
}




// -----------------------------------------------------------------------------
// Returns the number of points to write the sphere in a Paraview format
template <typename T>
__HOST__
int Sphere<T>::numberOfPoints_PARAVIEW() const
{
    return ( 4 * visuNodeNbPerQar * ( 2 * visuNodeNbPerQar - 1 ) + 3 );
}




// -----------------------------------------------------------------------------
// Returns the number of elementary polytopes to write the sphere in a Paraview 
// format
template <typename T>
__HOST__
int Sphere<T>::numberOfCells_PARAVIEW() const
{
    return ( 8 * visuNodeNbPerQar * visuNodeNbPerQar );
}




// -----------------------------------------------------------------------------
// Returns a list of points describing the sphere in a Paraview format
template <typename T>
__HOST__
std::list<Vector3<T>> Sphere<T>::writePoints_PARAVIEW( 
                                        Transform3<T> const& transform,
                                        Vector3<T> const* translation ) const
{
    std::list<Vector3<T>> ParaviewPoints;
    T angle = T( PI ) / ( T( 2 ) * visuNodeNbPerQar ) ;
    T angleZ = T( 0 ), local_radius = T( 0 );
    int k, i, ptsPerlevel =  4 * visuNodeNbPerQar;
    Vector3<T> pp, pptrans;
    
    // Regular points on the surface
    for ( k = 0; k < 2 * visuNodeNbPerQar - 1 ; ++k ) 
    {  
        angleZ = -PI / T( 2 ) + ( k + 1 ) * angle;
        local_radius = m_radius * cos( angleZ );
        pp[Z] = m_radius * sin( angleZ );
        for ( i = 0; i < ptsPerlevel ; ++i )
        {
            pp[X] = local_radius * cos( i * angle );
            pp[Y] = local_radius * sin( i * angle );
            pptrans = transform( pp );
            if ( translation ) 
                pptrans += *translation;
            ParaviewPoints.push_back( pptrans );
        }
    }
    
    pp[X] = T( 0 );
    pp[Y] = T( 0 );
    // Bottom point
    pp[Z] = - m_radius;
    pptrans = transform( pp );
    if ( translation ) 
        pptrans += *translation;
    ParaviewPoints.push_back( pptrans );
        
    // Top point
    pp[Z] = m_radius;
    pptrans = transform( pp );
    if ( translation ) 
        pptrans += *translation;
    ParaviewPoints.push_back( pptrans );
        
    // Gravity center
    pp[Z] = T( 0 );
    pptrans = transform( pp );  
    if ( translation ) 
        pptrans += *translation;
    ParaviewPoints.push_back( pptrans );
    
    return ( ParaviewPoints ); 
}




// -----------------------------------------------------------------------------
// Writes the connectivity of the sphere in a Paraview format
template <typename T>
__HOST__
void Sphere<T>::writeConnection_PARAVIEW( std::list<int>& connectivity,
    	                                  std::list<int>& offsets, 
                                          std::list<int>& cellstype, 
                                          int& firstpoint_globalnumber,
                                          int& last_offset ) const
{
    int i, k, 
    ptsPerlevel = 4 * visuNodeNbPerQar,
    Bottom_number = ptsPerlevel * ( 2 * visuNodeNbPerQar - 1 ),
    Top_number = ptsPerlevel * ( 2 * visuNodeNbPerQar - 1 ) + 1,  
    GC_number = ptsPerlevel * ( 2 * visuNodeNbPerQar - 1 ) + 2;

    // Regular cells: Pyramid
    for ( k = 0; k < 2 * visuNodeNbPerQar - 2 ; ++k ) 
    {  
        for ( i = 0; i < ptsPerlevel - 1 ; ++i )
        {
            int id = firstpoint_globalnumber + k * ptsPerlevel + i;
            connectivity.push_back( id ); 
            connectivity.push_back( id + 1 );
            connectivity.push_back( id + ptsPerlevel + 1 );	
            connectivity.push_back( id + ptsPerlevel );
            connectivity.push_back( firstpoint_globalnumber + GC_number );
            last_offset += 5;
            offsets.push_back( last_offset );
            cellstype.push_back( 14 );		
        }
        connectivity.push_back( firstpoint_globalnumber + k * ptsPerlevel + 
                                ptsPerlevel - 1 );
        connectivity.push_back( firstpoint_globalnumber + k * ptsPerlevel );
        connectivity.push_back( firstpoint_globalnumber + 
                                ( k + 1 ) * ptsPerlevel );
        connectivity.push_back( firstpoint_globalnumber + 
                                ( k + 1 ) * ptsPerlevel + ptsPerlevel - 1 );
        connectivity.push_back( firstpoint_globalnumber + GC_number );
        last_offset += 5;
        offsets.push_back( last_offset );
        cellstype.push_back( 14 );    
    }  

    // Bottom cells: tetrahedron
    for ( i = 0; i < ptsPerlevel-1 ; ++i )
    {
        connectivity.push_back( firstpoint_globalnumber + i ); 
        connectivity.push_back( firstpoint_globalnumber + i + 1 );
        connectivity.push_back( firstpoint_globalnumber + Bottom_number );	
        connectivity.push_back( firstpoint_globalnumber + GC_number );
        last_offset += 4;
        offsets.push_back( last_offset );
        cellstype.push_back( 10 );   
    }
    connectivity.push_back( firstpoint_globalnumber + ptsPerlevel - 1 );
    connectivity.push_back( firstpoint_globalnumber );
    connectivity.push_back( firstpoint_globalnumber + Bottom_number );	
    connectivity.push_back( firstpoint_globalnumber + GC_number );
    last_offset += 4;
    offsets.push_back( last_offset );
    cellstype.push_back( 10 );  

    // Top cells: tetrahedron  
    for ( i = 0; i < ptsPerlevel-1 ; ++i )
    {
        connectivity.push_back( firstpoint_globalnumber
                        + ( 2 * visuNodeNbPerQar - 2 ) * ptsPerlevel + i );
        connectivity.push_back( firstpoint_globalnumber
                    + ( 2 * visuNodeNbPerQar - 2 ) * ptsPerlevel + i + 1 );
        connectivity.push_back( firstpoint_globalnumber + Top_number );	
        connectivity.push_back( firstpoint_globalnumber + GC_number );
        last_offset += 4;
        offsets.push_back( last_offset );
        cellstype.push_back( 10 ); 
    }
    connectivity.push_back( firstpoint_globalnumber
                        + ( 2 * visuNodeNbPerQar - 1 ) * ptsPerlevel - 1 );
    connectivity.push_back( firstpoint_globalnumber
                            + ( 2 * visuNodeNbPerQar - 2 ) * ptsPerlevel );
    connectivity.push_back( firstpoint_globalnumber + Top_number );	
    connectivity.push_back( firstpoint_globalnumber + GC_number );
    last_offset += 4;
    offsets.push_back( last_offset );
    cellstype.push_back( 10 ); 

    firstpoint_globalnumber += 4 * visuNodeNbPerQar * 
                                            ( 2 * visuNodeNbPerQar - 1 ) + 3;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Sphere<float>;
template class Sphere<double>;

#undef visuNodeNbPerQar