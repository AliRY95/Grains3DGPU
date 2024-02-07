#include "Convex.hh"
#include "Vector3.hh"

// -----------------------------------------------------------------------------
// Default constructor
__host__ __device__ Box::Box()
{}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
__host__ __device__ Box::Box( Vec3d const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
__host__ __device__ Box::Box( double x, double y, double z )
: m_extent( Vec3d( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__ Box::~Box()
{}




// -----------------------------------------------------------------------------
// Sets values of the edge length
__host__ __device__ void Box::setExtent( double x, double y, double z )
{
    m_extent = Vec3d( x, y, z );
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
__host__ __device__ Vec3d const Box::getExtent() const
{
    return ( m_extent );
}




// -----------------------------------------------------------------------------
// Returns the convex type
__host__ __device__ ConvexType Box::getConvexType() const
{
    return ( BOX );
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the box
__host__ __device__ double Box::computeCircumscribedRadius() const
{
    return ( m_extent.norm() );
}




// -----------------------------------------------------------------------------
// Returns the volume of the box
__host__ __device__ double Box::computeVolume() const
{
    return ( 8.0 * m_extent[X] * m_extent[Y] * m_extent[Z] );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
__host__ __device__ bool Box::buildInertia( double* inertia, 
                                            double* inertia_1 ) const
{
    inertia[1] = inertia[2] = inertia[4] = 0.0;
    inertia[0] = 8.0 * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[Z] * m_extent[Z] ) / 3.0;
    inertia[3] = 8.0 * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[X] * m_extent[X] + m_extent[Z] * m_extent[Z] ) / 3.0;
    inertia[5] = 8.0 * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[X] * m_extent[X] ) / 3.0;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = 0.0;
    inertia_1[0] = 1.0 / inertia[0];
    inertia_1[3] = 1.0 / inertia[3];
    inertia_1[5] = 1.0 / inertia[5];

    return ( true );
}




// // -----------------------------------------------------------------------------
// // Returns the bounding volume to box
// __host__ __device__ BVolume const* Box::computeBVolume( unsigned int type ) const
// {
//     BVolume* bvol = NULL;
//     if ( type == 1 ) // OBB
//         bvol = new OBB( m_extent, Matrix() );
//     else if ( type == 2 ) // OBC
//     {
//         double a[2];
//         int axis = ( a[X] = fabs( m_extent[X] ) ) < ( a[Y] = fabs( m_extent[Y] ) )
//             ? Y : X;
//         int i = a[axis] < fabs( m_extent[Z] ) ? Z : axis;
//         Vector3 e( 0., 0., 0. );
//         e[i] = 1.;
//         double h = 2. * m_extent[i];
//         double r = sqrt( norm2(m_extent) - m_extent[i]*m_extent[i]);
//         bvol = new OBC( r, h, e );
//     }

//     return ( bvol );
// }




// -----------------------------------------------------------------------------
// Box support function, returns the support point P, i.e. the point on the
// surface of the box that satisfies max(P.v)
__host__ __device__ Vec3d Box::support( Vec3d const& v ) const
{
    double norm = v.norm2();
    if ( norm < EPSILON3 )
        return ( Vec3d() );
    else
        return ( Vec3d( v[X] < 0. ? -m_extent[X] : m_extent[X],
                        v[Y] < 0. ? -m_extent[Y] : m_extent[Y],
                        v[Z] < 0. ? -m_extent[Z] : m_extent[Z] ) );
}