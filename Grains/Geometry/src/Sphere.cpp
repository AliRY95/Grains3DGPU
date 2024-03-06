#include "Convex.hh"
#include "Sphere.hh"


// -----------------------------------------------------------------------------
// Constructor with radius
__host__ __device__ 
Sphere::Sphere( double r )
: m_radius( r )
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__
Sphere::~Sphere()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
__host__ __device__
ConvexType Sphere::getConvexType() const
{
    return ( SPHERE );
}




// -----------------------------------------------------------------------------
// Returns the radius
__host__ __device__
double Sphere::getRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Sets the radius
__host__ __device__
void Sphere::setRadius( double r )
{
    m_radius = r;
}




// -----------------------------------------------------------------------------
// Returns the volume of the Sphere
__host__ __device__
double Sphere::computeVolume() const
{
    return ( 4. * M_PI * m_radius * m_radius * m_radius / 3. );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
__host__ __device__
bool Sphere::computeInertia( double* inertia, 
                             double* inertia_1 ) const
{
    inertia[1] = inertia[2] = inertia[4] = 0.;
    inertia[5] = inertia[3] = inertia[0] = 8. * M_PI / 15. * pow( m_radius, 5.);

    inertia_1[1] = inertia_1[2] = inertia_1[4] = 0.;
    inertia_1[5] = inertia_1[3] = inertia_1[0] = 1. / inertia[0];

    return ( true );
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Sphere
__host__ __device__
double Sphere::computeCircumscribedRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to Sphere
__host__ __device__
Vec3f Sphere::computeBoundingBox() const
{
    return ( Vec3f( (float) m_radius, 
                    (float) m_radius, 
                    (float) m_radius ) );
}




// -----------------------------------------------------------------------------
// Sphere support function, returns the support point P, i.e. the point on the
// surface of the Sphere that satisfies max(P.v)
__host__ __device__
Vec3d Sphere::support( Vec3d const& v ) const
{
    double norm = v.norm2();
    if ( norm < EPSILON3 )
        return ( Vec3d() );
    else
    {
        double r = m_radius / norm;
        return ( Vec3d( v[X] * r, v[Y] * r, v[Z] * r ) );
    }
}