#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1
#include <bits/stdc++.h>
#include "Convex.hh"
#include "Superquadric.hh"


// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
__host__ __device__
Superquadric::Superquadric( double a, 
                            double b, 
                            double c, 
                            double n1, 
                            double n2 )
: m_a( a )
, m_b( b )
, m_c( c )
, m_n1( n1 )
, m_n2( n2 )
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__
Superquadric::~Superquadric()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
__host__ __device__
ConvexType Superquadric::getConvexType() const
{
    return ( SUPERQUADRIC );
}




// -----------------------------------------------------------------------------
// Returns the volume of the Superquadric
// TODO: beta function for GPU
__host__ __device__
double Superquadric::computeVolume() const
{
#ifdef __CUDA_ARCH__

#else
    double const C = 2.0 / 3.0 * m_a * m_b * m_c;
    double const eps1 = 2.0 / m_n1;
    double const eps2 = 2.0 / m_n2;

    return ( C * eps1 * eps2 * std::beta( eps1, 0.5 * eps1 ) * 
                               std::beta( 0.5 * eps2, 0.5 * eps2 ) );
#endif                            
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
// TODO: beta function for GPU
__host__ __device__
bool Superquadric::computeInertia( double* inertia, 
                                   double* inertia_1 ) const
{
#ifdef __CUDA_ARCH__

#else
    double const eps1 = 2.0 / m_n1;
    double const eps2 = 2.0 / m_n2;
    double const C = 0.4 * m_a * m_b * m_c * eps1 * eps2 ;

    double const prod1 = std::beta( 1.5 * eps2, 0.5 * eps2 ) * 
                         std::beta( 2.0 * eps1, 0.5 * eps1 );
    double const prod2 = m_c * m_c * std::beta( 0.5 * eps2, 0.5 * eps2 ) * 
                                     std::beta( 1.5 * eps1, eps1 );

    inertia[1] = inertia[2] = inertia[4] = 0.0;
    inertia[0] = C * ( m_b * m_b * prod1 + prod2 );
    inertia[3] = C * ( m_a * m_a * prod1  + prod2 );
    inertia[5] = C * ( m_a * m_a + m_b * m_b ) * prod1;

    inertia_1[1] = inertia_1[2] = inertia_1[4] = 0.0;
    inertia_1[0] = 1.0 / inertia[0];
    inertia_1[3] = 1.0 / inertia[3];
    inertia_1[5] = 1.0 / inertia[5];

    return ( true );
#endif
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Superquadric
__host__ __device__
double Superquadric::computeCircumscribedRadius() const
{
    if ( ( m_n1 == 2.0 ) && ( m_n2 == 2.0 ) )
        return ( max( m_a, max( m_b, m_c ) ) );
    else if ( m_n1 == 2.0 )
    {
        double const alpha = pow( m_b / m_a, 2.0 / ( m_n2 - 2.0 ) );
        double const xt = 1.0 / pow( 1 + pow( alpha, m_n2 ), 1.0 / m_n2 );
        return ( max( m_c, 
        sqrt( m_a * xt * m_a * xt + alpha * m_b * xt * alpha * m_b * xt ) ) );
    }
    else if ( m_n2 == 2.0 )
    {
        double const m = max( m_a, m_b );
        double const beta = pow( m_c * m_c / ( m * m ), 1.0 / ( m_n1 - 2.0 ) );
        double const xt = 1.0 / pow( 1.0 + pow( beta, m_n1 ), 1.0 / m_n1 );
        return ( sqrt( m * xt * m * xt + beta * m_c * xt * beta * m_c * xt ) );
    }
    else
    {
        double const alpha = pow( m_b / m_a, 2. / ( m_n2 - 2. ) );
        double const gamma = pow( 1. + pow( alpha, m_n2 ), m_n1 / m_n2 - 1. );
        double const beta = pow( gamma * m_c * m_c / ( m_a * m_a ), 
                                                            1. / (m_n1 - 2. ) );
        double const xt = 1. / pow( pow( 1. + pow( alpha, m_n2 ), m_n1 / m_n2 )
                                            + pow( beta, m_n1 ), 1.0 / m_n1 );

        return ( sqrt( m_a * xt * m_a * xt + alpha * m_b * xt * alpha * m_b * xt
            + beta * m_c * xt * beta * m_c * xt ) );
    }
}




// -----------------------------------------------------------------------------
// Returns the bounding box to Superquadric
__host__ __device__
Vec3f Superquadric::computeBoundingBox() const
{
    return ( Vec3f( (float) m_a, 
                    (float) m_b, 
                    (float) m_c ) );
}




// -----------------------------------------------------------------------------
// Superquadric support function, returns the support point P, i.e. the point on
// the surface of the Superquadric that satisfies max(P.v)
__host__ __device__
Vec3d Superquadric::support( Vec3d const& v ) const
{
    double norm = v.norm2();
    if ( norm > EPSILON3 )
    {
        double const abvx = abs( v[X] );
        double const abvy = abs( v[Y] );
        double const abvz = abs( v[Z] );
        double const signx = copysign( 1.0, v[X] );
        double const signy = copysign( 1.0, v[Y] );
        double const signz = copysign( 1.0, v[Z] );

        Vec3d sup;

        if ( abvx == 0. )
        {
            if ( abvy == 0. )
                return ( Vec3d( 0., 0., signz * m_c ) );
            else
            {
                double const alpha = pow( m_c / m_b * abvz / abvy, 
                                                        1. / ( m_n1 - 1. ) );
                double const yt = 1. / pow( 1. + pow( alpha, m_n1 ), 1. / m_n1);
                return ( Vec3d( 0., 
                                signy * m_b * yt, 
                                signz * alpha * m_c * yt ) );
            }
        }
        else
        {
            double const alpha = pow( m_b / m_a * abvy / abvx, 
                                                        1. / ( m_n2 - 1. ) );
            double const temp = 1. + pow( alpha, m_n2 );
            double const gamma = pow( temp, 
                                ( m_n1 - m_n2 ) / ( m_n2 * ( m_n1 - 1. ) ) );
            double const beta = gamma * pow( m_c / m_a * abvz / abvx, 
                                                        1. / ( m_n1 - 1. ) );
            double const xt = 1. / pow( pow( temp, m_n1 / m_n2 )
                                      + pow( beta, m_n1 ) , 1. / m_n1 );
            return ( Vec3d( signx * m_a * xt, 
                            signy * alpha * m_b * xt, 
                            signz * beta * m_c * xt ) );
        }
    }
    else
    {
        return ( Vec3d() );
    }
}