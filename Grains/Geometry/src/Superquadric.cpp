#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1
#include <bits/stdc++.h>
#include "Convex.hh"
#include "Superquadric.hh"


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
// Returns the volume of the Superquadric
template <typename T>
__HOSTDEVICE__
T Superquadric<T>::computeVolume() const
{
#ifdef __CUDA_ARCH__
    // TODO: beta function for GPU
#else
    T const C = T( 2 ) / T( 3 ) * m_a * m_b * m_c;
    T const eps1 = T( 2 ) / m_n1;
    T const eps2 = T( 2 ) / m_n2;

    return ( C * eps1 * eps2 * std::beta( eps1, 0.5 * eps1 ) * 
                               std::beta( 0.5 * eps2, 0.5 * eps2 ) );
#endif                            
}




// -----------------------------------------------------------------------------
// Returns the volume of the Superquadric - specialized for floats
template <>
__HOSTDEVICE__
float Superquadric<float>::computeVolume() const
{
#ifdef __CUDA_ARCH__
    // TODO: beta function for GPU
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
bool Superquadric<T>::computeInertia( T* inertia, 
                                              T* inertia_1 ) const
{
#ifdef __CUDA_ARCH__
    // TODO: beta function for GPU
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

    return ( true );
#endif
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor -
// specialized for floats
template <>
__HOSTDEVICE__
bool Superquadric<float>::computeInertia( float* inertia, 
                                                  float* inertia_1 ) const
{
#ifdef __CUDA_ARCH__
    // TODO: beta function for GPU
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

    return ( true );
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
    T norm = v.norm2();
    if ( norm > HIGHEPS )
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
    else
    {
        return ( Vector3<T>() );
    }
}




// -----------------------------------------------------------------------------
// Superquadric support function, returns the support point P, i.e. the point on
// the surface of the Superquadric that satisfies max(P.v) - 
template <>
__HOSTDEVICE__
Vector3<float> Superquadric<float>::support( Vector3<float> const& v ) const
{
    float norm = v.norm2();
    if ( norm > HIGHEPS )
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
                return ( Vector3<float>( 0.f, 
                                         0.f,
                                         signz * m_c ) );
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
    else
    {
        return ( Vector3<float>() );
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Superquadric<float>;
template class Superquadric<double>;