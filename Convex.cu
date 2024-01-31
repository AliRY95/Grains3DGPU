#ifndef _CONVEX_CUH_
#define _CONVEX_CUH_

#include "Basic.cuh"
#include "Vector3.cu"


// =============================================================================
/** @brief The class Convex.

    Convex bodies.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
template <typename T>
class Convex
{
  protected:
    /**@name Parameters */
    //@{
    Vector3<T> m_extent; /**< vector containing the half-legnth of the edges */
    //@}

  public:
    /** @name Constructors */
    //@{
    /** @brief Default constructor */
    __host__ __device__ Convex();

    /** @brief Constructor with 3 components as inputs
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ Convex( T x, T y, T z );

    /** @brief Constructor with a vector containing the edge half-lengths
    @param extent_ vector of half-lengths */
    __host__ __device__ Convex( Vector3<T> const& extent_ );

    /** @brief Destructor */
    __host__ __device__ ~Convex();
    //@}


    /** @name Methods */
    //@{
    /** @brief Sets values of the edge length
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ void setExtent( T x, T y, T z );

    /** @brief Gets values of the edge length
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ Vector3<T> const getExtent() const;

    /** @brief Box support function, returns the support point P, i.e. the
    point on the surface of the box that satisfies max(P.v)
    @param v direction */
    __host__ __device__ Vector3<T> support( Vector3<T> const& v ) const;
    //@}
};


/** @name Convex : External methods for the GJK algorithm */
//@{
/** @brief Returns whether 2 convex shapes intersect
 @param a convex shape A
 @param b convex shape B
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
template <typename T>
__host__ __device__ bool intersectGJK( Convex<T> const* a, 
                                       Convex<T> const* b,
                                       Vector3<T> const& a2w,
	                                   Vector3<T> const& b2w );

/** @brief Returns whether the bounding boxex are in contact or not
 @param a bounding box of A
 @param b bounding box of B
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
template <typename T>
__host__ __device__ bool intersectAABB( Convex<T> const* a, 
                                        Convex<T> const* b,
                                        Vector3<T> const& a2w,
	                                    Vector3<T> const& b2w );


//@}

/* ========================================================================== */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__ Convex<T>::Convex()
{}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
template <typename T>
__host__ __device__ Convex<T>::Convex( Vector3<T> const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
template <typename T>
__host__ __device__ Convex<T>::Convex( T x, T y, T z )
: m_extent( Vector3<T>( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__ Convex<T>::~Convex()
{}




// -----------------------------------------------------------------------------
// Sets values of the edge length
template <typename T>
__host__ __device__ void Convex<T>::setExtent( T x, T y, T z )
{
    m_extent[X] = x;
    m_extent[Y] = y;
    m_extent[Z] = z;
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
template <typename T>
__host__ __device__ Vector3<T> const Convex<T>::getExtent() const
{
    return( m_extent );
}




// -----------------------------------------------------------------------------
// Box support function, returns the support point P, i.e. the point on the
// surface of the box that satisfies max(P.v)
template <typename T>
__host__ __device__ Vector3<T> Convex<T>::support( Vector3<T> const& v ) const
{
    T norm = Norm( v );
    if ( norm < EPSILON1 )
        return ( Vector3<T>() );
    else
        return ( Vector3<T>( v[X] < 0. ? -m_extent[X] : m_extent[X],
                             v[Y] < 0. ? -m_extent[Y] : m_extent[Y],
                             v[Z] < 0. ? -m_extent[Z] : m_extent[Z] ) );
}




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
template <typename T>
__host__ __device__ void computeDet( int const bits,
                                     int const last,
                                     int const last_bit,
                                     int const all_bits,
                                     Vector3<T> const y[4],
                                     T dp[4][4],
                                     T det[16][4] )
{
    for( int i = 0, bit = 1; i < 4; ++i, bit <<=1 )
        if (bits & bit) 
            dp[i][last] = dp[last][i] = y[i] * y[last];
    dp[last][last] = y[last] * y[last];

    det[last_bit][last] = T( 1 );
    for( int j = 0, sj = 1; j < 4; ++j, sj <<= 1 )
    {
        if( bits & sj )
        {
            int s2 = sj | last_bit;
            det[s2][j] = dp[last][last] - dp[last][j];
            det[s2][last] = dp[j][j] - dp[j][last];
            for( int k = 0, sk = 1; k < j; ++k, sk <<= 1 )
            {
                if( bits & sk )
                {
                    int s3 = sk | s2;
                    det[s3][k] = det[s2][j] * (dp[j][j] - dp[j][k]) +
                        det[s2][last] * (dp[last][j] - dp[last][k]);
                    det[s3][j] = det[sk|last_bit][k] * (dp[k][k] - dp[k][j]) +
                        det[sk|last_bit][last] * (dp[last][k] - dp[last][j]);
                    det[s3][last] = det[sk|sj][k] * (dp[k][k] - dp[k][last]) +
                        det[sk|sj][j] * (dp[j][k] - dp[j][last]);
                }
            }
        }
    }

    if( all_bits == 15 )
    {
        det[15][0] = det[14][1] * (dp[1][1] - dp[1][0]) +
                     det[14][2] * (dp[2][1] - dp[2][0]) +
                     det[14][3] * (dp[3][1] - dp[3][0]);
        det[15][1] = det[13][0] * (dp[0][0] - dp[0][1]) +
                     det[13][2] * (dp[2][0] - dp[2][1]) +
                     det[13][3] * (dp[3][0] - dp[3][1]);
        det[15][2] = det[11][0] * (dp[0][0] - dp[0][2]) +
                     det[11][1] * (dp[1][0] - dp[1][2]) +
                     det[11][3] * (dp[3][0] - dp[3][2]);
        det[15][3] = det[7][0] * (dp[0][0] - dp[0][3]) +
                     det[7][1] * (dp[1][0] - dp[1][3]) +
                     det[7][2] * (dp[2][0] - dp[2][3]);
    }
}




// -----------------------------------------------------------------------------
template <typename T>
__host__ __device__  inline bool valid( int const s,
                                        int const all_bits,
                                        T const det[16][4] )
{
    for ( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
    {
        if ( all_bits & bit )
        {
            if ( s & bit )
            {
                if ( det[s][i] <= EPSILON1 )
                    return ( false );
            }
            else if ( det[s|bit][i] > T( 0 ) )
                return ( false );
        }
    }
    return ( true );
}




// -----------------------------------------------------------------------------
template <typename T>
__host__ __device__ inline void computeVec( int const bits_,
                                            Vector3<T> const y[4],
                                            T const det[16][4],
                                            Vector3<T>& v )
{
    T sum = T( 0 );
    v.setValue( T( 0 ), T( 0 ), T( 0 ) );
    for ( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
    {
        if ( bits_ & bit )
        {
            sum += det[bits_][i];
            v += y[i] * det[bits_][i];
        }
    }
    v *= T( 1 ) / sum;
}




// -----------------------------------------------------------------------------
template <typename T>
__host__ __device__ inline bool proper( int const s,
                                        T const det[16][4] )
{
    for( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
        if( ( s & bit ) && det[s][i] <= EPSILON3 )
            return ( false );
    return ( true );
}




// -----------------------------------------------------------------------------
template <typename T>
__host__ __device__ inline bool closest( int& bits,
                                         int const last,
                                         int const last_bit,
                                         int const all_bits,
                                         Vector3<T> const y[4],
                                         T dp[4][4],
                                         T det[16][4],
                                         Vector3<T>& v )
{
    int s;
    computeDet( bits, last, last_bit, all_bits, y, dp, det );
    for ( s = bits; s; --s )
    {
        if ( ( s & bits ) == s )
        {
            if ( valid( s | last_bit, all_bits, det ) )
            {
                bits = s | last_bit;
                computeVec( bits, y, det, v );
                return( true );
            }
        }
    }
    if ( valid( last_bit, all_bits, det ) )
    {
        bits = last_bit;
        v = y[last];
        return( true );
    }
    // Original GJK calls the backup procedure at this point.
    T min_dist2 = INFINITY;
    for ( s = all_bits; s; --s )
    {
        if ( ( s & all_bits ) == s )
        {
            if ( proper( s, det ) )
            {
                Vector3<T> u;
                computeVec( s, y, det, u );
                T dist2 = Norm2( u );
                if ( dist2 < min_dist2 )
                {
                    min_dist2 = dist2;
                    bits = s;
                    v = u;
                }
            }
        }
    }
    return ( false );
}




// ----------------------------------------------------------------------------
// The next function is used for detecting degenerate cases that cause
// termination problems due to rounding errors.
template <typename T>
__host__ __device__ inline bool degenerate( int const all_bits,
                                            Vector3<T> const y[4],
                                            Vector3<T> const& w )
{
  for ( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
    if ( (all_bits & bit) && y[i] == w )
        return ( true );
  return ( false );
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Returns whether 2 convex shapes intersect
template <typename T>
__host__ __device__  bool intersectGJK( Convex<T> const* a,
                                        Convex<T> const* b,
                                        Vector3<T> const& a2w,
                                        Vector3<T> const& b2w )
{
    int bits = 0;           // identifies current simplex
    int last = 0;           // identifies last found support point
    int last_bit = 0;       // last_bit = 1<<last
    int all_bits = 0;       // all_bits = bits|last_bit
    Vector3<T> y[4];        // support points of A - B in world coordinates
    T det[16][4] = {T( 0 )};// cached sub-determinants
    T dp[4][4] = {T( 0 )};

    Vector3<T> v( T( 1 ), T( 1 ), T( 1 )), w;
    T prod;
    
    do {        
        last = 0;
        last_bit = 1;
        while( bits & last_bit )
        {
            ++last;
            last_bit <<= 1;
        }
        w = ( a2w + a->support( -v ) ) - ( b2w + b->support( v ) ); // Change!
        // w = a->support( -v ) - b->support( v ); // Change!
        // printf("[%f %f %f], [%f %f %f] \n", v[X], v[Y] , v[Z],
        //                                     w[X], w[Y] , w[Z] );
        prod = v * w;
        if( prod > T( 0 ) || fabs( prod ) < EPSILON2 )
            return ( false );
        if ( degenerate( all_bits, y, w ) )
            return ( false );      
        y[last] = w;
        all_bits = bits | last_bit;
        if ( !closest( bits, last, last_bit, all_bits, y, dp, det, v ) )
            return ( false );
    } while ( bits < 15 && !approxZero( v ) );
    return ( true );
}




// ----------------------------------------------------------------------------
// Returns whether the bounding boxes of 2 convex shapes intersect
template <typename T>
__host__ __device__  bool intersectAABB( Convex<T> const* a,
                                         Convex<T> const* b,
                                         Vector3<T> const& a2w,
                                         Vector3<T> const& b2w )
{
    Vector3<T> const AABB1 = a->getExtent();
    Vector3<T> const AABB2 = b->getExtent();
    if ( fabs( a2w[X] - b2w[X] ) > ( AABB1[X] + AABB2[X] ) )
        return ( false );
    else if ( fabs( a2w[Y] - b2w[Y] ) > ( AABB1[Y] + AABB2[Y] ) )
        return ( false );
    else if ( fabs( a2w[Z] - b2w[Z] ) > ( AABB1[Z] + AABB2[Z] ) )
        return ( false );
    else // We have an overlap
        return ( true );
}

#endif