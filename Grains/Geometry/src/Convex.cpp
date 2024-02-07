#include "Convex.hh"
#include "Vector3.hh"

// -----------------------------------------------------------------------------
// Default constructor
__host__ __device__ Convex::Convex()
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__ Convex::~Convex()
{}




// // -----------------------------------------------------------------------------
// // Returns the convex shape bounding volume
// __host__ __device__ BVolume const* Convex::computeBVolume( unsigned int type ) 
//                                                                            const
// {
//   cout << "Warning for this Convex the method Convex::computeBVolume() "
//        << "is not yet implemented !\n";
//   return( nullptr );
// }






/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
__host__ __device__ void computeDet( int const bits,
                                     int const last,
                                     int const last_bit,
                                     int const all_bits,
                                     Vec3d const y[4],
                                     double dp[4][4],
                                     double det[16][4] )
{
    for( int i = 0, bit = 1; i < 4; ++i, bit <<=1 )
        if (bits & bit) 
            dp[i][last] = dp[last][i] = y[i] * y[last];
    dp[last][last] = y[last] * y[last];

    det[last_bit][last] = 1.;
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
__host__ __device__  inline bool valid( int const s,
                                        int const all_bits,
                                        double const det[16][4] )
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
            else if ( det[s|bit][i] > 0. )
                return ( false );
        }
    }
    return ( true );
}




// -----------------------------------------------------------------------------
__host__ __device__ inline void computeVec( int const bits_,
                                            Vec3d const y[4],
                                            double const det[16][4],
                                            Vec3d& v )
{
    double sum = 0.;
    v.setValue( 0., 0., 0. );
    for ( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
    {
        if ( bits_ & bit )
        {
            sum += det[bits_][i];
            v += y[i] * det[bits_][i];
        }
    }
    v *= 1. / sum;
}




// -----------------------------------------------------------------------------
__host__ __device__ inline bool proper( int const s,
                                        double const det[16][4] )
{
    for( int i = 0, bit = 1; i < 4; ++i, bit <<= 1 )
        if( ( s & bit ) && det[s][i] <= EPSILON3 )
            return ( false );
    return ( true );
}




// -----------------------------------------------------------------------------
__host__ __device__ inline bool closest( int& bits,
                                         int const last,
                                         int const last_bit,
                                         int const all_bits,
                                         Vec3d const y[4],
                                         double dp[4][4],
                                         double det[16][4],
                                         Vec3d& v )
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
    double min_dist2 = INFINITY;
    for ( s = all_bits; s; --s )
    {
        if ( ( s & all_bits ) == s )
        {
            if ( proper( s, det ) )
            {
                Vec3d u;
                computeVec( s, y, det, u );
                double dist2 = u.Norm2();
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
__host__ __device__ inline bool degenerate( int const all_bits,
                                            Vec3d const y[4],
                                            Vec3d const& w )
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
__host__ __device__  bool intersectGJK( Convex const* a,
                                        Convex const* b,
                                        Vec3d const& a2w,
                                        Vec3d const& b2w )
{
    int bits = 0;           // identifies current simplex
    int last = 0;           // identifies last found support point
    int last_bit = 0;       // last_bit = 1<<last
    int all_bits = 0;       // all_bits = bits|last_bit
    Vec3d y[4];        // support points of A - B in world coordinates
    double det[16][4] = { 0. };// cached sub-determinants
    double dp[4][4] = { 0. };

    Vec3d v( 1., 1., 1. ), w;
    double prod;
    
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
        if( prod > 0. || fabs( prod ) < EPSILON2 )
            return ( false );
        if ( degenerate( all_bits, y, w ) )
            return ( false );      
        y[last] = w;
        all_bits = bits | last_bit;
        if ( !closest( bits, last, last_bit, all_bits, y, dp, det, v ) )
            return ( false );
    } while ( bits < 15 && !v.isApproxZero() );
    return ( true );
}




// ----------------------------------------------------------------------------
// Returns whether the bounding boxes of 2 convex shapes intersect
__host__ __device__  bool intersectAABB( Convex const* a,
                                         Convex const* b,
                                         Vec3d const& a2w,
                                         Vec3d const& b2w )
{
    Vec3d const AABB1 = a->getExtent();
    Vec3d const AABB2 = b->getExtent();
    if ( fabs( a2w[X] - b2w[X] ) > ( AABB1[X] + AABB2[X] ) )
        return ( false );
    else if ( fabs( a2w[Y] - b2w[Y] ) > ( AABB1[Y] + AABB2[Y] ) )
        return ( false );
    else if ( fabs( a2w[Z] - b2w[Z] ) > ( AABB1[Z] + AABB2[Z] ) )
        return ( false );
    else // We have an overlap
        return ( true );
}