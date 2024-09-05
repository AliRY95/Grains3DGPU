#include "GJK_AY.hh"
#include "MatrixMath.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE T determinant( Vector3<T> const p,
                             Vector3<T> const q,
                             Vector3<T> const r )
{
  return ( p[0] * ( (q[1] * r[2] ) - ( r[1] * q[2] ) ) - 
           p[1] * ( q[0] * r[2] - r[0] * q[2] ) +
           p[2] * ( q[0] * r[1] - r[0] * q[1] ) );
}




// -----------------------------------------------------------------------------
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE void projectOnLine( Vector3<T> const p,
                                  Vector3<T> const q,
                                  Vector3<T>& v )
{
    Vector3<T> pq = p - q;
    T const tmp = ( p * pq ) / ( pq * pq);
    v = p - tmp * pq;
}




// -----------------------------------------------------------------------------
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE void projectOnPlane( Vector3<T> const p,
                                   Vector3<T> const q,
                                   Vector3<T> const r, 
                                   Vector3<T>& v )
{
    Vector3<T> n = ( p - q ) ^ ( p - r );
    T const tmp = ( n * p ) / ( n * n );
    v = tmp * n;
}




// -----------------------------------------------------------------------------
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE int hff1( Vector3<T> const p, 
                        Vector3<T> const q )
{
    if ( norm2( p ) - p * q > 0 )
        return 1; // keep q
    return 0;
}




// -----------------------------------------------------------------------------
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE int hff2( Vector3<T> const p,
                        Vector3<T> const q,
                        Vector3<T> const r )
{
    Vector3<T> pq = q - p;
    Vector3<T> nTemp = pq ^ ( r - p );
    Vector3<T> n = pq ^ nTemp;
    return ( p * n < 0 ); // Discard r if true
}




// -----------------------------------------------------------------------------
// TODO: COMMENT ???
template <typename T>
__HOSTDEVICE__
static INLINE int hff3( Vector3<T> const p,
                        Vector3<T> const q,
                        Vector3<T> const r )
{
    Vector3<T> n = ( q - p ) ^ ( r - p );
    return ( p * n <= 0 ); // discard s if true
}




// -----------------------------------------------------------------------------
// Handling the case where the simplex is of kind 1-simplex ( line )
template <typename T>
__HOSTDEVICE__
static INLINE void S1D( Simplex<T>& s, 
                        Vector3<T>& v )
{
    Vector3<T> const s1p = s.vrtx[1];
    Vector3<T> const s2p = s.vrtx[0];

    if ( hff1( s1p, s2p ) ) 
    {
        // update v, no need to update s, return V{1,2}
        projectOnLine( s1p, s2p, v );
        return;
    } 
    else 
    {
        // Update v and s, return V{1}
        v = s.vrtx[1];
        s.nvrtx = 1;
        s.vrtx[0] = s.vrtx[1];
        return;
    }
}




// -----------------------------------------------------------------------------
// Handling the case where the simplex is of kind 2-simplex ( triangle )
template <typename T>
__HOSTDEVICE__
static INLINE void S2D( Simplex<T>& s,
                        Vector3<T>& v ) 
{
    Vector3<T> const s1p = s.vrtx[2];
    Vector3<T> const s2p = s.vrtx[1];
    Vector3<T> const s3p = s.vrtx[0];
    int const hff1f_s12 = hff1( s1p, s2p );
    int const hff1f_s13 = hff1( s1p, s3p );

    if ( hff1f_s12 ) 
    {
        int const hff2f_23 = !hff2( s1p, s2p, s3p );
        if ( hff2f_23 )
        {
            if ( hff1f_s13 )
            {
                int const hff2f_32 = !hff2( s1p, s3p, s2p );
                if ( hff2f_32 )
                {
                    projectOnPlane( s1p, s2p, s3p, v ); // update s, 
                                                        // no need to update c
                    return;                             // return V{1,2,3}
                } 
                else
                {
                    // update v, update s, return V{1,3}
                    projectOnLine( s1p, s3p, v );
                    s.nvrtx = 2;
                    s.vrtx[1] = s.vrtx[2];
                    return;
                }
            } 
            else
            {
                projectOnPlane( s1p, s2p, s3p, v ); // update s, 
                                                    // no need to update c
                return;                             // return V{1,2,3}
            }
        } 
        else
        {
            // update v, update s, return V{1,2}
            projectOnLine( s1p, s2p, v );
            s.nvrtx = 2;
            s.vrtx[0] = s.vrtx[2];
            return;
        }
    } 
    else if ( hff1f_s13 ) 
    {
        int const hff2f_32 = !hff2( s1p, s3p, s2p );
        if ( hff2f_32 ) 
        {
            projectOnPlane( s1p, s2p, s3p, v ); // update s, no need to update v
            return;                             // return V{1,2,3}
        }
        else
        {
            // update v, update s, return V{1,3}
            projectOnLine( s1p, s3p, v );
            s.nvrtx = 2;
            s.vrtx[1] = s.vrtx[2];
            return;
        }
    }
    else
    {
        // update s and v, return V{1}
        v = s.vrtx[2];
        s.nvrtx = 1;
        s.vrtx[0] = s.vrtx[2];
        return;
    }
}





// -----------------------------------------------------------------------------
// Handling the case where the simplex is of kind 3-simplex ( tetrahedron )
template <typename T>
__HOSTDEVICE__
static INLINE void S3D( Simplex<T>& s, 
                        Vector3<T>& v )
{
    Vector3<T> s1, s2, s3, s4, s1s2, s1s3, s1s4;
    Vector3<T> si, sj, sk;
    int testLineThree, testLineFour, testPlaneTwo, 
        testPlaneThree, testPlaneFour, dotTotal;
    int i, j, k;

    s1 = s.vrtx[3];
    s2 = s.vrtx[2];
    s3 = s.vrtx[1];
    s4 = s.vrtx[0];
    s1s2 = s2 - s1;
    s1s3 = s3 - s1;
    s1s4 = s4 - s1;

    int hff1_tests[3];
    hff1_tests[2] = hff1( s1, s2 );
    hff1_tests[1] = hff1( s1, s3 );
    hff1_tests[0] = hff1( s1, s4 );
    testLineThree = hff1_tests[1];
    testLineFour  = hff1_tests[0];

    dotTotal = hff1_tests[2] + testLineThree + testLineFour;
    if ( dotTotal == 0 )
    {
        v = s1;
        s.nvrtx = 1;
        s.vrtx[0] = s1;
        return;
    }

    T const det134 = determinant( s1s3, s1s4, s1s2 );
    int const sss = ( det134 <= 0 );

    testPlaneTwo = hff3( s1, s3, s4 ) - sss;
    testPlaneTwo = testPlaneTwo * testPlaneTwo;
    testPlaneThree = hff3( s1, s4, s2 ) - sss;
    testPlaneThree = testPlaneThree * testPlaneThree;
    testPlaneFour = hff3( s1, s2, s3 ) - sss;
    testPlaneFour = testPlaneFour * testPlaneFour;

    switch ( testPlaneTwo + testPlaneThree + testPlaneFour )
    {
        case 3:
            v = zeroVector3T;
            s.nvrtx = 4;
            break;
        case 2:
            // Only one facing the origin
            // 1,i,j, are the indices of the points on the triangle and remove k
            // from simplex
            s.nvrtx = 3;
            if ( !testPlaneTwo ) // removes s2
                s.vrtx[2] = s.vrtx[3];
            else if ( !testPlaneThree ) // removes s3
            { 
                s.vrtx[1] = s2;
                s.vrtx[2] = s.vrtx[3];
            }
            else if ( !testPlaneFour ) // removes s4 - no need to reorder
            {
                s.vrtx[0] = s3;
                s.vrtx[1] = s2;
                s.vrtx[2] = s.vrtx[3];
            }
            // Call S2D
            S2D( s, v );
            break;
        case 1:
            // Two triangles face the origins:
            // The only positive hff3 is for triangle 1,i,j, therefore k must be
            // in the solution as it supports the the point of minimum norm.
            // 1,i,j, are the indices of the points on the triangle and remove k
            // from simplex
            s.nvrtx = 3;
            if ( testPlaneTwo )
            {
                k = 2; // s2
                i = 1;
                j = 0;
            }
            else if ( testPlaneThree )
            {
                k = 1; // s3
                i = 0;
                j = 2;
            }
            else
            {
                k = 0; // s4
                i = 2;
                j = 1;
            }

            si = s.vrtx[i];
            sj = s.vrtx[j];
            sk = s.vrtx[k];

            if ( dotTotal == 1 )
            {
                if ( hff1_tests[k] )
                {
                    if ( !hff2( s1, sk, si ) )
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = si;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, si, sk, v );
                    }
                    else if ( !hff2( s1, sk, sj ) )
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = sj;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sj, sk, v );
                    } 
                    else 
                    {
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = sk;
                        projectOnLine(s1, sk, v);
                    }
                } 
                else if ( hff1_tests[i] )
                {
                    if ( !hff2( s1, si, sk ) )
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = si;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, si, sk, v );
                    } 
                    else
                    {
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = si;
                        projectOnLine( s1, si, v );
                    }
                } 
                else
                {
                    if ( !hff2( s1, sj, sk ) )
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = sj;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sj, sk, v );
                    }
                    else
                    {
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = sj;
                        projectOnLine( s1, sj, v );
                    }
                }
            }
            else if ( dotTotal == 2 )
            {
                // Two edges have positive hff1, meaning that for two edges the
                // origin's project fall on the segement.
                // Certainly the edge 1,k supports the the point of minimum norm,
                // and so hff1_1k is positive.
                if ( hff1_tests[i] )
                {
                    if ( !hff2( s1, sk, si ) )
                    {
                        if ( !hff2( s1, si, sk ) ) 
                        {
                            s.nvrtx = 3;
                            s.vrtx[2] = s.vrtx[3];
                            s.vrtx[1] = si;
                            s.vrtx[0] = sk;
                            projectOnPlane( s1, si, sk, v );
                        }
                        else
                        {
                            s.nvrtx = 2;
                            s.vrtx[1] = s.vrtx[3];
                            s.vrtx[0] = sk;
                            projectOnLine( s1, sk, v );
                        }
                    }
                    else
                    {
                        if ( !hff2( s1, sk, sj ) )
                        {
                            s.nvrtx = 3;
                            s.vrtx[2] = s.vrtx[3];
                            s.vrtx[1] = sj;
                            s.vrtx[0] = sk;
                            projectOnPlane( s1, sj, sk, v );
                        } 
                        else
                        {
                            s.nvrtx = 2;
                            s.vrtx[1] = s.vrtx[3];
                            s.vrtx[0] = sk;
                            projectOnLine( s1, sk, v );
                        }
                    }
                }
                else if ( hff1_tests[j] ) // there is no other choice
                { 
                    if ( !hff2( s1, sk, sj ) )
                    {
                        if (!hff2(s1, sj, sk))
                        {
                            s.nvrtx = 3;
                            s.vrtx[2] = s.vrtx[3];
                            s.vrtx[1] = sj;
                            s.vrtx[0] = sk;
                            projectOnPlane( s1, sj, sk, v );
                        }
                        else
                        {
                            s.nvrtx = 2;
                            s.vrtx[1] = s.vrtx[3];
                            s.vrtx[0] = sj;
                            projectOnLine( s1, sj, v );
                        }
                    } 
                    else
                    {
                        if (!hff2(s1, sk, si)) 
                        {
                            s.nvrtx = 3;
                            s.vrtx[2] = s.vrtx[3];
                            s.vrtx[1] = si;
                            s.vrtx[0] = sk;
                            projectOnPlane( s1, si, sk, v );
                        }
                        else
                        {
                            s.nvrtx = 2;
                            s.vrtx[1] = s.vrtx[3];
                            s.vrtx[0] = sk;
                            projectOnLine( s1, sk, v );
                        }
                    }
                } 
                else
                {
                    // ERROR;
                }

            } 
            else if ( dotTotal == 3 )
            {
                // MM : ALL THIS HYPHOTESIS IS FALSE
                // sk is s.t. hff3 for sk < 0. So, sk must support the origin 
                // because there are 2 triangles facing the origin.
                int hff2_ik = hff2( s1, si, sk );
                int hff2_jk = hff2( s1, sj, sk );
                int hff2_ki = hff2( s1, sk, si );
                int hff2_kj = hff2( s1, sk, sj );

                // if ( hff2_ki == 0 && hff2_kj == 0 )
                // {
                //     mexPrintf("\n\n UNEXPECTED VALUES!!! \n\n");
                // }
                if ( hff2_ki == 1 && hff2_kj == 1 )
                {
                    s.nvrtx = 2;
                    s.vrtx[1] = s.vrtx[3];
                    s.vrtx[0] = sk;
                    projectOnLine( s1, sk, v );
                } 
                else if ( hff2_ki ) // discard i
                {
                    if ( hff2_jk ) // discard k
                    {       
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = sj;
                        projectOnLine( s1, sj, v );
                    }
                    else
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = sj;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sk, sj, v );
                    }
                } 
                else // discard j
                {
                    if ( hff2_ik ) // discard k
                    {
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = si;
                        projectOnLine( s1, si, v );
                    }
                    else
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = si;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sk, si, v );
                    }
                }
            }
            break;
        case 0:
            // The origin is outside all 3 triangles
            if ( dotTotal == 1 )
            {
                if (testLineThree) // Here si is set such that hff(s1,si) > 0
                {
                    k = 2;
                    i = 1; // s3
                    j = 0;
                }
                else if ( testLineFour ) 
                {
                    k = 1; // s3
                    i = 0;
                    j = 2;
                }
                else
                {
                    k = 0;
                    i = 2; // s2
                    j = 1;
                }
                si = s.vrtx[i];
                sj = s.vrtx[j];
                sk = s.vrtx[k];

                if ( !hff2( s1, si, sj ) )
                {
                    s.nvrtx = 3;
                    s.vrtx[2] = s.vrtx[3];
                    s.vrtx[1] = si;
                    s.vrtx[0] = sj;
                    projectOnPlane( s1, si, sj, v );
                }
                else if ( !hff2( s1, si, sk ) )
                {
                    s.nvrtx = 3;
                    s.vrtx[2] = s.vrtx[3];
                    s.vrtx[1] = si;
                    s.vrtx[0] = sk;
                    projectOnPlane( s1, si, sk, v );
                }
                else
                {
                    s.nvrtx = 2;
                    s.vrtx[1] = s.vrtx[3];
                    s.vrtx[0] = si;
                    projectOnLine( s1, si, v );
                }
            }
            else if ( dotTotal == 2 )
            {
                // Here si is set such that hff(s1,si) < 0
                s.nvrtx = 3;
                if ( !testLineThree )
                {
                    k = 2;
                    i = 1; // s3
                    j = 0;
                } 
                else if ( !testLineFour ) 
                {
                    k = 1;
                    i = 0; // s4
                    j = 2;
                } 
                else
                {
                    k = 0;
                    i = 2; // s2
                    j = 1;
                }
                si = s.vrtx[i];
                sj = s.vrtx[j];
                sk = s.vrtx[k];

                if ( !hff2( s1, sj, sk ) )
                {
                    if ( !hff2( s1, sk, sj ) ) 
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = sj;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sj, sk, v );
                    } 
                    else if ( !hff2( s1, sk, si ) )
                    {
                        s.nvrtx = 3;
                        s.vrtx[2] = s.vrtx[3];
                        s.vrtx[1] = si;
                        s.vrtx[0] = sk;
                        projectOnPlane( s1, sk, si, v );
                    } 
                    else 
                    {
                        s.nvrtx = 2;
                        s.vrtx[1] = s.vrtx[3];
                        s.vrtx[0] = sk;
                        projectOnLine( s1, sk, v );
                    }
                } 
                else if ( !hff2( s1, sj, si ) )
                {
                    s.nvrtx = 3;
                    s.vrtx[2] = s.vrtx[3];
                    s.vrtx[1] = si;
                    s.vrtx[0] = sj;
                    projectOnPlane( s1, si, sj, v );
                }
                else
                {
                    s.nvrtx = 2;
                    s.vrtx[1] = s.vrtx[3];
                    s.vrtx[0] = sj;
                    projectOnLine( s1, sj, v );
                }
            }
            break;
        default:
            printf("\nERROR:\tunhandled");
    }
}




// -----------------------------------------------------------------------------
// Distance subalgorithm using the signed volume method
template <typename T>
__HOSTDEVICE__
static INLINE void subalgorithm( Simplex<T>& s,
                                 Vector3<T>& v )
{
    switch ( s.nvrtx )
    {
        case 4:
            S3D( s, v );
            break;
        case 3:
            S2D( s, v );
            break;
        case 2:
            S1D( s, v );
            break;
        default:
            printf("\nERROR:\t invalid simplex\n");
    }
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Returns whether 2 convex shapes intersect using the GJK_SV algorithm
template <typename T>
__HOSTDEVICE__
T computeClosestPoints_GJK_AY( Convex<T> const& a, 
                               Convex<T> const& b, 
                               Transform3<T> const& a2w,
                               Transform3<T> const& b2w, 
                               Vector3<T>& pa,
                               Vector3<T>& pb,
                               int& nbIter ) 
{
    /* GJK parameters */
    // iteration counter
    unsigned int numIterations = 0;
    // maximum number of GJK iterations
    constexpr unsigned int maxNumIterations = 30;
    // counter for number of vertices
    unsigned int i = 0;
    // relative tolerance
    constexpr T relError = 1.e-4;
    // absolute tolerance
    constexpr T absError = 1.e-4 * relError;
    // optimality gap
    T mu = T( 0 );

    /* Acceleration parameters */
    bool acceleration = false;
    T momentum = T( 0 ), oneMinusMomentum = T( 1 );

    /* Initialization */
    // Grains3D parameters
    // Vector3<T> c2c( a2w.getOrigin() - b2w.getOrigin() );
    // Vector3<T> w( a2w( a.support( ( -c2c ) * a2w.getBasis() ) ) - 
    //               b2w( b.support( (  c2c ) * b2w.getBasis() ) ) );
    Vector3<T> w( b2w.getOrigin() - a2w.getOrigin() );
    Vector3<T> v( w );
    Vector3<T> d( w );
    
    /* Initialise simplex */
    T dist = norm( v );
    Simplex<T> s = { 1, { zeroVector3T } };
    for ( int t = 0; t < 3; ++t )
        s.vrtx[0][t] = w[t];


    /* Begin GJK iteration */
    do {
        numIterations++;

        // Finding the suitable direction using either Nesterov or original
        // The number 8 is hard-coded. Emprically, it shows the best convergence
        // for superquadrics. For the rest of shapes, we really do not need to 
        // use Nesterov as the improvemenet is marginal.
        if ( acceleration && numIterations % 8 != 0 )
        {
            momentum = numIterations / ( numIterations + 2. );
            oneMinusMomentum = T( 1 ) - momentum;
            d = momentum * d + 
                   momentum * oneMinusMomentum * v +
                   oneMinusMomentum * oneMinusMomentum * w;
        }
        else
            d = v;
    
        w = a2w( a.support( ( -d ) * a2w.getBasis() ) ) - 
            b2w( b.support( (  d ) * b2w.getBasis() ) );
    
        // termination criteria
        mu = dist - v * w / dist;
        if ( mu < dist * relError || mu < absError )
        {
            if ( acceleration )
            {
                // if d == v
                if ( norm( d - v ) < LOWEPS<T> )
                    break;
                // otherwise turn off the acceleration
                else
                {
                    acceleration = false;
                    w = a2w( a.support( ( -v ) * a2w.getBasis() ) ) - 
                        b2w( b.support( (  v ) * b2w.getBasis() ) );
                }
            }
            // if the acceleration is off, we have converged.
            else
                break;
        }

        // Add the new vertex to simplex
        i = s.nvrtx;
        for ( int t = 0; t < 3; ++t )
            s.vrtx[i][t] = w[t];
        s.nvrtx++;

        // Invoke distance sub-algorithm
        subalgorithm( s, v );
        dist = norm( v );

    } while ( ( s.nvrtx != 4 ) && 
              ( numIterations != maxNumIterations ) && 
              ( dist > HIGHEPS<T> ) );
    /* Outputs: witness points and nbIter */
    // It is not the best way to return the witness points.
    // It seems to be inconsistent with the way that it has been done in the
    // original GJK.
    // We can use the latest update on w to get a better estimate on witness
    // points.
    pa = a.support( ( -v ) * a2w.getBasis() );
    pb = a.support( (  v ) * b2w.getBasis() );
    nbIter = numIterations;
    return ( dist );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T ) \
template                                                                       \
__HOSTDEVICE__                                                                 \
T computeClosestPoints_GJK_AY( Convex<T> const& a,                             \
                               Convex<T> const& b,                             \
                               Transform3<T> const& a2w,                       \
                               Transform3<T> const& b2w,                       \
                               Vector3<T>& pa,                                 \
                               Vector3<T>& pb,                                 \
                               int& nbIter );
X( float )
X( double )
#undef X
