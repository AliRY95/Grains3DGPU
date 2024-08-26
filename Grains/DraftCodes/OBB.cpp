#include "MatrixMath.hh"
#include "BoundingBox.hh"
#include "Transform3.hh"
#include "OBB.hh"


/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Low-level methods for OBB as macros in double precision
#define TESTCASE1_D( i )                                                       \
( fabs( cen[i] ) >                                                             \
( a[i] +                                                                       \
  b[0] * fabs( ori[0][i] ) +                                                   \
  b[1] * fabs( ori[1][i] ) +                                                   \
  b[2] * fabs( ori[2][i] ) ) )

#define TESTCASE2_D( i )                                                       \
( fabs( cen[0] * ori[i][0] + cen[1] * ori[i][1] + cen[2] * ori[i][2] ) >       \
( b[i] +                                                                       \
  a[0] * fabs( ori[i][0] ) +                                                   \
  a[1] * fabs( ori[i][1] ) +                                                   \
  a[2] * fabs( ori[i][2] ) ) )

#define TESTCASE3_D(i, j)                                                      \
( fabs( cen[(j+2)%3] * ori[i][(j+1)%3] - cen[(j+1)%3] * ori[i][(j+2)%3] ) >    \
( a[(i+1)%3] * oriAbs[(i+2)%3][j] + a[(i+2)%3] * oriAbs[(i+1)%3][j] +          \
  b[(j+1)%3] * oriAbs[i][(j+2)%3] + b[(j+2)%3] * oriAbs[i][(j+1)%3] ) )




// -----------------------------------------------------------------------------
// Low-level methods for OBB as macros in single precision
#define TESTCASE1_F( i )                                                       \
( fabsf( cen[i] ) >                                                            \
( a[i] +                                                                       \
  b[0] * fabsf( ori[0][i] ) +                                                  \
  b[1] * fabsf( ori[1][i] ) +                                                  \
  b[2] * fabsf( ori[2][i] ) ) )

#define TESTCASE2_F( i )                                                       \
( fabsf( cen[0] * ori[i][0] + cen[1] * ori[i][1] + cen[2] * ori[i][2] ) >      \
( b[i] +                                                                       \
  a[0] * fabsf( ori[i][0] ) +                                                  \
  a[1] * fabsf( ori[i][1] ) +                                                  \
  a[2] * fabsf( ori[i][2] ) ) )

#define TESTCASE3_F(i, j)                                                      \
( fabsf( cen[(j+2)%3] * ori[i][(j+1)%3] - cen[(j+1)%3] * ori[i][(j+2)%3] ) >   \
( a[(i+1)%3] * oriAbs[(i+2)%3][j] + a[(i+2)%3] * oriAbs[(i+1)%3][j] +          \
  b[(j+1)%3] * oriAbs[i][(j+2)%3] + b[(j+2)%3] * oriAbs[i][(j+1)%3] ) )




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Returns whether the bounding boxes are in contact using OBB test
template <typename T>
__HOSTDEVICE__
bool intersectOrientedBoundingBox( BoundingBox<T> const& bbA, 
                                   BoundingBox<T> const& bbB,
                                   Transform3<T> const& trA2W,
                                   Transform3<T> const& trB2W )
{
    Vector3<T> const& a = bbA.getExtent();
    Vector3<T> const& b = bbB.getExtent();
    Vector3<T> const& cen = ( trA2W.getBasis() ).transpose() * 
                            ( trB2W.getOrigin() - trA2W.getOrigin() );
    Matrix3<T> const& ori = ( trB2W.getBasis() ).transpose() * trA2W.getBasis();
    Matrix3<T> const 
            oriAbs( fabs(ori[0][0]), fabs(ori[0][1]), fabs(ori[0][2]),
                    fabs(ori[1][0]), fabs(ori[1][1]), fabs(ori[1][2]),
                    fabs(ori[2][0]), fabs(ori[2][1]), fabs(ori[2][2]) );

    // CASE 1: ( three of them )
    if TESTCASE1_D( 0 ) return ( false );
    if TESTCASE1_D( 1 ) return ( false );
    if TESTCASE1_D( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2_D( 0 ) return ( false );
    if TESTCASE2_D( 1 ) return ( false );
    if TESTCASE2_D( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3_D( 0, 0 ) return ( false );
    if TESTCASE3_D( 1, 0 ) return ( false );
    if TESTCASE3_D( 2, 0 ) return ( false );
    if TESTCASE3_D( 0, 1 ) return ( false );
    if TESTCASE3_D( 1, 1 ) return ( false );
    if TESTCASE3_D( 2, 1 ) return ( false );
    if TESTCASE3_D( 0, 2 ) return ( false );
    if TESTCASE3_D( 1, 2 ) return ( false );
    if TESTCASE3_D( 2, 2 ) return ( false );
    
    return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using OBB test - 
// specialized for float
template <>
__HOSTDEVICE__
bool intersectOrientedBoundingBox<float>( BoundingBox<float> const& bbA, 
                                          BoundingBox<float> const& bbB,
                                          Transform3<float> const& trA2W,
                                          Transform3<float> const& trB2W )
{
    Vector3<float> const& a = bbA.getExtent();
    Vector3<float> const& b = bbB.getExtent();
    Vector3<float> const& cen = ( trA2W.getBasis() ).transpose() * 
                                ( trB2W.getOrigin() - trA2W.getOrigin() );
    Matrix3<float> const& ori = ( trB2W.getBasis() ).transpose() * 
                                trA2W.getBasis();
    Matrix3<float> const 
            oriAbs( fabsf(ori[0][0]), fabsf(ori[0][1]), fabsf(ori[0][2]),
                    fabsf(ori[1][0]), fabsf(ori[1][1]), fabsf(ori[1][2]),
                    fabsf(ori[2][0]), fabsf(ori[2][1]), fabsf(ori[2][2]) );

    // CASE 1: ( three of them )
    if TESTCASE1_F( 0 ) return ( false );
    if TESTCASE1_F( 1 ) return ( false );
    if TESTCASE1_F( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2_F( 0 ) return ( false );
    if TESTCASE2_F( 1 ) return ( false );
    if TESTCASE2_F( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3_F( 0, 0 ) return ( false );
    if TESTCASE3_F( 1, 0 ) return ( false );
    if TESTCASE3_F( 2, 0 ) return ( false );
    if TESTCASE3_F( 0, 1 ) return ( false );
    if TESTCASE3_F( 1, 1 ) return ( false );
    if TESTCASE3_F( 2, 1 ) return ( false );
    if TESTCASE3_F( 0, 2 ) return ( false );
    if TESTCASE3_F( 1, 2 ) return ( false );
    if TESTCASE3_F( 2, 2 ) return ( false );
    
    return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using OBB test - relative
// transformation
template <typename T>
__HOSTDEVICE__
bool intersectOrientedBoundingBox( BoundingBox<T> const& bbA, 
                                   BoundingBox<T> const& bbB,
                                   Transform3<T> const& trB2A )
{
    Vector3<T> const& a = bbA.getExtent();
    Vector3<T> const& b = bbB.getExtent();
    Vector3<T> const& cen = trB2A.getOrigin();
    Matrix3<T> const& ori = ( trB2A.getBasis() ).transpose();
    Matrix3<T> const 
                oriAbs( fabs(ori[0][0]), fabs(ori[0][1]), fabs(ori[0][2]),
                        fabs(ori[1][0]), fabs(ori[1][1]), fabs(ori[1][2]),
                        fabs(ori[2][0]), fabs(ori[2][1]), fabs(ori[2][2]) );

    // CASE 1: ( three of them )
    if TESTCASE1_D( 0 ) return ( false );
    if TESTCASE1_D( 1 ) return ( false );
    if TESTCASE1_D( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2_D( 0 ) return ( false );
    if TESTCASE2_D( 1 ) return ( false );
    if TESTCASE2_D( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3_D( 0, 0 ) return ( false );
    if TESTCASE3_D( 1, 0 ) return ( false );
    if TESTCASE3_D( 2, 0 ) return ( false );
    if TESTCASE3_D( 0, 1 ) return ( false );
    if TESTCASE3_D( 1, 1 ) return ( false );
    if TESTCASE3_D( 2, 1 ) return ( false );
    if TESTCASE3_D( 0, 2 ) return ( false );
    if TESTCASE3_D( 1, 2 ) return ( false );
    if TESTCASE3_D( 2, 2 ) return ( false );
    
    return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using OBB test - relative
// transformation - specialized for float
template <>
__HOSTDEVICE__
bool intersectOrientedBoundingBox<float>( BoundingBox<float> const& bbA, 
                                          BoundingBox<float> const& bbB,
                                          Transform3<float> const& trB2A )
{
    Vector3<float> const& a = bbA.getExtent();
    Vector3<float> const& b = bbB.getExtent();
    Vector3<float> const& cen = trB2A.getOrigin();
    Matrix3<float> const& ori = ( trB2A.getBasis() ).transpose();
    Matrix3<float> const 
                oriAbs( fabsf(ori[0][0]), fabsf(ori[0][1]), fabsf(ori[0][2]),
                        fabsf(ori[1][0]), fabsf(ori[1][1]), fabsf(ori[1][2]),
                        fabsf(ori[2][0]), fabsf(ori[2][1]), fabsf(ori[2][2]) );

    // CASE 1: ( three of them )
    if TESTCASE1_F( 0 ) return ( false );
    if TESTCASE1_F( 1 ) return ( false );
    if TESTCASE1_F( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2_F( 0 ) return ( false );
    if TESTCASE2_F( 1 ) return ( false );
    if TESTCASE2_F( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3_F( 0, 0 ) return ( false );
    if TESTCASE3_F( 1, 0 ) return ( false );
    if TESTCASE3_F( 2, 0 ) return ( false );
    if TESTCASE3_F( 0, 1 ) return ( false );
    if TESTCASE3_F( 1, 1 ) return ( false );
    if TESTCASE3_F( 2, 1 ) return ( false );
    if TESTCASE3_F( 0, 2 ) return ( false );
    if TESTCASE3_F( 1, 2 ) return ( false );
    if TESTCASE3_F( 2, 2 ) return ( false );
    
    return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using AABB test
template <typename T>
__HOSTDEVICE__
bool intersectAxisAlignedBoundingBox( BoundingBox<T> const& bbA, 
                                      BoundingBox<T> const& bbB,
                                      Transform3<T> const& trA2W,
                                      Transform3<T> const& trB2W )
{
    // TODO: lenA and lenB should be modified according to trA2W and trB2W
    // TODO: should we do len = bbA.getExtent() + bbB.getExtent()?
    Vector3<T> const lenA = bbA.getExtent();
    Vector3<T> const lenB = bbB.getExtent();
    Vector3<T> const posA = trA2W.getOrigin();
    Vector3<T> const posB = trB2W.getOrigin();
    if      ( fabs( posA[X] - posB[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabs( posA[Y] - posB[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabs( posA[Z] - posB[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using AABB test - relative
// transformation
template <typename T>
__HOSTDEVICE__
bool intersectAxisAlignedBoundingBox( BoundingBox<T> const& bbA, 
                                      BoundingBox<T> const& bbB,
                                      Transform3<T> const& trB2A )
{
    // TODO: lenA and lenB should be modified according to trA2W and trB2W
    // TODO: should we do len = bbA.getExtent() + bbB.getExtent()?
    Vector3<T> const lenA = bbA.getExtent();
    Vector3<T> const lenB = bbB.getExtent();
    Vector3<T> const pos = trB2A.getOrigin();
    if      ( fabs( pos[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabs( pos[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabs( pos[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}




// -----------------------------------------------------------------------------
// Undefining the low-level methods
#undef TESTCASE1_D
#undef TESTCASE2_D
#undef TESTCASE3_D
#undef TESTCASE1_F
#undef TESTCASE2_F
#undef TESTCASE3_F




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T ) \
template                                                                       \
__HOSTDEVICE__                                                                 \
bool intersectOrientedBoundingBox( BoundingBox<T> const& bbA,                  \
                                   BoundingBox<T> const& bbB,                  \
                                   Transform3<T> const& trA2W,                 \
                                   Transform3<T> const& trB2W );
X( float )
X( double )
#undef X