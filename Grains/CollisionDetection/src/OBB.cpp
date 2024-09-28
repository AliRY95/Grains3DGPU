#include "MatrixMath.hh"
#include "BoundingBox.hh"
#include "Transform3.hh"
#include "OBB.hh"


/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Low-level methods for OBB as macros in double precision
#define TESTCASE1( i )                                                         \
( fabs( cen[i] ) >                                                             \
( a[i] +                                                                       \
  b[0] * oriAbs[i][0] +                                                        \
  b[1] * oriAbs[i][1] +                                                        \
  b[2] * oriAbs[i][2] ) )

#define TESTCASE2( i )                                                         \
( fabs( cen[0] * ori[0][i] + cen[1] * ori[1][i] + cen[2] * ori[2][i] ) >       \
( b[i] +                                                                       \
  a[0] * oriAbs[0][i] +                                                        \
  a[1] * oriAbs[1][i] +                                                        \
  a[2] * oriAbs[2][i] ) )

#define TESTCASE3(i, j)                                                        \
( fabs( cen[(i+2)%3] * ori[(i+1)%3][j] - cen[(i+1)%3] * ori[(i+2)%3][j] ) >    \
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
    Vector3<T> const a = bbA.getExtent();
    Vector3<T> const b = bbB.getExtent();
    // First, we compute the transpose of trA2W basis and store it in ori
    Matrix3<T> ori = transpose( trA2W.getBasis() );
    // Then, the center is
    Vector3<T> const cen =  ori * ( trB2W.getOrigin() - trA2W.getOrigin() );
    // Finally, we compute the actual relative rotation matrix
    ori *= trB2W.getBasis();
    // And, we compute the absolute value of the matrix + some noise to 
    // encounter arithmetic errors.
    Matrix3<T> const oriAbs( fabs( ori[0][0] ) + LOWEPS<T>, 
                             fabs( ori[0][1] ) + LOWEPS<T>, 
                             fabs( ori[0][2] ) + LOWEPS<T>,
                             fabs( ori[1][0] ) + LOWEPS<T>, 
                             fabs( ori[1][1] ) + LOWEPS<T>, 
                             fabs( ori[1][2] ) + LOWEPS<T>, 
                             fabs( ori[2][0] ) + LOWEPS<T>, 
                             fabs( ori[2][1] ) + LOWEPS<T>, 
                             fabs( ori[2][2] ) + LOWEPS<T>);

    // CASE 1: ( three of them )
    if TESTCASE1( 0 ) return ( false );
    if TESTCASE1( 1 ) return ( false );
    if TESTCASE1( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2( 0 ) return ( false );
    if TESTCASE2( 1 ) return ( false );
    if TESTCASE2( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3( 0, 0 ) return ( false );
    if TESTCASE3( 1, 0 ) return ( false );
    if TESTCASE3( 2, 0 ) return ( false );
    if TESTCASE3( 0, 1 ) return ( false );
    if TESTCASE3( 1, 1 ) return ( false );
    if TESTCASE3( 2, 1 ) return ( false );
    if TESTCASE3( 0, 2 ) return ( false );
    if TESTCASE3( 1, 2 ) return ( false );
    if TESTCASE3( 2, 2 ) return ( false );
    
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
    Vector3<T> const a = bbA.getExtent();
    Vector3<T> const b = bbB.getExtent();
    Vector3<T> const cen = trB2A.getOrigin();
    Matrix3<T> const ori = trB2A.getBasis();
    Matrix3<T> const oriAbs( fabs( ori[0][0] ) + LOWEPS<T>, 
                             fabs( ori[0][1] ) + LOWEPS<T>, 
                             fabs( ori[0][2] ) + LOWEPS<T>,
                             fabs( ori[1][0] ) + LOWEPS<T>, 
                             fabs( ori[1][1] ) + LOWEPS<T>, 
                             fabs( ori[1][2] ) + LOWEPS<T>, 
                             fabs( ori[2][0] ) + LOWEPS<T>, 
                             fabs( ori[2][1] ) + LOWEPS<T>, 
                             fabs( ori[2][2] ) + LOWEPS<T>);

    // CASE 1: ( three of them )
    if TESTCASE1( 0 ) return ( false );
    if TESTCASE1( 1 ) return ( false );
    if TESTCASE1( 2 ) return ( false );

    // CASE 2: ( three of them )
    if TESTCASE2( 0 ) return ( false );
    if TESTCASE2( 1 ) return ( false );
    if TESTCASE2( 2 ) return ( false );

    // CASE 3: ( nine of them )
    if TESTCASE3( 0, 0 ) return ( false );
    if TESTCASE3( 1, 0 ) return ( false );
    if TESTCASE3( 2, 0 ) return ( false );
    if TESTCASE3( 0, 1 ) return ( false );
    if TESTCASE3( 1, 1 ) return ( false );
    if TESTCASE3( 2, 1 ) return ( false );
    if TESTCASE3( 0, 2 ) return ( false );
    if TESTCASE3( 1, 2 ) return ( false );
    if TESTCASE3( 2, 2 ) return ( false );
    
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
#undef TESTCASE1
#undef TESTCASE2
#undef TESTCASE3




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