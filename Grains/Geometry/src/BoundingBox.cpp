#include "Transform3.hh"
#include "BoundingBox.hh"


// -----------------------------------------------------------------------------
// Constructor
__host__ __device__
BoundingBox::BoundingBox()
{}




// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
__host__ __device__
BoundingBox::BoundingBox( float x, float y, float z )
: m_extent( Vec3f( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
__host__ __device__
BoundingBox::BoundingBox( Vec3f const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__
BoundingBox::~BoundingBox()
{}




// -----------------------------------------------------------------------------
// Sets values of the edge length
__host__ __device__
void BoundingBox::setExtent( float x, float y, float z )
{
    m_extent = Vec3f( x, y, z );
}




// -----------------------------------------------------------------------------
// Sets extent values using ccording to the convex which we want to fit the 
// bounding box around.
__host__ __device__
void BoundingBox::setExtent( Convex const& convex )
{
    m_extent = convex.computeBoundingBox();
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
__host__ __device__
Vec3f const BoundingBox::getExtent() const
{
    return ( m_extent );
}






/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether the bounding boxes are in contact using OBB test
#define TESTCASE1( i ) \
( fabs( cen[i] ) > \
( a[i] + b[0]*fabs(ori[0][i]) + b[1]*fabs(ori[1][i]) + b[2]*fabs(ori[2][i]) ) )

#define TESTCASE2( i ) \
( fabs( cen[0]*ori[i][0] + cen[1]*ori[i][1] + cen[2]*ori[i][2] ) > \
( b[i] + a[0]*fabs(ori[i][0]) + a[1]*fabs(ori[i][1]) + a[2]*fabs(ori[i][2]) ) )

#define TESTCASE3(i, j) \
( fabs( cen[(j+2)%3]*ori[i][(j+1)%3] - cen[(j+1)%3]*ori[i][(j+2)%3] ) > \
     ( a[(i+1)%3]*oriAbs[(i+2)%3][j] + a[(i+2)%3]*oriAbs[(i+1)%3][j] + \
       b[(j+1)%3]*oriAbs[i][(j+2)%3] + b[(j+2)%3]*oriAbs[i][(j+1)%3] ) )

__host__ __device__
bool intersectOrientedBoundingBox( BoundingBox const& bbA, 
                                   BoundingBox const& bbB,
                                   Transform3d const& trA2W,
                                   Transform3d const& trB2W )
{
    Vec3f const& a = bbA.getExtent();
    Vec3f const& b = bbB.getExtent();
    Vec3d const& cen = ( trA2W.getBasis() ).transpose() * 
                       ( trB2W.getOrigin() - trA2W.getOrigin() );
    Mat3d const& ori = ( trB2W.getBasis() ).transpose() * trA2W.getBasis();
    Mat3d const oriAbs( fabs(ori[0][0]), fabs(ori[0][1]), fabs(ori[0][2]),
                        fabs(ori[1][0]), fabs(ori[1][1]), fabs(ori[1][2]),
                        fabs(ori[2][0]), fabs(ori[2][1]), fabs(ori[2][2]) );

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
__host__ __device__
bool intersectOrientedBoundingBox( BoundingBox const& bbA, 
                                   BoundingBox const& bbB,
                                   Transform3d const& trB2A )
{
    Vec3f const& a = bbA.getExtent();
    Vec3f const& b = bbB.getExtent();
    Vec3d const& cen = trB2A.getOrigin();
    Mat3d const& ori = ( trB2A.getBasis() ).transpose();
    Mat3d const oriAbs( fabs(ori[0][0]), fabs(ori[0][1]), fabs(ori[0][2]),
                        fabs(ori[1][0]), fabs(ori[1][1]), fabs(ori[1][2]),
                        fabs(ori[2][0]), fabs(ori[2][1]), fabs(ori[2][2]) );

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

#undef TESTCASE1
#undef TESTCASE2
#undef TESTCASE3




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using AABB test
__host__ __device__
bool intersectAxisAlignedBoundingBox( BoundingBox const& bbA, 
                                      BoundingBox const& bbB,
                                      Transform3d const& trA2W,
                                      Transform3d const& trB2W )
{
    // TODO: lenA and lenB should be modified according to trA2W and trB2W
    // TODO: should we do len = bbA.getExtent() + bbB.getExtent()?
    Vec3f const lenA = bbA.getExtent();
    Vec3f const lenB = bbB.getExtent();
    Vec3f const posA = trA2W.getOrigin();
    Vec3f const posB = trB2W.getOrigin();
    if      ( fabsf( posA[X] - posB[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabsf( posA[Y] - posB[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabsf( posA[Z] - posB[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the bounding boxes are in contact using AABB test - relative
// transformation
__host__ __device__
bool intersectAxisAlignedBoundingBox( BoundingBox const& bbA, 
                                      BoundingBox const& bbB,
                                      Transform3d const& trB2A )
{
    // TODO: lenA and lenB should be modified according to trA2W and trB2W
    // TODO: should we do len = bbA.getExtent() + bbB.getExtent()?
    Vec3f const lenA = bbA.getExtent();
    Vec3f const lenB = bbB.getExtent();
    Vec3f const pos = trB2A.getOrigin();
    if      ( fabsf( pos[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabsf( pos[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabsf( pos[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}