#ifndef _BOUNDINGBOX_HH_
#define _BOUNDINGBOX_HH_


#include "Vector3.hh"
#include "Transform3.hh"
#include "Convex.hh"


class Convex;

// =============================================================================
/** @brief The class BoundingBox.

    Bounding Boxes (AABB/BoundingBox) collision detection for convex bodies in single 
    precision.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class BoundingBox
{
    protected:
        /** @name Parameters */
        //@{
        Vec3f m_extent; /**< vector containing the half-legnth of the edges */
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor */
        __host__ __device__
        BoundingBox();

        /** @brief Constructor with 3 components as inputs
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __host__ __device__
        BoundingBox( float x, 
                     float y,
                     float z );

        /** @brief Constructor with a vector containing the edge half-lengths
        @param extent_ vector of half-lengths */
        __host__ __device__
        BoundingBox( Vec3f const& extent_ );

        /** @brief Destructor */
        __host__ __device__
        ~BoundingBox();
        //@}


        /** @name Methods */
        //@{
        /** @brief Sets values of the edge length
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __host__ __device__
        void setExtent( float x, 
                        float y,
                        float z );

        /** @brief Sets the extent values according to the convex which we want
        to fit the BoundingBox around.
        @param convex convex object */
        __host__ __device__
        void setExtent( Convex const& convex );


        /** @brief Gets values of the edge length
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __host__ __device__
        Vec3f const getExtent() const;
        //@}
};


/** @name BoundingBox : External methods */
//@{
/** @brief Returns whether the bounding boxes are in contact using OBB test
 @param bbA first bounding box
 @param bbB second bounding box
 @param trA2W transformation of first bounding box
 @param trB2W transformation of second bounding box */
__host__ __device__
bool intersectOrientedBoundingBox( BoundingBox const& bbA, 
                                   BoundingBox const& bbB,
                                   Transform3d const& trA2W,
                                   Transform3d const& trB2W );
                                        
/** @brief Returns whether the bounding boxes are in contact using OBB test - 
 relative transformation
 @param bbA first bounding box
 @param bbB second bounding box
 @param trB2A transformation of the second bounding box wrt the first bounding
 box */
__host__ __device__
bool intersectOrientedBoundingBox( BoundingBox const& bbA, 
                                   BoundingBox const& bbB,
                                   Transform3d const& trB2A );

/** @brief Returns whether the bounding boxes are in contact using AABB test
 @param bbA first bounding box
 @param bbB second bounding box
 @param trA2W transformation of first bounding box
 @param trB2W transformation of second bounding box */
__host__ __device__
bool intersectAxisAlignedBoundingBox( BoundingBox const& bbA, 
                                      BoundingBox const& bbB,
                                      Transform3d const& trA2W,
                                      Transform3d const& trB2W );
                                        
/** @brief Returns whether the bounding boxes are in contact using AABB test - 
 relative transformation
 @param bbA first bounding box
 @param bbB second bounding box
 @param trB2A transformation of the second bounding box wrt the first bounding
 box */
__host__ __device__
bool intersectAxisAlignedBoundingBox( BoundingBox const& bbA, 
                                      BoundingBox const& bbB,
                                      Transform3d const& trB2A );
//@}


#endif