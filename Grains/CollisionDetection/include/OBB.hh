#ifndef _OBB_HH_
#define _OBB_HH_


#include "Transform3.hh"
#include "BoundingBox.hh"


// =============================================================================
/** @brief The header for the axis-aligned and oriented bounding boxes collision
    detection.

    Axis-aligned Bounding Boxes (AABB) and Oriented Bounding Boxes (OBB)
    routines to find whether bounding boxes are in contact or not.
    AABB is deprecated, so try to use OBB.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name OBB : External methods */
//@{
/** @brief Returns whether the bounding boxes are in contact using OBB test
@param bbA first bounding box
@param bbB second bounding box
@param trA2W transformation of first bounding box
@param trB2W transformation of second bounding box */
template <typename T>
__HOSTDEVICE__
bool intersectOrientedBoundingBox( BoundingBox<T> const& bbA, 
                                   BoundingBox<T> const& bbB,
                                   Transform3<T> const& trA2W,
                                   Transform3<T> const& trB2W );
                                        
/** @brief Returns whether the bounding boxes are in contact using OBB test - 
relative transformation
@param bbA first bounding box
@param bbB second bounding box
@param trB2A transformation of the second bounding box wrt the first bounding
box */
template <typename T>
__HOSTDEVICE__
bool intersectOrientedBoundingBox( BoundingBox<T> const& bbA, 
                                   BoundingBox<T> const& bbB,
                                   Transform3<T> const& trB2A );

/** @brief Returns whether the bounding boxes are in contact using AABB test
@param bbA first bounding box
@param bbB second bounding box
@param trA2W transformation of first bounding box
@param trB2W transformation of second bounding box */
template <typename T>
__HOSTDEVICE__
bool intersectAxisAlignedBoundingBox( BoundingBox<T> const& bbA, 
                                      BoundingBox<T> const& bbB,
                                      Transform3<T> const& trA2W,
                                      Transform3<T> const& trB2W );
                                        
/** @brief Returns whether the bounding boxes are in contact using AABB test - 
relative transformation
@param bbA first bounding box
@param bbB second bounding box
@param trB2A transformation of the second bounding box wrt the first bounding
box */
template <typename T>
__HOSTDEVICE__
bool intersectAxisAlignedBoundingBox( BoundingBox<T> const& bbA, 
                                      BoundingBox<T> const& bbB,
                                      Transform3<T> const& trB2A );
//@}


#endif