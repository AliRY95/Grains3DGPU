#ifndef _COLLISIONDETECTION_HH_
#define _COLLISIONDETECTION_HH_


#include "RigidBody.hh"
#include "Transform3.hh"
#include "ContactInfo.hh"


// =============================================================================
/** @brief The header for Rigid bodies collision detections.

    Functions for collision detection between two rigid bodies.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name CollisionDetection : External methods */
//@{
/** @brief Returns whether 2 rigid bodies intersect
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric transformation describing convex A in the world reference
 frame
 @param b2w geometric transformation describing convex B in the world reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__ 
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
                           Transform3<T> const& a2w,
                           Transform3<T> const& b2w );
                           
/** @brief Returns whether 2 rigid bodies intersect - relative transformation
 @param rbA first rigid body
 @param rbB second rigid body
 @param b2a geometric tramsformation describing convex B in the A's reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
                           Transform3<T> const& b2a );

/** @brief Returns the contact information (if any) for 2 rigid bodies
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, U> const& rbA,
                                         RigidBody<T, U> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w );
                                         
// TODO: LATER
// /** @brief Returns the contact information (if any) for 2 rigid bodies - 
// relative transformation
//  @param rbA first rigid body
//  @param rbB second rigid body
//  @param b2a geometric tramsformation describing convex B in the A's reference
//  frame */
// __HOSTDEVICE__
// ContactInfo closestPointsRigidBodies( RigidBody const& rbA,
//                                       RigidBody const& rbB,
//                                       Transform3d const& b2a );
//@}


#endif
