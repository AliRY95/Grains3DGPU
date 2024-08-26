#ifndef _GJK_JH_HH_
#define _GJK_JH_HH_


#include "Transform3.hh"
#include "Convex.hh"


// =============================================================================
/** @brief The header for the original GJK distance query algorithm.

    The original GJK distance query algorithm using the Johnson subalgorithm 
    with the backup procedure. It supports both single and double floating point
    operations, but it is not recommended to use the single precision version as
    it is prone to numerical instabilities.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name GJK : External methods */
//@{
/** @brief Returns whether 2 convex shapes intersect
@param a convex shape A
@param b convex shape B
@param a2w geometric tramsformation describing convex A in the world reference
frame
@param b2w geometric tramsformation describing convex B in the world reference
frame */
template <typename T>
__HOSTDEVICE__
bool intersectGJK( Convex<T> const& a, 
                   Convex<T> const& b,
                   Transform3<T> const& a2w,
                   Transform3<T> const& b2w );

/** @brief Returns whether 2 convex shapes intersect - relative transformation
@param a convex shape A
@param b convex shape B
@param b2a geometric tramsformation describing convex B in the A's reference
frame */
template <typename T>
__HOSTDEVICE__
bool intersectGJK( Convex<T> const& a, 
                   Convex<T> const& b,
                   Transform3<T> const& b2a );

/** @brief Returns the minimal distance between 2 convex shapes and a point per
convex shape that represents the tips of the minimal distance segment
@param a convex shape A
@param b convex shape B
@param a2w geometric tramsformation describing convex A in the world reference
frame
@param b2w geometric tramsformation describing convex B in the world reference
frame
@param pa point representing one tip of the minimal distance segment on A
@param pb point representing the other tip of the minimal distance segment on
B
@param nbIter number of iterations of GJK for convergence */
template <typename T>
__HOSTDEVICE__
T computeClosestPoints_GJK_JH( Convex<T> const& a, 
                               Convex<T> const& b, 
                               Transform3<T> const& a2w,
                               Transform3<T> const& b2w, 
                               Vector3<T>& pa,
                               Vector3<T>& pb,
                               int& nbIter );
//@}


#endif