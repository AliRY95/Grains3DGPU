#ifndef _GJK_AY_HH_
#define _GJK_AY_HH_

#include "Transform3.hh"
#include "Convex.hh"


// =============================================================================
/** @brief The header for the GJK distance query algorithm with signed volume.

    The GJK distance query algorithm using the Signed Volume (SV) subalgorithm.
    It supports both single and double floating point operations.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name GJK_SV : Data structure for simplex */
//@{
template <typename T>
struct Simplex {
	int nvrtx;                /**< Number of points defining the simplex */
	Vector3<T> vrtx[4];       /**< Coordinates of the points of the simplex */
};
//@}

/** @name GJK_SV : External methods */
//@{
/** @brief Returns the minimal distance between 2 convex shapes and a point per
convex shape that represents the tips of the minimal distance segment using the
signed volume distance subalgorithm
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
T computeClosestPoints_GJK_AY( Convex<T> const& a, 
                               Convex<T> const& b, 
                               Transform3<T> const& a2w,
                               Transform3<T> const& b2w, 
                               Vector3<T>& pa,
                               Vector3<T>& pb,
                               int& nbIter );
//@}


#endif
