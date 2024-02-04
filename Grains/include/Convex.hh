#ifndef _CONVEX_HH_
#define _CONVEX_HH_

#include "Basic.hh"
#include "Vector3.hh"


// =============================================================================
/** @brief The class Convex.

    Convex bodies.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
class Convex
{
  protected:
    /**@name Parameters */
    //@{
    Vec3d m_extent; /**< vector containing the half-legnth of the edges */
    //@}

  public:
    /** @name Constructors */
    //@{
    /** @brief Default constructor */
    __host__ __device__ Convex();

    /** @brief Constructor with 3 components as inputs
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ Convex( double x, double y, double z );

    /** @brief Constructor with a vector containing the edge half-lengths
    @param extent_ vector of half-lengths */
    __host__ __device__ Convex( Vec3d const& extent_ );

    /** @brief Destructor */
    __host__ __device__ ~Convex();
    //@}


    /** @name Methods */
    //@{
    /** @brief Sets values of the edge length
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ void setExtent( double x, double y, double z );

    /** @brief Gets values of the edge length
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ Vec3d const getExtent() const;

    /** @brief Box support function, returns the support point P, i.e. the
    point on the surface of the box that satisfies max(P.v)
    @param v direction */
    __host__ __device__ Vec3d support( Vec3d const& v ) const;
    //@}
};


/** @name Convex : External methods for the GJK algorithm */
//@{
/** @brief Returns whether 2 convex shapes intersect
 @param a convex shape A
 @param b convex shape B
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
__host__ __device__ bool intersectGJK( Convex const* a, 
                                       Convex const* b,
                                       Vec3d const& a2w,
	                                     Vec3d const& b2w );

/** @brief Returns whether the bounding boxex are in contact or not
 @param a bounding box of A
 @param b bounding box of B
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
__host__ __device__ bool intersectAABB( Convex const* a, 
                                        Convex const* b,
                                        Vec3d const& a2w,
	                                      Vec3d const& b2w );


//@}

#endif