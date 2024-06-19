#ifndef _AABB_HH_
#define _AABB_HH_


#include "Vector3.hh"
#include "Convex.hh"

class Convex;

// =============================================================================
/** @brief The class AABB.

    Axis Aligned Bounding Boxes (AABB) for convex bodies in single precision.
    It is now deprecated as we use BoundingBox class for both AABB and OBB.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class AABB
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
        AABB();

        /** @brief Constructor with 3 components as inputs
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __host__ __device__
        AABB( float x, 
              float y,
              float z );

        /** @brief Constructor with a vector containing the edge half-lengths
        @param extent_ vector of half-lengths */
        __host__ __device__
        AABB( Vec3f const& extent_ );

        /** @brief Destructor */
        __host__ __device__
        ~AABB();
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
        to fit the AABB around.
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


/** @name AABB : External methods */
//@{
/** @brief Returns whether the AABBs are in contact
 @param a first AABB
 @param b second AABB
 @param posA position of first AABB
 @param posB position of second AABB */
__host__ __device__
bool intersectAABB( AABB const& a, 
                    AABB const& b,
                    Vec3f const& posA,
                    Vec3f const& posB );
                                        
/** @brief Returns whether the AABBs are in contact - relative position
 @param a first AABB
 @param b second AABB
 @param posB2A position of the second AABB wrt the first AABB */
__host__ __device__
bool intersectAABB( AABB const& a, 
                    AABB const& b,
                    Vec3f const& posB2A );
//@}


#endif