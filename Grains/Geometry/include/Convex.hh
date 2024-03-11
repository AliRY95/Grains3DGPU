#ifndef _CONVEX_HH_
#define _CONVEX_HH_


#include "Transform3.hh"
#include "BoundingBox.hh"


// Convex types
enum ConvexType {
    SPHERE,
    BOX,
    CYLINDER,
    CONE,
    SUPERQUADRIC,
    POLYHEDRON,
    RECTANGLE2D
};


// =============================================================================
/** @brief The class Convex.

    Convex bodies in double precision.

    @author A.Yazdani - 2023 - Construction 
    @author A.Yazdani - 2024 - Modificiation */
// =============================================================================
class Convex
{
    protected:
        /**@name Contructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        __host__ __device__ 
        Convex();
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Destructor */
        __host__ __device__
        virtual ~Convex();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Returns the convex type */
        __host__ __device__
        virtual ConvexType getConvexType() const = 0;
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the volume of the convex shape */
        __host__ __device__
        virtual double computeVolume() const = 0;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __host__ __device__
        virtual bool computeInertia( double* inertia, 
                                     double* inertia_1 ) const = 0;

        /** @brief Computes and returns the circumscribed radius of the 
        reference convex shape */
        __host__ __device__
        virtual double computeCircumscribedRadius() const = 0;

        /** @brief Returns the half-length of the bounding box fitted to the 
        convex without considering the transformation */
        __host__ __device__
        virtual Vec3f computeBoundingBox() const = 0;

        /** @brief Convex support function, returns the support point P, i.e. 
        the point on the surface of the convex shape that satisfies max(P.v)
        @param v direction vector */
        __host__ __device__
        virtual Vec3d support( Vec3d const& v ) const = 0;
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
__host__ __device__
bool intersectGJK( Convex const& a, 
                   Convex const& b,
                   Transform3d const& a2w,
                   Transform3d const& b2w );

/** @brief Returns whether 2 convex shapes intersect - relative transformation
@param a convex shape A
@param b convex shape B
@param b2a geometric tramsformation describing convex B in the A's reference
 frame */
__host__ __device__
bool intersectGJK( Convex const& a, 
                   Convex const& b,
                   Transform3d const& b2a );

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
double closestPointsGJK( Convex const& a, 
                         Convex const& b, 
                         Transform3d const& a2w,
	                     Transform3d const& b2w, 
                         Vec3d& pa,
                         Vec3d& pb,
                         int& nbIter );
//@}

#endif