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

    Convex bodies - The base class for various particle shapes.

    @author A.Yazdani - 2023 - Construction 
    @author A.Yazdani - 2024 - Modificiation */
// =============================================================================
template <typename T>
class Convex
{
    protected:
        /**@name Contructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        __HOSTDEVICE__ 
        Convex();
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Destructor */
        __HOSTDEVICE__
        virtual ~Convex();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Returns the convex type */
        __HOSTDEVICE__
        virtual ConvexType getConvexType() const = 0;
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the volume of the convex shape */
        __HOSTDEVICE__
        virtual T computeVolume() const = 0;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        virtual bool computeInertia( T* inertia, 
                                     T* inertia_1 ) const = 0;

        /** @brief Computes and returns the circumscribed radius of the 
        reference convex shape */
        __HOSTDEVICE__
        virtual T computeCircumscribedRadius() const = 0;

        /** @brief Returns the half-length of the bounding box fitted to the 
        convex without considering the transformation */
        __HOSTDEVICE__
        virtual Vector3<T> computeBoundingBox() const = 0;

        /** @brief Convex support function, returns the support point P, i.e. 
        the point on the surface of the convex shape that satisfies max(P.v)
        @param v direction vector */
        __HOSTDEVICE__
        virtual Vector3<T> support( Vector3<T> const& v ) const = 0;
        //@}
};


typedef Convex<float> ConvexF;
typedef Convex<double> ConvexD;


#endif