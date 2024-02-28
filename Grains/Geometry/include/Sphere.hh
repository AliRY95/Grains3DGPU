#ifndef _SPHERE_HH_
#define _SPHERE_HH_


#include "Vector3.hh"
#include "Convex.hh"


// =============================================================================
/** @brief The class Sphere.

    Convex with the shape of a sphere in double precision.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class Sphere : public Convex
{
    protected:
        /** @name Parameters */
        //@{
        double m_radius; /**< radius of the sphere */
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with radius
        @param r radius */
        __host__ __device__
        Sphere( double r );

        /** @brief Destructor */
        __host__ __device__
        ~Sphere();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the convex type */
        __host__ __device__
        ConvexType getConvexType() const;

        /** @brief Gets the radius */
        __host__ __device__
        double getRadius() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the radius
        @param r radius */
        __host__ __device__
        void setRadius( double r );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the circumscribed radius of the sphere */
        __host__ __device__
        double computeCircumscribedRadius() const;

        /** @brief Returns the sphere volume */
        __host__ __device__
        double computeVolume() const;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __host__ __device__
        bool computeInertia( double* inertia, 
                             double* inertia_1 ) const;

        /** @ Returns the half-length of the AABB fitted to the box */
        __host__ __device__
        Vec3f computeAABB() const;

        /** @brief Sphere support function, returns the support point P, i.e. 
        the point on the surface of the box that satisfies max(P.v)
        @param v direction */
        __host__ __device__
        Vec3d support( Vec3d const& v ) const;
        //@}
};


#endif
