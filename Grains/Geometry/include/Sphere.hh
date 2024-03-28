#ifndef _SPHERE_HH_
#define _SPHERE_HH_


#include "Convex.hh"


// =============================================================================
/** @brief The class Sphere.

    Convex with the shape of a sphere.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Sphere : public Convex<T>
{
    protected:
        /** @name Parameters */
        //@{
        T m_radius; /**< radius of the sphere */
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with radius
        @param r radius */
        __HOSTDEVICE__
        Sphere( T r );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Sphere();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the convex type */
        __HOSTDEVICE__
        ConvexType getConvexType() const final;

        /** @brief Gets the radius */
        __HOSTDEVICE__
        T getRadius() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the radius
        @param r radius */
        __HOSTDEVICE__
        void setRadius( T r );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the circumscribed radius of the sphere */
        __HOSTDEVICE__
        T computeCircumscribedRadius() const final;

        /** @brief Returns the sphere volume */
        __HOSTDEVICE__
        T computeVolume() const final;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        bool computeInertia( T* inertia, 
                             T* inertia_1 ) const final;

        /** @ Returns the half-length of the bounding box fitted to the sphere 
        without considering the transformation */
        __HOSTDEVICE__
        Vector3<T> computeBoundingBox() const final;

        /** @brief Sphere support function, returns the support point P, i.e. 
        the point on the surface of the box that satisfies max(P.v)
        @param v direction */
        __HOSTDEVICE__
        Vector3<T> support( Vector3<T> const& v ) const final;
        //@}
};


typedef Sphere<float> SphereF;
typedef Sphere<double> SphereD;


#endif
