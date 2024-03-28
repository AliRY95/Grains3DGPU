#ifndef _SUPERQUADRIC_HH_
#define _SUPERQUADRIC_HH_


#include "Convex.hh"


// =============================================================================
/** @brief The class Superquadric.

    Convex with the shape of a superquadric.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Superquadric : public Convex<T>
{
    protected:
        /** @name Parameters */
        //@{
        T m_a; /**< scale parameter along the x-axis */
        T m_b; /**< scale parameter along the y-axis */
        T m_c; /**< scale parameter along the z-axis */
        T m_n1; /**< first exponent */
        T m_n2; /**< second exponent */
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with inputs
        @param a scale parameter along the x-axis
        @param b scale parameter along the y-axis
        @param c scale parameter along the z-axis
        @param n1 first exponent
        @param n2 second exponent */
        __HOSTDEVICE__ 
        Superquadric( T a = 0., 
                      T b = 0., 
                      T c = 0.,
                      T n1 = 2., 
                      T n2 = 2. );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Superquadric();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the convex type */
        __HOSTDEVICE__
        ConvexType getConvexType() const final;
        //@}


        /** @name Methods */
        //@{
        /** @brief Computes and returns the circumscribed radius of the 
        Superquadric */
        __HOSTDEVICE__
        T computeCircumscribedRadius() const final;

        /** @brief Returns the Superquadric volume */
        __HOSTDEVICE__
        T computeVolume() const final;

        /** @brief Computes the inertia tensor and the inverse of the inertia 
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        bool computeInertia( T* inertia, 
                             T* inertia_1 ) const final;

        /** @ Returns the half-length of the bounding box fitted to the
        superquadric without considering the transformation */
        __HOSTDEVICE__
        Vector3<T> computeBoundingBox() const final;

        /** @brief Superquadric support function, returns the support point P,
        i.e. point on the surface of the Superquadric that satisfies max(P.v)
        @param v direction */
        __HOSTDEVICE__
        Vector3<T> support( Vector3<T> const& v ) const final;
        //@}
};


typedef Superquadric<float> SuperquadricF;
typedef Superquadric<double> SuperquadricD;


#endif
