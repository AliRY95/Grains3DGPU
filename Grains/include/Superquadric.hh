#ifndef _SUPERQUADRIC_HH_
#define _SUPERQUADRIC_HH_


#include "Convex.hh"


// =============================================================================
/** @brief The class Superquadric.

    Convex with the shape of a superquadric in double precision.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class Superquadric : public Convex
{
    protected:
        /** @name Parameters */
        //@{
        double m_a; /**< scale parameter along the x-axis */
        double m_b; /**< scale parameter along the y-axis */
        double m_c; /**< scale parameter along the z-axis */
        double m_n1; /**< first exponent */
        double m_n2; /**< second exponent */
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
        __host__ __device__ Superquadric( double a = 0., 
                                          double b = 0., 
                                          double c = 0.,
                                          double n1 = 2., 
                                          double n2 = 2. );

        /** @brief Destructor */
        __host__ __device__ ~Superquadric();
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the convex type */
        __host__ __device__ ConvexType getConvexType() const;

        /** @brief Computes and returns the circumscribed radius of the 
        Superquadric */
        __host__ __device__ double computeCircumscribedRadius() const;

        /** @brief Returns the Superquadric volume */
        __host__ __device__ double computeVolume() const;

        /** @brief Computes the inertia tensor and the inverse of the inertia 
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __host__ __device__ bool computeInertia( double* inertia, 
                                                 double* inertia_1 ) const;

        /** @ Returns the half-length of the AABB fitted to the Superquadric */
        __host__ __device__ Vec3f computeAABB() const;

        /** @brief Superquadric support function, returns the support point P,
        i.e. point on the surface of the Superquadric that satisfies max(P.v)
        @param v direction */
        __host__ __device__ Vec3d support( Vec3d const& v ) const;
        //@}
};

#endif
