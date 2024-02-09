#ifndef _BOX_HH_
#define _BOX_HH_

#include "Convex.hh"
#include "Vector3.hh"


// =============================================================================
/** @brief The class Box.

    Convex with the shape of a box in double precision.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class Box : public Convex
{
  protected:
    /** @name Parameters */
    //@{
    Vec3d m_extent; /**< vector containing the half-legnth of the edges */
    //@}


  public:
    /** @name Constructors */
    //@{
    /** @brief Constructor with 3 components as inputs
    @param x 1st component
    @param y 2nd component
    @param z 3rd component */
    __host__ __device__ Box( double x, double y, double z );

    /** @brief Constructor with a vector containing the edge half-lengths
    @param extent_ vector of half-lengths */
    __host__ __device__ Box( Vec3d const& extent_ );

    /** @brief Destructor */
    __host__ __device__ ~Box();
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

    /** @brief Returns the convex type */
    __host__ __device__ ConvexType getConvexType() const;

    /** @brief Computes and returns the circumscribed radius of the box */
    __host__ __device__ double computeCircumscribedRadius() const;

    /** @brief Returns the box volume */
    __host__ __device__ double computeVolume() const;

    /** @brief Computes the inertia tensor and the inverse of the inertia tensor
    @param inertia inertia tensor
    @param inertia_1 inverse of the inertia tensor */
    __host__ __device__ bool buildInertia( double* inertia, 
                                           double* inertia_1 ) const;

    /** @ Returns the half-length of the AABB fitted to the box */
    __host__ __device__ Vec3f computeAABB() const;

    /** @brief Box support function, returns the support point P, i.e. the
    point on the surface of the box that satisfies max(P.v)
    @param v direction */
    __host__ __device__ Vec3d support( Vec3d const& v ) const;
    //@}
};

#endif
