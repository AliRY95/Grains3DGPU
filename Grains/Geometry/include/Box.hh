#ifndef _BOX_HH_
#define _BOX_HH_


#include "Convex.hh"


// =============================================================================
/** @brief The class Box.

    Convex with the shape of a box.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Box : public Convex<T>
{
    protected:
        /** @name Parameters */
        //@{
        Vector3<T> m_extent; /**< vector containing the half-legnth of edges */
        //@}


    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with 3 components as inputs
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __HOSTDEVICE__
        Box( T x,
             T y,
             T z );

        /** @brief Constructor with a vector containing the edge half-lengths
        @param extent_ vector of half-lengths */
        __HOSTDEVICE__
        Box( Vector3<T> const& extent_ );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Box();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Returns the convex type */
        __HOSTDEVICE__
        ConvexType getConvexType() const final;

        /** @brief Gets values of the edge length
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __HOSTDEVICE__
        Vector3<T> getExtent() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets values of the edge length
        @param x 1st component
        @param y 2nd component
        @param z 3rd component */
        __HOSTDEVICE__
        void setExtent( T x,
                        T y,
                        T z );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the circumscribed radius of the box */
        __HOSTDEVICE__
        T computeCircumscribedRadius() const final;

        /** @brief Returns the box volume */
        __HOSTDEVICE__
        T computeVolume() const final;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        bool computeInertia( T* inertia, 
                             T* inertia_1 ) const final;

        /** @ Returns the half-length of the bounding box fitted to the box 
        without considering the transformation */
        __HOSTDEVICE__
        Vector3<T> computeBoundingBox() const final;

        /** @brief Box support function, returns the support point P, i.e. the
        point on the surface of the box that satisfies max(P.v)
        @param v direction */
        __HOSTDEVICE__
        Vector3<T> support( Vector3<T> const& v ) const final;
        //@}
};


typedef Box<float> BoxF;
typedef Box<double> BoxD;


#endif
