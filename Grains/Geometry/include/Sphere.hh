#ifndef _SPHERE_HH_
#define _SPHERE_HH_


#include "ReaderXML.hh"
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

        /** @brief Constructor with an input stream
        @param fileIn input stream */
        __HOST__
        Sphere( std::istream& fileIn );

        /** @brief Constructor with an XML node as an input parameter
        @param root XML node */
        __HOST__
        Sphere( DOMNode* root );

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
        /** @brief Returns a clone of the sphere */
        __HOSTDEVICE__
        Convex<T>* clone() const final;

        /** @brief Returns the sphere volume */
        __HOSTDEVICE__
        T computeVolume() const final;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        void computeInertia( T (&inertia)[6], 
                             T (&inertia_1)[6] ) const final;

        /** @brief Returns the circumscribed radius of the sphere */
        __HOSTDEVICE__
        T computeCircumscribedRadius() const final;

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


        /** @name I/O methods */
        //@{
        /** @brief Input operator
        @param fileIn input stream */
        __HOST__
        void readConvex( std::istream& fileIn ) final;

        /** @brief Output operator
        @param fileOut output stream */
        __HOST__
        void writeConvex( std::ostream& fileOut ) const final;

        /** @brief Returns the number of points to write the sphere in a 
        Paraview format */
        __HOST__
        int numberOfPoints_PARAVIEW() const final;

        /** @brief Returns the number of elementary polytopes to write the 
        sphere in a Paraview format */
        __HOST__
        int numberOfCells_PARAVIEW() const final;

        /** @brief Returns a list of points describing the sphere in a Paraview
        format
        @param transform geometric transformation
        @param translation additional center of mass translation */
        __HOST__
        std::list<Vector3<T>> writePoints_PARAVIEW( 
                                                Transform3<T> const& transform,
                                                Vector3<T> const* translation )
                                                const final;

        /** @brief Writes the connectivity of the sphere in a Paraview format
        @param connectivity connectivity of Paraview polytopes
        @param offsets connectivity offsets
        @param cellstype Paraview polytopes type
        @param firstpoint_globalnumber global number of the 1st point
        @param last_offset last offset used for the previous convex shape */
        __HOST__
        void writeConnection_PARAVIEW( std::list<int>& connectivity,
                                       std::list<int>& offsets, 
                                       std::list<int>& cellstype, 
                                       int& firstpoint_globalnumber,
                                       int& last_offset ) const final;
        //@}
};


typedef Sphere<float> SphereF;
typedef Sphere<double> SphereD;


#endif
