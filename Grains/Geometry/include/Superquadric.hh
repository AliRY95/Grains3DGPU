#ifndef _SUPERQUADRIC_HH_
#define _SUPERQUADRIC_HH_


#include "ReaderXML.hh"
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

        /** @brief Constructor with an input stream
        @param fileIn input stream */
        __HOST__
        Superquadric( std::istream& fileIn );

        /** @brief Constructor with an XML node as an input parameter
        @param root XML node */
        __HOST__
        Superquadric( DOMNode* root );

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

        /** @brief Returns the number of points to write the superquadric in a 
        Paraview format */
        __HOST__
        int numberOfPoints_PARAVIEW() const final;

        /** @brief Returns the number of elementary polytopes to write the 
        superquadric in a Paraview format */
        __HOST__
        int numberOfCells_PARAVIEW() const final;

        /** @brief Returns a list of points describing the superquadric in a 
        Paraview format
        @param transform geometric transformation
        @param translation additional center of mass translation */
        __HOST__
        std::list<Vector3<T>> writePoints_PARAVIEW( 
                                                Transform3<T> const& transform,
                                                Vector3<T> const* translation )
                                                const final;

        /** @brief Writes the connectivity of the superquadric in a Paraview 
        format
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


typedef Superquadric<float> SuperquadricF;
typedef Superquadric<double> SuperquadricD;


#endif
