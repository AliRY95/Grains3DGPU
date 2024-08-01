#ifndef _BOX_HH_
#define _BOX_HH_


#include "ReaderXML.hh"
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

        /** @brief Constructor with an input stream
        @param fileIn input stream */
        __HOST__
        Box( std::istream& fileIn );

        /** @brief Constructor with an XML node as an input parameter
        @param root XML node */
        __HOST__
        Box( DOMNode* root );

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
        /** @brief Returns a clone of the box */
        __HOSTDEVICE__
        Convex<T>* clone() const final;

        /** @brief Returns the box volume */
        __HOSTDEVICE__
        T computeVolume() const final;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        void computeInertia( T (&inertia)[6], 
                             T (&inertia_1)[6] ) const final;

        /** @brief Returns the circumscribed radius of the box */
        __HOSTDEVICE__
        T computeCircumscribedRadius() const final;

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

        /** @brief Returns the number of points to write the box in a Paraview 
        format */
        __HOST__
        int numberOfPoints_PARAVIEW() const final;

        /** @brief Returns the number of elementary polytopes to write the box
        in a Paraview format */
        __HOST__
        int numberOfCells_PARAVIEW() const final;

        /** @brief Returns a list of points describing the box in a Paraview
        format
        @param transform geometric transformation
        @param translation additional center of mass translation */
        __HOST__
        std::list<Vector3<T>> writePoints_PARAVIEW( 
                                                Transform3<T> const& transform,
                                                Vector3<T> const* translation )
                                                const final;

        /** @brief Writes the connectivity of the box in a Paraview format
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


typedef Box<float> BoxF;
typedef Box<double> BoxD;


#endif
