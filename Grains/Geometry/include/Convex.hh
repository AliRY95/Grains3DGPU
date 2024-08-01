#ifndef _CONVEX_HH_
#define _CONVEX_HH_


#include "Transform3.hh"


// Convex types
enum ConvexType {
    SPHERE,
    BOX,
    CYLINDER,
    CONE,
    SUPERQUADRIC,
    POLYHEDRON,
    RECTANGLE
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
        /** @brief Returns a clone of the convex */
        __HOSTDEVICE__
        virtual Convex<T>* clone() const = 0;

        /** @brief Returns the volume of the convex shape */
        __HOSTDEVICE__
        virtual T computeVolume() const = 0;

        /** @brief Computes the inertia tensor and the inverse of the inertia
        tensor
        @param inertia inertia tensor
        @param inertia_1 inverse of the inertia tensor */
        __HOSTDEVICE__
        virtual void computeInertia( T (&inertia)[6], 
                                     T (&inertia_1)[6] ) const = 0;

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


        /** @name I/O methods */
        //@{
        /** @brief Input operator
        @param fileIn input stream */
        __HOST__
        virtual void readConvex( std::istream& fileIn ) = 0;

        /** @brief Output operator
        @param fileOut output stream */
        __HOST__
        virtual void writeConvex( std::ostream& fileOut ) const = 0;

        /** @brief Returns the number of points to write the convex in a 
        Paraview format */
        __HOST__
        virtual int numberOfPoints_PARAVIEW() const = 0;

        /** @brief Returns the number of elementary polytopes to write the 
        convex in a Paraview format */
        __HOST__
        virtual int numberOfCells_PARAVIEW() const = 0;

        /** @brief Returns a list of points describing the convex in a Paraview
        format
        @param transform geometric transformation
        @param translation additional center of mass translation */
        __HOST__
        virtual std::list<Vector3<T>> writePoints_PARAVIEW( 
                                                Transform3<T> const& transform,
                                                Vector3<T> const* translation )
                                                const = 0;

        /** @brief Writes the connectivity of the convex in a Paraview format
        @param connectivity connectivity of Paraview polytopes
        @param offsets connectivity offsets
        @param cellstype Paraview polytopes type
        @param firstpoint_globalnumber global number of the 1st point
        @param last_offset last offset used for the previous convex shape */
        __HOST__
        virtual void writeConnection_PARAVIEW( std::list<int>& connectivity,
                                               std::list<int>& offsets, 
                                               std::list<int>& cellstype, 
                                               int& firstpoint_globalnumber,
                                               int& last_offset ) const = 0;
        //@}


        // /** @name Operators */
        // //@{
        // /** @brief Input operator
        // @param fileIn input stream
        // @param convex Convex object*/
        // std::istream& operator >> ( std::istream& fileIn,
        //                             Convex<T>& convex );

        // /** @brief Output operator
        // @param fileOut output stream
        // @param convex Convex object */
        // std::ostream& operator << ( std::ostream& fileOut, 
        //                             Convex<T> const& convex );
        // //@}        
};


typedef Convex<float> ConvexF;
typedef Convex<double> ConvexD;


#endif