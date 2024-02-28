#ifndef _TRANSFORM3_HH_
#define _TRANSFORM3_HH_


#include "Matrix3.hh"


// =============================================================================
/** @brief The class Transform3.

    A position/origin described by a Vector3 and an orientation described by a 
    Matrix3. Note that the Matrix3 portion can also contain a scaling component
    and is hence not necessarily unitary. 

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Transform3
{
    private:
        /**@name Parameters */
        //@{
        Matrix3<T> m_basis;  /**< Orientation of the transformation */
        Vector3<T> m_origin; /**< Center of the transformation */
        //@}


    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor. Origin is def and matrix is identity
        @param def value of all 3 components for origin; x = y = z = def */
        __host__ __device__
        Transform3( T def = T() );

        /** @brief Constructor with origin coordinates as inputs and matrix is 
        identity
        @param x origin x-coordinate 
        @param y origin y-coordinate
        @param z origin z-coordinate */
        __host__ __device__
        Transform3( T x,
                    T y,
                    T z );

        /** @brief Constructor with a 1D array of 12 values as inputs containing
        the rotation matrix coefficients following by the origin coordinates
        @param m 1D array contanining 12 values */
        __host__ __device__
        Transform3( T const t[12] );

        /** @brief Copy constructor
        @param t the transformation to be copied */
        __host__ __device__
        Transform3( Transform3<T> const& t );

        /** @brief Destructor */
        __host__ __device__ ~Transform3();
        //@}


        /**@name Get methods */
        //@{
        /** @brief Gets the orientation of the transformation */
        __host__ __device__
        Matrix3<T> getBasis() const;

        /** @brief Gets the origin of the transformation */
        __host__ __device__
        Vector3<T> getOrigin() const;
        //@}


        /**@name Set methods */
        //@{
        /** @brief Sets the transformation with an 1D array of 12 values as 
        inputs. The 1D array must be organized as: 0=Mxx, 1=Mxy, 2=Mxz, 3=Myx,
        4=Myy, 5=Myz, 6=Mzx, 7=Mzy, 8=Mzz, 9=Ox, 10=Oy, 11=Oz 
        @param t the 1D array of values containing the tranformation
        coefficients */
        __host__ __device__
        void setValue( T const t[12] );

        /** @brief Sets the matrix part of the transformation
        @param m matrix part of the transformation */
        __host__ __device__
        void setBasis( Matrix3<T> const& m );

        /** @brief Sets the matrix part of the transformation with specified
        rotations around each principal axis
        @param aX rotation around the x-axis
        @param aY rotation around the y-axis
        @param aZ rotation around the z-axis */
        __host__ __device__
        void setBasis( T aX,
                       T aY,
                       T aZ );

        /** @brief Sets the origin of the transformation
        @param v origin of the transformation */
        __host__ __device__
        void setOrigin( Vector3<T> const& v );

        /** @brief Sets the transformation to the identity */
        __host__ __device__
        void setIdentity();

        /** @brief Sets the transformation to the inverse of another 
        transformation
        @param t the other transformation */ 
        __host__ __device__
        void setToInverseTransform( Transform3<T> const& t );

        /** @brief Sets the transformation composition of affine transformations
        this = t2 o t1 (t1 first followed by t2)
        @param t1 1st affine transformation
        @param t2 2nd affine transformation */
        __host__ __device__
        void setToTransformsComposition( Transform3<T> const& t1, 
                                         Transform3<T> const& t2 );
        //@}


        /**@name Methods */
        //@{
        /** @brief Composition with a scaling transformation: 
        this = this o scaling
        @param x scaling factor in x
        @param y scaling factor in y
        @param z scaling factor in z */
        __host__ __device__
        void composeWithScaling( T x,
                                 T y,
                                 T z );  

        /** @brief Composition on the left by a rotation described by a 
        transform:
        this = rot o this (this first followed by rot).
        This composition leaves the origin unchanged but does not check that rot
        is indeed a rotation
        @param t the other transformation describing a rotation */
        __host__ __device__
        void composeLeftByRotation( Transform3<T> const& t );  

        /** @brief Composition on the left by a translation:
        this = trans(vector) o this (this first followed by trans(vector))
        @param v translation vector */
        __host__ __device__
        void composeLeftByTranslation( Vector3<T> const& v );

        /** @brief Composition on the left by another affine transformation: 
        this = t o this (this first followed by t)
        @param t the other affine transformation */
        __host__ __device__
        void composeLeftByTransform( Transform3<T> const& t );

        /** @brief Composition on the right by another affine transformation: 
        this = this o t (t first followed by this)
        @param t the other affine transformation */
        __host__ __device__
        void composeRightByTransform( Transform3<T> const& t );

        /** @brief Composition in a way that it is now the relative
        transformation with respect to t
        @param t the other affine transformation */
        __host__ __device__
        void relativeToTransform( Transform3<T> const& t );
        //@}


        /**@name Operators */
        //@{
        /** @brief Returns the result of applying the transformation to the input 
        vector
        @param v input vector */
        __host__ __device__ Vector3<T> operator () ( Vector3<T> const& v ) const;

        /** @brief Equal operator to another Transform3 object
        @param t the other Transform object */
        __host__ __device__ Transform3<T>& operator = ( Transform3<T> const& t );
        //@}  
};


typedef Transform3<float> Transform3f;
typedef Transform3<double> Transform3d;


#endif
