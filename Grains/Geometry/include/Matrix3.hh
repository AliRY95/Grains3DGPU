#ifndef _MATRIX3_HH_
#define _MATRIX3_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief The class Matrix3.

    3x3 real matrix.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Matrix3
{
    protected:
        /**@name Parameters */
        //@{
        T m_comp[3][3]; /**< 3x3 array containing the matrix components */
        //@}

    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor. Matrix is initialized to the identity
        matrix */
        __host__ __device__
        Matrix3();

        /** @brief Constructor with a 1D array of values as input
        @param mat the 1D array of values containing the matrix components ordered as
        0=Mxx, 1=Mxy, 2=Mxz, 3=Myx, 4=Myy, 5=Myz, 6=Mzx, 7=Mzy, 8=Mzz */
        __host__ __device__
        Matrix3( T const* mat );

        /** @brief Constructor with 9 components as inputs
        @param xx (1,1) coefficient
        @param xy (1,2) coefficient
        @param xz (1,3) coefficient
        @param yx (2,1) coefficient
        @param yy (2,2) coefficient
        @param yz (2,3) coefficient
        @param zx (3,1) coefficient
        @param zy (3,2) coefficient
        @param zz (3,3) coefficient */
        __host__ __device__ 
        Matrix3( T xx, T xy, T xz,
                 T yx, T yy, T yz,
                 T zx, T zy, T zz );

        /** @brief Copy constructor
        @param mat the copied matrix */
        __host__ __device__
        Matrix3( Matrix3<T> const& mat );

        /** @brief Destructor */
        __host__ __device__
        ~Matrix3();
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the matrix to a 1D array of 9 values as input
        @param mat the 1D array of values ordered as : 0=Mxx, 1=Mxy, 2=Mxz, 3=Myx,
        4=Myy, 5=Myz, 6=Mzx, 7=Mzy, 8=Mzz */
        __host__ __device__ 
        void setValue( T const* mat );

        /** @brief Sets the matrix with all 9 components as inputs
        @param xx (1,1) coefficient
        @param xy (1,2) coefficient
        @param xz (1,3) coefficient
        @param yx (2,1) coefficient
        @param yy (2,2) coefficient
        @param yz (2,3) coefficient
        @param zx (3,1) coefficient
        @param zy (3,2) coefficient
        @param zz (3,3) coefficient */
        __host__ __device__
        void setValue( T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns a matrix with positive components */
        __host__ __device__
        Matrix3<T> absolute() const;

        /** @brief Returns the determinant of the matrix */
        __host__ __device__
        T determinant() const;

        /** @brief Returns the inverse of the matrix */
        __host__ __device__
        Matrix3<T> inverse() const;

        /** @brief Returns the transposed matrix */
        __host__ __device__
        Matrix3<T> transpose() const;
        //@}


        /**@name Operators */
        //@{
        /** @brief Operator +=
        @param mat 2nd Matrix3 object */
        __host__ __device__
        Matrix3<T>& operator += ( Matrix3<T> const& mat );

        /** @brief Operator -=
        @param mat 2nd Matrix3 object */
        __host__ __device__
        Matrix3<T>& operator -= ( Matrix3<T> const& mat );
        
        /** @brief Unitary operator *= by a scalar
        @param d multiplication factor */
        __host__ __device__
        Matrix3<T>& operator *= ( T d );

        /** @brief Operator *= by a matrix
        @param mat 2nd Matrix3 object */
        __host__ __device__
        Matrix3<T>& operator *= ( Matrix3<T> const& mat );

        /** @brief i-th row accessor
        @param i row number */
        __host__ __device__
        Vector3<T>& operator [] ( unsigned int i ) const;

        /** @brief Assign operator to another matrix
        @param mat rhs Matrix3 object */
        __host__ __device__
        Matrix3<T>& operator = ( Matrix3<T> const& mat );

        // /** @brief Unitary operator -. Returns an object with negative components */
        // __host__ __device__ Matrix3<T> operator - () const;
};


/** @name Matrix3 : External methods */
//@{
/** @brief Matrces sum
@param m1 first matrix
@param m2 second matrix */
template <typename T>
__host__ __device__
Matrix3<T> operator + ( Matrix3<T> const& m1, 
                        Matrix3<T> const& m2 );

/** @brief Scalar-matrix product
@param c the scalar
@param m the matrix */
template <typename T>
__host__ __device__
Matrix3<T> operator * ( T c, Matrix3<T> const& m );

/** @brief Matrix-vector product
@param m the matrix
@param v the vector */
template <typename T>
__host__ __device__ 
Vector3<T> operator * ( Matrix3<T> const& m,
                        Vector3<T> const& v );

/** @brief Vector-matrix product
@param m the matrix
@param v the vector */
template <typename T>
__host__ __device__
Vector3<T> operator * ( Vector3<T> const& v, 
                        Matrix3<T> const& m );

/** @brief Matrix-matrix product
@param m1 left matrix
@param m2 right matrix */
template <typename T>
__host__ __device__
Matrix3<T> operator * ( Matrix3<T> const& m1, 
                        Matrix3<T> const& m2 );

// /** @brief Returns the matrix that rotates vector src to vector dest,
// i.e. dest = mat * src
// @param src the source vector
// @param dest the destination vector */
// Matrix getRotationMatrix( Vector3 const& src, Vector3 const& dest );
//@}


typedef Matrix3<float> Mat3f;
typedef Matrix3<double> Mat3d;


#endif
