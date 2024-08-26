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
        __HOSTDEVICE__
        Matrix3();

        /** @brief Constructor with a 1D array of values as input
        @param mat the 1D array of values containing the matrix components ordered as
        0=Mxx, 1=Mxy, 2=Mxz, 3=Myx, 4=Myy, 5=Myz, 6=Mzx, 7=Mzy, 8=Mzz */
        __HOSTDEVICE__
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
        __HOSTDEVICE__ 
        Matrix3( T xx, T xy, T xz,
                 T yx, T yy, T yz,
                 T zx, T zy, T zz );

        /** @brief Copy constructor
        @param mat the copied matrix */
        __HOSTDEVICE__
        Matrix3( Matrix3<T> const& mat );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Matrix3();
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the matrix to a 1D array of 9 values as input
        @param mat the 1D array of values ordered as : 0=Mxx, 1=Mxy, 2=Mxz, 3=Myx,
        4=Myy, 5=Myz, 6=Mzx, 7=Mzy, 8=Mzz */
        __HOSTDEVICE__ 
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
        __HOSTDEVICE__
        void setValue( T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns a matrix with positive components */
        __HOSTDEVICE__
        Matrix3<T> absolute() const;

        /** @brief Returns the determinant of the matrix */
        __HOSTDEVICE__
        T determinant() const;

        /** @brief Returns the inverse of the matrix */
        __HOSTDEVICE__
        Matrix3<T> inverse() const;

        /** @brief Returns the transposed matrix */
        __HOSTDEVICE__
        Matrix3<T> transpose() const;
        //@}


        /**@name Operators */
        //@{
        /** @brief Operator +=
        @param mat 2nd Matrix3 object */
        __HOSTDEVICE__
        Matrix3<T>& operator += ( Matrix3<T> const& mat );

        /** @brief Operator -=
        @param mat 2nd Matrix3 object */
        __HOSTDEVICE__
        Matrix3<T>& operator -= ( Matrix3<T> const& mat );
        
        /** @brief Unitary operator *= by a scalar
        @param d multiplication factor */
        __HOSTDEVICE__
        Matrix3<T>& operator *= ( T d );

        /** @brief Operator *= by a matrix
        @param mat 2nd Matrix3 object */
        __HOSTDEVICE__
        Matrix3<T>& operator *= ( Matrix3<T> const& mat );

        /** @brief i-th row accessor
        @param i row number */
        __HOSTDEVICE__
        Vector3<T>& operator [] ( unsigned int i ) const;

        /** @brief Assign operator to another matrix
        @param mat rhs Matrix3 object */
        __HOSTDEVICE__
        Matrix3<T>& operator = ( Matrix3<T> const& mat );

        /** @brief Unitary operator -. Returns an object with negative 
        components */
        __HOSTDEVICE__
        Matrix3<T>& operator - ();
};


/** @name External Methods - I/O methods */
//@{
/** @brief Input operator
@param fileIn input stream
@param v vector */
template <typename T>
__HOST__
std::istream& operator >> ( std::istream& fileIn, 
                            Matrix3<T>& m );

/** @brief Output operator
@param fileOut output stream
@param v vector */
template <typename T>
__HOST__
std::ostream& operator << ( std::ostream& fileOut, 
                            Matrix3<T> const& m );
//@}


typedef Matrix3<float> Mat3F;
typedef Matrix3<double> Mat3D;


#endif
