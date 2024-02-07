#ifndef _VECTOR3_HH_
#define _VECTOR3_HH_

#include "Basic.hh"


// =============================================================================
/** @brief The class Vector3.

    Vector/Point in a 3D space.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
template <typename T>
class Vector3
{
  protected:
    /**@name Parameters */
    //@{
    T m_comp[3]; /**< array of 3 components */
    //@}


  public:
    /**@name Constructors */
    //@{
    /** @brief Default constructor
    @param def value of all 3 components */
    __host__ __device__ Vector3( T def = T() );

    /** @brief Constructor with 3 components as inputs
    @param x 1st component
    @param y 2nd component
    @param z 3rd component*/
    __host__ __device__ Vector3( T x, T y, T z );

    /** @brief Copy constructor
    @param vec copied Vector3 object */
    __host__ __device__ Vector3( Vector3<T> const& vec );

    /** @brief Destructor */
    __host__ __device__ ~Vector3();
    //@}


    /** @name Methods */
    //@{
    /** @brief Sets the components */
    __host__ __device__ void setValue( T x, T y, T z );

    /** @brief Unitary nomalization operator */
    __host__ __device__ void normalize();

    /** @brief Returns a vector corresponding to the normalized vector */
    __host__ __device__ Vector3<T> normalized() const;

    /** @brief Returns the norm of the vector */
    __host__ __device__ T norm() const;

    /** @brief Returns the norm square of the vector */
    __host__ __device__ T norm2() const;

    /** @brief Returns whether the vector norm is less than EPSILON2
    where EPSILON2 is defined in Basic.hh */
    __host__ __device__ bool isApproxZero() const;
    //@}


    /** @name Operators */
    //@{
    /** @brief Addition
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T> operator + ( Vector3<T> const& vec ) const;

    /** @brief Subtraction
    @param vec 2nd object */
    __host__ __device__ Vector3<T> operator - ( Vector3<T> const& vec ) const;
    
    /** @brief Multiplication by a scalar
    @param d multiplication factor */
    __host__ __device__ Vector3<T> operator * ( T d ) const;
    
    /** @brief Division by a scalar
    @param d division factor */
    __host__ __device__ Vector3<T> operator / ( T d ) const;

    /** @brief Operator +=
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T>& operator += ( Vector3<T> const& vec );

    /** @brief Operator -=
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T>& operator -= ( Vector3<T> const& vec );
    
    /** @brief Unitary operator *= by a scalar
    @param d multiplication factor */
    __host__ __device__ Vector3<T>& operator *= ( T d );

    /** @brief Unitary operator /= by a scalar
    @param d division factor */
    __host__ __device__ Vector3<T>& operator /= ( T d );

    /** @brief dot product
    @param vec 2nd Vector3 object */
    __host__ __device__ T operator * ( Vector3<T> const& vec ) const;
    
    /** @brief Cross product this x rhv
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T> operator ^ ( Vector3<T> const& vec ) const;

    /** @brief ith component accessor
    @param i component index */
    __host__ __device__ T operator [] ( size_t i ) const;

    /** @brief Assign operator to another Vector3 object
    @param vec rhs Vector3 object */
    __host__ __device__ Vector3<T>& operator = ( Vector3<T> const& vec );    

    /** @brief Unitary operator -. Returns an object with negative components */
    __host__ __device__ Vector3<T> operator - () const;

    /** @brief Comparaison operator
    @param vec 2nd Vector3 object */
    __host__ __device__ bool operator == ( Vector3<T> const& vec ) const;

    /** @brief Difference operator
    @param vec 2nd Vector3 object */
    __host__ __device__ bool operator != ( Vector3<T> const& vec ) const;
    //@}
};

// static Vector3 Vector3Nul; /**< Vector3 nul (0.,0.,0.)  */
typedef Vector3<float> Vec3f;
typedef Vector3<double> Vec3d;

#endif
