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
    T m_comp[3]; /**< Array of 3 components */
    //@}

public:
    /**@name Constructors */
    //@{
    /** @brief Default constructor
        @param def value of all 3 components */
    __HOSTDEVICE__
    Vector3(T def = T());

    /** @brief Constructor with the buffer
        @param buffer buffer */
    __HOSTDEVICE__
    Vector3(const T* buffer);

    /** @brief Constructor with 3 components as inputs
        @param x 1st component
        @param y 2nd component
        @param z 3rd component*/
    __HOSTDEVICE__
    Vector3(T x, T y, T z);

    /** @brief Copy constructor
        @param vec copied Vector3 object */
    __HOSTDEVICE__
    Vector3(const Vector3<T>& vec);

    /** @brief Destructor */
    __HOSTDEVICE__
    ~Vector3();
    //@}

    /** @name Get methods */
    //@{
    /** @brief Gets the pointer to the buffer */
    __HOSTDEVICE__
    const T* getBuffer() const;
    //@}

    /** @name Set methods */
    //@{
    /** @brief Sets the vector to a 1D array of 3 values as input
        @param buffer the 1D array of values ordered as: 0=Vx, 1=Vy, 2=Vz */
    __HOSTDEVICE__
    void setValue(const T* buffer);

    /** @brief Sets the components
        @param x the x component
        @param y the y component
        @param z the z component */
    __HOSTDEVICE__
    void setValue(T x, T y, T z);
    //@}

    /** @name Methods */
    //@{
    /** @brief Unitary nomalization operator */
    __HOSTDEVICE__
    void normalize();

    /** @brief Returns a vector corresponding to the normalized vector */
    __HOSTDEVICE__
    Vector3<T> normalized() const;

    /** @brief Returns the norm of the vector */
    __HOSTDEVICE__
    T norm() const;

    /** @brief Returns the norm squared of the vector */
    __HOSTDEVICE__
    T norm2() const;

    /** @brief Returns whether the vector norm is less than a given tol
        @param tol tolerance -- HIGHEPS defined in Basic.hh is the default */
    __HOSTDEVICE__
    bool isApproxZero(T tol = HIGHEPS<T>) const;

    /** @brief Rounds components to +-tol
        @param tol tolerance -- EPS defined in Basic.hh is the default */
    __HOSTDEVICE__
    void round(T tol = EPS<T>);

    /** @brief set all components to zero */
    __HOSTDEVICE__
    void reset();
    //@}

    /** @name Operators */
    //@{
    /** @brief Operator +=
        @param vec 2nd Vector3 object */
    __HOSTDEVICE__
    Vector3<T>& operator+=(const Vector3<T>& vec);

    /** @brief Operator -=
        @param vec 2nd Vector3 object */
    __HOSTDEVICE__
    Vector3<T>& operator-=(const Vector3<T>& vec);

    /** @brief Unitary operator *= by a scalar
        @param d multiplication factor */
    __HOSTDEVICE__
    Vector3<T>& operator*=(T d);

    /** @brief Unitary operator /= by a scalar
        @param d division factor */
    __HOSTDEVICE__
    Vector3<T>& operator/=(T d);

    /** @brief ith component accessor
        @param i component index */
    __HOSTDEVICE__
    T const& operator[](size_t i) const;

    /** @brief ith component accessor - modifiable lvalue
        @param i component index */
    __HOSTDEVICE__
    T& operator[](size_t i);

    /** @brief Assign operator to another Vector3 object
        @param vec rhs Vector3 object */
    __HOSTDEVICE__
    Vector3<T>& operator=(const Vector3<T>& vec);

    /** @brief Unitary operator -. Returns an object with negative 
        components */
    __HOSTDEVICE__
    Vector3<T> operator-() const;

    /** @brief Comparaison operator
        @param vec 2nd Vector3 object */
    __HOSTDEVICE__
    bool operator==(const Vector3<T>& vec) const;

    /** @brief Difference operator
        @param vec 2nd Vector3 object */
    __HOSTDEVICE__
    bool operator!=(const Vector3<T>& vec) const;

    /** @brief Conversion operator float */
    __HOSTDEVICE__
    operator Vector3<float>() const;
    //@}
};

/** @name External Methods - I/O methods */
//@{
/** @brief Input operator
@param fileIn input stream
@param v vector */
template <typename T>
__HOST__ std::istream& operator>>(std::istream& fileIn, Vector3<T>& v);

/** @brief Output operator
@param fileOut output stream
@param v vector */
template <typename T>
__HOST__ std::ostream& operator<<(std::ostream& fileOut, const Vector3<T>& v);
//@}

typedef Vector3<float>  Vec3F;
typedef Vector3<double> Vec3D;

#endif
