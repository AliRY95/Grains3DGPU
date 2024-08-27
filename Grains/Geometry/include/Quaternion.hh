#ifndef _QUATERNION_HH_
#define _QUATERNION_HH_

#include "Vector3.hh"
#include "Matrix3.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class Quaternion.

    A quaternion in a 3D space, i.e., a scalar w plus a Vector3 vector vqt as
    [ w, vqt ].

    @author F.PRADEL - Institut Francais du Petrole - 2000 - Modification
    @author A.WACHS  - 2019 - Modification
    @author A.Yazdani - 2024 - Major modification */
// =============================================================================
template <typename T>
class Quaternion
{
	protected:
		/**@name Parameters */
		//@{
		Vector3<T> m_vqt; /**< Vectorial part of the quaternion */
		T m_w; /**< scalar part of the quaternion */
		//@}


	public:
		/**@name Constructors */
		//@{
		/** @brief Default constructor */
		__HOSTDEVICE__
		Quaternion();

		/** @brief Constructor with 2 scalar as input parameters q and d.
		Quaternion is initialized as [ d, (q,q,q) ]
		@param q value of all 3 components of the vector
		@param d value of the scalar */
		__HOSTDEVICE__
		Quaternion( T q,
					T d = T( 0 ) );

		/** @brief Constructor with a Vector3 vec and a scalar d. Quaternion is
		initialized as [ d, vec ]
		@param vec the Vector3 vector
		@param d value of the scalar */
		__HOSTDEVICE__
		Quaternion( Vector3<T> const& vec, 
					T d = T( 0 ) );

		/** @brief Constructor with a vector given by its 3 components (x,y,z) 
		and a scalar d. Quaternion is initialized as [ d, (x,y,z) ]
		@param x x-component of the vector
		@param y y-component of the vector
		@param z z-component of the vector
		@param d value of the scalar */
		__HOSTDEVICE__
		Quaternion( T x, 
					T y, 
					T z, 
					T d );

		/** @brief Constructor with a rotation matrix 		
		@param rot rotation matrix */
		__HOSTDEVICE__
		Quaternion( Matrix3<T> const& rot );

		/** @brief Copy constructor
		@param q copied Quaternion object */
		__HOSTDEVICE__
		Quaternion( Quaternion<T> const& q );

		/** @brief Destructor */
		__HOSTDEVICE__
		~Quaternion();
		//@}


		/**@name Get methods */
        //@{
		/** @brief Returns the vectorial part of the quaternion */
		__HOSTDEVICE__
		Vector3<T> getVector() const;

		/** @brief Returns the value of the scalar part of the quaternion */
		__HOSTDEVICE__
		T getScalar() const;
		//@}


		/**@name Set methods */
		//@{
		/** @brief Sets the vectorial part of the quaternion
		@param vec the Vector3 vector */
		__HOSTDEVICE__
		void setVector( Vector3<T> const& vec );

		/** @brief Sets the scalar part of the quaternion
		@param d value of the scalar */
		__HOSTDEVICE__
		void setScalar( T d );

		/** @brief Sets the quaternion with a Vector3 vector vec and a scalar d.
		Quaternion is set to [ d, vec ]
		@param vec the Vector3 vector
		@param d value of the scalar */
		__HOSTDEVICE__
		void setQuaternion( Vector3<T> const& vec, 
							T d );

		/** @brief Sets the quaternion with a vector given by its 3 components
		(x,y,z) and a scalar d. Quaternion is set to [ d, (x,y,z) ]
		@param x x-component of the vector
		@param y y-component of the vector
		@param z z-component of the vector
		@param d value of the scalar */
		__HOSTDEVICE__
		void setQuaternion( T x, 
							T y, 
							T z, 
							T d );

		/** @brief Sets the quaternion with a rotation matrix
		@param rot rotation matrix */
		__HOSTDEVICE__
		void setQuaternion( Matrix3<T> const& rot );

		/** @brief Builds a unit quaternion representing the rotation, from
		u to v. The input vectors need not to be normalised.
		@param u First vector
		@param v Second vector */
		__HOSTDEVICE__
		void setRotFromTwoVectors( Vector3<T> const& u, 
								   Vector3<T> const& v );
		//@}


		/**@name Methods */
		//@{
		/** @brief Returns the norm of the quaternion
		@param q the quaternion */
		__HOSTDEVICE__
		T norm() const;

		/** @brief Returns the norm square of the quaternion
		@param q the quaternion */
		__HOSTDEVICE__
		T norm2() const;

		/** @brief Returns the conjugate of the quaternion */
		__HOSTDEVICE__
		Quaternion<T> conjugate() const;

		/** @brief Returns the inverse of the quaternion */
		__HOSTDEVICE__
		Quaternion<T> inverse() const;

		/** @brief Multiplies the quaternion on the left by a vector lhs, i.e.,
		performs [ 0, lhs ] x this and return the product that is a quaternion
		@param lhs the left hand side vector */
		__HOSTDEVICE__
		Quaternion<T> multLeftVec( Vector3<T> const& lhs ) const;

		/** @brief Multiplies the quaternion on the right by another quaternion 
		rhs, i.e., performs this x rhs, and return the vectorial part of 
		this x rhs
		@param q the other quaternion */
		__HOSTDEVICE__
		Vector3<T> multToVector3( Quaternion<T> const& q ) const;

		/** @brief Multiplies the quaternion on the right by the conjugate of
		another quaternion rhs, i.e., perform this x rhs^t, and return the
		vectorial part of this x rhs^t
		@param q the other quaternion */
		__HOSTDEVICE__
		Vector3<T> multConjugateToVector3( Quaternion<T> const& q ) const;
		
		/** @brief Rotates a vector using the quaternion *this
		@param v The vector to be rotated */
		__HOSTDEVICE__
		Vector3<T> rotateVector( Vector3<T> const& v ) const;    
		//@}
		

		/**@name Operators */
		//@{
		/** @brief Operator +=
		@param q the other quaternion */
		__HOSTDEVICE__
		Quaternion<T>& operator += ( Quaternion<T> const& q );

		/** @brief Operator -=
		@param q the other quaternion */
		__HOSTDEVICE__
		Quaternion<T>& operator -= ( Quaternion<T> const& q );

		/** @brief Unitary operator *= by a scalar
		@param d multiplication factor */
		__HOSTDEVICE__
		Quaternion<T>& operator *= ( T d );

		/** @brief ith component accessor
        @param i component index */
        __HOSTDEVICE__
        T operator [] ( size_t i ) const;
		
		/** @brief ith-component accessor: (0,1,2) for the vector components and
		3 for the scalar - modifiable lvalue
		@param i index */
		__HOSTDEVICE__
		T& operator [] ( size_t i );

		/** @brief Assign operator to another Quaternion object
		@param q the other Quaternion object */
		__HOSTDEVICE__
		Quaternion<T>& operator = ( Quaternion<T> const& q );
		
		/** @brief Unitary operator -. Return a quaternion with negative 
		elements */
		__HOSTDEVICE__
		Quaternion<T> operator - ();

		/** @brief Comparison operator
		@param q the other quaternion */
		__HOSTDEVICE__
		bool operator == ( Quaternion<T> const& q );

		/** @brief Difference operator
		@param q the other quaternion */
		__HOSTDEVICE__
		bool operator != ( Quaternion<T> const& q );
		//@}
};


/** @name External Methods - I/O methods */
//@{
/** @brief Input operator
@param fileIn input stream
@param q quaternion */
template <typename T>
std::istream& operator >> ( std::istream& fileIn, 
                            Quaternion<T>& q );

/** @brief Output operator
@param fileOut output stream
@param q quaternion */
template <typename T>
std::ostream& operator << ( std::ostream& fileOut, 
                            Quaternion<T> const& q );
//@}


typedef Quaternion<float> QuaternionF;
typedef Quaternion<double> QuaternionD;

#endif
