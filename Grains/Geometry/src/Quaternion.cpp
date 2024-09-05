#include "Quaternion.hh"
#include "VectorMath.hh"
#include "QuaternionMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion()
: m_w( T( 1 ) )
, m_vqt( T( 0 ) )
{}




// -----------------------------------------------------------------------------
// Constructor with 2 scalar as input parameters q and d. Quaternion is
// initialized as [ d, (q,q,q) ]
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion( T q, 
						   T d )
: m_w( d )
, m_vqt( q )
{}




// -----------------------------------------------------------------------------
// Constructor with a Vector3 vector vec and a scalar d. Quaternion is 
// initialized as [ d, vec ]
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion( Vector3<T> const& vec, 
						   T d )
: m_w( d )
, m_vqt( vec )
{}




// -----------------------------------------------------------------------------
// Constructor with a vector given by its 3 components (x,y,z) and a scalar d.
// Quaternion is initialized as [ d, (x,y,z) ]
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion( T x, 
						   T y, 
						   T z, 
						   T d )
: m_w( d )
, m_vqt( Vector3<T>( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Constructor with a rotation matrix
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion( Matrix3<T> const& rot )
{
	this->setQuaternion( rot );
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
Quaternion<T>::Quaternion( Quaternion<T> const& q )
: m_w( q.m_w )
, m_vqt( q.m_vqt )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Quaternion<T>::~Quaternion()
{}




// -----------------------------------------------------------------------------
// Returns the vectorial part of the quaternion
template <typename T>
__HOSTDEVICE__
Vector3<T> Quaternion<T>::getVector() const
{
  return ( m_vqt );
}




// -----------------------------------------------------------------------------
// Returns the value of the scalar part of the quaternion
template <typename T>
__HOSTDEVICE__
T Quaternion<T>::getScalar() const
{
	return ( m_w );
}




// -----------------------------------------------------------------------------
// Sets the vectorial part of the quaternion
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setVector( Vector3<T> const& vec )
{
	m_vqt = vec;
}




// -----------------------------------------------------------------------------
// Sets the scalar part of the quaternion
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setScalar( T d )
{
	m_w = d;
}




// -----------------------------------------------------------------------------
// Sets the quaternion with a Vector3 vector vec and a scalar d.
// Quaternion is set to [ d, vec ]
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setQuaternion( Vector3<T> const& vec, 
								   T d )
{
	m_w = d;
	m_vqt = vec;
}




// -----------------------------------------------------------------------------
// Sets the quaternion with a vector given by its 3 components (x,y,z)
// and a scalar d. Quaternion is set to [ d, (x,y,z) ]
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setQuaternion( T x, 
								   T y, 
								   T z, 
								   T d )
{
	m_vqt[X] = x;
	m_vqt[Y] = y;
	m_vqt[Z] = z;
	m_w = d;
}




// -----------------------------------------------------------------------------
// Sets the quaternion with a rotation matrix
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setQuaternion( Matrix3<T> const& rot )
{
	T den = T( 0 );

    // Case rotYY > - rotZZ, rotXX > - rotYY and rotXX > - rotZZ
    if ( rot[Y][Y] > - rot[Z][Z] && 
		 rot[X][X] > - rot[Y][Y] && 
		 rot[X][X] > - rot[Z][Z] )
    {
		den = pow( T( 1 ) + rot[X][X] + rot[Y][Y] + rot[Z][Z], T( 0.5 ) );
		m_w = T( 0.5 ) * den;
		m_vqt[X] = T( 0.5 ) * ( rot[Z][Y] - rot[Y][Z] ) / den;
		m_vqt[Y] = T( 0.5 ) * ( rot[X][Z] - rot[Z][X] ) / den;
		m_vqt[Z] = T( 0.5 ) * ( rot[Y][X] - rot[X][Y] ) / den;
    }
    // Case rotYY < - rotZZ, rotXX > rotYY and rotXX > rotZZ
    else if ( rot[Y][Y] < - rot[Z][Z] && 
			  rot[X][X] > rot[Y][Y] && 
			  rot[X][X] > rot[Z][Z] )
    {
		den = pow( T( 1 ) + rot[X][X] - rot[Y][Y] - rot[Z][Z], T( 0.5 ) );
		m_w = T( 0.5 ) * ( rot[Z][Y] - rot[Y][Z] ) / den;
		m_vqt[X] = T( 0.5 ) * den;
		m_vqt[Y] = T( 0.5 ) * ( rot[X][Y] + rot[Y][X] ) / den;
		m_vqt[Z] = T( 0.5 ) * ( rot[Z][X] + rot[X][Z] ) / den;
    }
    // Case rotYY > rotZZ, rotXX < rotYY and rotXX < - rotZZ
    else if ( rot[Y][Y] > rot[Z][Z] && 
			  rot[X][X] < rot[Y][Y] && 
			  rot[X][X] < - rot[Z][Z] )
    {
		den = pow( T( 1 ) - rot[X][X] + rot[Y][Y] - rot[Z][Z], T( 0.5 ) );
		m_w = T( 0.5 ) * ( rot[X][Z] - rot[Z][X] ) / den;
		m_vqt[X] = T( 0.5 ) * ( rot[X][Y] + rot[Y][X] ) / den;
		m_vqt[Y] = T( 0.5 ) * den;
		m_vqt[Z] = T( 0.5 ) * ( rot[Y][Z] + rot[Z][Y] ) / den;
    }
    // Case rotYY < rotZZ, rotXX < - rotYY and rotXX < rotZZ
    else if ( rot[Y][Y] < rot[Z][Z] && 
			  rot[X][X] < - rot[Y][Y] && 
			  rot[X][X] < rot[Z][Z] )
    {
		den = pow( T( 1 ) - rot[X][X] - rot[Y][Y] + rot[Z][Z], T( 0.5 ) );
		m_w = T( 0.5 ) * ( rot[Y][X] - rot[X][Y] ) / den;
		m_vqt[X] = T( 0.5 ) * ( rot[Z][X] + rot[X][Z] ) / den;
		m_vqt[Y] = T( 0.5 ) * ( rot[Y][Z] + rot[Z][Y] ) / den;
		m_vqt[Z] = T( 0.5 ) * den;
    }
    else
		printf( "Warning: case not covered in Quaternion::setQuaternion( Matrix rot )!\n" );
}




// -----------------------------------------------------------------------------
// Build a unit quaternion representing the rotation from u to v. 
// The input vectors need not be normalised. */
// TODO: if the input vectors aren't normalized, normalize them and warn the
// user.
template <typename T>
__HOSTDEVICE__
void Quaternion<T>::setRotFromTwoVectors( Vector3<T> const& u, 
										  Vector3<T> const& v )
{
	T norm_u_norm_v = sqrt( ( u * u ) * ( v * v ) );
	T real_part = norm_u_norm_v + u * v;
	Vector3<T> vect;

	if ( real_part < 1.e-6 * norm_u_norm_v )
	{
		/* If u and v are exactly opposite, rotate 180 degrees
		around an arbitrary orthogonal axis. Axis normalisation
		can happen later, when we normalise the quaternion. */
		real_part = T( 0 );
		vect = fabs( u[0] ) > fabs( u[2] ) ? Vector3<T>( -u[1], u[0], T( 0 ) )
										   : Vector3<T>( T( 0 ), -u[2], u[1] );
	}
	else
	{
		/* Otherwise, build quaternion the standard way. */
		vect = u ^ v;
	}

	Quaternion<T> qq( vect[0], vect[1], vect[2], real_part );
	*this = ( T( 1 ) / qq.norm() ) * qq;
}




// -----------------------------------------------------------------------------
// Returns the norm of the quaternion
template <typename T>
__HOSTDEVICE__
T Quaternion<T>::norm() const
{
	return ( sqrt( m_vqt[X] * m_vqt[X] + 
				   m_vqt[Y] * m_vqt[Y] +
				   m_vqt[Z] * m_vqt[Z] + 
				   m_w * m_w ) );
}




// -----------------------------------------------------------------------------
// Returns the norm square of the quaternion
template <typename T>
__HOSTDEVICE__
T Quaternion<T>::norm2() const
{
  return ( m_vqt[X] * m_vqt[X] + 
		   m_vqt[Y] * m_vqt[Y] +
		   m_vqt[Z] * m_vqt[Z] + 
		   m_w * m_w );
}




// -----------------------------------------------------------------------------
// Returns the conjugate of the quaternion
template <typename T>
__HOSTDEVICE__
Quaternion<T> Quaternion<T>::conjugate() const
{
	return ( Quaternion<T>( -m_vqt, m_w ) );
}




// -----------------------------------------------------------------------------
// Returns the inverse of the quaternion
template <typename T>
__HOSTDEVICE__
Quaternion<T> Quaternion<T>::inverse() const
{
	return ( ( T( 1 ) / this->norm() ) * this->conjugate() );
}




// -----------------------------------------------------------------------------
// Multiplies the quaternion on the left by a vector lhs, i.e., perform
// [ 0, lhs ] x this and return the product that is a quaternion
template <typename T>
__HOSTDEVICE__
Quaternion<T> Quaternion<T>::multLeftVec( Vector3<T> const& lhs ) const
{
	T tmp = -lhs * m_vqt;
	Vector3<T> vtmp = ( lhs ^ m_vqt ) + ( m_w * lhs ) ;
	return ( Quaternion<T>( vtmp, tmp ) );
}




// -----------------------------------------------------------------------------
// Multiplies the quaternion on the right by another quaternion rhs,
// i.e., perform this x rhs, and return the vectorial part of this x rhs
template <typename T>
__HOSTDEVICE__
Vector3<T> Quaternion<T>::multToVector3( Quaternion<T> const& q ) const
{
	Vector3<T> vtmp
		( ( m_vqt ^ q.m_vqt ) + ( m_w * q.m_vqt ) + ( q.m_w * m_vqt ) );
	return ( vtmp );
}




// -----------------------------------------------------------------------------
// Multiplies the quaternion on the right by the conjugate of another
// quaternion rhs, i.e., perform this x rhs^t, and return the vectorial part of
// this x rhs^t
template <typename T>
__HOSTDEVICE__
Vector3<T> Quaternion<T>::multConjugateToVector3( Quaternion<T> const& q ) const
{
	Vector3<T> vtmp
		( - ( m_vqt ^ q.m_vqt ) - ( m_w * q.m_vqt ) + ( q.m_w * m_vqt ) );
	return ( vtmp );
}




// -----------------------------------------------------------------------------
// Rotates a vector using the quaternion *this
template <typename T>
__HOSTDEVICE__
Vector3<T> Quaternion<T>::rotateVector( Vector3<T> const& v ) const
{
	Vector3<T> v_rotated = ( m_w * m_w - m_vqt.norm2() ) * v +
				T( 2 ) * ( v * m_vqt ) * m_vqt + T( 2 ) * m_w * ( m_vqt ^ v );
	return ( v_rotated );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__HOSTDEVICE__
Quaternion<T>& Quaternion<T>::operator += ( Quaternion<T> const& q )
{
	m_w += q.m_w;
	m_vqt += q.m_vqt;
	return ( *this );
}




// -----------------------------------------------------------------------------
// Operator -=
template <typename T>
__HOSTDEVICE__
Quaternion<T>& Quaternion<T>::operator -= ( Quaternion<T> const& q )
{
	m_w -= q.m_w;
	m_vqt -= q.m_vqt;
	return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator *= by a scalar
template <typename T>
__HOSTDEVICE__
Quaternion<T>& Quaternion<T>::operator *= ( T d )
{
	m_w *= d;
	m_vqt *= d;
	return ( *this );
}




// -----------------------------------------------------------------------------
// ith-component accessor: (0,1,2) for the vector components and 3 forthe scalar
template <typename T>
__HOSTDEVICE__
T Quaternion<T>::operator [] ( size_t i ) const
{
	return ( i == 3 ? m_w :  m_vqt[i] );
}




// -----------------------------------------------------------------------------
// ith-component accessor: (0,1,2) for the vector components and 3 for the
// scalar - modifiable lvalue
template <typename T>
__HOSTDEVICE__
T& Quaternion<T>::operator [] ( size_t i )
{
	return ( i == 3 ? m_w : m_vqt[i] );
}




// -----------------------------------------------------------------------------
// Equal operator to another quaternion
template <typename T>
__HOSTDEVICE__
Quaternion<T>& Quaternion<T>::operator = ( Quaternion<T> const& q )
{
	m_w = q.m_w;
	m_vqt = q.m_vqt;
	return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator -. Return a quaternion with negative elements
template <typename T>
__HOSTDEVICE__
Quaternion<T> Quaternion<T>::operator - ()
{
	return ( Quaternion<T>( - m_vqt, - m_w ) );
}




// -----------------------------------------------------------------------------
// Comparison operator
template <typename T>
__HOSTDEVICE__
bool Quaternion<T>::operator == ( Quaternion<T> const& q )
{
	return ( m_w == q.m_w && 
			 m_vqt[0] == q.m_vqt[0] && 
			 m_vqt[1] == q.m_vqt[1] && 
			 m_vqt[2] == q.m_vqt[2] );
}




// -----------------------------------------------------------------------------
// Difference operator
template <typename T>
__HOSTDEVICE__
bool Quaternion<T>::operator != ( Quaternion<T> const& q )
{
	return ( ! ( *this == q ) );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
std::ostream& operator << ( std::ostream& fileOut, 
							Quaternion<T> const& q )
{
	fileOut << q.getScalar() << "\t" << q.getVector();
	return ( fileOut );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
std::istream& operator >> ( std::istream& fileIn, 
							Quaternion<T>& q )
{
	Vector3<T> vec;
	T scalar;
	fileIn >> scalar >> vec;
	q.setScalar( scalar );
	q.setVector( vec );
	return ( fileIn );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Quaternion<float>;
template class Quaternion<double>;

#define X( T ) \
template std::ostream& operator << <T>( std::ostream& fileOut,                 \
                                        Quaternion<T> const& q );              \
                                                                               \
template std::istream& operator >> <T>( std::istream& fileIn,                  \
                                        Quaternion<T>& q );
X( float )
X( double )
#undef X