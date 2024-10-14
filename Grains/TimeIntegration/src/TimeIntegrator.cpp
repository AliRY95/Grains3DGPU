#include "TimeIntegrator.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::TimeIntegrator()
{}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::TimeIntegrator( TimeIntegrator<T> const& ti )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
TimeIntegrator<T>::~TimeIntegrator()
{}




// -----------------------------------------------------------------------------
// Computes the quaternion change over the time step
template <typename T>
__HOSTDEVICE__
Quaternion<T> TimeIntegrator<T>::computeQuaternionChange( 
                                            Vector3<T> const& avgAngVel ) const
{
    // Quaternion change over dt
    Quaternion<T> qRotChange;
    T nOmega = norm( avgAngVel );
    if ( nOmega > HIGHEPS<T> ) 
    {
        T c = cos( nOmega * m_dt / T( 2 ) );
        T s = sin( nOmega * m_dt / T( 2 ) );
        // T c = cos( nOmega * m_dt );
        // T s = sin( nOmega * m_dt );
        return ( Quaternion<T>( ( s / nOmega ) * avgAngVel, c ) );
    } 
    else 
        return ( Quaternion<T>( T( 0 ), T( 1 ) ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class TimeIntegrator<float>;
template class TimeIntegrator<double>;