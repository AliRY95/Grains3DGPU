#include "FirstOrderExplicit.hh"
#include "VectorMath.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
FirstOrderExplicit<T>::FirstOrderExplicit()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
FirstOrderExplicit<T>::~FirstOrderExplicit()
{}




// ----------------------------------------------------------------------------
// Computes the new velocity and position at time t+dt
template <typename T>
__HOSTDEVICE__
void FirstOrderExplicit<T>::Move( Vector3<T> const& transAcc,
								  Vector3<T> const& AngAcc,
                                  Kinematics<T>& kin,
								  Vector3<T>& transMotion,
								  Vector3<T>& avgAngVel ) const
{
	// Translational velocity and motion
	transMotion = m_dt * kin.getTranslationalVelocity();
	kin.addTranslationalVelocity( m_dt * transAcc );

	// Angular velocity and motion
	avgAngVel = kin.getAngularVelocity();
	kin.addAngularVelocity( m_dt * AngAcc );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class FirstOrderExplicit<float>;
template class FirstOrderExplicit<double>;
