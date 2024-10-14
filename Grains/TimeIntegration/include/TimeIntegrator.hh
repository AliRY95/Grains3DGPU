#ifndef _TIMEINTEGRATOR_HH_
#define _TIMEINTEGRATOR_HH_


#include "Transform3.hh"
#include "Quaternion.hh"
#include "Torce.hh"
#include "Kinematics.hh"


// TimeIntegrator types
enum TimeIntegratorType {
    FIRSTORDEREXPLICIT,
    SECONDORDEREXPLICIT
};


// =============================================================================
/** @brief The class TimeIntegrator.

    Numerical scheme for the time integration of the Newton's law and the
    kinematic equations. 

    @author A.YAZDANI - 2024 - Construction */
// =============================================================================
template <typename T>
class TimeIntegrator
{
	protected:
        /** @name Parameters */
        //@{
        T m_dt; /**< time step */
        //@}


		/** @name Contructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        __HOSTDEVICE__ 
        TimeIntegrator();

		/** @brief Copy constructor
        @param ti TimeIntegrator object to be copied */
        __HOSTDEVICE__
        TimeIntegrator( TimeIntegrator<T> const& ti );
        //@}


	public:
		/** @name Contructors */
		//@{
		/** @brief Destructor */
		__HOSTDEVICE__
		virtual ~TimeIntegrator();
		//@}


		/** @name Get methods */
        //@{
        /** @brief Returns the time integrator type */
        __HOSTDEVICE__
        virtual TimeIntegratorType getTimeIntegratorType() const = 0;
        //@}


		/** @name Methods */
		//@{
		/** @brief Creates and returns a clone of the time integrator */
		__HOSTDEVICE__
		virtual TimeIntegrator<T>* clone() const = 0;

		/** @brief Computes the quaternion change over the time step given an
		average for the angular velocity over the time step
		@param v average of the angular velocity over the time step */
		__HOSTDEVICE__
		Quaternion<T> computeQuaternionChange( Vector3<T> const& v ) const;

		/** @brief Computes the new velocity and transformation change over dt
		@param momentum acceleration
		@param velocity velocity 
		@param transMotion translational motion over dt
		@param rotMotion rotational motion over dt */
		__HOSTDEVICE__    
		virtual void Move( Kinematics<T> const& momentum,
						   Kinematics<T>& velocity,
						   Vector3<T>& transMotion,
						   Quaternion<T>& rotMotion ) const = 0;
		//@}
};

#endif
