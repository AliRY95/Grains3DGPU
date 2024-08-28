#ifndef _TIMEINTEGRATOR_HH_
#define _TIMEINTEGRATOR_HH_


#include "Torce.hh"
#include "Transform3.hh"
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

    @author A.WACHS - Institut Francais du Petrole - 2011 - Creation 
    @author A.WACHS - 2019 - Major cleaning & refactoring
    @author A.YAZDANI - 2024 - Major cleaning for porting to GPU */
// =============================================================================
template <typename T>
class TimeIntegrator
{
	protected:
        /** @name Parameters */
        //@{
        T m_dt; /**< time step */
        //@}


		/**@name Contructors */
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
		/**@name Contructors */
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

		/** @brief Computes the new velocity and position at time t+dt
		@param acceleration acceleration
		@param velocity velocity 
		@param transMotion translation motion
		@param avgAngVel average angular velocity in interval [t,t+dt] */
		__HOSTDEVICE__    
		virtual void Move( Kinematics<T> const& acceleration,
						   Kinematics<T>& velocity,
						   Vector3<T>& transMotion,
						   Vector3<T>& avgAngVel ) const = 0;
		//@}
};

#endif
