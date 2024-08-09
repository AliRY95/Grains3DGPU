#ifndef _FIRSTORDEREXPLICIT_HH_
#define _FIRSTORDEREXPLICIT_HH_


#include "Torce.hh"
#include "Transform3.hh"
#include "Kinematics.hh"


// =============================================================================
/** @brief The class FirstOrderExplicit.

    First order explicit integration scheme: x(t+dt) = x(t) + dt*v(t) and 
    v(t+dt) = v(t) + dt*a(t). 
    
    @author A.Yazdani - 2024 - Construction */
// ============================================================================
template <typename T>
class FirstOrderExplicit
{
	protected:
        /** @name Parameters */
        //@{
        T m_dt; /**< time step */
        //@}


    public:
        /**@name Contructors & Destructor */
        //@{
        /** @brief Destructor */
        __HOSTDEVICE__
        ~FirstOrderExplicit();
      
        /** @brief Default constructor */
        __HOSTDEVICE__
        FirstOrderExplicit();
        //@}


        /** @name Methods */
        //@{
        /** @brief Computes the new velocity and position at time t+dt
        @param torce torce acting on the component
        @param mass mass of the component
		@param inertia inertia tensor of the component
		@param tr transformation of the component
        @param kin kinematics of the component */
        __HOSTDEVICE__
        void Move( Vector3<T> const& transAcc,
								  Vector3<T> const& AngAcc,
                                  Kinematics<T>& kin,
								  Vector3<T>& transMotion,
								  Vector3<T>& avgAngVel ) const;
        //@}
};

#endif
