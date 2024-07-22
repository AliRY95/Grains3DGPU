#ifndef _KINEMATICS_HH_
#define _KINEMATICS_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief The class Kinematics.

    Manages the kinematics of components.

    @author G.FERRER - Institut Francais du Petrole - 2000 - Creation 
    @author A.WACHS - 2019 - Major cleaning & refactoring */
// ============================================================================
template <typename T>
class Kinematics
{
    protected:
        /** @name Parameters */
        //@{
        Vector3<T> m_translationalVelocity; /**< Translational velocity */
        Vector3<T> m_angularVelocity; /**< Angular velocity */  
        // Vector3 m_dUdt; /**< Translational velocity variation dU/dt */
        // Vector3 m_dOmegadt; /**< Angular velocity variation dom/dt */
        // Vector3 m_translationalDisplacementOverDt; /**< Translational displacement
        // over dt */
        // Vector3 m_averageAngularVelocityOverDt; /**< average angular velocity over 
        // dt */
        //@}
    
    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        Kinematics();

        /** @brief Constructor with a u and a omega as input parameters
        @param u translational velocity
        @param omega angular velocity */
        __HOSTDEVICE__
        Kinematics( Vector3<T> const& u, 
                    Vector3<T> const& omega );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Kinematics();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets translational velocity of the kinematics */
        __HOSTDEVICE__
        Vector3<T> getTranslationalVelocity() const;
        
        /** @brief Gets angular velocity of the kinematics */
        __HOSTDEVICE__
        Vector3<T> getAngularVelocity() const;
        //@}
    

        /** @name Set methods */
        //@{
        /** @brief Sets the translational velocity of the kinematics
        @param u translation velocity */
        __HOSTDEVICE__
        void setTranslationalVelocity( Vector3<T> const& u );

        /** @brief Sets the angular velocity of the kinematics
        @param omega angular velocity */
        __HOSTDEVICE__
        void setAngularVelocity( Vector3<T> const& omega );
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the total velocity U + om x R given R 
        @param R arm vector */
        __HOSTDEVICE__
        Vector3<T> Velocity( Vector3<T> const& R ) const;
        //@}    
};


typedef Kinematics<float> KinematicsF;
typedef Kinematics<double> KinematicsD;


#endif
