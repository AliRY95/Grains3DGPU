#ifndef _CONTACTFORCEMODEL_HH_
#define _CONTACTFORCEMODEL_HH_


#include "Basic.hh"
#include "Vector3.hh"
#include "ContactInfo.hh"


// =============================================================================
/** The class ContactForceModel.

    Contact force model involving a normal Hookean spring, a normal Dashpot and
    a tangential Coulomb friction (HO-D-C) to compute the force and torque 
    induced by the contact between two rigid components.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ContactForceModel
{
    protected:
        /** @name Parameters */
        //@{
        T stiff; /**< stiffness coefficient */  
        T en; /**< restitution coefficient */ 
        T muet; /**< tangential damping coefficient */
        T muec; /**< Coulomb friction coefficient */
        T k_m_s; /**< rolling friction coefficient */
        //@}

    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with contact parameters as inputs
        @param stiff stiffness coefficient
        @param en restitution coefficient
        @param muet tangential damping coefficient
        @param muec Coulomb friction coefficient
        @param k_m_s rolling friction coefficient */
        __HOSTDEVICE__
        ContactForceModel( T stiff,
                           T en,
                           T muet,
                           T muec,
                           T k_m_s );

        /** @brief Destructor */
        __HOSTDEVICE__ 
        ~ContactForceModel();
        //@}


        /** @name Methods */
        //@{
        /** @brief Performs forces & torques computation
        @param p0_ first Component (Particle)
        @param p1_ second Component (Particle ou Obstacle)
        @param contactInfos geometric contact features
        @param delFN normal force
        @param delFT tangential force
        @param delM torque */
        void performForcesCalculus( Component* p0_,  Component* p1_,
        PointContact const& contactInfos,
        Vector3& delFN, Vector3& delFT, Vector3& delM );

        /** @brief Computes forces & torques 
        @param p1 first component
        @param p2 second component
        @param contactInfo geometric contact features */
        bool computeForces( Component* p1, Component* p1_,
        ContactInfo const& contactInfo );
        //@}
};


#endif
