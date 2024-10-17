#ifndef _HODCCONTACTFORCEMODEL_HH_
#define _HODCCONTACTFORCEMODEL_HH_


#include "ContactForceModel.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class HODCContactForceModel.

    Contact force model involving a normal Hookean spring, a normal Dashpot and
    a tangential Coulomb friction (HO-D-C) to compute the force and torque 
    induced by the contact between two rigid components.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class HODCContactForceModel : public ContactForceModel<T>
{
    private:
        /**@name Parameter */
        //@{
        T m_stiff; /**< Coefficient of elasticity */  
        T m_en;    /**< Coefficient of restitution */
        T m_muen;  /**< Coefficient of ??? */
        T m_muet;  /**< Coefficient of tangential friction */
        T m_muec;  /**< Coefficient of Coulomb friction */
        T m_kms;   /**< Coefficient of rolling friction */
        //@}
    
    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        HODCContactForceModel();

        /** @brief Constructor with an XML node
        @param root XML node */
        __HOST__
        HODCContactForceModel( DOMNode* root );

        /** @brief Constructor with five values as contact parameters
        @param stiff coefficient of elasticity
        @param en coefficient of restitution
        @param muet coefficient of tangential friction
        @param muec coefficient of Coulomb friction
        @param kms coefficient of rolling friction */
        __HOSTDEVICE__
        HODCContactForceModel( T stiff,
                               T en, 
                               T muet, 
                               T muec, 
                               T kms ); 

        /** @brief Destructor */
        __HOSTDEVICE__
        ~HODCContactForceModel();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the ContactForceModel type */
        __HOSTDEVICE__
        ContactForceModelType getContactForceModelType() const final;

        /** @brief Gets the parameters of the HODC contact force model
        @param stiff coefficient of elasticity
        @param en coefficient of restitution
        @param muet coefficient of tangential friction
        @param muec coefficient of Coulomb friction
        @param kms coefficient of rolling friction */
        __HOSTDEVICE__
        void getContactForceModelParameters( T& stiff,
                                             T& en, 
                                             T& muet, 
                                             T& muec, 
                                             T& kms ) const;
        //@}


        /**@name Methods */
        //@{        
        /** @brief Returns a torce based on the contact information
        @param contactInfos geometric contact features
        @param relVelocityAtContact relative velocity at the contact point
        @param relAngVelocity relative angular velocity
        @param m1 mass of the first component (Particle)
        @param m2 mass of the second component (Particle ou Obstacle)
        @param delFN normal force
        @param delFT tangential force
        @param delM torque */
        __HOSTDEVICE__
        void performForcesCalculus( ContactInfo<T> const& contactInfos,
                                    Vector3<T> const& relVelocityAtContact,
                                    Vector3<T> const& relAngVelocity,
                                    T m1,
                                    T m2, 
                                    Vector3<T>& delFN,
                                    Vector3<T>& delFT,
                                    Vector3<T>& delM ) const;
        
        /** @brief Returns a torce based on the contact information
        @param contactInfos geometric contact features
        @param relVelocityAtContact relative velocity at the contact point
        @param relAngVelocity relative angular velocity
        @param m1 mass of the first component (Particle)
        @param m2 mass of the second component (Particle ou Obstacle)
        @param torce computed force and torque */
        // @param nbContact number of contact points for composite particles */
        __HOSTDEVICE__
        void computeForces( ContactInfo<T> const& contactInfos,
                            Vector3<T> const& relVelocityAtContact,
                            Vector3<T> const& relAngVelocity,
                            T m1,
                            T m2, 
                            Vector3<T> const& trOrigin,
                            Torce<T>& torce ) const final;
        //@}
};


typedef HODCContactForceModel<float> HODCContactForceModelF;
typedef HODCContactForceModel<double> HODCContactForceModelD;


#endif
