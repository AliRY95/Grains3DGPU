#ifndef _HOOKECONTACTFORCEMODEL_HH_
#define _HOOKECONTACTFORCEMODEL_HH_


#include "ContactForceModel.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class HookeContactForceModel.

    Contact force model involving a normal Hookean spring, a normal Dashpot and
    a tangential Coulomb friction (HO-D-C) to compute the force and torque 
    induced by the contact between two rigid components.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class HookeContactForceModel : public ContactForceModel<T>
{
    private:
        /**@name Parameter */
        //@{
        T m_kn;    /**< Normal stiffness coefficient */  
        T m_en;    /**< Normal restitution coefficient */
        T m_muen;  /**< log(m_en) / sqrt( PI * PI + log(m_en) * log(m_en) ) */
        T m_etat;  /**< Tangential damping coefficient */
        T m_muc;   /**< Tangential Coulomb friction coefficient */
        T m_kr;    /**< Rolling resistance coefficient */
        //@}
    
    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        HookeContactForceModel();

        /** @brief Constructor with an XML node
        @param root XML node */
        __HOST__
        HookeContactForceModel( DOMNode* root );

        /** @brief Constructor with five values as contact parameters
        @param kn normal stiffness coefficient
        @param en normal restitution coefficient
        @param etat tangential damping coefficient
        @param muc tangential Coulomb friction coefficient
        @param kr rolling resistance coefficient */
        __HOSTDEVICE__
        HookeContactForceModel( T kn,
                                T en, 
                                T etat, 
                                T muc, 
                                T kr ); 

        /** @brief Destructor */
        __HOSTDEVICE__
        ~HookeContactForceModel();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the ContactForceModel type */
        __HOSTDEVICE__
        ContactForceModelType getContactForceModelType() const final;

        /** @brief Gets the parameters of the Hooke contact force model
        @param kn normal stiffness coefficient
        @param en normal restitution coefficient
        @param etat tangential damping coefficient
        @param muc tangential Coulomb friction coefficient
        @param kr rolling resistance coefficient */
        __HOSTDEVICE__
        void getContactForceModelParameters( T& kn,
                                             T& en, 
                                             T& etat, 
                                             T& muc, 
                                             T& kr ) const;
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


typedef HookeContactForceModel<float>  HookeContactForceModelF;
typedef HookeContactForceModel<double> HookeContactForceModelD;


#endif
