#ifndef _CONTACTFORCEMODEL_HH_
#define _CONTACTFORCEMODEL_HH_


#include "ContactInfo.hh"
#include "Torce.hh"
#include "Vector3.hh"


// ContactForceModel types
enum ContactForceModelType {
    HODC
};


// =============================================================================
/** @brief The class ContactForceModel.

    Defines the contact forces between two colliding components and computes
    these contact forces.

    @author A.YAZDANI - 2024 - Construction */
// =============================================================================
template <typename T>
class ContactForceModel
{
	protected:
        /**@name Contructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        __HOSTDEVICE__ 
        ContactForceModel();

		/** @brief Copy constructor
        @param cf ContactForceModel object to be copied */
        __HOSTDEVICE__
        ContactForceModel( ContactForceModel<T> const& cf );
        //@}


	public:
		/**@name Contructors */
		//@{
		/** @brief Destructor */
		__HOSTDEVICE__
		virtual ~ContactForceModel();
		//@}


		/** @name Get methods */
        //@{
        /** @brief Returns the ContactForceModel type */
        __HOSTDEVICE__
        virtual ContactForceModelType getContactForceModelType() const = 0;
        //@}


		/** @name Methods */
		//@{
		// /** @brief Creates and returns a clone of the object */
		// __HOSTDEVICE__
		// virtual ContactForceModel<T>* clone() const = 0;

		/** @brief Returns a torce based on the contact information
        @param contactInfos geometric contact features
        @param relVelocityAtContact relative velocity at the contact point
        @param relAngVelocity relative angular velocity
        @param m1 mass of the first component (Particle)
        @param m2 mass of the second component (Particle ou Obstacle)
        @param torce computed force and torque */
        // @param nbContact number of contact points for composite particles */
        __HOSTDEVICE__
        virtual void computeForces( ContactInfo<T> const& contactInfos,
									Vector3<T> const& relVelocityAtContact,
									Vector3<T> const& relAngVelocity,
									T m1,
									T m2, 
                                    Vector3<T> const& trOrigin,
									Torce<T>& torce ) const = 0;
		//@}
};

#endif
