#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_

#include "Transform3.hh"
#include "RigidBody.hh"

#include "GrainsParameters.hh"


// =============================================================================
/** @brief The class ComponentManager.

    Components in the simulation.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class ComponentManager
{
    protected:
        /** @name Parameters */
        //@{
        Transform3d* m_transform; /**< array of components transformation */
        // Mat3d* orientation; /**< array of components orientation */
        // Vec3d* position; /**< array of components position */
        // Vec3d* force; /**< array of components force */
        // Vec3d* torque; /**< array of components torque */
        // Vec3d* translationalVelocity; /**< array of components velocity */
        // Vec3d* angularVelocity; /**< array of components angular velocity */
        unsigned int* m_neighborsId; /**< array of components neighbor Id */ 
        unsigned int* m_rigidBodyId; /**< array of components rigid body Id */
        unsigned int* m_componentId; /**< array of component Ids */
        unsigned int* m_neighborsCount; /**< array of components neighbor count */
        // bool* isActive; /**< array of components activity in the simulation */
        // bool* isObstacle; /**< array of components flag for being obstacle */
        // unsigned int m_numComponents; /**< number of components */
        //@}

    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        ComponentManager();

        /** @brief Destructor */
        virtual ~ComponentManager();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets components transformation */
        virtual Transform3d* getTransform() const = 0;

        /** @brief Gets the array of components neighbor Id */
        virtual unsigned int* getNeighborsId() const = 0;

        /** @brief Gets the array of components rigid body Id */
        virtual unsigned int* getRigidBodyId() const = 0;

        /** @brief Gets the array of component Ids */
        virtual unsigned int* getComponentId() const = 0;

        /** @brief Gets the array of components neighbor count */
        virtual unsigned int* getNeighborsCount() const = 0;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets components transformation */
        virtual void setTransform( const Transform3d* tr ) = 0;

        /** @brief Sets the array of components neighbor Id */
        virtual void setNeighborsId( const unsigned int* id ) = 0;

        /** @brief Sets the array of components rigid body Id */
        virtual void setRigidBodyId( const unsigned int* id ) = 0;

        /** @brief Sets the array of component Ids */
        virtual void setComponentId( const unsigned int* id ) = 0;

        /** @brief Sets the array of components neighbor count */
        virtual void setNeighborsCount( const unsigned int* id ) = 0;
        //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Detects collision between particles */
        virtual void detectCollision( const RigidBody* const* rb, 
                                      bool* result ) = 0;

        // /** @brief Computes impact forces */
        // void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};


#endif