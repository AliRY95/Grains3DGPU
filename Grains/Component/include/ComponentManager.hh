#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_


#include "GrainsParameters.hh"
#include "Transform3.hh"
#include "RigidBody.hh"
#include "LinkedCell.hh"
#include "Kinematics.hh"
#include "Torce.hh"
#include "ContactForceModel.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "TimeIntegrator.hh"


// =============================================================================
/** @brief The class ComponentManager.

    This is just an abstract class to make sure all derived classess follow the
    same set of methods.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ComponentManager
{
    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        ComponentManager();

        /** @brief Destructor */
        virtual ~ComponentManager();
        //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Updates links between components and linked cell */
        // template <typename U>
        // TODO: @param
        virtual void updateLinks( LinkedCell<T> const* const* LC ) = 0;

        /** @brief Detects collision between components and computes forces */
        // template <typename U>
        // TODO: @param
        virtual void detectCollisionAndComputeForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB, 
                                        ContactForceModel<T> const* const* CF,
                                        int* result ) = 0;

        /** @brief Updates the position and velocities of components */
        // TODO: @param
        virtual void moveComponents( TimeIntegrator<T> const* const* TI,
                                     RigidBody<T, T> const* const* RB ) = 0;
        //@}
};

typedef ComponentManager<float> ComponentManager_d;
typedef ComponentManager<double> ComponentManager_f;

#endif