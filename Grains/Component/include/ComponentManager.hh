#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_


#include "GrainsParameters.hh"
#include "Transform3.hh"
#include "RigidBody.hh"
#include "LinkedCell.hh"
#include "Kinematics.hh"
#include "Torce.hh"
#include "HODCContactForceModel.hh"
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


        // /** @name Get methods */
        // //@{
        // /** @brief Gets components transformation */
        // virtual Transform3<T>* getTransform() const = 0;

        // /** @brief Gets the array of components neighbor Id */
        // virtual unsigned int* getNeighborsId() const = 0;

        // /** @brief Gets the array of components rigid body Id */
        // virtual unsigned int* getRigidBodyId() const = 0;

        // /** @brief Gets the array of component Ids */
        // virtual unsigned int* getComponentId() const = 0;

        // /** @brief Gets the array of components cell hash */
        // virtual unsigned int* getComponentCellHash() const = 0;

        // /** @brief Gets the array of components neighbor count */
        // virtual unsigned int* getNeighborsCount() const = 0;

        // /** @brief Gets the array of cells hash start */
        // virtual unsigned int* getCellHashStart() const = 0;
        // //@}


        // /** @name Set methods */
        // //@{
        // /** @brief Sets components transformation */
        // virtual void setTransform( Transform3<T> const* tr ) = 0;

        // /** @brief Sets the array of components neighbor Id */
        // virtual void setNeighborsId( unsigned int const* id ) = 0;

        // /** @brief Sets the array of components rigid body Id */
        // virtual void setRigidBodyId( unsigned int const* id ) = 0;

        // /** @brief Sets the array of component Ids */
        // virtual void setComponentId( unsigned int const* id ) = 0;

        // /** @brief Sets the array of components cell hash */
        // virtual void setComponentCellHash( unsigned int const* hash ) = 0;
        
        // /** @brief Sets the array of components neighbor count */
        // virtual void setNeighborsCount( unsigned int const* count ) = 0;

        // /** @brief Sets the array of cells hash start */
        // virtual void setCellHashStart( unsigned int const* id ) = 0;
        // //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Detects collision between particles */
        // template <typename U>
        virtual void detectCollision( LinkedCell<T> const* const* LC,
                                      RigidBody<T, T> const* const* rb, 
                                      HODCContactForceModel<T> const* const* CF,
                                      int* result ) = 0;

        // /** @brief Computes impact forces */
        // void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};

typedef ComponentManager<float> ComponentManager_d;
typedef ComponentManager<double> ComponentManager_f;

#endif