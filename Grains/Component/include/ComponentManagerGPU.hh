#ifndef _COMPONENTMANAGERGPU_HH_
#define _COMPONENTMANAGERGPU_HH_


#include "ComponentManager.hh"


// =============================================================================
/** @brief The class ComponentManager.

    Components in the simulation running on GPU.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ComponentManagerGPU : public ComponentManager<T>
{
    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with the number of particles randomly positioned 
        in the computational domain - Change to XML later !!!! */
        ComponentManagerGPU();

        /** @brief Constructor with host data as input
        @param cm component manager on host */
        ComponentManagerGPU( ComponentManager<T> const& cm );

        /** @brief Destructor */
        ~ComponentManagerGPU();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets components transformation */
        Transform3<T>* getTransform() const;

        /** @brief Gets the array of components neighbor Id */
        unsigned int* getNeighborsId() const;

        /** @brief Gets the array of components rigid body Id */
        unsigned int* getRigidBodyId() const;

        /** @brief Gets the array of component Ids */
        unsigned int* getComponentId() const;

        /** @brief Gets the array of components cell hash */
        unsigned int* getComponentCellHash() const;

        /** @brief Gets the array of components neighbor count */
        unsigned int* getNeighborsCount() const;

        /** @brief Gets the array of cells hash start */
        unsigned int* getCellHashStart() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets components transformation */
        void setTransform( Transform3<T> const* tr );

        /** @brief Sets the array of components neighbor Id */
        void setNeighborsId( unsigned int const* id );

        /** @brief Sets the array of components rigid body Id */
        void setRigidBodyId( unsigned int const* id );

        /** @brief Sets the array of component Ids */
        void setComponentId( unsigned int const* id );

        /** @brief Sets the array of components cell hash */
        void setComponentCellHash( unsigned int const* hash );

        /** @brief Sets the array of components neighbor count */
        void setNeighborsCount( unsigned int const* count );

        /** @brief Sets the array of cells hash start */
        void setCellHashStart( unsigned int const* id );
        //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        /** @brief Updates the linked cell list according to the linked cell 
        provided
        @param LC linked cell */
        void updateLinkedCellList( LinkedCell<T> const* const* LC );

        /** @brief Detects collision between particles */
        void detectCollision( LinkedCell<T> const* const* LC,
                              RigidBody<T> const* const* rb, 
                              int* results );

        // /** @brief Computes impact forces */
        // void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};


#endif