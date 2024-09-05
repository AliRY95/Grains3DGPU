#ifndef _COMPONENTMANAGERGPU_HH_
#define _COMPONENTMANAGERGPU_HH_


#include "ComponentManager.hh"
#include "ComponentManagerCPU.hh"


// =============================================================================
/** @brief The class ComponentManagerGPU.

    Components in the simulation running on GPU.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ComponentManagerGPU : public ComponentManager<T>
{
    protected:
        /** @name Parameters */
        //@{
        unsigned int* m_rigidBodyId; /**< array of components rigid body Id */
        Transform3<T>* m_transform; /**< array of components transformation */
        Kinematics<T>* m_kinematics; /**< array of components kinematics */
        Torce<T>* m_torce; /**< array of components torce */
        int* m_componentId; /**< array of component Ids */
        unsigned int* m_componentCellHash; /**< array of components cell hash */
        unsigned int* m_cellHashStart; /**< array of cells hash start */
        unsigned int* m_cellHashEnd; /**< array of cells hash start */
        // bool* isActive; /**< array of components activity in the simulation */
        // bool* isObstacle; /**< array of components flag for being obstacle */
        // unsigned int m_numComponents; /**< number of components */
        //@}

        
    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with the number of particles randomly positioned 
        in the computational domain - Change to XML later !!!! */
        ComponentManagerGPU();

        /** @brief Constructor with host data as input
        @param cm component manager on host */
        ComponentManagerGPU( ComponentManagerCPU<T> const& cm );

        /** @brief Destructor */
        ~ComponentManagerGPU();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the array of components rigid body Id */
        unsigned int* getRigidBodyId() const;

        /** @brief Gets components transformation */
        Transform3<T>* getTransform() const;

        /** @brief Gets the array of component Ids */
        int* getComponentId() const;

        /** @brief Gets the array of components cell hash */
        unsigned int* getComponentCellHash() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the array of components rigid body Id */
        void setRigidBodyId( unsigned int const* id );
        
        /** @brief Sets components transformation */
        void setTransform( Transform3<T> const* tr );

        /** @brief Sets the array of component Ids */
        void setComponentId( int const* id );
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
        // template <typename U>
        void detectCollision( LinkedCell<T> const* const* LC,
                              RigidBody<T, T> const* const* RB, 
                              ContactForceModel<T> const* const* CF,
                              int* results );

        /** @brief Computes impact forces */
        void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};


typedef ComponentManagerGPU<float> ComponentManagerGPU_d;
typedef ComponentManagerGPU<double> ComponentManagerGPU_f;


#endif