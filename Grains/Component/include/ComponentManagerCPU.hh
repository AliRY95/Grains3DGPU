#ifndef _COMPONENTMANAGERCPU_HH_
#define _COMPONENTMANAGERCPU_HH_


#include "ComponentManager.hh"


// =============================================================================
/** @brief The class ComponentManagerCPU.

    Components in the simulation running on CPU.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ComponentManagerCPU : public ComponentManager<T>
{
    protected:
        /** @name Parameters */
        //@{
        std::vector<Transform3<T>> m_transform; /**< components transformation */
        // Mat3d* orientation; /**< array of components orientation */
        // Vec3d* position; /**< array of components position */
        // Vec3d* force; /**< array of components force */
        // Vec3d* torque; /**< array of components torque */
        // Vec3d* translationalVelocity; /**< array of components velocity */
        // Vec3d* angularVelocity; /**< array of components angular velocity */
        std::vector<unsigned int> m_neighborsId; /**< components neighbor Id */ 
        std::vector<unsigned int> m_rigidBodyId; /**< components rigid body Id */
        std::vector<unsigned int> m_componentId; /**< components Id */
        std::vector<unsigned int> m_componentCellHash; /**< components cell hash */
        std::vector<unsigned int> m_neighborsCount; /**< components neighbor count */
        std::vector<unsigned int> m_cellHashStart; /**< cells hash start */
        std::vector<unsigned int> m_cellHashEnd; /**< cells hash start */
        // bool* isActive; /**< array of components activity in the simulation */
        // bool* isObstacle; /**< array of components flag for being obstacle */
        // unsigned int m_numComponents; /**< number of components */
        //@}

        
    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with the number of particles randomly positioned 
        in the computational domain - Change to XML later !!!! */
        ComponentManagerCPU();

        /** @brief Destructor */
        ~ComponentManagerCPU();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets components transformation */
        std::vector<Transform3<T>> getTransform() const;

        /** @brief Gets the array of components neighbor Id */
        std::vector<unsigned int> getNeighborsId() const;

        /** @brief Gets the array of components rigid body Id */
        std::vector<unsigned int> getRigidBodyId() const;

        /** @brief Gets the array of component Ids */
        std::vector<unsigned int> getComponentId() const;

        /** @brief Gets the array of components cell hash */
        std::vector<unsigned int> getComponentCellHash() const;

        /** @brief Gets the array of components neighbor count */
        std::vector<unsigned int> getNeighborsCount() const;

        /** @brief Gets the array of cells hash start */
        std::vector<unsigned int> getCellHashStart() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets components transformation */
        void setTransform( std::vector<Transform3<T>> const& tr );

        /** @brief Sets the array of components neighbor Id */
        void setNeighborsId( std::vector<unsigned int> const& id );

        /** @brief Sets the array of components rigid body Id */
        void setRigidBodyId( std::vector<unsigned int> const& id );

        /** @brief Sets the array of component Ids */
        void setComponentId( std::vector<unsigned int> const& id );

        /** @brief Sets the array of components cell hash */
        void setComponentCellHash( std::vector<unsigned int> const& hash );

        /** @brief Sets the array of components neighbor count */
        void setNeighborsCount( std::vector<unsigned int> const& count );

        /** @brief Sets the array of cells hash start */
        void setCellHashStart( std::vector<unsigned int> const& id );
        //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Detects collision between particles */
        // template <typename U>
        void detectCollision( LinkedCell<T> const* const* LC,
                              RigidBody<T, T> const* const* rb, 
                              int* result );

        // /** @brief Computes impact forces */
        // void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};


typedef ComponentManagerCPU<float> ComponentManagerCPU_d;
typedef ComponentManagerCPU<double> ComponentManagerCPU_f;


#endif