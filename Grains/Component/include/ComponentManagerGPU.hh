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
        /** \brief number of particles in manager (HOST VARIABLE) */
        unsigned int m_nParticles;
        /** \brief number of obstacles in manager (HOST VARIABLE) */
        unsigned int m_nObstacles;
        /** \brief number of cells in manager (HOST VARIABLE) */
        unsigned int m_nCells;
        /** \brief components rigid body Id */
        unsigned int* m_rigidBodyId; 
        /** \brief components transformation */
        Transform3<T>* m_transform; 
        /** \brief array of components velocities */
        Kinematics<T>* m_velocity;
        /** \brief array of components torce */
        Torce<T>* m_torce;
        /** \brief components Id with positive values for particles and negative
        values for obstacles. */
        int* m_componentId; 
        /** \brief components cell hash */
        unsigned int* m_componentCellHash; 
        /** \brief cells hash start */
        unsigned int* m_cellHashStart; 
        /** \brief cells hash end */
        unsigned int* m_cellHashEnd; 


        // bool* isActive; /**< array of components activity in the simulation */
        // bool* isObstacle; /**< array of components flag for being obstacle */
        // unsigned int m_numComponents; /**< number of components */
        //@}

        
    public:
        /** @name Constructors */
        //@{
        /** @brief Constructor with the number of particles randomly positioned 
        in the computational domain - Change to XML later !!!! */
        ComponentManagerGPU( unsigned int nParticles,
                             unsigned int nObstacles,
                             unsigned int nCells );

        /** @brief Destructor */
        ~ComponentManagerGPU();
        //@}


        /** @name Get methods */
        //@{
        /** @name Get methods */
        //@{
        /** @brief Gets the number of particles in manager */
        unsigned int getNumberOfParticles() const;

        /** @brief Gets the number of obstacles in manager */
        unsigned int getNumberOfObstacles() const;

        /** @brief Gets the number of cells in manager */
        unsigned int getNumberOfCells() const;

        /** @brief Gets components rigid body Id */
        std::vector<unsigned int> getRigidBodyId() const;

        /** @brief Gets components transformation */
        std::vector<Transform3<T>> getTransform() const;

        /** @brief Gets components velocities */
        std::vector<Kinematics<T>> getVelocity() const;

        /** @brief Gets components torce */
        std::vector<Torce<T>> getTorce() const;

        /** @brief Gets the array of component Ids */
        std::vector<int> getComponentId() const;

        // /** @brief Gets the array of components neighbor Id */
        // std::vector<unsigned int> getNeighborsId() const;

        // /** @brief Gets the array of components neighbor count */
        // std::vector<unsigned int> getNeighborsCount() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the array of components rigid body Id */
        void setRigidBodyId( std::vector<unsigned int> const& id );

        /** @brief Sets components transformation */
        void setTransform( std::vector<Transform3<T>> const& t );

        /** @brief Sets components velocities */
        void setVelocity( std::vector<Kinematics<T>> const& v );

        /** @brief Sets components torce */
        void setTorce( std::vector<Torce<T>> const& t );

        /** @brief Sets the array of component Ids */
        void setComponentId( std::vector<int> const& id );

        // /** @brief Sets the array of components neighbor Id */
        // void setNeighborsId( std::vector<unsigned int> const& id );
        
        // /** @brief Sets the array of components neighbor count */
        // void setNeighborsCount( std::vector<unsigned int> const& count );
        //@}
        

        /** @name Methods */
        //@{
        /** @brief Copies data from a host ComponentManager
        @param cm component manager on host */
        void copyFromHost( ComponentManagerCPU<T> const* cm );

        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        /** @brief Updates the linked cell list according to the linked cell 
        provided
        @param LC linked cell */
        void updateLinks( LinkedCell<T> const* const* LC ) final;

        /** @brief Detects collision between particles */
        // template <typename U>
        void detectCollisionAndComputeForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB, 
                                        ContactForceModel<T> const* const* CF,
                                        int* results ) final;

        /** @brief Updates the position and velocities of components */
        // TODO: @param
        void moveComponents( TimeIntegrator<T> const* const* TI,
                             RigidBody<T, T> const* const* RB ) final;
        //@}
};


typedef ComponentManagerGPU<float> ComponentManagerGPU_d;
typedef ComponentManagerGPU<double> ComponentManagerGPU_f;


#endif