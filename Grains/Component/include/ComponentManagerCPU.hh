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
        /** \brief number of particles in manager */
        unsigned int m_nParticles;
        /** \brief number of obstacles in manager */
        unsigned int m_nObstacles;
        /** \brief number of cells in manager */
        unsigned int m_nCells;
        /** \brief components rigid body Id */
        std::vector<unsigned int> m_rigidBodyId; 
        /** \brief components transformation */
        std::vector<Transform3<T>> m_transform; 
        /** \brief array of components velocities */
        std::vector<Kinematics<T>> m_velocity;
        /** \brief array of components torce */
        std::vector<Torce<T>> m_torce;
        /** \brief components Id with positive values for particles and negative
        values for obstacles. */
        std::vector<int> m_componentId; 
        /** \brief components cell hash */
        std::vector<unsigned int> m_componentCellHash; 
        /** \brief cells hash start */
        std::vector<unsigned int> m_cellHashStart; 
        /** \brief cells hash end */
        std::vector<unsigned int> m_cellHashEnd; 

        // /**< components neighbor Id */ 
        // std::vector<unsigned int> m_neighborsId; 
        // /**< components neighbor count */
        // std::vector<unsigned int> m_neighborsCount; 
        /**< array of components activity in the simulation */
        // std::vector<bool> m_isActive; 
        //@}

        
    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor */
        ComponentManagerCPU();

        /** @brief Constructor with the number of particles, obstacles, and
        cells. Particles are assumed to have the same shape (rigid body ID = 0). 
        Transformations are all identity, velocities and torce are set to zero.
        It is not checked whether the particles are intersecting. 
        This constructor must be used only when we want to later modify the 
        transformations, otherwise having all particles at the origin is not a 
        physical configuration. */
        ComponentManagerCPU( unsigned int nParticles,
                             unsigned int nObstacles,
                             unsigned int nCells );

        /** @brief Constructor given the number of each rigid body, number of 
        obstacles, and number of cells. rigidBodyId is set according to the
        number of each rigid body.
        Transformations are randomly chosen in the global domain taken from 
        GrainsParameters, while velocities and torce are set to zero.
        It is not checked whether the particles are intersecting. */
        ComponentManagerCPU( std::vector<unsigned int> numEachRigidBody,
                             unsigned int nObstacles,
                             unsigned int nCells );

        /** @brief Constructor given an insertion policy, the number of each 
        rigid body, number of obstacles, and number of cells.
        It is not checked whether the particles are intersecting. */
        ComponentManagerCPU( std::vector<unsigned int> numEachRigidBody,
                             unsigned int nObstacles,
                             unsigned int nCells,
                             std::vector<Transform3<T>> initTr );

        /** @brief Destructor */
        ~ComponentManagerCPU();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the number of particles in manager */
        unsigned int getNumberOfParticles() const final;

        /** @brief Gets the number of obstacles in manager */
        unsigned int getNumberOfObstacles() const final;

        /** @brief Gets the number of cells in manager */
        unsigned int getNumberOfCells() const final;

        /** @brief Gets components rigid body Id */
        std::vector<unsigned int> getRigidBodyId() const final;

        /** @brief Gets components transformation */
        std::vector<Transform3<T>> getTransform() const final;

        /** @brief Gets components velocities */
        std::vector<Kinematics<T>> getVelocity() const final;

        /** @brief Gets components torce */
        std::vector<Torce<T>> getTorce() const final;

        /** @brief Gets the array of component Ids */
        std::vector<int> getComponentId() const final;

        // /** @brief Gets the array of components neighbor Id */
        // std::vector<unsigned int> getNeighborsId() const;

        // /** @brief Gets the array of components neighbor count */
        // std::vector<unsigned int> getNeighborsCount() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the array of components rigid body Id */
        void setRigidBodyId( std::vector<unsigned int> const& id ) final;

        /** @brief Sets components transformation */
        void setTransform( std::vector<Transform3<T>> const& t ) final;

        /** @brief Sets components velocities */
        void setVelocity( std::vector<Kinematics<T>> const& v ) final;

        /** @brief Sets components torce */
        void setTorce( std::vector<Torce<T>> const& t ) final;

        /** @brief Sets the array of component Ids */
        void setComponentId( std::vector<int> const& id ) final;

        // /** @brief Sets the array of components neighbor Id */
        // void setNeighborsId( std::vector<unsigned int> const& id );
        
        // /** @brief Sets the array of components neighbor count */
        // void setNeighborsCount( std::vector<unsigned int> const& count );
        //@}


        /** @name Methods */
        //@{
        /** @brief Inserts particles according to a given insertion policy
        @param ins insertion policy */
        void insertParticles( Insertion<T> const* ins ) final;

        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Updates links between components and linked cell */
        // template <typename U>
        void updateLinks( LinkedCell<T> const* const* LC ) final;
        
        /** @brief Detects collision between components and computes forces */
        // template <typename U>
        // TODO: @param
        void detectCollisionAndComputeContactForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB,
                                        ContactForceModel<T> const* const* CF,
                                        int* result ) final;

        /** @brief Updates the position and velocities of components */
        void moveComponents( TimeIntegrator<T> const* const* TI,
                             RigidBody<T, T> const* const* RB ) final;
        //@}
};


typedef ComponentManagerCPU<float> ComponentManagerCPU_d;
typedef ComponentManagerCPU<double> ComponentManagerCPU_f;


#endif