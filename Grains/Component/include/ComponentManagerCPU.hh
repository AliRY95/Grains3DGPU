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
        /** \brief Particles rigid body Id */
        std::vector<unsigned int> m_rigidBodyId;
        /** \brief Obsatcles rigid body Id */
        std::vector<unsigned int> m_obstacleRigidBodyId;
        /** \brief Particles transformation */
        std::vector<Transform3<T>> m_transform;
        /** \brief Obstacles transformation */
        std::vector<Transform3<T>> m_obstacleTransform;
        /** \brief Particles velocities */
        std::vector<Kinematics<T>> m_velocity;
        /** \brief Particles torce */
        std::vector<Torce<T>> m_torce;
        /** \brief Particles Id */
        std::vector<unsigned int> m_particleId;
        /** \brief Particles cell hash */
        std::vector<unsigned int> m_particleCellHash;
        /** \brief Cells and particle Ids each cell contains */
        std::vector<std::vector<unsigned int>> m_cell;
        /** \brief Number of particles in manager */
        unsigned int m_nParticles;
        /** \brief Number of obstacles in manager */
        unsigned int m_nObstacles;
        /** \brief Number of cells in manager */
        unsigned int m_nCells;

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
        cells. All other data members are set to default. */
        ComponentManagerCPU( unsigned int nParticles,
                             unsigned int nObstacles,
                             unsigned int nCells );

        /** @brief Destructor */
        ~ComponentManagerCPU();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets particles rigid body Id */
        std::vector<unsigned int> getRigidBodyId() const final;

        /** @brief Gets obstacles rigid body Id */
        std::vector<unsigned int> getRigidBodyIdObstacles() const final;

        /** @brief Gets particles transformation */
        std::vector<Transform3<T>> getTransform() const final;

        /** @brief Gets obstacles transformation */
        std::vector<Transform3<T>> getTransformObstacles() const final;

        /** @brief Gets particles velocities */
        std::vector<Kinematics<T>> getVelocity() const final;

        /** @brief Gets particles torces */
        std::vector<Torce<T>> getTorce() const final;

        /** @brief Gets the array of particles Ids */
        std::vector<unsigned int> getParticleId() const final;

        /** @brief Gets the number of particles in manager */
        unsigned int getNumberOfParticles() const final;

        /** @brief Gets the number of obstacles in manager */
        unsigned int getNumberOfObstacles() const final;

        /** @brief Gets the number of cells in manager */
        unsigned int getNumberOfCells() const final;

        // /** @brief Gets the array of components neighbor Id */
        // std::vector<unsigned int> getNeighborsId() const;

        // /** @brief Gets the array of components neighbor count */
        // std::vector<unsigned int> getNeighborsCount() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the array of particles rigid body Ids */
        void setRigidBodyId( std::vector<unsigned int> const& id ) final;

        /** @brief Sets the array of obstacles rigid body Ids */
        void setRigidBodyIdObstacles( 
                                    std::vector<unsigned int> const& id ) final;

        /** @brief Sets particles transformations */
        void setTransform( std::vector<Transform3<T>> const& t ) final;

        /** @brief Sets obstacles transformations */
        void setTransformObstacles( std::vector<Transform3<T>> const& t ) final;

        /** @brief Sets particles velocities */
        void setVelocity( std::vector<Kinematics<T>> const& v ) final;

        /** @brief Sets particles torces */
        void setTorce( std::vector<Torce<T>> const& t ) final;

        /** @brief Sets the array of particles Ids */
        void setParticleId( std::vector<unsigned int> const& id ) final;

        // /** @brief Sets the array of components neighbor Id */
        // void setNeighborsId( std::vector<unsigned int> const& id );
        
        // /** @brief Sets the array of components neighbor count */
        // void setNeighborsCount( std::vector<unsigned int> const& count );
        //@}


        /** @name Methods */
        //@{
        /** @brief Initializes the RigidBody IDs and transformations for 
        // obstacles in the simulation
        @param numEachUniqueObstacles accumulating vector for number of 
        different RB 
        @param initTr initial transformation of obstacles */
        void initializeObstacles( 
                            std::vector<unsigned int> numEachUniqueObstacles,
                            std::vector<Transform3<T>> initTr ) final;

        /** @brief Initializes the RigidBody IDs and transformations for 
        // particles in the simulation
        @param numEachUniqueParticles accumulating vector for number of 
        different RB 
        @param initTr initial transformation of particles */
        void initializeParticles( 
                            std::vector<unsigned int> numEachUniqueParticles,
                            std::vector<Transform3<T>> initTr ) final;

        /** @brief Inserts particles according to a given insertion policy
        @param ins insertion policy */
        void insertParticles( Insertion<T>* ins ) final;

        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Updates links between particles and linked cell
        @param LC linked cell */
        void updateLinks( LinkedCell<T> const* const* LC ) final;
        
        /** @brief Detects collision between particles and obstacles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param CF array of all contact force models */
        void detectCollisionAndComputeContactForcesObstacles( 
                                RigidBody<T, T> const* const* particleRB,
                                RigidBody<T, T> const* const* obstacleRB,
                                ContactForceModel<T> const* const* CF ) final;

        /** @brief Detects collision between particles and particles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param LC linked cell
        @param CF array of all contact force models */
        void detectCollisionAndComputeContactForcesParticles( 
                                RigidBody<T, T> const* const* particleRB,
                                LinkedCell<T> const* const* LC,
                                ContactForceModel<T> const* const* CF,
                                int* result ) final;
                                        
        /** @brief Detects collision between components and computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param LC linked cell
        @param CF array of all contact force models */
        void detectCollisionAndComputeContactForces( 
                                    RigidBody<T, T> const* const* particleRB,
                                    RigidBody<T, T> const* const* obstacleRB,
                                    LinkedCell<T> const* const* LC,
                                    ContactForceModel<T> const* const* CF,
                                    int* result ) final;

        /** @brief Adds external forces such as gravity
        @param particleRB array of rigid bodies for particles
        @param g gravity field */
        void addExternalForces( RigidBody<T, T> const* const* particleRB,
                                Vector3<T> const& g ) final;

        /** @brief Updates the position and velocities of particles
        @param particleRB array of rigid bodies for particles
        @param TI time integration scheme */
        void moveParticles( RigidBody<T, T> const* const* particleRB,
                            TimeIntegrator<T> const* const* TI ) final;
        //@}
};


typedef ComponentManagerCPU<float> ComponentManagerCPU_d;
typedef ComponentManagerCPU<double> ComponentManagerCPU_f;


#endif