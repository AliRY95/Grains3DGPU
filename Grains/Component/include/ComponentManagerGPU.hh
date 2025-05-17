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
    /** \brief Particles rigid body Id */
    uint* m_rigidBodyId;
    /** \brief Obstacles rigid body Id */
    uint* m_obstacleRigidBodyId;
    /** \brief Particles transformation */
    Transform3<T>* m_transform;
    /** \brief Obstacles transformation */
    Transform3<T>* m_obstacleTransform;
    /** \brief Particles velocities */
    Kinematics<T>* m_velocity;
    /** \brief Particles torce */
    Torce<T>* m_torce;
    /** \brief Particles Id */
    uint* m_particleId;
    /** \brief Particles cell hash */
    uint* m_particleCellHash;
    /** \brief cells hash start */
    uint* m_cellHashStart;
    /** \brief cells hash end */
    uint* m_cellHashEnd;
    /** \brief number of particles in manager (HOST VARIABLE) */
    uint m_nParticles;
    /** \brief number of obstacles in manager (HOST VARIABLE) */
    uint m_nObstacles;
    /** \brief number of cells in manager (HOST VARIABLE) */
    uint m_nCells;

    // bool* isActive; /**< array of components activity in the simulation */
    // bool* isObstacle; /**< array of components flag for being obstacle */
    //@}

public:
    /** @name Constructors */
    //@{
    /** @brief Constructor with the number of particles, obstacles, and
        cells. All other data members are set to default. */
    ComponentManagerGPU(uint nParticles, uint nObstacles, uint nCells);

    /** @brief Destructor */
    ~ComponentManagerGPU();
    //@}

    /** @name Get methods */
    //@{
    /** @brief Gets particles rigid body Id */
    std::vector<uint> getRigidBodyId() const final;

    /** @brief Gets obstacles rigid body Id */
    std::vector<uint> getRigidBodyIdObstacles() const final;

    /** @brief Gets particles transformation */
    std::vector<Transform3<T>> getTransform() const final;

    /** @brief Gets obstacles transformation */
    std::vector<Transform3<T>> getTransformObstacles() const final;

    /** @brief Gets particles velocities */
    std::vector<Kinematics<T>> getVelocity() const final;

    /** @brief Gets particles torces */
    std::vector<Torce<T>> getTorce() const final;

    /** @brief Gets the array of particles Ids */
    std::vector<uint> getParticleId() const final;

    /** @brief Gets the number of particles in manager */
    uint getNumberOfParticles() const final;

    /** @brief Gets the number of obstacles in manager */
    uint getNumberOfObstacles() const final;

    /** @brief Gets the number of cells in manager */
    uint getNumberOfCells() const final;

    // /** @brief Gets the array of components neighbor Id */
    // std::vector<uint> getNeighborsId() const;

    // /** @brief Gets the array of components neighbor count */
    // std::vector<uint> getNeighborsCount() const;
    //@}

    /** @name Set methods */
    //@{
    /** @brief Sets the array of particles rigid body Ids */
    void setRigidBodyId(std::vector<uint> const& id) final;

    /** @brief Sets the array of obstacles rigid body Ids */
    void setRigidBodyIdObstacles(std::vector<uint> const& id) final;

    /** @brief Sets particles transformations */
    void setTransform(std::vector<Transform3<T>> const& t) final;

    /** @brief Sets obstacles transformations */
    void setTransformObstacles(std::vector<Transform3<T>> const& t) final;

    /** @brief Sets particles velocities */
    void setVelocity(std::vector<Kinematics<T>> const& v) final;

    /** @brief Sets particles torces */
    void setTorce(std::vector<Torce<T>> const& t) final;

    /** @brief Sets the array of particles Ids */
    void setParticleId(std::vector<uint> const& id) final;

    // /** @brief Sets the array of components neighbor Id */
    // void setNeighborsId( std::vector<uint> const& id );

    // /** @brief Sets the array of components neighbor count */
    // void setNeighborsCount( std::vector<uint> const& count );
    //@}

    /** @name Methods */
    //@{
    // /** @brief Sorts particles based on a Z-curve */
    // void sortParticles();

    // /** @brief Creates a neighbor list */
    // void createNeighborList();

    /** @brief Updates links between particles and linked cell
        @param LC linked cell */
    void updateLinks(LinkedCell<T> const* const* LC) final;

    /** @brief Detects collision between particles and obstacles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param CF array of all contact force models */
    void detectCollisionAndComputeContactForcesObstacles(
        RigidBody<T, T> const* const*      particleRB,
        RigidBody<T, T> const* const*      obstacleRB,
        ContactForceModel<T> const* const* CF) final;

    /** @brief Detects collision between particles and particles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param LC linked cell
        @param CF array of all contact force models */
    void detectCollisionAndComputeContactForcesParticles(
        RigidBody<T, T> const* const*      particleRB,
        LinkedCell<T> const* const*        LC,
        ContactForceModel<T> const* const* CF) final;

    /** @brief Detects collision between components and computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param LC linked cell
        @param CF array of all contact force models */
    void detectCollisionAndComputeContactForces(
        RigidBody<T, T> const* const*      particleRB,
        RigidBody<T, T> const* const*      obstacleRB,
        LinkedCell<T> const* const*        LC,
        ContactForceModel<T> const* const* CF) final;

    /** @brief Adds external forces such as gravity
        @param particleRB array of rigid bodies for particles
        @param g gravity field */
    void addExternalForces(RigidBody<T, T> const* const* particleRB) final;

    /** @brief Updates the position and velocities of particles
        @param particleRB array of rigid bodies for particles
        @param TI time integration scheme */
    void moveParticles(RigidBody<T, T> const* const*   particleRB,
                       TimeIntegrator<T> const* const* TI) final;
    //@}
};

typedef ComponentManagerGPU<float>  ComponentManagerGPU_d;
typedef ComponentManagerGPU<double> ComponentManagerGPU_f;

#endif