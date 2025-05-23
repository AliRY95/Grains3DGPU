#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_

#include "CollisionDetection.hh"
#include "ContactForceModel.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "GrainsParameters.hh"
#include "Insertion.hh"
#include "Kinematics.hh"
#include "LinkedCell.hh"
#include "RigidBody.hh"
#include "TimeIntegrator.hh"
#include "Torce.hh"
#include "Transform3.hh"

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

    /** @name Get methods */
    //@{
    /** @brief Gets particles rigid body Ids */
    virtual std::vector<uint> getRigidBodyId() const = 0;

    /** @brief Gets obstacles rigid body Id */
    virtual std::vector<uint> getRigidBodyIdObstacles() const = 0;

    /** @brief Gets particles transformations */
    virtual std::vector<Transform3<T>> getTransform() const = 0;

    /** @brief Gets obstacles transformation */
    virtual std::vector<Transform3<T>> getTransformObstacles() const = 0;

    /** @brief Gets particles velocities */
    virtual std::vector<Kinematics<T>> getVelocity() const = 0;

    /** @brief Gets particles torces */
    virtual std::vector<Torce<T>> getTorce() const = 0;

    /** @brief Gets the array of particles Ids */
    virtual std::vector<uint> getParticleId() const = 0;

    /** @brief Gets the number of particles in manager */
    virtual uint getNumberOfParticles() const = 0;

    /** @brief Gets the number of obstacles in manager */
    virtual uint getNumberOfObstacles() const = 0;

    /** @brief Gets the number of cells in manager */
    virtual uint getNumberOfCells() const = 0;

    // /** @brief Gets the array of components neighbor Id */
    // std::vector<uint> getNeighborsId() const;

    // /** @brief Gets the array of components neighbor count */
    // std::vector<uint> getNeighborsCount() const;
    //@}

    /** @name Set methods */
    //@{
    /** @brief Sets the array of particles rigid body Ids */
    virtual void setRigidBodyId(std::vector<uint> const& id) = 0;

    /** @brief Sets the array of obstacles rigid body Ids */
    virtual void setRigidBodyIdObstacles(std::vector<uint> const& id) = 0;

    /** @brief Sets particles transformations */
    virtual void setTransform(std::vector<Transform3<T>> const& t) = 0;

    /** @brief Sets obstacles transformations */
    virtual void setTransformObstacles(std::vector<Transform3<T>> const& t) = 0;

    /** @brief Sets particles velocities */
    virtual void setVelocity(std::vector<Kinematics<T>> const& v) = 0;

    /** @brief Sets particles torces */
    virtual void setTorce(std::vector<Torce<T>> const& t) = 0;

    /** @brief Sets the array of particles Ids */
    virtual void setParticleId(std::vector<uint> const& id) = 0;

    // /** @brief Sets the array of components neighbor Id */
    // void setNeighborsId( std::vector<uint> const& id );

    // /** @brief Sets the array of components neighbor count */
    // void setNeighborsCount( std::vector<uint> const& count );
    //@}

    /** @name Methods */
    //@{
    /** @brief Copies data from another ComponentManager object.
        @param cm component manager on device */
    void copy(ComponentManager<T> const* cm);

    /** @brief Initializes transformations for obstacles in the simulation
        @param initTr initial transformation of obstacles */
    virtual void initializeObstacles(std::vector<Transform3<T>> initTr);

    /** @brief Initializes transformations for particles in the simulation
        @param initTr initial transformation of particles */
    virtual void initializeParticles(std::vector<Transform3<T>> initTr);

    /** @brief Inserts particles according to a given insertion policy
        @param ins insertion policy */
    virtual void insertParticles(Insertion<T>* ins);

    // /** @brief Sorts particles based on a Z-curve */
    // void sortParticles();

    // /** @brief Creates a neighbor list */
    // void createNeighborList();

    /** @brief Updates links between particles and linked cell
        @param LC linked cell */
    virtual void updateLinks(LinkedCell<T> const* const* LC) = 0;

    /** @brief Detects collision between particles and obstacles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param CF array of all contact force models */
    virtual void detectCollisionAndComputeContactForcesObstacles(
        RigidBody<T, T> const* const*      particleRB,
        RigidBody<T, T> const* const*      obstacleRB,
        ContactForceModel<T> const* const* CF)
        = 0;

    /** @brief Detects collision between particles and particles and 
        // computes forces
        @param particleRB array of rigid bodies for particles
        @param LC linked cell
        @param CF array of all contact force models */
    virtual void detectCollisionAndComputeContactForcesParticles(
        RigidBody<T, T> const* const*      particleRB,
        LinkedCell<T> const* const*        LC,
        ContactForceModel<T> const* const* CF,
        int*                               result)
        = 0;

    /** @brief Detects collision between components and computes forces
        @param particleRB array of rigid bodies for particles
        @param obstacleRB array of rigid bodies for obstacles
        @param LC linked cell
        @param CF array of all contact force models */
    virtual void detectCollisionAndComputeContactForces(
        RigidBody<T, T> const* const*      particleRB,
        RigidBody<T, T> const* const*      obstacleRB,
        LinkedCell<T> const* const*        LC,
        ContactForceModel<T> const* const* CF,
        int*                               result)
        = 0;

    /** @brief Adds external forces such as gravity
        @param particleRB array of rigid bodies for particles
        @param g gravity field */
    virtual void addExternalForces(RigidBody<T, T> const* const* particleRB,
                                   const Vector3<T>&             g)
        = 0;

    /** @brief Updates the position and velocities of particles
        @param particleRB array of rigid bodies for particles
        @param TI time integration scheme */
    virtual void moveParticles(RigidBody<T, T> const* const*   particleRB,
                               TimeIntegrator<T> const* const* TI)
        = 0;
    //@}
};

typedef ComponentManager<float>  ComponentManager_d;
typedef ComponentManager<double> ComponentManager_f;

#endif