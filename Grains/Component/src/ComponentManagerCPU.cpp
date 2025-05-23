#include <algorithm>
#include <execution>
#include <omp.h>
#include <random>

#include "ComponentManagerCPU.hh"
#include "Quaternion.hh"
#include "QuaternionMath.hh"
#include "VectorMath.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU()
{
}

// -----------------------------------------------------------------------------
// Constructor with the number of particles, number of obstacles, and number of
// cells with all other data members initialized as default.
template <typename T>
ComponentManagerCPU<T>::ComponentManagerCPU(uint nParticles,
                                            uint nObstacles,
                                            uint nCells)
    : m_nParticles(nParticles)
    , m_nObstacles(nObstacles)
    , m_nCells(nCells)
{
    m_rigidBodyId.reserve(m_nParticles);
    m_obstacleRigidBodyId.reserve(m_nObstacles);
    m_transform.reserve(m_nParticles);
    m_obstacleTransform.reserve(m_nObstacles);
    m_velocity.reserve(m_nParticles);
    m_torce.reserve(m_nParticles);
    m_particleId.reserve(m_nParticles);
    m_particleCellHash.reserve(m_nParticles);
    m_cell.reserve(m_nCells);

    // Initializing the vectors for particles
    for(int i = 0; i < m_nParticles; i++)
    {
        m_rigidBodyId.push_back(0);
        m_transform.push_back(Transform3<T>());
        m_velocity.push_back(Kinematics<T>());
        m_torce.push_back(Torce<T>());
        m_particleId.push_back(i);
        // m_isActive.push_back( 0 );
    }

    // Initializing the vectors for obstacles
    for(int i = 0; i < m_nObstacles; i++)
    {
        m_obstacleRigidBodyId.push_back(0);
        m_obstacleTransform.push_back(Transform3<T>());
    }

    // Initializing the vectors for cells
    // The size of these vectors is one bigger that nCells because we reserve
    // cellID = 0.
    std::vector<uint> zeroVec(1, UINT_MAX);
    for(int i = 0; i < m_nCells; i++)
    {
        m_cell.push_back(zeroVec);
    }
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
ComponentManagerCPU<T>::~ComponentManagerCPU()
{
}

// -----------------------------------------------------------------------------
// Gets particles rigid body Ids
template <typename T>
std::vector<uint> ComponentManagerCPU<T>::getRigidBodyId() const
{
    return (m_rigidBodyId);
}

// -----------------------------------------------------------------------------
// Gets obstacles rigid body Ids
template <typename T>
std::vector<uint> ComponentManagerCPU<T>::getRigidBodyIdObstacles() const
{
    return (m_obstacleRigidBodyId);
}

// -----------------------------------------------------------------------------
// Gets particles transformations
template <typename T>
std::vector<Transform3<T>> ComponentManagerCPU<T>::getTransform() const
{
    return (m_transform);
}

// -----------------------------------------------------------------------------
// Gets obstacles transformations
template <typename T>
std::vector<Transform3<T>> ComponentManagerCPU<T>::getTransformObstacles() const
{
    return (m_obstacleTransform);
}

// -----------------------------------------------------------------------------
// Gets particles velocities
template <typename T>
std::vector<Kinematics<T>> ComponentManagerCPU<T>::getVelocity() const
{
    return (m_velocity);
}

// -----------------------------------------------------------------------------
// Gets particles torces
template <typename T>
std::vector<Torce<T>> ComponentManagerCPU<T>::getTorce() const
{
    return (m_torce);
}

// -----------------------------------------------------------------------------
// Gets the array of particle Ids
template <typename T>
std::vector<uint> ComponentManagerCPU<T>::getParticleId() const
{
    return (m_particleId);
}

// -----------------------------------------------------------------------------
// Gets the number of particles in manager
template <typename T>
uint ComponentManagerCPU<T>::getNumberOfParticles() const
{
    return (m_nParticles);
}

// -----------------------------------------------------------------------------
// Gets the number of obstacles in manager
template <typename T>
uint ComponentManagerCPU<T>::getNumberOfObstacles() const
{
    return (m_nObstacles);
}

// -----------------------------------------------------------------------------
// Gets the number of cells in manager
template <typename T>
uint ComponentManagerCPU<T>::getNumberOfCells() const
{
    return (m_nCells);
}

// -----------------------------------------------------------------------------
// Sets the array of particles rigid body Ids
template <typename T>
void ComponentManagerCPU<T>::setRigidBodyId(std::vector<uint> const& id)
{
    m_rigidBodyId = id;
}

// -----------------------------------------------------------------------------
// Sets the array of obstacles rigid body Ids
template <typename T>
void ComponentManagerCPU<T>::setRigidBodyIdObstacles(
    std::vector<uint> const& id)
{
    m_obstacleRigidBodyId = id;
}

// -----------------------------------------------------------------------------
// Sets particles transformations
template <typename T>
void ComponentManagerCPU<T>::setTransform(std::vector<Transform3<T>> const& t)
{
    m_transform = t;
}

// -----------------------------------------------------------------------------
// Sets obstacles transformations
template <typename T>
void ComponentManagerCPU<T>::setTransformObstacles(
    std::vector<Transform3<T>> const& t)
{
    m_obstacleTransform = t;
}

// -----------------------------------------------------------------------------
// Sets particles velocities
template <typename T>
void ComponentManagerCPU<T>::setVelocity(std::vector<Kinematics<T>> const& v)
{
    m_velocity = v;
}

// -----------------------------------------------------------------------------
// Sets particles torces
template <typename T>
void ComponentManagerCPU<T>::setTorce(std::vector<Torce<T>> const& t)
{
    m_torce = t;
}

// -----------------------------------------------------------------------------
// Sets the array of particle Ids
template <typename T>
void ComponentManagerCPU<T>::setParticleId(std::vector<uint> const& id)
{
    m_particleId = id;
}

// -----------------------------------------------------------------------------
// Initializes the RigidBody IDs and transformations of the obstacles
template <typename T>
void ComponentManagerCPU<T>::initializeObstacles(
    std::vector<Transform3<T>> initTr)
{
    // Making sure that we have data for all obstacles and the number of initial
    // TR matches the number of RBs
    assert(initTr.size() == m_nObstacles);

    // Assigning
    for(int i = 0; i < m_nObstacles; i++)
    {
        // m_rigidBodyId
        m_obstacleRigidBodyId[i] = i;

        // m_transform
        m_obstacleTransform[i] = initTr[i];
    }
}

// -----------------------------------------------------------------------------
// Initializes the RigidBody IDs and transformations of the particles
template <typename T>
void ComponentManagerCPU<T>::initializeParticles(
    std::vector<Transform3<T>> initTr)
{
    // Making sure that we have data for all particles and the number of initial
    // TR matches the number of RBs
    assert(initTr.size() == m_nParticles);

    // Assigning
    for(uint i = 0; i < m_nParticles; i++)
    {
        // m_rigidBodyId
        m_rigidBodyId[i] = i;

        // m_transform
        m_transform[i] = initTr[i];
    }
}

// -----------------------------------------------------------------------------
// Inserts particles according to a given insertion policy
template <typename T>
void ComponentManagerCPU<T>::insertParticles(Insertion<T>* ins)
{
    std::pair<Transform3<T>, Kinematics<T>> insData;
    // Inserting particles
    for(int i = 0; i < m_nParticles; i++)
    {
        // Fetching insertion data from ins
        insData = ins->fetchInsertionData();

        // m_transform
        m_transform[i].composeLeftByRotation(insData.first);
        m_transform[i].setOrigin(insData.first.getOrigin());

        // m_velocity
        m_velocity[i] = insData.second;
    }
}

// -----------------------------------------------------------------------------
// Updates links between components and linked cell
template <typename T>
void ComponentManagerCPU<T>::updateLinks(LinkedCell<T> const* const* LC)
{
    // Reset
    for(int i = 0; i < m_nCells + 1; i++)
        m_cell[i].clear();

    // Updating m_particleCellHash according to the linkedCell. That is,
    // assigning a hash value to each particle based on the cell it belongs to.
    (*LC)->computeLinearLinkedCellHashCPU(m_transform,
                                          m_nParticles,
                                          m_particleCellHash);

    // Update cells
    for(int i = 0; i < m_nParticles; i++)
    {
        uint cellId = m_particleCellHash[i];
        m_cell[cellId].push_back(i);
    }
}

// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and obstacles
template <typename T>
void ComponentManagerCPU<T>::detectCollisionAndComputeContactForcesObstacles(
    RigidBody<T, T> const* const*      particleRB,
    RigidBody<T, T> const* const*      obstacleRB,
    ContactForceModel<T> const* const* CF)
{
    // Loop over all particles
    for(int pId = 0; pId < m_nParticles; pId++)
    {
        // Parameters of the particle
        const RigidBody<T, T>& rbA   = *(particleRB[m_rigidBodyId[pId]]);
        const Transform3<T>&   trA   = m_transform[pId];
        T                      massA = rbA.getMass();
        uint                   matA  = rbA.getMaterial();

        // Loop over all obstacles
        for(int oId = 0; oId < m_nObstacles; oId++)
        {
            RigidBody<T, T> const& rbB
                = *(obstacleRB[m_obstacleRigidBodyId[oId]]);
            const Transform3<T>& trB = m_obstacleTransform[oId];
            ContactInfo<T> ci = closestPointsRigidBodies(rbA, rbB, trA, trB);
            if(ci.getOverlapDistance() < T(0))
            {
                // CF ID given materialIDs
                uint contactForceID
                    = ContactForceModelBuilderFactory<T>::computeHash(
                        matA,
                        rbB.getMaterial());
                // velocities of the particles
                Kinematics<T> v1(m_velocity[pId]);
                // Kinematics<T> v2( m_velocity[ oId ] );
                Kinematics<T> v2;
                // geometric point of contact
                Vector3<T> contactPt(ci.getContactPoint());
                // relative velocity at contact point
                Vector3<T> relVel(v1.kinematicsAtPoint(contactPt)
                                  - v2.kinematicsAtPoint(contactPt));
                // relative angular velocity
                Vector3<T> relAngVel(v1.getAngularComponent()
                                     - v2.getAngularComponent());
                CF[contactForceID]->computeForces(ci,
                                                  relVel,
                                                  relAngVel,
                                                  massA,
                                                  rbB.getMass(),
                                                  trA.getOrigin(),
                                                  m_torce[pId]);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and particles
template <typename T>
void ComponentManagerCPU<T>::detectCollisionAndComputeContactForcesParticles(
    RigidBody<T, T> const* const*      particleRB,
    LinkedCell<T> const* const*        LC,
    ContactForceModel<T> const* const* CF,
    int*                               result)
{
    // Loop over all particles
    // #pragma omp parallel for
    for(int pId = 0; pId < m_nParticles; pId++)
    {
        // Parameters of the primary particle
        const uint             particleId = m_particleId[pId];
        const uint             cellHash   = m_particleCellHash[pId];
        const RigidBody<T, T>& rbA   = *(particleRB[m_rigidBodyId[particleId]]);
        const Transform3<T>&   trA   = m_transform[particleId];
        T                      massA = rbA.getMass();
        uint                   matA  = rbA.getMaterial();
        const uint*            neighborsList = (*LC)->getNeighbors(cellHash);
        // Loop over all neighboring particles
        for(int i = 0; i < 27; ++i)
        {
            // Get the neighboring cell hash
            uint neighborCellHash = neighborsList[i];
            // Check if the neighboring cell is valid
            if(neighborCellHash == UINT_MAX)
                continue;
            for(auto id : m_cell[neighborCellHash])
            {
                const uint secondaryId = id;
                // To skip self-collision
                if(secondaryId == particleId)
                    continue;
                const RigidBody<T, T>& rbB
                    = *(particleRB[m_rigidBodyId[secondaryId]]);
                const Transform3<T>& trB = m_transform[secondaryId];
                ContactInfo<T>       ci
                    = closestPointsRigidBodies(rbA, rbB, trA, trB);
                if(ci.getOverlapDistance() < T(0))
                {
                    // CF ID given materialIDs
                    uint contactForceID
                        = ContactForceModelBuilderFactory<T>::computeHash(
                            matA,
                            rbB.getMaterial());
                    // velocities of the particles
                    Kinematics<T> v1(m_velocity[particleId]);
                    Kinematics<T> v2(m_velocity[secondaryId]);
                    // geometric point of contact
                    Vector3<T> contactPt(ci.getContactPoint());
                    // relative velocity at contact point
                    Vector3<T> relVel(v1.kinematicsAtPoint(contactPt)
                                      - v2.kinematicsAtPoint(contactPt));
                    // relative angular velocity
                    Vector3<T> relAngVel(v1.getAngularComponent()
                                         - v2.getAngularComponent());
                    CF[contactForceID]->computeForces(ci,
                                                      relVel,
                                                      relAngVel,
                                                      massA,
                                                      rbB.getMass(),
                                                      trA.getOrigin(),
                                                      m_torce[particleId]);
                }
                result[particleId] += (ci.getOverlapDistance() < T(0));
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Detects collision and computes forces between all components
template <typename T>
void ComponentManagerCPU<T>::detectCollisionAndComputeContactForces(
    RigidBody<T, T> const* const*      particleRB,
    RigidBody<T, T> const* const*      obstacleRB,
    LinkedCell<T> const* const*        LC,
    ContactForceModel<T> const* const* CF,
    int*                               result)
{
    // Updates links between components and linked cell
    updateLinks(LC);

    // Particle-particle interactions
    detectCollisionAndComputeContactForcesParticles(particleRB, LC, CF, result);

    // Particle-obstacle interactions
    detectCollisionAndComputeContactForcesObstacles(particleRB, obstacleRB, CF);
}

// -----------------------------------------------------------------------------
// Adds external forces such as gravity
template <typename T>
void ComponentManagerCPU<T>::addExternalForces(
    RigidBody<T, T> const* const* particleRB, const Vector3<T>& g)
{
    // #pragma omp parallel for
    for(int pId = 0; pId < m_nParticles; pId++)
    {
        // Parameters of the primary particle
        const RigidBody<T, T>& rb   = *(particleRB[m_rigidBodyId[pId]]);
        T                      mass = rb.getMass();
        // Adding the gravitational force to the torce
        m_torce[pId].addForce(mass * g);
    }
}

// -----------------------------------------------------------------------------
// Updates the position and velocities of particles
template <typename T>
void ComponentManagerCPU<T>::moveParticles(
    RigidBody<T, T> const* const*   particleRB,
    TimeIntegrator<T> const* const* TI)
{
    // #pragma omp parallel for
    for(int pId = 0; pId < m_nParticles; pId++)
    {
        // Rigid body
        const RigidBody<T, T>* rb = particleRB[m_rigidBodyId[pId]];

        // First, we compute quaternion of orientation
        Quaternion<T> qRot(m_transform[pId].getBasis());
        // Computing momentums in the space-fixed coordinate
        const Kinematics<T>& momentum
            = rb->computeMomentum(m_velocity[pId].getAngularComponent(),
                                  m_torce[pId],
                                  qRot);
        // Reset torces
        m_torce[pId].reset();
        // Finally, we move particles using the given time integration
        Vector3<T>    transMotion;
        Quaternion<T> rotMotion;
        (*TI)->Move(momentum, m_velocity[pId], transMotion, rotMotion);

        // Quaternion and rotation quaternion conjugate
        Vector3<T>    om = m_velocity[pId].getAngularComponent();
        Quaternion<T> qRotCon(qRot.conjugate());
        // Write torque in body-fixed coordinates system
        Vector3<T> angAcc(qRot.multToVector3(om * qRotCon));
        // and update the transformation of the component
        m_transform[pId].updateTransform(transMotion, rotMotion);
        // TODO
        // qRot = qRotChange * qRot;
        // qRotChange = T( 0.5 ) * ( m_velocity[ pId ].getAngularComponent() * qRot );
    }
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class ComponentManagerCPU<float>;
template class ComponentManagerCPU<double>;