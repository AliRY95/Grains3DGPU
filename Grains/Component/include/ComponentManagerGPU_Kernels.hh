// TODO: CHANGE THE FORMAT FROM HH TO CUH LATER.
#ifndef _COMPONENTMANAGERGPU_KERNLES_HH_
#define _COMPONENTMANAGERGPU_KERNLES_HH_

#include "ContactForceModel.hh"
#include "LinkedCell.hh"
#include "RigidBody.hh"
#include "TimeIntegrator.hh"
#include "Transform3.hh"
#include "Vector3.hh"

// =============================================================================
/** @brief The header for GPU kernels used in the ComponentManagerGPU class.

    Various GPU kernels used in the ComponentManagerGPU class.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name ComponentManagerGPU_Kernels : External methods */
//@{
/** @brief Zeros out the array
@param array array to be zero-ed out
@param numElements number of elements in the array */
__GLOBAL__
void zeroOutArray_kernel(uint* array, uint numElements);

/** @brief Returns the start Id for each hash value in cellStart
@param componentCellHash sorted array of cell hash values
@param numComponents number of components
@param cellStartAndEnd start and end indices as s1, e1, s2, e2, ... */
__GLOBAL__
void sortComponentsAndFindCellStart_kernel(uint const* componentCellHash,
                                           uint        numComponents,
                                           uint*       cellStart,
                                           uint*       cellEnd);

/** @brief Detects collision between particles and obstacles and computes forces
@param particleRB array of rigid bodies for particles
@param obstacleRB array of rigid bodies for obstacles
@param CF array of all contact force models
@param rigidBodyId array of rigid body IDs for particles
@param transform array of particles transformations
@param velocity array of particles velocities
@param torce array of particles torces
@param obstacleTransform array of obstacles transformations
@param nParticles number of particles
@param nObstacles number of obstacles */
template <typename T, typename U>
__GLOBAL__ void detectCollisionAndComputeContactForcesObstacles_kernel(
    RigidBody<T, U> const* const*      particleRB,
    RigidBody<T, U> const* const*      obstacleRB,
    ContactForceModel<T> const* const* CF,
    uint*                              rigidBodyId,
    Transform3<T> const*               transform,
    Kinematics<T> const*               velocity,
    Torce<T>*                          torce,
    uint*                              obstacleRigidBodyId,
    Transform3<T> const*               obstacleTransform,
    int                                nParticles,
    int                                nObstacles);

/** @brief Detects collision between particles and particles and computes forces
@param particleRB array of rigid bodies for particles
@param LC linked cell
@param CF array of all contact force models
@param rigidBodyId array of rigid body IDs for particles
@param transform array of particles transformations
@param velocity array of particles velocities
@param torce array of particles torces
@param particleId array of particles ids
@param particleCellHash array of particles cell hashes
@param cellHashStart array of cells starting index
@param cellHashEnd array of cells ending index
@param nParticles number of particles */
template <typename T, typename U>
__GLOBAL__ void detectCollisionAndComputeContactForcesParticles_kernel(
    RigidBody<T, U> const* const*      particleRB,
    LinkedCell<T> const* const*        LC,
    ContactForceModel<T> const* const* CF,
    uint*                              rigidBodyId,
    Transform3<T> const*               transform,
    Kinematics<T> const*               velocity,
    Torce<T>*                          torce,
    uint*                              particleId,
    uint*                              particleCellHash,
    uint*                              cellHashStart,
    uint*                              cellHashEnd,
    int                                nParticles);

/** @brief Adds external forces such as gravity
@param particleRB array of rigid bodies for particles
@param rigidBodyId array of rigid body IDs for particles
@param g the gravity field
@param torce array of particles torces
@param nParticles number of particles */
template <typename T, typename U>
__GLOBAL__ void
    addExternalForces_kernel(RigidBody<T, U> const* const* particleRB,
                             const uint*                   rigidBodyId,
                             const T                       gX,
                             const T                       gY,
                             const T                       gZ,
                             Torce<T>*                     torce,
                             const uint                    nParticles);

/** @brief Updates the position and velocities of particles
@param particleRB array of rigid bodies for particles
@param TI time integrator scheme
@param rigidBodyId array of rigid body IDs for particles
@param transform array of particles transformations
@param velocity array of particles velocities
@param torce array of particles torces
@param nParticles number of particles */
template <typename T, typename U>
__GLOBAL__ void moveParticles_kernel(RigidBody<T, U> const* const*   particleRB,
                                     TimeIntegrator<T> const* const* TI,
                                     uint*          rigidBodyId,
                                     Transform3<T>* transform,
                                     Kinematics<T>* velocity,
                                     Torce<T>*      torce,
                                     int            nParticles);
//@}

#endif