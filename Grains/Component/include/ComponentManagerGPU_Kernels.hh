// TODO: CHANGE THE FORMAT FROM HH TO CUH LATER.
#ifndef _COMPONENTMANAGERGPU_KERNLES_HH_
#define _COMPONENTMANAGERGPU_KERNLES_HH_


#include "Vector3.hh"
#include "Transform3.hh"
#include "RigidBody.hh"
#include "LinkedCell.hh"
#include "ContactForceModel.hh"
#include "TimeIntegrator.hh"


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
void zeroOutArray_kernel( unsigned int* array,
                          unsigned int numElements );

/** @brief Returns the start Id for each hash value in cellStart
@param componentCellHash sorted array of cell hash values
@param numComponents number of components
@param cellStartAndEnd start and end indices as s1, e1, s2, e2, ... */
__GLOBAL__
void sortComponentsAndFindCellStart_kernel( 
                                        unsigned int const* componentCellHash,
                                        unsigned int numComponents,
                                        unsigned int* cellStart,
                                        unsigned int* cellEnd );

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
__GLOBAL__ 
void detectCollisionAndComputeContactForcesObstacles_kernel( 
                                    RigidBody<T, U> const* const* particleRB,
                                    RigidBody<T, U> const* const* obstacleRB,
                                    ContactForceModel<T> const* const* CF,
                                    unsigned int* rigidBodyId,
                                    Transform3<T> const* transform,
                                    Kinematics<T> const* velocity,
                                    Torce<T>* torce,
                                    unsigned int* obstacleRigidBodyId,
                                    Transform3<T> const* obstacleTransform,
                                    int nParticles,
                                    int nObstacles );
                                    
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
__GLOBAL__ 
void detectCollisionAndComputeContactForcesParticles_kernel( 
                                    RigidBody<T, U> const* const* particleRB,
                                    LinkedCell<T> const* const* LC,
                                    ContactForceModel<T> const* const* CF,
                                    unsigned int* rigidBodyId,
                                    Transform3<T> const* transform,
                                    Kinematics<T> const* velocity,
                                    Torce<T>* torce,
                                    unsigned int* particleId,
                                    unsigned int* particleCellHash,
                                    unsigned int* cellHashStart,
                                    unsigned int* cellHashEnd,
                                    int nParticles,
                                    int* result );

/** @brief Adds external forces such as gravity
@param particleRB array of rigid bodies for particles
@param rigidBodyId array of rigid body IDs for particles
@param torce array of particles torces
@param g the gravity field
@param nParticles number of particles */
template <typename T, typename U>
__GLOBAL__ 
void addExternalForces_kernel( RigidBody<T, U> const* const* particleRB,
                               unsigned int* rigidBodyId,
                               Torce<T>* torce,
                               T gX, T gY, T gZ,
                               int nParticles );
                           
/** @brief Updates the position and velocities of particles
@param particleRB array of rigid bodies for particles
@param TI time integrator scheme
@param rigidBodyId array of rigid body IDs for particles
@param transform array of particles transformations
@param velocity array of particles velocities
@param torce array of particles torces
@param nParticles number of particles */
template <typename T, typename U>
__GLOBAL__ 
void moveParticles_kernel( RigidBody<T, U> const* const* particleRB,
                           TimeIntegrator<T> const* const* TI,
                           unsigned int* rigidBodyId,
                           Transform3<T>* transform,
                           Kinematics<T>* velocity,
                           Torce<T>* torce,
                           int nParticles );
//@}


#endif