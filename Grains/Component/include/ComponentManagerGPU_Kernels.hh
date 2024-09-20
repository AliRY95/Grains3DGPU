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

/** @brief Computes the collision forces for all given components using 
LinkedCell and a thread-per-particle policy
@param TODO */
template <typename T, typename U>
__GLOBAL__ 
void detectCollisionAndComputeContactForces_kernel( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, U> const* const* RB,
                                        ContactForceModel<T> const* const* CF,
                                        unsigned int* m_rigidBodyId,
                                        Transform3<T> const* m_transform,
                                        Kinematics<T> const* m_velocity,
                                        Torce<T>* m_torce,
                                        int* m_compId,
                                        unsigned int* m_componentCellHash,
                                        unsigned int* m_cellHashStart,
                                        unsigned int* m_cellHashEnd,
                                        int numComponents,
                                        T gx, T gy, T gz,
                                        int* result );

/** @brief Updates components transformation and velocities
@param TODO */
template <typename T, typename U>
__GLOBAL__ 
void moveComponents_kernel( RigidBody<T, U> const* const* RB,
                            TimeIntegrator<T> const* const* TI,
                            unsigned int* m_rigidBodyId,
                            Transform3<T>* m_transform,
                            Kinematics<T>* m_velocity,
                            Torce<T>* m_torce,
                            int numComponents );
//@}


#endif