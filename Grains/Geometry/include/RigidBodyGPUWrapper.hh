#ifndef _RIGIDBODYGPUWRAPPER_HH_
#define _RIGIDBODYGPUWRAPPER_HH_


#include "RigidBody.hh"


// =============================================================================
/** @brief Header for RigidBodyGPUWrapper.

    RigidBody objects must be instantiated on device, if we want to use them on
    device. This is because of the runtime polymorphism for the class Convex 
    which is a data member of the RigidBody class.
    Herein, we implement a device kernel to instantiate the RigidBody objects
    and a wrapper to mimic a host side array of RigidBody objects on device.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name RigidBodyGPUWrapper : External methods */
//@{
/** @brief Constructs an array of rigidBody objects d_rb on device identical to 
h_rb which is an array of RigidBody objects on host
@param h_rb host-side rigid body arrays
@param d_rb host-side rigid body arrays
@param numRigidBodies number of rigid bodies in the array */
template <typename T, typename U>
__HOST__
void RigidBodyCopyHostToDevice( RigidBody<T, U>** h_rb,
                                RigidBody<T, U>** d_rb,
                                int numRigidBodies );
//@}


#endif
