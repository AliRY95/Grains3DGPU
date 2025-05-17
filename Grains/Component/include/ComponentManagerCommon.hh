#ifndef _COMPONENTMANAGERCOMMON_HH_
#define _COMPONENTMANAGERCOMMON_HH_

#include "CollisionDetection.hh"
#include "ContactForceModel.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "GrainsParameters.hh"
#include "Kinematics.hh"
#include "LinkedCell.hh"
#include "Quaternion.hh"
#include "QuaternionMath.hh"
#include "RigidBody.hh"
#include "TimeIntegrator.hh"
#include "Torce.hh"
#include "Transform3.hh"
#include "Vector3.hh"

// =============================================================================
/** @brief ComponentManager common functions between host and device.

    This is a header-only file that contains common functions between CPU/GPU
    for the ComponentManager class. The functions are templated to allow
    for flexibility in usage. The functions are marked as inline to allow for 
    better optimization by the compiler.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name ComponentManager common functions between host and device */
//@{
/** @brief Adds gravity to the particle
@param particleRB the rigid body of the particle
@param rigidBodyId the rigid body ID of the particle
@param g the gravitational acceleration vector
@param torce the torce acting on the particle */
template <typename T, typename U>
__HOSTDEVICE__ static INLINE void
    addGravity(RigidBody<T, U> const* const* particleRB,
               const uint                    rigidBodyId,
               const Vector3<T>&             g,
               Torce<T>&                     torce)
{
    const RigidBody<T, U>* rb   = particleRB[rigidBodyId];
    const T                mass = rb->getMass();
    // Adding the gravitational force to the torce
    torce.addForce(mass * g);
}

// -----------------------------------------------------------------------------
/** @brief Moves a particle using the given time integration method
@param particleRB the rigid body of the particle
@param TI the time integrator
@param transform the transformation of the particle
@param kinematics the kinematics of the particle
@param torce the torce acting on the particle
@param rigidBodyId the rigid body ID of the particle
@param pId the ID of the particle */
template <typename T, typename U>
__HOSTDEVICE__ static INLINE void
    moveParticle(RigidBody<T, U> const* const*   particleRB,
                 TimeIntegrator<T> const* const* TI,
                 Transform3<T>&                  transform,
                 Kinematics<T>&                  kinematics,
                 Torce<T>&                       torce,
                 const uint                      rigidBodyId,
                 const uint                      pId)
{
    // Rigid body
    const RigidBody<T, U>* rb = particleRB[rigidBodyId];

    // First, we compute quaternion of orientation
    Quaternion<T> qRot(transform.getBasis());
    // Computing momentums in the space-fixed coordinate
    const Kinematics<T>& momentum
        = rb->computeMomentum(kinematics.getAngularComponent(), torce, qRot);
    // Reset torces
    torce.reset();
    // Finally, we move particles using the given time integration
    Vector3<T>    transMotion;
    Quaternion<T> rotMotion;
    (*TI)->Move(momentum, kinematics, transMotion, rotMotion);

    // Quaternion and rotation quaternion conjugate
    Vector3<T>    om = kinematics.getAngularComponent();
    Quaternion<T> qRotCon(qRot.conjugate());
    // Write torque in body-fixed coordinates system
    Vector3<T> angAcc(qRot.multToVector3(om * qRotCon));
    // and update the transformation of the component
    transform.updateTransform(transMotion, rotMotion);
    // TODO
    // qRot = qRotChange * qRot;
    // qRotChange = T( 0.5 ) * ( m_velocity[ pId ].getAngularComponent() * qRot );
}

// /** @brief Resets the particle configurations
// @param transform the transformation of the particle
// */
// template <typename T, typename U>
// __HOSTDEVICE__ static INLINE void resetParticle(
//     const Transform3<T>& transform,
//     Torce<T>& torce
// )
// {
//     torce.reset();
// }
#endif