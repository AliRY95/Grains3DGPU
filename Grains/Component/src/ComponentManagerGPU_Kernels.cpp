// TODO: CHANGE THE FORMAT FROM CPP TO CU LATER.
#include <cooperative_groups.h>

#include "ComponentManagerGPU_Kernels.hh"
#include "CollisionDetection.hh"
#include "LinkedCell.hh"
#include "LinkedCellGPUWrapper.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Zeros out the array
__GLOBAL__ 
void zeroOutArray_kernel( unsigned int* array,
                          unsigned int numElements )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if ( tid >= numElements )
        return;

    array[tid] = 0;
}




// -----------------------------------------------------------------------------
// Returns the start Id for each hash value in cellStart
__GLOBAL__ 
void sortComponentsAndFindCellStart_kernel( 
                                     unsigned int const* componentCellHash,
                                     unsigned int numComponents,
                                     unsigned int* cellStart,
                                     unsigned int* cellEnd )
{
    // Handle to thread block group
    cooperative_groups::thread_block cta = 
                                        cooperative_groups::this_thread_block();
    extern __shared__ unsigned int sharedHash[];  // blockSize + 1 elements
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    unsigned int hash;
    if ( tid < numComponents )
    {
        hash = componentCellHash[ tid ];
        // Load hash data into shared memory so that we can look at neighboring 
        // component's hash value without loading two hash values per thread
        sharedHash[ threadIdx.x + 1 ] = hash;
        // first thread in block must load neighboring component hash as well
        if ( tid > 0 && threadIdx.x == 0 ) 
            sharedHash[ 0 ] = componentCellHash[ tid - 1 ];
    }
    cooperative_groups::sync( cta );

    if ( tid < numComponents )
    {
        // If this component has a different cell hash value to the previous
        // component then it must be the first component in the cell.
        // As it isn't the first component, it must also be the end of the 
        // previous component's cell.

        if ( tid == 0 || hash != sharedHash[threadIdx.x] )
        {
            cellStart[ hash ] = tid;
            if ( tid > 0 )
                cellEnd[ sharedHash[ threadIdx.x ] ] = tid; // excluding
        }
        if ( tid == numComponents - 1 )
            cellEnd[ hash ] = tid + 1;
    }
    // // Now use the sorted index to reorder the pos and vel data
    // uint sortedIndex = gridParticleIndex[index];
    // float4 pos = oldPos[sortedIndex];
    // float4 vel = oldVel[sortedIndex];

    // sortedPos[index] = pos;
    // sortedVel[index] = vel;
}




// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and components
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
                                    int nObstacles )
{
    unsigned int pId = blockIdx.x * blockDim.x + threadIdx.x;

    if ( pId >= nParticles )
        return;
    
    RigidBody<T, U> const& rbA = *( particleRB[ rigidBodyId[ pId ] ] );
    Transform3<T> const& trA = transform[ pId ];
    T massA = rbA.getMass();
    unsigned int matA = rbA.getMaterial();

    for ( int oId = 0; oId < nObstacles; oId++ )
    {
        RigidBody<T, U> const& rbB = 
                                *( obstacleRB[ obstacleRigidBodyId[ oId ] ] );
        Transform3<T> const& trB = obstacleTransform[ oId ];
        ContactInfo<T> ci = closestPointsRigidBodies( rbA,
                                                      rbB,
                                                      trA, 
                                                      trB );
        if ( ci.getOverlapDistance() < T( 0 ) )
        {
            // CF ID given materialIDs
            unsigned int contactForceID = 
            ContactForceModelBuilderFactory<T>::computeHash( 
                                                            matA, 
                                                            rbB.getMaterial() );
            // velocities of the particles
            Kinematics<T> v1( velocity[ pId ] );
            // Kinematics<T> v2( m_velocity[ oId ] );
            Kinematics<T> v2;
            // geometric point of contact
            Vector3<T> contactPt( ci.getContactPoint() );
            // relative velocity at contact point
            Vector3<T> relVel( v1.kinematicsAtPoint( contactPt ) -
                               v2.kinematicsAtPoint( contactPt ) );
            // relative angular velocity
            Vector3<T> relAngVel( v1.getAngularComponent() - 
                                  v2.getAngularComponent() );
            CF[contactForceID]->computeForces( ci, 
                                               relVel,
                                               relAngVel,
                                               massA,
                                               rbB.getMass(),
                                               trA.getOrigin(),
                                               torce[ pId ] );
        }
    }
}




// -----------------------------------------------------------------------------
// Detects collision and computes forces between particles and particles
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
                                   int* result )
{
    unsigned int pId = blockIdx.x * blockDim.x + threadIdx.x;

    if ( pId >= nParticles )
        return;
    
    unsigned int const primaryId = particleId[ pId ];
    unsigned int const cellHash = particleCellHash[ pId ];
    RigidBody<T, U> const& rbA = *( particleRB[ rigidBodyId[ primaryId ] ] );
    Transform3<T> const& trA = transform[ primaryId ];
    T massA = rbA.getMass();
    unsigned int matA = rbA.getMaterial();

    for ( int k = -1; k < 2; k++ ) {
    for ( int j = -1; j < 2; j++ ) {
    for ( int i = -1; i < 2; i++ ) {
        int neighboringCellHash =
        (*LC)->computeNeighboringCellLinearHash( cellHash, i, j, k );
        int startId = cellHashStart[ neighboringCellHash ];
        int endId = cellHashEnd[ neighboringCellHash ];
        for ( int id = startId; id < endId; id++ )
        {
            int secondaryId = particleId[ id ];
            // To skip the self-collision
            if ( secondaryId == primaryId )
                continue;
            RigidBody<T, U> const& rbB = 
                                *( particleRB[ rigidBodyId[ secondaryId ] ] );
            Transform3<T> const& trB = transform[ secondaryId ];
            // result[compId] += intersectRigidBodies( rigidBodyA,
            //                                      rigidBodyA,
            //                                      transformA, 
            //                                      transformB );
            ContactInfo<T> ci = closestPointsRigidBodies( rbA,
                                                          rbB,
                                                          trA, 
                                                          trB );
            if ( ci.getOverlapDistance() < T( 0 ) )
            {
                // CF ID given materialIDs
                unsigned int contactForceID = 
                ContactForceModelBuilderFactory<T>::computeHash( 
                                                            matA, 
                                                            rbB.getMaterial() );
                // velocities of the particles
                Kinematics<T> v1( velocity[ primaryId ] );
                Kinematics<T> v2( velocity[ secondaryId ] );
                // relative velocity at contact point
                Vector3<T> relVel( 
                                v1.kinematicsAtPoint( ci.getContactPoint() ) -
                                v2.kinematicsAtPoint( ci.getContactPoint() ) );
                // relative angular velocity
                Vector3<T> relAngVel( v1.getAngularComponent() - 
                                      v2.getAngularComponent() );
                CF[contactForceID]->computeForces( ci, 
                                                   relVel,
                                                   relAngVel,
                                                   massA,
                                                   rbB.getMass(),
                                                   trA.getOrigin(),
                                                   torce[ primaryId ] );
            }
            result[ primaryId ] += ( ci.getOverlapDistance() < T( 0 ) );
        }
    } } }
}




// -----------------------------------------------------------------------------
// Adds external forces such as gravity
template <typename T, typename U>
__GLOBAL__ 
void addExternalForces_kernel( RigidBody<T, U> const* const* particleRB,
                               unsigned int* rigidBodyId,
                               Torce<T>* torce,
                               T gX, T gY, T gZ,
                               int nParticles )
{
    unsigned int pId = blockIdx.x * blockDim.x + threadIdx.x;

    if ( pId >= nParticles )
        return;
    
    // // Rigid body
    // RigidBody<T, U> const* rb = particleRB[ rigidBodyId[ pId ] ];
    // T mass = rb->getMass();
    // // T mass = particleRB[ rigidBodyId[ pId ] ]->getMass();

    // // Add to torce
    // torce[ pId ].addForce( mass * Vector3<T>( gX, gY, gZ ) );
}




// -----------------------------------------------------------------------------
// Updates the position and velocities of particles
// TODO: CLEAN -- A LOT OF THINGS
template <typename T, typename U>
__GLOBAL__ 
void moveParticles_kernel( RigidBody<T, U> const* const* RB,
                           TimeIntegrator<T> const* const* TI,
                           unsigned int* rigidBodyId,
                           Transform3<T>* transform,
                           Kinematics<T>* velocity,
                           Torce<T>* torce,
                           int nParticles )
{
    unsigned int pId = blockIdx.x * blockDim.x + threadIdx.x;

    if ( pId >= nParticles )
        return;
    
    // Rigid body
    RigidBody<T, U> const* rb = RB[ rigidBodyId[ pId ] ];

    T mass = rb->getMass();
    // Add to torce
    torce[ pId ].addForce( mass * Vector3<T>( 0, 0, -0.1 ) );


    // First, we compute quaternion of orientation
    Quaternion<T> qRot( transform[ pId ].getBasis() );
    // Next, we compute accelerations and reset torces
    Kinematics<T> const& momentum = rb->computeMomentum( 
                                        velocity[ pId ].getAngularComponent(),
                                        torce[ pId ], 
                                        qRot );
    torce[ pId ].reset();
    // Finally, we move particles using the given time integration
    Vector3<T> transMotion;
    Quaternion<T> rotMotion;
    (*TI)->Move( momentum, 
                 velocity[ pId ],
                 transMotion, 
                 rotMotion );
    // and update the transformation of the component
    transform[ pId ].updateTransform( transMotion, rotMotion );
    // TODO
    // qRot = qRotChange * qRot;
    // qRotChange = T( 0.5 ) * ( m_velocity[ pId ].getAngularComponent() * qRot );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T, U )                                                              \
template                                                                       \
__GLOBAL__                                                                     \
void detectCollisionAndComputeContactForcesObstacles_kernel(                   \
                                    RigidBody<T, U> const* const* particleRB,  \
                                    RigidBody<T, U> const* const* obstacleRB,  \
                                    ContactForceModel<T> const* const* CF,     \
                                    unsigned int* rigidBodyId,                 \
                                    Transform3<T> const* transform,            \
                                    Kinematics<T> const* velocity,             \
                                    Torce<T>* torce,                           \
                                    unsigned int* obstacleRigidBodyId,         \
                                    Transform3<T> const* obstacleTransform,    \
                                    int nParticles,                            \
                                    int nObstacles );                          \
                                                                               \
template                                                                       \
__GLOBAL__                                                                     \
void detectCollisionAndComputeContactForcesParticles_kernel(                   \
                                   RigidBody<T, U> const* const* particleRB,   \
                                   LinkedCell<T> const* const* LC,             \
                                   ContactForceModel<T> const* const* CF,      \
                                   unsigned int* rigidBodyId,                  \
                                   Transform3<T> const* transform,             \
                                   Kinematics<T> const* velocity,              \
                                   Torce<T>* torce,                            \
                                   unsigned int* particleId,                   \
                                   unsigned int* particleCellHash,             \
                                   unsigned int* cellHashStart,                \
                                   unsigned int* cellHashEnd,                  \
                                   int nParticles,                             \
                                   int* result );                              \
                                                                               \
template                                                                       \
__GLOBAL__                                                                     \
void addExternalForces_kernel( RigidBody<T, U> const* const* particleRB,       \
                               unsigned int* rigidBodyId,                      \
                               Torce<T>* torce,                                \
                               T gX, T gY, T gZ,                               \
                               int nParticles );                               \
                                                                               \
template                                                                       \
__GLOBAL__                                                                     \
void moveParticles_kernel( RigidBody<T, U> const* const* RB,                   \
                           TimeIntegrator<T> const* const* TI,                 \
                           unsigned int* rigidBodyId,                          \
                           Transform3<T>* transform,                           \
                           Kinematics<T>* velocity,                            \
                           Torce<T>* torce,                                    \
                           int nParticles );
X( float, float )
X( double, float )
X( double, double )
#undef X