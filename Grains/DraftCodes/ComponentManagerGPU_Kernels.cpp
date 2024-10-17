// TODO: CHANGE THE FORMAT FROM CPP TO CU LATER.
#include <cooperative_groups.h>

#include "ComponentManagerGPU_Kernels.hh"
#include "CollisionDetection.hh"
#include "LinkedCell.hh"
#include "LinkedCellGPUWrapper.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "GrainsParameters.hh"


// -----------------------------------------------------------------------------
// Zeros out the array
__GLOBAL__ 
void zeroOutArray( unsigned int* array,
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
// N-squared collision detection kernel using a thread-per-particle policy
// TODO: REMOVE LATER
template <typename T, typename U>
__GLOBAL__ 
void collisionDetectionN2( RigidBody<T, U> const* const* a,
                           Transform3<T> const* tr3d,
                           int numComponents,
                           int* result )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    RigidBody<T, U> const& AA = **a;
    Transform3<T> const& trA = tr3d[tid];
    ContactInfo<T> ci;
    for ( int j = 0; j < numComponents; j++ )
    {
        // result[tid] += intersectRigidBodies( AA, AA, trA, tr3d[j] );
        ci = closestPointsRigidBodies( AA, AA, trA, tr3d[j] );
        result[j] += ( ci.getOverlapDistance() < 0. );
    }
}




// -----------------------------------------------------------------------------
// N-squared collision detection kernel with relative transformation
// TODO: REMOVE LATER
template <typename T, typename U>
__GLOBAL__ 
void collisionDetectionRelativeN2( RigidBody<T, U> const* const* a,
                                   Transform3<T> const* tr3d,
                                   int numComponents,
                                   int* result )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    RigidBody<T, U> const& AA = **a;
    Transform3<T> const& trA = tr3d[tid];
    Transform3<T> trB2A;
    for ( int j = 0; j < numComponents; j++ )
    {
        trB2A = tr3d[j];
        trB2A.relativeToTransform( trA );
        result[tid] += intersectRigidBodies( AA, AA, trB2A );
    }
}




// -----------------------------------------------------------------------------
// LinkedCell collision detection kernel 
// TODO: CLEAN -- A LOT OF THINGS
template <typename T, typename U>
__GLOBAL__ 
void detectCollisionAndComputeContactForces_kernel( 
                                   LinkedCell<T> const* const* LC,
                                   RigidBody<T, U> const* const* RB,
                                   ContactForceModel<T> const* const* CF,
                                   unsigned int* m_rigidBodyId,
                                   Transform3<T> const* tr3d,
                                   Torce<T>* m_torce,
                                   int* m_compId,
                                   unsigned int* m_componentCellHash,
                                   unsigned int* m_cellHashStart,
                                   unsigned int* m_cellHashEnd,
                                   int numComponents,
                                   int* result )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if ( tid >= numComponents )
        return;
    
    unsigned int const compId = m_compId[ tid ];
    unsigned int const cellHash = m_componentCellHash[ tid ];
    RigidBody<T, U> const& rbA = *( RB[ m_rigidBodyId[ compId ] ] );
    Transform3<T> const& trA = tr3d[ compId ];
    T massA = rbA.getMass();
    unsigned int matA = rbA.getMaterial();

    for ( int k = -1; k < 2; k++ ) {
    for ( int j = -1; j < 2; j++ ) {
    for ( int i = -1; i < 2; i++ ) {
        int neighboringCellHash =
        (*LC)->computeNeighboringCellLinearHash( cellHash, i, j, k );
        int startId = m_cellHashStart[ neighboringCellHash ];
        int endId = m_cellHashEnd[ neighboringCellHash ];
        for ( int id = startId; id < endId; id++ )
        {
            int secondaryId = m_compId[ id ];
            // To skip the self-collision
            if ( secondaryId == compId )
                continue;
            RigidBody<T, U> const& rbB = *( RB[ m_rigidBodyId[ secondaryId ] ] );
            Transform3<T> const& trB = tr3d[ secondaryId ];
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
                unsigned int contactForceID = 0;
                // ContactForceModelBuilderFactory<T>::computeHash( matA, 
                //                                         rbB.getMaterial(),
                //                                         GrainsParameters<T>::m_materialMap.size() );
                CF[contactForceID]->computeForces( ci, 
                                                    zeroVector3T,
                                                    zeroVector3T,
                                                    massA,
                                                    rbB.getMass(),
                                                    trA.getOrigin(),
                                                    m_torce[ compId ] );
            }
            result[compId] += ( ci.getOverlapDistance() < T( 0 ) );
            // Vector3<T> relVelocityAtContact = 
            // m_kinematics[compId].getVelocityAtPoint( ci.getContactPoint() ) -
            // m_kinematics[secondaryId].getVelocityAtPoint( ci.getContactPoint() );
            // Vector3<T> relAngVelocity = 
        }
    } } }
    // Adding the gravitational force to the torce
    // m_torce[compId].addForce( massA * GrainsParameters<T>::m_gravity );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T, U )                                                              \
template                                                                       \
__GLOBAL__                                                                     \
void collisionDetectionN2( RigidBody<T, U> const* const* a,                    \
                           Transform3<T> const* tr3d,                          \
                           int numComponents,                                  \
                           int* result );                                      \
template                                                                       \
__GLOBAL__                                                                     \
void detectCollisionAndComputeConatactForces_kernel(                           \
                                   LinkedCell<T> const* const* LC,             \
                                   RigidBody<T, U> const* const* RB,           \
                                   ContactForceModel<T> const* const* CF,      \
                                   unsigned int* m_rigidBodyId,                \
                                   Transform3<T> const* tr3d,                  \
                                   Torce<T>* m_torce,                          \
                                   int* m_compId,                              \
                                   unsigned int* m_componentCellHash,          \
                                   unsigned int* m_cellHashStart,              \
                                   unsigned int* m_cellHashEnd,                \
                                   int numComponents,                          \
                                   int* result );                              
X( float, float )
X( double, float )
X( double, double )
#undef X