#include "GrainsTestDev.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"
#include "ConvexBuilderFactory.hh"
#include "GJK_AY.hh"
#include "RigidBodyGPUWrapper.hh"
#include "Box.hh"
#include "Superquadric.hh"

namespace GrainsCPU
{
// ...
template <typename T>
void collisionDetectionGJK( RigidBody<T, T> const* const* rb,
                            Transform3<T> const* t1,
                            Transform3<T> const* t2,
                            T* dist,
                            int const method,
                            int const N )
{
    for ( int i = 0; i < N; i++ )
    {
        dist[i] = distanceRigidBodies( *( rb[0] ), 
                                       *( rb[1] ), 
                                       t1[i], 
                                       t2[i],
                                       method );
    }
};
} // GrainsCPU namespace end




// -----------------------------------------------------------------------------
// ...
namespace GrainsGPU
{
// ...
template <typename T>
__global__ 
void collisionDetectionGJK( RigidBody<T, T> const* const* rb,
                            Transform3<T> const* t1,
                            Transform3<T> const* t2,
                            T* dist,
                            int const method,
                            int const N )
{
    int bid = gridDim.x * gridDim.y * blockIdx.z + 
              blockIdx.y * gridDim.x + 
              blockIdx.x;
    int tid = bid * blockDim.x + threadIdx.x;
    
    dist[tid] = distanceRigidBodies( *( rb[0] ), 
                                     *( rb[1] ), 
                                     t1[tid], 
                                     t2[tid],
                                     method );
};
} // GrainsGPU namespace end




// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
GrainsTestDev<T>::GrainsTestDev()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
GrainsTestDev<T>::~GrainsTestDev()
{}




// -----------------------------------------------------------------------------
// Runs the simulation over the prescribed time interval
template <typename T>
void GrainsTestDev<T>::simulate()
{
    int const method = 3;
    int userN = 2;
    int const N = userN * 24 * 256; // No. pair particles
    // int const N = 10;
    T r1 = 0.05, r2 = 0.05, r3 = 0.05; // Radii for now!
    
    /* ====================================================================== */
    /* Creating two random Vector3 arrays for centers of pair-particles       */
    /* ====================================================================== */
    default_random_engine generator;
    normal_distribution<T> distribution1( -0.05, 0.05 );
    normal_distribution<T> distribution2( +0.05, 0.05 );

    // Allocating memory on host and device
    Transform3<T> *h_tr1 = new Transform3<T>[N];
    Transform3<T> *h_tr2 = new Transform3<T>[N];
    Transform3<T> *d_tr1;
    Transform3<T> *d_tr2;
    cudaErrCheck( cudaMalloc( (void**)&d_tr1,
                              N * sizeof( Transform3<T> ) ) );
    cudaErrCheck( cudaMalloc( (void**)&d_tr2,
                              N * sizeof( Transform3<T> ) ) );
                             
    // Randomize array on host
    for( int i = 0; i < N; i++ )
    {
        h_tr1[i].setBasis( distribution1( generator ),
                           distribution1( generator ),
                           distribution1( generator ) );
        h_tr1[i].setOrigin( Vector3<T>( distribution1( generator ), 
                                        distribution1( generator ), 
                                        distribution1( generator ) ) );
        h_tr2[i].setBasis( distribution2( generator ),
                           distribution2( generator ),
                           distribution2( generator ) );
        h_tr2[i].setOrigin( Vector3<T>( distribution2( generator ), 
                                        distribution2( generator ), 
                                        distribution2( generator ) ) );
    }

    // Copying the arrays from host to device
    cudaErrCheck( cudaMemcpy( d_tr1,
                              h_tr1,
                              N * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );
    cudaErrCheck( cudaMemcpy( d_tr2,
                              h_tr2,
                              N * sizeof( Transform3<T> ), 
                              cudaMemcpyHostToDevice ) );

    /* ====================================================================== */
    /* Creating Particles                                                     */
    /* ====================================================================== */
    // Convex
    Convex<T> *h_convex1 = new Superquadric<T>( r1, r2, r3, 3., 3. );
    Convex<T> *h_convex2 = new Superquadric<T>( r1, r2, r3, 3., 3. );
    RigidBody<T, T>** h_rb = ( RigidBody<T, T>** ) malloc( 2 * sizeof( RigidBody<T, T>* ) );
    h_rb[0] = new RigidBody<T, T>( h_convex1, T( 0 ), 0, 1 );
    h_rb[1] = new RigidBody<T, T>( h_convex2, T( 0 ), 0, 1 );

    RigidBody<T, T>** d_rb;
    cudaErrCheck( cudaMalloc( (void**) &d_rb, 2 * sizeof( RigidBody<T, T>* ) ) );
    RigidBodyCopyHostToDevice( h_rb, 
                               d_rb,
                               2 );
    cudaDeviceSynchronize();

    /* ====================================================================== */
    /* Collision detection                                                    */
    /* ====================================================================== */
    T* h_collision = new T[N];
    // Zeroing out      
    for( int i = 0; i < N; i++ )
    {
        h_collision[i] = 0;
    }
    T* d_collision;
    cudaErrCheck( cudaMalloc( (void**)&d_collision,
                              N * sizeof( T ) ) );
    cudaErrCheck( cudaMemcpy( d_collision,
                              h_collision,
                              N * sizeof( T ), 
                              cudaMemcpyHostToDevice ) );

    // Collision detection on host
    auto h_start = chrono::high_resolution_clock::now();
    GrainsCPU::collisionDetectionGJK<T>( h_rb,
                                         h_tr1,
                                         h_tr2,
                                         h_collision,
                                         method,
                                         N );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    dim3 dimBlock( 256, 1, 1 );
    dim3 dimGrid( N / 256, 1, 1 );
    auto d_start = chrono::high_resolution_clock::now();
    GrainsGPU::collisionDetectionGJK<<< dimGrid, dimBlock >>> ( d_rb, 
                                                                d_tr1,
                                                                d_tr2,
                                                                d_collision,
                                                                method,
                                                                N );
    cudaErrCheck( cudaDeviceSynchronize() );
    auto d_end = chrono::high_resolution_clock::now();

    /* ====================================================================== */
    /* Results                                                                */
    /* ====================================================================== */
    // Time comparison
    chrono::duration<double> h_time = h_end - h_start;
    chrono::duration<double> d_time = d_end - d_start;
    std::cout << "CPU: " << h_time.count()
              << ", GPU: " << d_time.count() 
              << ", Speedup: " << h_time.count() / d_time.count() << endl;

    // accuracy
    T* h_d_collision = new T[N];
    cudaErrCheck( cudaMemcpy( h_d_collision,
                              d_collision,
                              N * sizeof( T ), 
                              cudaMemcpyDeviceToHost ) );

    int trueCount = 0;
    for( int i = 0; i < N; i++ )
    {
        if( fabs( h_collision[i] - h_d_collision[i] ) < 1.e-6 )
            trueCount++;
    }
    cout << N << " Particles, "
         << trueCount << " Consistent collisions on host and device." << endl;

    delete[] h_tr1;
    delete[] h_tr2;
    delete[] h_collision;
    delete[] h_d_collision;
    cudaFree( d_tr1 );
    cudaFree( d_tr2 );
    cudaFree( d_collision );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsTestDev<float>;
template class GrainsTestDev<double>;