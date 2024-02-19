/* ========================================================================== */
/*                Discrete Element Method Using NVIDIA CUDA                   */
/*                      Alireza Yazdani, July 2023                            */
/* ========================================================================== */
#include <cuda.h>
#include <cuda_runtime.h>
#include "Kernels.cu"


using namespace std;


/* ========================================================================== */
/*                                    Main                                    */
/* ========================================================================== */
/* Main Function */
int main(int argc, char* argv[])
{
    double userN = stod( argv[1] );
    int const N = round( userN * 24 * 256 ); // No. pair particles
    double r1 = 0.05, r2 = 0.05, r3 = 0.05; // Radii for now!

    ConvexType particleType;
    switch ( stoi( argv[2] ) )
    {
    case ( 1 ):
        particleType = SPHERE;
        break;
    case ( 2 ):
        particleType = BOX;
        break;
    case ( 3 ):
        particleType = SUPERQUADRIC;
        break;
    default:
        particleType = BOX;
        break;
    }
    /* ====================================================================== */
    /* Creating two random Transform3 for each particles                      */
    /* ====================================================================== */

    default_random_engine generator;
    uniform_real_distribution<double> location( 0.0, 1.0 );
    uniform_real_distribution<double> angle( 0.0, 2. * M_PI );

    // Allocating memory on host and device
    Transform3d *h_tr3d = new Transform3d[N];
    Transform3d *d_tr3d; // Device array
    cudaErrCheck( cudaMalloc( (void**)&d_tr3d,
                              N * sizeof( Transform3d ) ) );
                             
    // Randomize array on host
    for( int i = 0; i < N; i++ )
    {
        // Random orientation
        double aX = angle( generator );
        double aY = angle( generator );
        double aZ = angle( generator );
        h_tr3d[i].setBasis( aX, aY, aZ );
        h_tr3d[i].setOrigin( Vec3d( location( generator ),
                                    location( generator ),
                                    location( generator ) ) );
    }

    // Copying the arrays from host to device
    cudaErrCheck( cudaMemcpy( d_tr3d,
                              h_tr3d,
                              N * sizeof( Transform3d ), 
                              cudaMemcpyHostToDevice ) );

    /* ====================================================================== */
    /* Creating convex bodies                                                 */
    /* ====================================================================== */

    RigidBody** h_rb;
    h_rb = ( RigidBody** ) malloc( sizeof( RigidBody* ) );
    GrainsCPU::setupRigidBody( particleType, r1, r2, r3, 1.e-3, h_rb );

    // Copying the array from host to device
    // __constant__ RigidBody d_rb[1];
    RigidBody** d_rb;
    cudaErrCheck( cudaMalloc( (void**)&d_rb,
                              sizeof( RigidBody* ) ) );
    GrainsGPU::setupRigidBody<<<1, 1>>>( particleType, r1, r2, r3, 1.e-3, d_rb );

    /* ====================================================================== */
    /* Collision detection                                                    */
    /* ====================================================================== */

    bool *h_collision = new bool[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
    {
        h_collision[i] = false;
    }
    bool *d_collision;
    cudaErrCheck( cudaMalloc( (void**)&d_collision,
                              N * sizeof(bool) ) );
    cudaErrCheck( cudaMemcpy( d_collision,
                              h_collision,
                              N * sizeof(bool), 
                              cudaMemcpyHostToDevice ) );

    // Collision detection on host
    auto h_start = chrono::high_resolution_clock::now();
    GrainsCPU::collisionDetectionGJK( h_rb,
                                      h_tr3d,
                                      h_collision,
                                      N );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    dim3 dimBlock( 256, 1, 1 );
    dim3 dimGrid( N / 256, 1, 1 );
    auto d_start = chrono::high_resolution_clock::now();
    GrainsGPU::collisionDetectionGJK<<< dimGrid, dimBlock >>> ( d_rb, 
                                                                d_tr3d,
                                                                d_collision,
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
    bool *h_d_collision = new bool[N];
    cudaErrCheck( cudaMemcpy( h_d_collision,
                              d_collision,
                              N * sizeof(bool), 
                              cudaMemcpyDeviceToHost ) );

    int diffCount = 0, trueHostCount = 0, trueDeviceCount = 0;
    for( int i = 0; i < N; i++ )
    {
        if( h_collision[i] == true )
            trueHostCount++;
        if( h_d_collision[i] == true )
            trueDeviceCount++;
        if( h_collision[i] != h_d_collision[i] )
            diffCount++;
    }
    cout << N << " Particles, "
         << trueHostCount << " Collision on host, "
         << trueDeviceCount << " Collision on device, "
         << diffCount << " Different results." << endl;

    delete[] h_tr3d;
    delete[] h_collision;
    cudaFree( d_tr3d );
    cudaFree( d_collision );
    
  return 0;
}