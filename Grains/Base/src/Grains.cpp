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
    double r1 = 0.1, r2 = 0.1, r3 = 0.1; // Radii for now!

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
        h_tr3d[i].setBasis( 
            Mat3d( cos(aZ)*cos(aY),
                   cos(aZ)*sin(aY)*sin(aX) - sin(aZ)*cos(aX),
                   cos(aZ)*sin(aY)*cos(aX) + sin(aZ)*sin(aX),
                   sin(aZ)*cos(aY),
                   sin(aZ)*sin(aY)*sin(aX) + cos(aZ)*cos(aX),
                   sin(aZ)*sin(aY)*cos(aX) - cos(aZ)*sin(aX),
                   -sin(aY),
                   cos(aY)*sin(aX),
                   cos(aY)*cos(aX) ) );
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

    Convex* h_convex = new Box( r1, r2, r3 );

    // Copying the array from host to device
    Convex** d_convex;
    cudaErrCheck( cudaMalloc( (void**)&d_convex,
                              sizeof( Convex** ) ) );
    setupConvex<<< 1, 1 >>>( r1, r2, r3, d_convex );

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
    GrainsCPU::collisionDetectionGJK( h_convex,
                                      h_tr3d,
                                      h_collision,
                                      N );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    dim3 dimBlock( 256, 1, 1 );
    dim3 dimGrid( N / 256, 1, 1 );
    auto d_start = chrono::high_resolution_clock::now();
    GrainsGPU::collisionDetectionGJK<<< dimGrid, dimBlock >>> ( d_convex, 
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