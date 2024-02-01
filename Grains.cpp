/* ========================================================================== */
/*                Discrete Element Method Using NVIDIA CUDA                   */
/*                      Alireza Yazdani, July 2023                            */
/* ========================================================================== */
#include <cuda.h>
#include <cuda_runtime.h>
#include "Kernels.cpp"

using namespace std;

#ifdef USE_SINGLE_PRECISION
typedef float userDefType;
#else
typedef double userDefType;
#endif


/* ========================================================================== */
/*                                    Main                                    */
/* ========================================================================== */
/* Main Function */
int main(int argc, char* argv[])
{
    int userN = stoi( argv[1] );
    int const N = userN * 24 * 256; // No. pair particles
    // int const N = 24 * 512; // No. pair particles
    // int const N = 1; // No. pair particles
    userDefType r1 = 0.05, r2 = 0.05, r3 = 0.05; // Radii for now!
    
    // const size_t malloc_limit = size_t( 4608 / 2 ) * size_t( 1024 ) * size_t( 1024 );
    // cudaErrCheck( cudaDeviceSetLimit( cudaLimitMallocHeapSize, malloc_limit ) );

    // size_t p;
    // cudaErrCheck( cudaDeviceGetLimit( &p, cudaLimitMallocHeapSize ) );
    // cout << p / 1028 << "KB" << endl;
    /* ====================================================================== */
    /* Creating two random Vector3 arrays for centers of pair-particles       */
    /* ====================================================================== */
    default_random_engine generator;
    // normal_distribution<userDefType> distribution( 0.5, 0.2 );
    uniform_real_distribution<userDefType> distribution( 0.0, 1.0 );

    // Allocating memory on host and device
    // Vector3<userDefType> h_firstCenters[N]; // Host array
    Vector3<userDefType> *h_centers = new Vector3<userDefType>[N];
    Vector3<userDefType> *d_centers; // Device array
    cudaErrCheck( cudaMalloc( (void**)&d_centers,
                              N * sizeof( Vector3<userDefType> ) ) );
                             
    // Randomize array on host
    for( int i = 0; i < N; i++ )
    {
        h_centers[i].setValue( distribution( generator ),
                               distribution( generator ),
                               distribution( generator ) );
    }

    // Copying the arrays from host to device
    cudaErrCheck( cudaMemcpy( d_centers,
                              h_centers,
                              N * sizeof( Vector3<userDefType> ), 
                              cudaMemcpyHostToDevice ) );

    /* ====================================================================== */
    /* Creating convex bodies                                             */
    /* ====================================================================== */
    Vector3<userDefType> *h_box = new Vector3<userDefType>( r1, r2, r3 );
    Vector3<userDefType> *d_box; // Device arrays
    cudaErrCheck( cudaMalloc( (void**)&d_box,
                              sizeof( Vector3<userDefType> ) ) );

    // Copying the particles from host to device
    cudaErrCheck( cudaMemcpy( d_box,
                              h_box,
                              sizeof( Vector3<userDefType> ), 
                              cudaMemcpyHostToDevice ) );

    Convex *h_convex = new Convex( r1, r2, r3 );

    // Copying the array from host to device
    Convex *d_convex;
    cudaErrCheck( cudaMalloc( (void**)&d_convex,
                              sizeof( Convex ) ) );
    // setupConvex<<< 1, 1 >>>( r1, r2, r3, d_convex );

    /* ====================================================================== */
    /* Collision detection                                                    */
    /* ====================================================================== */
    // bool *h_collision = new bool[N * N];
    // // Zeroing out
    // for( int i = 0; i < N * N; i++ )
    // {
    //     h_collision[i] = false;
    // }
    // bool *d_collision;
    // cudaErrCheck( cudaMalloc( (void**)&d_collision,
    //                           N * N * sizeof(bool) ) );
    // cudaErrCheck( cudaMemcpy( d_collision,
    //                           h_collision,
    //                           N * N * sizeof(bool), 
    //                           cudaMemcpyHostToDevice ) );
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
    // GrainsCPU::collisionDetectionSpherical( r1,
    //                                         h_centers,
    //                                         h_collision,
    //                                         N );             
    // GrainsCPU::collisionDetectionAABB( h_box,
    //                                   h_centers,
    //                                   h_collision,
    //                                   N );
    // GrainsCPU::collisionDetectionGJK( h_convex,
    //                                   h_centers,
    //                                   h_collision,
    //                                   N );
    GrainsCPU::collisionDetectionGJKwithAABB( h_convex,
                                              h_centers,
                                              h_collision,
                                              N );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    dim3 dimBlock( 256, 1, 1 );
    dim3 dimGrid( N / 256, 1, 1 );
    // dim3 dimGrid( 6, 2, 2 );
    // dim3 dimBlock( N / 24, 1, 1 );
    // if( N / 24 > 256 )
    // {
    //     dimBlock.x = 256;
    //     dimGrid.x = N / dimBlock.x / 4;
    // }
    auto d_start = chrono::high_resolution_clock::now();
    // GrainsGPU::collisionDetectionSpherical<<< dimGrid, dimBlock >>> ( r1, 
    //                                                                   d_centers,
    //                                                                   d_collision,
    //                                                                   N );
    // GrainsGPU::collisionDetectionAABB<<< dimGrid, dimBlock >>>( d_box,
    //                                                             d_centers,
    //                                                             d_collision,
    //                                                             N );
    // GrainsGPU::collisionDetectionGJK<<< dimGrid, dimBlock >>> ( d_convex, 
    //                                                             d_centers,
    //                                                             d_collision,
    //                                                             N );
    GrainsGPU::collisionDetectionGJKwithAABB<<< dimGrid, dimBlock >>> ( d_convex, 
                                                                        d_centers,
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

    // // accuracy
    // bool *h_d_collision = new bool[N * N];
    // cudaErrCheck( cudaMemcpy( h_d_collision,
    //                           d_collision,
    //                           N * N * sizeof(bool), 
    //                           cudaMemcpyDeviceToHost ) );

    // int diffCount = 0, trueHostCount = 0, trueDeviceCount = 0;
    // for( int i = 0; i < N * N; i++ )
    // {
    //     if( h_collision[i] == true )
    //         trueHostCount++;
    //     if( h_d_collision[i] == true )
    //         trueDeviceCount++;
    //     if( h_collision[i] != h_d_collision[i] )
    //         diffCount++;
    // }
    // cout << N << " Particles, "
    //      << trueHostCount << " Collision on host, "
    //      << trueDeviceCount << " Collision on device, "
    //      << diffCount << " Different results." << endl;

    delete[] h_centers;
    delete[] h_collision;
    cudaFree( d_centers );
    cudaFree( d_collision );
    
  return 0;
}