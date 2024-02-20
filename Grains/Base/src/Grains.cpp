/* ========================================================================== */
/*                Discrete Element Method Using NVIDIA CUDA                   */
/*                      Alireza Yazdani, July 2023                            */
/* ========================================================================== */
#include <cuda.h>
#include <cuda_runtime.h>

#include "Sphere.hh"
#include "Box.hh"
#include "Superquadric.hh"
#include "Convex.hh"

#include "Transform3.hh"

#include "RigidBody.hh"

#include "ComponentManagerCPU.hh"
#include "ComponentManagerGPU.hh"
#include "ComponentManager.hh"

#include "GrainsParameters.hh"


using namespace std;


/* ========================================================================== */
/*                                    TEMP                                    */
/* ========================================================================== */

namespace GrainsCPU {
void setupRigidBody( ConvexType cType,
                     double a,
                     double b,
                     double c,
                     double ct,
                     RigidBody** rb )
{
    Convex* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere( a );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box( a, b, c );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric( a, b, c, 2., 3. );
        *rb = new RigidBody( cvx, ct );
    }
}; } // GrainsCPU namespace end

namespace GrainsGPU {
__global__ void setupRigidBody( ConvexType cType,
                                double a,
                                double b,
                                double c,
                                double ct,
                                RigidBody** rb )
{
    Convex* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere( a );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box( a, b, c );
        *rb = new RigidBody( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric( a, b, c, 2., 3. );
        *rb = new RigidBody( cvx, ct );
    }
}; } // GrainsGPU namespace end

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

    GrainsParameters grainsParameters;
    grainsParameters.m_numComponents = N;

    /* ====================================================================== */
    /* Creating rigid bodies and component managers                           */
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

    ComponentManager* h_cm = new ComponentManagerCPU();
    ComponentManager* d_cm = new ComponentManagerGPU( *h_cm );

    /* ====================================================================== */
    /* Collision detection                                                    */
    /* ====================================================================== */

    bool *h_collision = new bool[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
        h_collision[i] = false;
    bool *d_collision;
    cudaErrCheck( cudaMalloc( (void**)&d_collision,
                              N * sizeof(bool) ) );
    cudaErrCheck( cudaMemcpy( d_collision,
                              h_collision,
                              N * sizeof(bool), 
                              cudaMemcpyHostToDevice ) );

    // Collision detection on host
    auto h_start = chrono::high_resolution_clock::now();
    h_cm->detectCollision( h_rb, h_collision );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    auto d_start = chrono::high_resolution_clock::now();
    d_cm->detectCollision( d_rb, d_collision );
    cudaDeviceSynchronize();
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
    
  return 0;
}