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
#include "ContactInfo.hh"
#include "LinkedCell.hh"
#include "ComponentManagerCPU.hh"
#include "ComponentManagerGPU.hh"
#include "ComponentManager.hh"

#include "GrainsParameters.hh"


using namespace std;


#ifdef SINGLE
typedef float BV_TYPE;
typedef float TYPE;
#else
typedef double BV_TYPE;
typedef double TYPE;
#endif
/* ========================================================================== */
/*                                    TEMP                                    */
/* ========================================================================== */

namespace GrainsCPU {
template <typename T, typename U>
void setupRigidBody( ConvexType cType,
                     T a,
                     T b,
                     T c,
                     T ct,
                     RigidBody<T, U>** rb )
{
    Convex<T>* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere<T>( a );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box<T>( a, b, c );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric<T>( a, b, c, T( 256 ), T( 256 ) );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
}; 

template <typename T>
void setupLinkedCell( T rMax,
                      LinkedCell<T>** lc )
{
    *lc = new LinkedCell<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                             Vector3<T>( T( 1 ), T( 1 ), T( 1 ) ), 
                             rMax );
}; } // GrainsCPU namespace end

namespace GrainsGPU {
template <typename T, typename U>
__global__ void setupRigidBody( ConvexType cType,
                                T a,
                                T b,
                                T c,
                                T ct,
                                RigidBody<T, U>** rb )
{
    Convex<T>* cvx;
    if ( cType == SPHERE )
    {
        cvx = new Sphere<T>( a );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
    else if ( cType == BOX )
    {
        cvx = new Box<T>( a, b, c );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
    else if ( cType == SUPERQUADRIC )
    {
        cvx = new Superquadric<T>( a, b, c, 2., 3. );
        *rb = new RigidBody<T, U>( cvx, ct );
    }
};

template <typename T>
__global__ void setupLinkedCell( T rMax,
                                 LinkedCell<T>** lc )
{
    *lc = new LinkedCell<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                             Vector3<T>( T( 1 ), T( 1 ), T( 1 ) ), 
                             rMax );
}; } // GrainsGPU namespace end

/* ========================================================================== */
/*                                    Main                                    */
/* ========================================================================== */
/* Main Function */
int main(int argc, char* argv[])
{
    double userN = stod( argv[1] );
    int const N = round( userN * 24 * 256 ); // No. pair particles
    // int const N = 7; // No. pair particles
    TYPE r1 = 0.05, r2 = 0.05, r3 = 0.05; // Radii for now!

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
    /* Creating rigid bodies                                                  */
    /* ====================================================================== */

    RigidBody<TYPE, BV_TYPE>** h_rb;
    h_rb = ( RigidBody<TYPE, BV_TYPE>** ) malloc( sizeof( RigidBody<TYPE, BV_TYPE>* ) );
    GrainsCPU::setupRigidBody( particleType, r1, r2, r3, TYPE( 1.e-3 ), h_rb );

    // Copying the array from host to device
    // __constant__ RigidBody d_rb[1];
    RigidBody<TYPE, BV_TYPE>** d_rb;
    cudaErrCheck( cudaMalloc( (void**)&d_rb,
                              sizeof( RigidBody<TYPE, BV_TYPE>* ) ) );
    GrainsGPU::setupRigidBody<<<1, 1>>>( particleType, r1, r2, r3, TYPE( 1.e-3 ), d_rb );

    /* ====================================================================== */
    /* Creating linked cells                                                  */
    /* ====================================================================== */

    TYPE maxRadius = (*h_rb)->getCircumscribedRadius();

    LinkedCell<TYPE>** h_lc;
    h_lc = ( LinkedCell<TYPE>** ) malloc( sizeof( LinkedCell<TYPE>* ) );
    GrainsCPU::setupLinkedCell( TYPE( 2 ) * maxRadius, h_lc );

    LinkedCell<TYPE>** d_lc;
    cudaErrCheck( cudaMalloc( (void**)&d_lc,
                              sizeof( LinkedCell<TYPE>* ) ) );
    GrainsGPU::setupLinkedCell<<<1, 1>>>( TYPE( 2 ) * maxRadius, d_lc );


    GrainsParameters grainsParameters;
    grainsParameters.m_numCells = (*h_lc)->getNumCells();
    grainsParameters.m_numComponents = N;

    ComponentManager<TYPE>* h_cm = new ComponentManagerCPU<TYPE>();
    ComponentManager<TYPE>* d_cm = new ComponentManagerGPU<TYPE>( *h_cm );

    /* ====================================================================== */
    /* Collision detection                                                    */
    /* ====================================================================== */

    int* h_collision = new int[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
        h_collision[i] = 0;
    int* d_collision;
    cudaErrCheck( cudaMalloc( (void**)&d_collision,
                              N * sizeof( int ) ) );
    cudaErrCheck( cudaMemcpy( d_collision,
                              h_collision,
                              N * sizeof( int ), 
                              cudaMemcpyHostToDevice ) );

    // Collision detection on host
    auto h_start = chrono::high_resolution_clock::now();
    h_cm->detectCollision( h_lc, h_rb, h_collision );
    auto h_end = chrono::high_resolution_clock::now();

    // Collision detection on device
    auto d_start = chrono::high_resolution_clock::now();
    d_cm->detectCollision( d_lc, d_rb, d_collision );
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
    int* h_d_collision = new int[N];
    cudaErrCheck( cudaMemcpy( h_d_collision,
                              d_collision,
                              N * sizeof( int ), 
                              cudaMemcpyDeviceToHost ) );

    int trueHostCount = 0, trueDeviceCount = 0;
    for( int i = 0; i < N; i++ )
    {
        trueHostCount += h_collision[i];
        trueDeviceCount += h_d_collision[i];
    }
    cout << N << " Particles, "
         << trueHostCount << " Collision on host, "
         << trueDeviceCount << " Collision on device" << endl;

    // for( int i = 0; i < N; i++ )
    // {
    //     cout << h_collision[i] << " ";
    // }
    // cout << "\n";
    // for( int i = 0; i < N; i++ )
    // {
    //     cout << h_d_collision[i] << " ";
    // }
    // cout << "\n";
  return 0;
}