#include "Particles.cuh"
#include <cstdlib>

// -----------------------------------------------------------------------------
// Default constructor
__host__ __device__ Particles::Particles()
{}




// -----------------------------------------------------------------------------
// Host constructor with No. particles
Particles::Particles( int num ) : m_num ( num )
{
    m_position = new Vector3[ num ];
}




// -----------------------------------------------------------------------------
// Device constructor with No. particles
__device__ Particles::Particles( int num ) : m_num ( num )
{
    cudaErrCheck( cudaMalloc( (void**)&m_num, sizeof( int ) ) );
    cudaErrCheck( cudaMalloc( (void**)&m_position, num * sizeof( Vector3 ) ) );
}




// -----------------------------------------------------------------------------
// Host destructor
Particles::~Particles()
{
    delete
}




// -----------------------------------------------------------------------------
// Device destructor
__device__ Particles::~Particles()
{
    cudaFree
}




// -----------------------------------------------------------------------------
// Host function to assign random positions
void Particles::setupParticles()
{
    
}




// -----------------------------------------------------------------------------
// Device function to assign random positions
__device__ void Particles::setupParticles()
{
    
}




