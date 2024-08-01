#include "LinkedCellGPUWrapper.hh"
#include "LinkedCell.hh"
#include "Vector3.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// GPU kernel to construct the linked cell on device.
template <typename T>
__GLOBAL__ 
void createLinkedCellOnDeviceKernel( T minX, T minY, T minZ,
                                     T maxX, T maxY, T maxZ,
                                     T size,
                                     LinkedCell<T>** LC, 
                                     int* numCells )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid > 0 )
        return;

    *LC = new LinkedCell<T>( Vector3<T>( minX, minY, minZ ),
                             Vector3<T>( maxX, maxY, maxZ ),
                             size );
    *numCells = (*LC)->getNumCells();
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Constructs a LinkedCell object on device given the corners and size 
template <typename T>
__HOST__
int createLinkedCellOnDevice( Vector3<T> min,
                              Vector3<T> max,
                              T size,
                              LinkedCell<T>** LC )
{
    // allocating memory for only one int for the number of cells
    int h_numCells;
    int* d_numCells;
    cudaMalloc( ( void** ) &d_numCells, sizeof( int ) );

    createLinkedCellOnDeviceKernel<<<1, 1>>>( min[X], min[Y], min[Z],
                                              max[X], max[Y], max[Z],
                                              size,
                                              LC,
                                              d_numCells );

    // copying the variable to host
    cudaMemcpy( &h_numCells, 
                d_numCells, 
                sizeof( int ), 
                cudaMemcpyDeviceToHost );
    
    return( h_numCells );
}




// -----------------------------------------------------------------------------
// Kernel for computing the linear linked cell hash values for all components
template <typename T>
__GLOBAL__ 
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<T> const* const* LC,
                                            Transform3<T> const* tr,
                                            unsigned int numComponents,
                                            unsigned int* componentCellHash )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid >= numComponents ) 
        return;

    componentCellHash[tid] = 
    (*LC)->computeLinearCellHash( (*LC)->computeCellId( tr[tid].getOrigin() ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T )                                                                 \
template                                                                       \
__HOST__                                                                       \
int createLinkedCellOnDevice( Vector3<T> min,                                  \
                            Vector3<T> max,                                    \
                            T size,                                            \
                            LinkedCell<T>** LC );                              \
                                                                               \
template                                                                       \
__GLOBAL__                                                                     \
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<T> const* const* LC,    \
                                            Transform3<T> const* tr,           \
                                            unsigned int numComponents,        \
                                            unsigned int* componentCellHash );                                                                               
X( float )
X( double )
#undef X