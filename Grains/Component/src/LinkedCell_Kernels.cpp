// TODO: CHANGE THE FORMAT FROM CPP TO CU LATER.
#include "LinkedCell_Kernels.hh"
#include "LinkedCell.hh"
// #include "Transform3.hh"
#include "Vector3.hh"


// -----------------------------------------------------------------------------
// Kernel for computing the linear linked cell hash values for all components
template <typename T>
__GLOBAL__ 
// void createLinkedCellonGPU( Vector3<T> const& min,
//                             Vector3<T> const& max,
void createLinkedCellOnGPU( T minX, T minY, T minZ,
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
__GLOBAL__                                                                     \
void createLinkedCellOnGPU( T minX, T minY, T minZ,                            \
                            T maxX, T maxY, T maxZ,                            \
                            T size,                                            \
                            LinkedCell<T>** LC,                                \
                            int* numCells );                                   \
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