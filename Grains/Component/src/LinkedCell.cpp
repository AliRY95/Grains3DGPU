#include "VectorMath.hh"
#include "Transform3.hh"
#include "LinkedCell.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__ 
LinkedCell<T>::LinkedCell()
{}




// -----------------------------------------------------------------------------
// Constructor with min and max points along with extent of each cell
template <typename T>
__host__ __device__ 
LinkedCell<T>::LinkedCell( Vector3<T> const& min, 
                           Vector3<T> const& max,
                           T extent )
: m_minCorner( min )
, m_maxCorner( max )
, m_cellExtent( extent )
{
    Vector3<T> numCellsPerDir = ( m_maxCorner - m_minCorner ) / m_cellExtent;
    // Not performant sensitive as it is called only once in the beggining.
    // So, just do a bit of unoptimized work is fine for now.
    m_numCellsPerDir.x = ceil( numCellsPerDir[X] );
    m_numCellsPerDir.y = ceil( numCellsPerDir[Y] );
    m_numCellsPerDir.z = ceil( numCellsPerDir[Z] );
    m_numCells = m_numCellsPerDir.x * m_numCellsPerDir.y * m_numCellsPerDir.z;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__ 
LinkedCell<T>::~LinkedCell()
{}




// -----------------------------------------------------------------------------
// Gets the number of cells
template <typename T>
__host__ __device__ 
int LinkedCell<T>::getNumCells() const
{
    return ( m_numCells );
}




// -----------------------------------------------------------------------------
// Returns the 3d Id of the cell which the point belongs to
template <typename T>
__host__ __device__ 
uint3 LinkedCell<T>::computeCellId( Vector3<T> const& p ) const
{
    uint3 cellId;
    cellId.x = floor( ( p[X] - m_minCorner[X] ) / m_cellExtent );
    cellId.y = floor( ( p[Y] - m_minCorner[Y] ) / m_cellExtent );
    cellId.z = floor( ( p[Z] - m_minCorner[Z] ) / m_cellExtent );
    return ( cellId );
}




// -----------------------------------------------------------------------------
// Returns the 3d Id of the cell which the point belongs to - specialized for 
// floats
template <>
__host__ __device__ 
uint3 LinkedCell<float>::computeCellId( Vector3<float> const& p ) const
{
    uint3 cellId;
    cellId.x = floorf( ( p[X] - m_minCorner[X] ) / m_cellExtent );
    cellId.y = floorf( ( p[Y] - m_minCorner[Y] ) / m_cellExtent );
    cellId.z = floorf( ( p[Z] - m_minCorner[Z] ) / m_cellExtent );
    return ( cellId );
}




// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the 3d Id of the cell
// Is it possible to perform the multiply operations faster? __umul24?
template <typename T>
__host__ __device__ 
int LinkedCell<T>::computeLinearCellHash( uint3 const& cellId ) const
{
    return ( ( cellId.z * m_numCellsPerDir.y + 
               cellId.y ) * m_numCellsPerDir.x + 
               cellId.x + 1 );
}




// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the 3d Id of the cell
// Is it possible to perform the multiply operations faster? __umul24?
template <typename T>
__host__ __device__ 
int LinkedCell<T>::computeLinearCellHash( int i,
                                          int j,
                                          int k ) const
{
    // out of bound
    if ( i < 0 || i >= m_numCellsPerDir.x || 
         j < 0 || j >= m_numCellsPerDir.y ||
         k < 0 || k >= m_numCellsPerDir.z )
        return ( 0 );
    else
        return ( ( k * m_numCellsPerDir.y + 
                j ) * m_numCellsPerDir.x + 
                i + 1 );
}




// -----------------------------------------------------------------------------
// Returns the linear cell hash value for a neighboring cell in the direction 
// given by (i, j, k)
// TODO: performance check!
template <typename T>
__host__ __device__ 
int LinkedCell<T>::computeNeighboringCellLinearHash( int cellHash,
                                                     int i,
                                                     int j,
                                                     int k ) const
{
    cellHash--;
    int z = cellHash / ( m_numCellsPerDir.x * m_numCellsPerDir.y );
    int y = ( cellHash / m_numCellsPerDir.x ) % m_numCellsPerDir.y;
    int x = cellHash % m_numCellsPerDir.x;
    return ( computeLinearCellHash( x + i, y + j, z + k ) );
}




// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using CPU
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashCPU( Transform3<T> const* tr,
                                              unsigned int numComponents,
                                              unsigned int* componentCellHash )
                                              const
{
    for ( int i = 0; i < numComponents; i++ )
        componentCellHash[i] = 
        this->computeLinearCellHash( this->computeCellId( tr[i].getOrigin() ) );
}




// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using GPU
// TODO: cudaMaxOccupancy -- function for numBlocks
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashGPU( Transform3<T> const* tr,
                                              unsigned int numComponents,
                                              unsigned int* componentCellHash )
                                              const
{
    unsigned int numThreads = 256;
    unsigned int numBlocks = ( numComponents + numThreads - 1 ) / numThreads;
    // computeLinearLinkedCellHashGPU_kernel<<< numBlocks, numThreads >>>
    //                                                       ( this, 
    //                                                         tr,
    //                                                         numComponents,
    //                                                         componentCellHash );
}




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Kernel for computing the linear linked cell hash values for all components
// TODO: CLEAN
template <typename T>
__global__ 
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<T> const* const* LC,
                                            Transform3<T> const* tr,
                                            unsigned int numComponents,
                                            unsigned int* componentCellHash )
{

    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid >= numComponents ) 
        return;

    componentCellHash[tid] = 
    (**LC).computeLinearCellHash( (**LC).computeCellId( tr[tid].getOrigin() ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class LinkedCell<float>;
template class LinkedCell<double>;

#define X( T )                                                                 \
template                                                                       \
__global__                                                                     \
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<T> const* const*  LC,   \
                                            Transform3<T> const* tr,           \
                                            unsigned int numComponents,        \
                                            unsigned int* componentCellHash );
X( float )
X( double )
#undef X