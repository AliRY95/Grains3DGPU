#include "LinkedCell.hh"
#include "LinkedCellGPUWrapper.hh"
#include "Transform3.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
LinkedCell<T>::LinkedCell()
{}




// -----------------------------------------------------------------------------
// Constructor with min and max points along with extent of each cell
template <typename T>
__HOSTDEVICE__
LinkedCell<T>::LinkedCell( Vector3<T> const& min, 
                           Vector3<T> const& max,
                           T extent )
: m_minCorner( min )
, m_maxCorner( max )
, m_cellExtent( extent )
{
    Vector3<T> numCellsPerDir( EPS<T>, EPS<T>, EPS<T> );
    numCellsPerDir += m_maxCorner - m_minCorner;
    numCellsPerDir /= m_cellExtent;
    m_numCellsPerDir.x = int( numCellsPerDir[X] );
    m_numCellsPerDir.y = int( numCellsPerDir[Y] );
    m_numCellsPerDir.z = int( numCellsPerDir[Z] );
    m_numCells = m_numCellsPerDir.x * m_numCellsPerDir.y * m_numCellsPerDir.z;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
LinkedCell<T>::~LinkedCell()
{}




// -----------------------------------------------------------------------------
// Gets the number of cells
template <typename T>
__HOSTDEVICE__
int LinkedCell<T>::getNumCells() const
{
    return ( m_numCells );
}




// -----------------------------------------------------------------------------
// Returns the 3d Id of the cell which the point belongs to
template <typename T>
__HOSTDEVICE__
uint3 LinkedCell<T>::computeCellId( Vector3<T> const& p ) const
{
    uint3 cellId;
    cellId.x = floor( ( p[X] - m_minCorner[X] ) / m_cellExtent );
    cellId.y = floor( ( p[Y] - m_minCorner[Y] ) / m_cellExtent );
    cellId.z = floor( ( p[Z] - m_minCorner[Z] ) / m_cellExtent );
    return ( cellId );
}




// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the 3d Id of the cell
// Is it possible to perform the multiply operations faster? __umul24?
template <typename T>
__HOSTDEVICE__
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
__HOSTDEVICE__
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
__HOSTDEVICE__
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
void LinkedCell<T>::computeLinearLinkedCellHashCPU( 
                                std::vector<Transform3<T>> const& tr,
                                unsigned int numComponents,
                                std::vector<unsigned int>& componentCellHash )
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




// -----------------------------------------------------------------------------
// Explicit instantiation
template class LinkedCell<float>;
template class LinkedCell<double>;