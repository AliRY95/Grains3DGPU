#include "LinkedCell.hh"
#include "LinkedCellGPUWrapper.hh"
#include "Transform3.hh"
#include "VectorMath.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__ LinkedCell<T>::LinkedCell()
{
}

// -----------------------------------------------------------------------------
// Constructor with min and max points along with extent of each cell
template <typename T>
__HOSTDEVICE__ LinkedCell<T>::LinkedCell(const Vector3<T>& min,
                                         const Vector3<T>& max,
                                         T                 extent)
    : m_minCorner(min)
    , m_maxCorner(max)
    , m_cellExtent(extent)
{
    Vector3<T> numCellsPerDir(EPS<T>, EPS<T>, EPS<T>);
    numCellsPerDir += m_maxCorner - m_minCorner;
    numCellsPerDir /= m_cellExtent;
    m_numCellsPerDir.x = int(numCellsPerDir[X]);
    m_numCellsPerDir.y = int(numCellsPerDir[Y]);
    m_numCellsPerDir.z = int(numCellsPerDir[Z]);
    m_numCells = m_numCellsPerDir.x * m_numCellsPerDir.y * m_numCellsPerDir.z;
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ LinkedCell<T>::~LinkedCell()
{
}

// -----------------------------------------------------------------------------
// Gets the number of cells
template <typename T>
__HOSTDEVICE__ int LinkedCell<T>::getNumCells() const
{
    return (m_numCells);
}

// -----------------------------------------------------------------------------
// Checks if a cell Id is in range
template <typename T>
__HOSTDEVICE__ void LinkedCell<T>::checkBound(int3 const& id) const
{
    assert(id.x > 0 && id.x < m_numCellsPerDir.x && id.y > 0
           && id.y < m_numCellsPerDir.y && id.z > 0
           && id.z < m_numCellsPerDir.z);
    // if ( id.x < 0 || id.x >= m_numCellsPerDir.x ||
    //      id.y < 0 || id.y >= m_numCellsPerDir.y ||
    //      id.z < 0 || id.z >= m_numCellsPerDir.z )
    // {
    //     printf( "Linked cell range exceeded!" );
    //     assert( 1 );
    // }
}

// -----------------------------------------------------------------------------
// Returns the 3d Id of the cell which the point belongs to
template <typename T>
__HOSTDEVICE__ int3 LinkedCell<T>::computeCellId(const Vector3<T>& p) const
{
    int3 cellId;
    // static_cast is faster than floor, though it comes with a cost ...
    // if the operand is -0.7, it gives 0.
    // cellId.x = static_cast<int>( ( p[X] - m_minCorner[X] ) / m_cellExtent );
    // cellId.y = static_cast<int>( ( p[Y] - m_minCorner[Y] ) / m_cellExtent );
    // cellId.z = static_cast<int>( ( p[Z] - m_minCorner[Z] ) / m_cellExtent );
    cellId.x = floor((p[X] - m_minCorner[X]) / m_cellExtent);
    cellId.y = floor((p[Y] - m_minCorner[Y]) / m_cellExtent);
    cellId.z = floor((p[Z] - m_minCorner[Z]) / m_cellExtent);
    checkBound(cellId);
    return (cellId);
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value of a given point
template <typename T>
__HOSTDEVICE__ int
    LinkedCell<T>::computeLinearCellHash(const Vector3<T>& p) const
{
    return (computeLinearCellHash(computeCellId(p)));
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the 3d Id of the cell
template <typename T>
__HOSTDEVICE__ int
    LinkedCell<T>::computeLinearCellHash(int3 const& cellId) const
{
    return ((cellId.z * m_numCellsPerDir.y + cellId.y) * m_numCellsPerDir.x
            + cellId.x + 1);
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the Id along each axis
template <typename T>
__HOSTDEVICE__ int
    LinkedCell<T>::computeLinearCellHash(int i, int j, int k) const
{
    return ((k * m_numCellsPerDir.y + j) * m_numCellsPerDir.x + i + 1);
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value for a neighboring cell in the direction
// given by (i, j, k)
// TODO: performance check!
template <typename T>
__HOSTDEVICE__ int LinkedCell<T>::computeNeighboringCellLinearHash(int cellHash,
                                                                   int i,
                                                                   int j,
                                                                   int k) const
{
    cellHash--;
    int z = cellHash / (m_numCellsPerDir.x * m_numCellsPerDir.y);
    int y = (cellHash / m_numCellsPerDir.x) % m_numCellsPerDir.y;
    int x = cellHash % m_numCellsPerDir.x;
    return (computeLinearCellHash(x + i, y + j, z + k));
}

// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using CPU
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashCPU(
    std::vector<Transform3<T>> const& tr,
    unsigned int                      numComponents,
    std::vector<unsigned int>&        componentCellHash) const
{
    for(int i = 0; i < numComponents; i++)
        componentCellHash[i] = computeLinearCellHash(tr[i].getOrigin());
}

// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using GPU
// TODO: cudaMaxOccupancy -- function for numBlocks
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashGPU(
    Transform3<T> const* tr,
    unsigned int         numComponents,
    unsigned int*        componentCellHash) const
{
    unsigned int numThreads = 256;
    unsigned int numBlocks  = (numComponents + numThreads - 1) / numThreads;
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