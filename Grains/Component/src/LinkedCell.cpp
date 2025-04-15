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
    m_numCellsPerDir.x = uint(numCellsPerDir[X]);
    m_numCellsPerDir.y = uint(numCellsPerDir[Y]);
    m_numCellsPerDir.z = uint(numCellsPerDir[Z]);
    m_numCells = m_numCellsPerDir.x * m_numCellsPerDir.y * m_numCellsPerDir.z;

    // Allocate memory for the flat array
    uint offset;
    m_neighborsList = new uint[27 * m_numCells];
    // Precompute neighbors for each cell
    for(uint cellHash = 0; cellHash < m_numCells; ++cellHash)
    {
        offset       = cellHash * 27;
        uint3 cellId = {cellHash % m_numCellsPerDir.x,
                        (cellHash / m_numCellsPerDir.x) % m_numCellsPerDir.y,
                        cellHash / (m_numCellsPerDir.x * m_numCellsPerDir.y)};
        for(int k = -1; k < 2; ++k)
        {
            for(int j = -1; j < 2; ++j)
            {
                for(int i = -1; i < 2; ++i)
                {
                    int nx = cellId.x + i;
                    int ny = cellId.y + j;
                    int nz = cellId.z + k;

                    // Check if the neighbor is within bounds
                    // clang-format off
                    if(nx >= 0 && nx < m_numCellsPerDir.x && 
                       ny >= 0 && ny < m_numCellsPerDir.y && 
                       nz >= 0 && nz < m_numCellsPerDir.z)
                    {
                        uint neighborHash = 
                            nx + 
                            ny * m_numCellsPerDir.x +
                            nz * m_numCellsPerDir.x * m_numCellsPerDir.y;
                        m_neighborsList[offset++] = neighborHash;
                    }
                    // clang-format on
                    else
                    {
                        // Invalid neighbor
                        m_neighborsList[offset++] = UINT_MAX;
                    }
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ LinkedCell<T>::~LinkedCell()
{
    delete[] m_neighborsList;
}

// -----------------------------------------------------------------------------
// Gets the neighbors of the cell with the given hash value
template <typename T>
__HOSTDEVICE__ const uint*
    LinkedCell<T>::getNeighbors(const uint cellHash) const
{
    const uint offset = cellHash * 27;
    return (&m_neighborsList[offset]);
}

// -----------------------------------------------------------------------------
// Gets the number of cells
template <typename T>
__HOSTDEVICE__ uint LinkedCell<T>::getNumCells() const
{
    return (m_numCells);
}

// -----------------------------------------------------------------------------
// Checks if a cell Id is in range
template <typename T>
__HOSTDEVICE__ void LinkedCell<T>::checkBound(const uint3& id) const
{
    if(id.x >= m_numCellsPerDir.x || id.y >= m_numCellsPerDir.y
       || id.z >= m_numCellsPerDir.z)
    {
        printf("Linked cell range exceeded!");
        // assert( 1 );
    }
}

// -----------------------------------------------------------------------------
// Returns the 3d Id of the cell which the point belongs to
template <typename T>
__HOSTDEVICE__ uint3 LinkedCell<T>::computeCellId(const Vector3<T>& p) const
{
    uint3 cellId;
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
__HOSTDEVICE__ uint
    LinkedCell<T>::computeLinearCellHash(const Vector3<T>& p) const
{
    return (computeLinearCellHash(computeCellId(p)));
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the 3d Id of the cell
template <typename T>
__HOSTDEVICE__ uint
    LinkedCell<T>::computeLinearCellHash(const uint3& cellId) const
{
    return ((cellId.z * m_numCellsPerDir.y + cellId.y) * m_numCellsPerDir.x
            + cellId.x);
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value from the Id along each axis
template <typename T>
__HOSTDEVICE__ uint LinkedCell<T>::computeLinearCellHash(const uint i,
                                                         const uint j,
                                                         const uint k) const
{
    return ((k * m_numCellsPerDir.y + j) * m_numCellsPerDir.x + i);
}

// -----------------------------------------------------------------------------
// Returns the linear cell hash value for a neighboring cell in the direction
// given by (i, j, k)
// TODO: performance check!
template <typename T>
__HOSTDEVICE__ uint LinkedCell<T>::computeNeighboringCellLinearHash(
    const uint cellHash, const uint i, const uint j, const uint k) const
{
    uint z = cellHash / (m_numCellsPerDir.x * m_numCellsPerDir.y);
    uint y = (cellHash / m_numCellsPerDir.x) % m_numCellsPerDir.y;
    uint x = cellHash % m_numCellsPerDir.x;
    return (computeLinearCellHash(x + i, y + j, z + k));
}

// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using CPU
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashCPU(
    const std::vector<Transform3<T>>& tr,
    uint                              numComponents,
    std::vector<uint>&                componentCellHash) const
{
    for(uint i = 0; i < numComponents; i++)
        componentCellHash[i] = computeLinearCellHash(tr[i].getOrigin());
}

// -----------------------------------------------------------------------------
// Computes and stores the linear cell hash values in componentCellHash for all
// components using GPU
// TODO: cudaMaxOccupancy -- function for numBlocks
template <typename T>
void LinkedCell<T>::computeLinearLinkedCellHashGPU(
    Transform3<T> const* tr, uint numComponents, uint* componentCellHash) const
{
    uint numThreads = 256;
    uint numBlocks  = (numComponents + numThreads - 1) / numThreads;
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