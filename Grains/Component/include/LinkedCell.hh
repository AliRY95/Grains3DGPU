#ifndef _LINKEDCELL_HH_
#define _LINKEDCELL_HH_

#include "Transform3.hh"
#include "Vector3.hh"

// =============================================================================
/** @brief The class LinkedCell.

    The broad-phase detection is done through LinkedCell class. It limits the 
     typename T  for collision to the neighboring components. Neighboring components
    are those who belong to adjacent cells given a uniform Cartesian grid for 
    cells.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class LinkedCell
{
protected:
    /** @name Parameters */
    //@{
    Vector3<T> m_minCorner; /**< min corner point of the linked cell */
    Vector3<T> m_maxCorner; /**< max corner point of the linked cell */
    T          m_cellExtent; /**< extent of each cell */
    uint3      m_numCellsPerDir; /**< number of cells per each direction */
    uint       m_numCells; /**< total number of cells */
    //@}

public:
    /** @name Constructors */
    //@{
    /** @brief Default constructor */
    __HOSTDEVICE__
    LinkedCell();

    /** @brief Constructor with min and max points along with extent of each
        cell 
        @param min min point of the linked cell 
        @param max max point of the linked cell 
        @param extent size of cells */
    __HOSTDEVICE__
    LinkedCell(const Vector3<T>& min, const Vector3<T>& max, T extent);

    /** @brief Destructor */
    __HOSTDEVICE__
    ~LinkedCell();
    //@}

    /** @name Get methods */
    //@{
    /** @brief Gets the number of cells */
    __HOSTDEVICE__
    uint getNumCells() const;
    //@}

    /** @name Methods */
    //@{
    /** @brief Checks if a cell Id is in range
        @param id 3D Id */
    __HOSTDEVICE__
    void checkBound(const uint3& id) const;

    /** @brief Returns the 3d Id of the cell which the point belongs to
        @param p point */
    __HOSTDEVICE__
    uint3 computeCellId(const Vector3<T>& p) const;

    /** @brief Returns the linear cell hash value of a given point
        @param p point */
    __HOSTDEVICE__
    uint computeLinearCellHash(const Vector3<T>& p) const;

    /** @brief Returns the linear cell hash value from the 3d Id of the cell
        @param cellId 3d cell Id */
    __HOSTDEVICE__
    uint computeLinearCellHash(const uint3& cellId) const;

    /** @brief Returns the linear cell hash value from the 3d Id of the cell
        @param i position of the cell in the x-direction
        @param j position of the cell in the y-direction
        @param k position of the cell in the z-direction */
    __HOSTDEVICE__
    uint computeLinearCellHash(uint i, uint j, uint k) const;

    /** @brief Returns the linear cell hash value for a neighboring cell in
        the direction given by (i, j, k)
        @param i relative position of the neighboring cell in the x-direction
        @param j relative position of the neighboring cell in the y-direction
        @param k relative position of the neighboring cell in the z-direction */
    __HOSTDEVICE__
    uint computeNeighboringCellLinearHash(uint cellHash,
                                          uint i,
                                          uint j,
                                          uint k) const;

    /** @brief Computes and stores the linear cell hash values in 
        componentCellHash for all components using CPU
        @param pos position of components
        @param numComponents number of components
        @param componentCellHash hash values for particles */
    void computeLinearLinkedCellHashCPU(
        const std::vector<Transform3<T>>& tr,
        uint                              numComponents,
        std::vector<uint>&                componentCellHash) const;

    /** @brief Computes and stores the linear cell hash values in 
        componentCellHash for all components using GPU - Wrapper
        @param pos position of components
        @param numComponents number of components
        @param componentCellHash hash values for particles */
    void computeLinearLinkedCellHashGPU(const Transform3<T>* pos,
                                        uint                 numComponents,
                                        uint* componentCellHash) const;

    /** @brief Returns the Morton cell hash value from the 3d Id of the cell
        // @param cellId 3d cell Id */
    // __HOSTDEVICE__
    // uint computeMortonCellHash( int i,
    //                                     int j,
    //                                     int k ) const;
    //@}
};

typedef LinkedCell<float>  LinkedCellF;
typedef LinkedCell<double> LinkedCellD;

#endif