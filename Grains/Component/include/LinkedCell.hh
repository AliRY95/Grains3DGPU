#ifndef _LINKEDCELL_HH_
#define _LINKEDCELL_HH_


#include "Vector3.hh"
#include "Transform3.hh"

// =============================================================================
/** @brief The class LinkedCell.

    The broad-phase detection is done through LinkedCell class. It limits the 
    search for collision to the neighboring components. Neighboring components
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
        T m_cellExtent; /**< extent of each cell */
        uint3 m_numCellsPerDir; /**< number of cells per each direction */
        int m_numCells; /**< total number of cells */
        //@}

    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor */
        __host__ __device__ 
        LinkedCell();

        /** @brief Constructor with min and max points along with extent of each
        cell 
        @param min min point of the linked cell 
        @param max max point of the linked cell 
        @param extent size of cells */
        __host__ __device__ 
        LinkedCell( Vector3<T> const& min, 
                    Vector3<T> const& max,
                    T extent );

        /** @brief Destructor */
        __host__ __device__
        ~LinkedCell();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the number of cells */
        __host__ __device__ 
        int getNumCells() const;
        //@}


        /** @name Methods */
        //@{
        /** @brief Returns the 3d Id of the cell which the point belongs to
        @param p point */
        __host__ __device__
        uint3 computeCellId( Vector3<T> const& p ) const;

        /** @brief Returns the linear cell hash value from the 3d Id of the cell
        @param cellId 3d cell Id */
        __host__ __device__
        int computeLinearCellHash( uint3 const& cellId ) const;

        /** @brief Returns the linear cell hash value from the 3d Id of the cell
        @param i position of the cell in the x-direction
        @param j position of the cell in the y-direction
        @param k position of the cell in the z-direction */
        __host__ __device__
        int computeLinearCellHash( int i,
                                   int j,
                                   int k ) const;

        /** @brief Returns the linear cell hash value for a neighboring cell in
        the direction given by (i, j, k) - note that
        @param i relative position of the neighboring cell in the x-direction
        @param j relative position of the neighboring cell in the y-direction
        @param k relative position of the neighboring cell in the z-direction */
        __host__ __device__ 
        int computeNeighboringCellLinearHash( int cellHash,
                                              int i,
                                              int j,
                                              int k ) const; 
                                                      
        
        /** @brief Computes and stores the linear cell hash values in 
        componentCellHash for all components using CPU
        @param pos position of components
        @param numComponents number of components
        @param componentCellHash hash values for particles */
        void computeLinearLinkedCellHashCPU( Transform3<T> const* tr,
                                             unsigned int numComponents,
                                             unsigned int* componentCellHash )
                                             const;

        /** @brief Computes and stores the linear cell hash values in 
        componentCellHash for all components using GPU - Wrapper
        @param pos position of components
        @param numComponents number of components
        @param componentCellHash hash values for particles */
        void computeLinearLinkedCellHashGPU( Transform3<T> const* pos,
                                             unsigned int numComponents,
                                             unsigned int* componentCellHash )
                                             const;

        // /** @brief Returns the Morton cell hash value from the 3d Id of the cell
        // @param cellId 3d cell Id */
        // __host__ __device__ unsigned int computeMortonCellHash( 
        //                                           Vec3ui8 const& cellId ) const;
        //@}
};


/** @name LinkedCell : External methods */
//@{
/** @brief Computes and stores the linear cell hash values in  componentCellHash
for all components using GPU - Kernel
@param LC linked cell
@param pos position of components
@param numComponents number of components
@param componentCellHash hash values for particles */
// template <typename T>
__global__ 
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<double> const* const* LC,
                                            Transform3d const* pos,
                                            unsigned int numComponents,
                                            unsigned int* componentCellHash );
//@}


// typedef LinkedCell<float> LinkedCellF;
typedef LinkedCell<double> LinkedCellD;


#endif