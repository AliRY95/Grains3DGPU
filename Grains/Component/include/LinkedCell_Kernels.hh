#ifndef _LINKEDCELL_KERNELS_HH_
#define _LINKEDCELL_KERNELS_HH_


#include "LinkedCell.hh"
#include "Transform3.hh"
#include "Vector3.hh"


// =============================================================================
/** @brief The header for GPU kernels used in the LinkedCell class.

    Various GPU kernels used in the LinkedCell class.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name LinkedCell_Kernels : External methods */
//@{
/** @brief Creates a LinkedCell object
@param min smallest corner of the domain
@param max greatest corner of the domain 
@param size size of each cell 
@param LC pointer to the linked cell object to be created
@param numCells number of cells after constructing the linked cell */
template <typename T>
__GLOBAL__ 
// void createLinkedCellonGPU( Vector3<T> const& min,
//                             Vector3<T> const& max,
void createLinkedCellOnGPU( T minX, T minY, T minZ,
                            T maxX, T maxY, T maxZ,
                            T size,
                            LinkedCell<T>** LC,
                            int* numCells );

/** @brief Computes the linear linked cell hash values for all components
@param LC linked cell
@param tr list of all transformations
@param numComponents number of all components in the simulation */
template <typename T>
__GLOBAL__ 
void computeLinearLinkedCellHashGPU_kernel( LinkedCell<T> const* const* LC,
                                            Transform3<T> const* tr,
                                            unsigned int numComponents,
                                            unsigned int* componentCellHash );
//@}


#endif