#ifndef _LINKEDCELLGPUWRAPPER_HH_
#define _LINKEDCELLGPUWRAPPER_HH_


#include "LinkedCell.hh"
#include "Transform3.hh"
#include "Vector3.hh"


// =============================================================================
/** @brief Header for LinkedCellGPUWrapper.

    Various GPU kernels used in the LinkedCell class.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name LinkedCellGPUWrapper : External methods */
//@{
/** @brief Creates a LinkedCell object on device and returns the number of cells
after the construction
@param min smallest corner of the domain
@param max greatest corner of the domain 
@param size size of each cell 
@param LC device pointer to the linked cell object to be created
@param numCells number of cells after constructing the linked cell */
template <typename T>
__HOST__
int createLinkedCellOnDevice( Vector3<T> min,
                              Vector3<T> max,
                              T size,
                              LinkedCell<T>** LC );

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