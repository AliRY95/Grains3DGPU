#ifndef _HODCCONTACTFORCEMODELGPUWRAPPER_HH_
#define _HODCCONTACTFORCEMODELGPUWRAPPER_HH_


#include "HODCContactForceModel.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief Header for HODCContactForceModelGPUWrapper

    Similar to LinkedCell objects, the contact force model requires to be 
    instantiated on device.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name LinkedCellGPUWrapper : External methods */
//@{
/** @brief Creates a ContactForce object on device given an XML input
@param root XML node
@param CF device pointer to the ContactForce object to be created */
template <typename T>
__HOST__
void createContactForceOnDevice( DOMNode* root,
                                 HODCContactForceModel<T>** CF );
//@}


#endif