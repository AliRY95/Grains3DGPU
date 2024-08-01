#include "HODCContactForceModelGPUWrapper.hh"
#include "HODCContactForceModel.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// GPU kernel to construct the linked cell on device.
template <typename T>
__GLOBAL__ 
void createContactForceOnDeviceKernel( T stiff,
                                       T en,
                                       T muet,
                                       T muec,
                                       T kms,
                                       HODCContactForceModel<T>** CF )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid > 0 )
        return;

    *CF = new HODCContactForceModel<T>( stiff, en, muet, muec, kms );
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Constructs a ContactForce object on device given an XML file
template <typename T>
__HOST__
void createContactForceOnDevice( DOMNode* root,
                                 HODCContactForceModel<T>** CF )
{
    DOMNode* parameter;
    parameter = ReaderXML::getNode( root, "stiff" );
    T stiff = T( ReaderXML::getNodeValue_Double( parameter ) );

    parameter = ReaderXML::getNode( root, "muc" );
    T muec = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    parameter = ReaderXML::getNode( root, "en" );
    T en = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    parameter = ReaderXML::getNode( root, "mut" );
    T muet = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    parameter = ReaderXML::getNode( root, "kms" );
    T kms = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    createContactForceOnDeviceKernel<<<1, 1>>>( stiff, en, muet, muec, kms, 
                                                CF );
    cudaDeviceSynchronize();
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T )                                                                 \
template                                                                       \
__HOST__                                                                       \
void createContactForceOnDevice( DOMNode* root,                                \
                                 HODCContactForceModel<T>** CF );
X( float )
X( double )
#undef X