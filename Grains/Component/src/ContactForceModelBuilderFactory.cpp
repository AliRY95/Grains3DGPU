#include "ContactForceModelBuilderFactory.hh"
#include "HODCContactForceModel.hh"
#include "GrainsParameters.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// GPU kernel to construct the ContactForceModel on device.
// This is mandatory as we cannot access device memory addresses on the host
// So, we pass a device memory address to a kernel.
// Memory address is then populated within the kernel.
// This kernel is not declared in any header file since we directly use it below
// It helps to NOT explicitly instantiate it.
template <typename T, typename... Arguments>
__GLOBAL__
void createContactForceModelKernel( ContactForceModel<T>** CF,
									unsigned int index,
									ContactForceModelType contactForceModelType,
									Arguments... args )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid > 0 ) 
        return;
    
	if constexpr ( sizeof...( args ) == 5 )
	{
		if ( contactForceModelType == HODC ) 
				CF[index] = new HODCContactForceModel<T>( args... );
	}
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Creates and returns the contact force model
template <typename T>
__HOST__
ContactForceModel<T>** ContactForceModelBuilderFactory<T>::create( 
															DOMElement* root )
{
	// number of materials stored after reading the particles and obstacles in
	// the XML file
	unsigned int numMaterials = GrainsParameters<T>::m_materialMap.size();
	unsigned int numContactPairs = 
            GrainsParameters<T>::m_materialMap.size() * 
            ( GrainsParameters<T>::m_materialMap.size() - 1 ) / 2;


	ContactForceModel<T>** CF = new ContactForceModel<T>*[numContactPairs];

	DOMNodeList* allContacts = ReaderXML::getNodes( root, "ContactForceModel" );
	for ( XMLSize_t i = 0; i < allContacts->getLength(); i++ ) 
	{
		DOMNode* contact = allContacts->item( i );
		std::string name = ReaderXML::getNodeName( contact );
		// Contact Pair
		DOMNode* material = ReaderXML::getNode( contact, "Material" );
		std::string matA = ReaderXML::getNodeAttr_String( material, "materialA" );
		std::string matB = ReaderXML::getNodeAttr_String( material, "materialB" );
		unsigned int matA_id = GrainsParameters<T>::m_materialMap[ matA ];
		unsigned int matB_id = GrainsParameters<T>::m_materialMap[ matB ];
		unsigned int index = computeHash( matA_id, matB_id, numMaterials );
		// TODO:
		DOMNode* contactType = ReaderXML::getNode( contact, "HODC" );
		CF[index] = new HODCContactForceModel<T>( contactType );
	}

	return( CF );
}




// -----------------------------------------------------------------------------
// Hash function to map a pair of material IDs x and y to a single ID to access
// the contact force model between them.
// In general, we have an array of ContactForceModel objects. The main purpose
// of this function is to know which element of this array should be accessed,
// given the material ID of two rigidBodies.
template <typename T>
__HOSTDEVICE__
unsigned int ContactForceModelBuilderFactory<T>::computeHash( unsigned int x,
															  unsigned int y,
															  unsigned int N )
{
	if ( x < N && y < N )
	{
		unsigned int small = min( x, y );
		unsigned int large = max( x, y );
		return ( ( N - 1 ) * small + large );
	}
	else
	{
		printf( "Wrong inputs in" );
		printf( "ContactForceModelBuilderFactory<T>::computeHash\n" );
		return( 0 );
	}
}




// -----------------------------------------------------------------------------
// Constructs a ContactForceModel object on device.
// It is assumed that appropriate memory is allocated to d_cf.
template <typename T>
__HOST__
void ContactForceModelBuilderFactory<T>::ContactForceModelCopyHostToDevice(
												ContactForceModel<T>** h_cf, 
												ContactForceModel<T>** d_cf,
												unsigned int numContactPairs )
{
    for ( int index = 0; index < numContactPairs; index++ )
    {

		if ( !h_cf[index] )
			continue;
        // Extracting info from the host side object
        ContactForceModel<T>* contact = h_cf[index];
        ContactForceModelType contactType = contact->getContactForceModelType();

        if ( contactType == HODC )
        {
            HODCContactForceModel<T>* c = 
							dynamic_cast<HODCContactForceModel<T>*>( contact );
			T stiff, en, muet, muec, kms;
            c->getContactForceModelParameters( stiff, en, muet, muec, kms );
            createContactForceModelKernel<<<1, 1>>>( d_cf, 
													 index, 
													 HODC, 
													 stiff, en, muet, muec, kms );
        }
        else
        {
            std::cout << "Contact force model type is not implemented for GPU!" 
					  << std::endl
				 	  << "Aborting Grains!" 
                 	  << std::endl;
            exit( 1 );
        }
    }
    cudaDeviceSynchronize();
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ContactForceModelBuilderFactory<float>;
template class ContactForceModelBuilderFactory<double>;
