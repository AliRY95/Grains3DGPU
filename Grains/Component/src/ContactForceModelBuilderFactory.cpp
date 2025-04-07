#include "ContactForceModelBuilderFactory.hh"
#include "GrainsParameters.hh"
#include "GrainsUtils.hh"
#include "HookeContactForceModel.hh"

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
__GLOBAL__ void createContactForceModelKernel(ContactForceModel<T>** CF,
                                              unsigned int           index,
                                              ContactForceModelType  contactForceModelType,
                                              Arguments... args)
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid > 0)
        return;

    if constexpr(sizeof...(args) == 5)
    {
        if(contactForceModelType == HOOKE)
            CF[index] = new HookeContactForceModel<T>(args...);
    }
}

/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Creates and returns the contact force model
template <typename T>
__HOST__ ContactForceModel<T>** ContactForceModelBuilderFactory<T>::create(DOMElement* root)
{
    unsigned int           numContactPairs = GrainsParameters<T>::m_numContactPairs;
    ContactForceModel<T>** CF              = new ContactForceModel<T>*[numContactPairs];
    DOMNodeList*           allContacts     = ReaderXML::getNodes(root, "ContactForceModel");
    for(XMLSize_t i = 0; i < allContacts->getLength(); i++)
    {
        DOMNode*    contact     = allContacts->item(i);
        std::string contactType = ReaderXML::getNodeAttr_String(contact, "Type");
        // Contact Pair
        DOMNode*     material = ReaderXML::getNode(contact, "Material");
        std::string  matA     = ReaderXML::getNodeAttr_String(material, "materialA");
        std::string  matB     = ReaderXML::getNodeAttr_String(material, "materialB");
        unsigned int matA_id  = GrainsParameters<T>::m_materialMap[matA];
        unsigned int matB_id  = GrainsParameters<T>::m_materialMap[matB];
        unsigned int index    = computeHash(matA_id, matB_id);
        // Contact Parameters
        DOMNode* parameters = ReaderXML::getNode(contact, "Parameters");
        if(contactType == "Hooke")
            CF[index] = new HookeContactForceModel<T>(parameters);
        else
        {
            Gout(0, 9, "Contact force model is not known.", "Aborting Grains!");
            exit(1);
        }
    }

    return (CF);
}

// -------------------------------------------------------------------------------------------------
// Hash function to map a pair of material IDs x and y to a single ID to access
// the proper contact force model between them.
// We DO NOT CHECK the inputs. We TRUST the user on this. However, inproper
// inputs would persumably crash the code by accessing memory accesses that are
// not valid.
template <typename T>
__HOSTDEVICE__ unsigned int ContactForceModelBuilderFactory<T>::computeHash(unsigned int x,
                                                                            unsigned int y)
{
    unsigned int s = min(x, y); // smaller one
    unsigned int l = max(x, y); // larger one
    return (l * (l + 1) / 2 + s);

    // unsigned int numPM = numParticleMaterials<T>;
    // unsigned int s = min( x, y ); // smaller one
    // unsigned int l = max( x, y ); // larger one
    // if ( l < numPM )
    // 	return ( l * ( l + 1 ) / 2 + s );
    // // offset if contact between a part and obst
    // else if ( l >= numPM )
    // 	return ( numPM * (numPM + 1) / 2 + s + ( l - numPM ) * numPM );
    // else
    // {
    // 	printf( "Wrong inputs in" );
    // 	printf( "ContactForceModelBuilderFactory<T>::computeHash\n" );
    // 	printf( "Contact between two obstacles is not handled!\n" );
    // 	return ( 0 );
    // }
}

// -------------------------------------------------------------------------------------------------
// Constructs a ContactForceModel object on device.
// It is assumed that appropriate memory is allocated to d_cf.
template <typename T>
__HOST__ void ContactForceModelBuilderFactory<T>::ContactForceModelCopyHostToDevice(
    ContactForceModel<T>** h_cf, ContactForceModel<T>** d_cf)
{
    for(int i = 0; i < GrainsParameters<T>::m_numContactPairs; i++)
    {
        if(!h_cf[i])
            continue;
        // Extracting info from the host side object
        ContactForceModel<T>* contact     = h_cf[i];
        ContactForceModelType contactType = contact->getContactForceModelType();

        if(contactType == HOOKE)
        {
            HookeContactForceModel<T>* c = dynamic_cast<HookeContactForceModel<T>*>(contact);
            T                          kn, en, etat, muc, kr;
            c->getContactForceModelParameters(kn, en, etat, muc, kr);
            createContactForceModelKernel<<<1, 1>>>(d_cf, i, HOOKE, kn, en, etat, muc, kr);
        }
        else
        {
            Gout(0, 9, "Contact force model is not implemented for GPU!", "Aborting Grains!");
            exit(1);
        }
    }
    cudaDeviceSynchronize();
}

// -------------------------------------------------------------------------------------------------
// Explicit instantiation
template class ContactForceModelBuilderFactory<float>;
template class ContactForceModelBuilderFactory<double>;
