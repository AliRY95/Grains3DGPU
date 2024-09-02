#ifndef _CONTACTFORCEMODELBUILDERFACTORY_HH_
#define _CONTACTFORCEMODELBUILDERFACTORY_HH_


#include "ContactForceModel.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class ContactForceModelBuilderFactory.

	Creates the contact force model for each pair of materials.

    @author A.YAZDANI - 2024 - Construction */
// =============================================================================
template <typename T>
class ContactForceModelBuilderFactory
{
	private:
		/**@name Contructors & Destructor */
		//@{
		/** @brief Default constructor (forbidden) */
		__HOST__
		ContactForceModelBuilderFactory();

		/** @brief Destructor (forbidden) */
		__HOST__
		~ContactForceModelBuilderFactory();
		//@}


	public:
		/**@name Methods */
		//@{
		/** @brief Creates and returns the contact force model given an XML node 
		@param root XML node */
		__HOST__
		static ContactForceModel<T>** create( DOMElement* root );

		/** @brief Hash function to map a pair of material IDs x and y to a 
		single ID to access the contact force model between them
		@param x 1st material ID
		@param y 2nd material ID */
		__HOSTDEVICE__
		static unsigned int computeHash( unsigned int x,
										 unsigned int y,
										 unsigned int N );
		
		/** @brief ContactForceModel objects must be instantiated on device, if 
		we want to use them on device. Copying from host is not supported due to 
		runtime polymorphism for this class.
		This function constructs a ContactForceModel object in a given device 
		memory from an XML node.
		It calls a deivce kernel that is implemented in the source file.
		@param root XML node
		@param d_CF double pointer to a device memory to construct the object */
		__HOST__
		static void ContactForceModelCopyHostToDevice( 
												ContactForceModel<T>** h_CF,
												ContactForceModel<T>** d_CF,
												unsigned int numContactPairs );
		//@}
};


typedef ContactForceModelBuilderFactory<float> ContactForceModelBuilderFactoryF;
typedef ContactForceModelBuilderFactory<double> ContactForceModelBuilderFactoryD;


#endif
