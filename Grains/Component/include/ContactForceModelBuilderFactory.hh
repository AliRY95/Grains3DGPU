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
		/** @brief Creates and returns the contact force model */
		__HOST__
		static ContactForceModel<T>* create( DOMNode* root );
		
		/** @brief ContactForceModel objects must be instantiated on device, if 
		we want to use them on device. Copying from host is not supported due to 
		runtime polymorphism for this class.
		This function constructs a ContactForceModel object in a given device 
		memory from an XML node.
		It calls a deivce kernel that is implemented in the source file.
		@param root XML node
		@param d_CF double pointer to a device memory to construct the object */
		static void createOnDevice( DOMNode* root,
									TimeIntegrator<T>** d_CF );
		//@}
};


typedef ContactForceModelBuilderFactory<float> ContactForceModelBuilderFactoryF;
typedef ContactForceModelBuilderFactory<double> ContactForceModelBuilderFactoryD;


#endif
