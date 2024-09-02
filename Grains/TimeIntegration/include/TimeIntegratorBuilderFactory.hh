#ifndef _TIMEINTEGRATORBUILDERFACTORY_HH_
#define _TIMEINTEGRATORBUILDERFACTORY_HH_


#include "TimeIntegrator.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class TimeIntegratorBuilderFactory.

    Creates the numerical scheme for the time integration of the Newton's law 
    and the kinematic equations. 

    @author A.WACHS - Institut Francais du Petrole - 2011 - Creation 
    @author A.WACHS - 2021 - Major cleaning & refactoring
    @author A.YAZDANI - 2024 - Major cleaning for porting to GPU */
// =============================================================================
template <typename T>
class TimeIntegratorBuilderFactory
{
	private:
		/**@name Contructors & Destructor */
		//@{
		/** @brief Default constructor (forbidden) */
		TimeIntegratorBuilderFactory();

		/** @brief Destructor (forbidden) */
		~TimeIntegratorBuilderFactory();
		//@}


	public:
		/**@name Methods */
		//@{
		/** @brief Creates and returns the time integration scheme */

		static TimeIntegrator<T>* create( DOMNode* root,
										  T dt );
		
		/** @brief TimeIntegrator objects must be instantiated on device, if we 
		want to use them on device. Copying from host is not supported due to 
		runtime polymorphism for this class.
		This function constructs a TimeIntegrator object in a given device 
		memory from an XML node.
		It calls a deivce kernel that is implemented in the source file.
		@param root XML node
		@param dt time step
		@param d_TI double pointer to a device memory to construct the object */
		__HOST__
		static void createOnDevice( DOMNode* root,
									T dt,
									TimeIntegrator<T>** d_TI );
		//@}
};


typedef TimeIntegratorBuilderFactory<float> TimeIntegratorBuilderFactoryF;
typedef TimeIntegratorBuilderFactory<double> TimeIntegratorBuilderFactoryD;


#endif
