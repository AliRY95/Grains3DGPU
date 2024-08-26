#include "TimeIntegratorBuilderFactory.hh"
#include "FirstOrderExplicit.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// GPU kernel to construct the timeIntegrator on device.
// This is mandatory as we cannot access device memory addresses on the host
// So, we pass a device memory address to a kernel.
// Memory address is then populated within the kernel.
// This kernel is not declared in any header file since we directly use it below
// It helps to NOT explicitly instantiate it.
template <typename T>
__GLOBAL__
void createTimeIntegratorKernel( TimeIntegratorType tiType, 
                                 T dt,
                                 TimeIntegrator<T>** TI )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid > 0 ) 
        return;
    
    if ( tiType == FIRSTORDEREXPLICIT ) 
            *TI = new FirstOrderExplicit<T>( dt );
    else
        printf( "TimeIntegrator is not supported on GPU! Aborting Grains!\n" );

}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Creates and returns the time integration scheme
template <typename T>
__HOST__
TimeIntegrator<T>* TimeIntegratorBuilderFactory<T>::create( DOMNode* root,
														  	T dt )
{
	TimeIntegrator<T>* TI;

    string type = ReaderXML::getNodeAttr_String( root, "Type" );
	if ( type == "FirstOrderExplicit" )
		TI = new FirstOrderExplicit<T>( dt );
	else {
		cout << "Wrong type of time integration schema";
		cout << " in <TimeIntegration Type=\"xxx\"/>" << endl; 
		cout << "Allowed entries for xxx are:" << endl;
		cout << "  * SecondOrderLeapFrog" << endl;
		cout << "  * FirstOrderExplicit" << endl;    
		cout << "  * SecondOrderExplicit" << endl;
		cout << "  * SecondOrderAdamsBashforth" << endl;    
		exit( 1 );
	}

	return ( TI );
}




// -----------------------------------------------------------------------------
// Constructs a TimeIntegrator object on device.
// It is assumed that appropriate memory is allocated to d_ti.
template <typename T>
__HOST__
void TimeIntegratorBuilderFactory<T>::createOnDevice( DOMNode* root,
													  T dt, 
													  TimeIntegrator<T>** d_TI )
{
    string type = ReaderXML::getNodeAttr_String( root, "Type" );
    TimeIntegratorType tiType;
	if ( type == "FirstOrderExplicit" )
		tiType = FIRSTORDEREXPLICIT;
	else {
		cout << "Wrong type of time integration schema";
		cout << " in <TimeIntegration Type=\"xxx\"/>" << endl; 
		cout << "Allowed entries for xxx are:" << endl;
		cout << "  * SecondOrderLeapFrog" << endl;
		cout << "  * FirstOrderExplicit" << endl;    
		cout << "  * SecondOrderExplicit" << endl;
		cout << "  * SecondOrderAdamsBashforth" << endl;    
		exit( 1 );
	}
	createTimeIntegratorKernel<<<1, 1>>>( tiType, dt, d_TI );
	cudaDeviceSynchronize();
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class TimeIntegratorBuilderFactory<float>;
template class TimeIntegratorBuilderFactory<double>;
