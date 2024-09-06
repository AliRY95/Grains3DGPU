#include "Grains.hh"
#include "RigidBodyGPUWrapper.hh"
#include "LinkedCellGPUWrapper.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "TimeIntegratorBuilderFactory.hh"
#include "VectorMath.hh"


/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Default constructor
template <typename T>
Grains<T>::Grains()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
Grains<T>::~Grains()
{}




// -----------------------------------------------------------------------------
// Initializes the simulation using the XML input
template <typename T>
void Grains<T>::initialize( DOMElement* rootElement )
{   
    // Read the input file
    // Type - CPU or GPU
    string option = ReaderXML::getNodeAttr_String( rootElement, "Type" );
    if ( option == "Standard" ) 
        GrainsParameters<T>::m_isGPU = false;
    else if ( option == "GPU" )
        GrainsParameters<T>::m_isGPU = true;
    

    /* */
    Construction( rootElement );
    Forces( rootElement );
    // AdditionalFeatures( rootElement );
}




/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Constructs the simulation -- Reads the Construction part of the XML input to
// set the parameters
template <typename T>
void Grains<T>::Construction( DOMElement* rootElement )
{
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
    if ( !root )
    {
        cout << shiftString0 << "Construction section cannot be found!" << endl;
        exit( 1 );
    }




    // -------------------------------------------------------------------------
    // Domain size: origin, max coordinates and periodicity
    DOMNode* domain = ReaderXML::getNode( root, "LinkedCell" );
    GrainsParameters<T>::m_dimension.setValue( 
        T( ReaderXML::getNodeAttr_Double( domain, "MX" ) ),
        T( ReaderXML::getNodeAttr_Double( domain, "MY" ) ),
        T( ReaderXML::getNodeAttr_Double( domain, "MZ" ) ) );

    DOMNode* domain_origin = ReaderXML::getNode( root, "Origin" );
    if ( domain_origin )
        GrainsParameters<T>::m_origin.setValue( 
            T( ReaderXML::getNodeAttr_Double( domain_origin, "OX" ) ),
            T( ReaderXML::getNodeAttr_Double( domain_origin, "OY" ) ),
            T( ReaderXML::getNodeAttr_Double( domain_origin, "OZ" ) ) );
    else
        GrainsParameters<T>::m_origin.setValue( T( 0 ), T( 0 ), T( 0 ) );

    // if the simulation is periodic
    GrainsParameters<T>::m_isPeriodic = false;




    // -------------------------------------------------------------------------
    // Particles
    DOMNode* particles = ReaderXML::getNode( root, "Particles" );
    DOMNodeList* allParticles = ReaderXML::getNodes( rootElement, "Particle" );
    // Number of unique shapes (rigid bodies) in the simulation
    unsigned int numRigidBodies = int( allParticles->getLength() );
    // Number of each unique shape in the simulation. For simplicity, we keep it
    // accumulative. Vector [2, 5] means we have 2 id0 ridid bodies and then 
    // 5 - 2 = 3 id1 rigid bodies. it also indicates that there are 5 particles
    // in the simulation.
    std::vector<unsigned int> numEachRigidBody( numRigidBodies, 0 );
    T linkedCellSize = T( 0 );
    if ( particles )
    {
        cout << shiftString6 << "Reading new particle types ..." << endl;

        // Memory allocation for m_rigidBodyList with respect to the number of 
        // shapes in the simulation
        m_rigidBodyList = ( RigidBody<T, T>** ) malloc( 
                            numRigidBodies * sizeof( RigidBody<T, T>* ) );
        
        // Populating the array with different kind of rigid bodies in the XML
        // file
        for ( int i = 0; i < numRigidBodies; i++ )
        {
            DOMNode* nParticle = allParticles->item( i );
            if ( i == 0 )
                numEachRigidBody[ i ] =  
                            ReaderXML::getNodeAttr_Int( nParticle, "Number" );
            else
                numEachRigidBody[ i ] = numEachRigidBody[ i - 1 ] + 
                            ReaderXML::getNodeAttr_Int( nParticle, "Number" );

            // Create the Rigid Body
            m_rigidBodyList[ i ] = new RigidBody<T, T>( nParticle );

            // Finding the max circumscribed radius among all shapes
            if ( m_rigidBodyList[i]->getCircumscribedRadius() > linkedCellSize )
                linkedCellSize = m_rigidBodyList[i]->getCircumscribedRadius();
        }

        // if it is a GPU simulation, we allocate memory on device as well 
        if ( GrainsParameters<T>::m_isGPU )
        {
            cudaErrCheck( cudaMalloc( (void**) &m_d_rigidBodyList,
                            numRigidBodies * sizeof( RigidBody<T, T>* ) ) );
            RigidBodyCopyHostToDevice( m_rigidBodyList, 
                                       m_d_rigidBodyList,
                                       numRigidBodies );
            cudaDeviceSynchronize();
        }

        cout << shiftString6 << "Reading particle types completed!" << endl;
    }




    // -------------------------------------------------------------------------
    // Scaling coefficient of linked cell size
    cout << shiftString6 << "Constructing linked cell ..." << endl;
    T LC_coeff = T( 1 );
    DOMNode* nLC = ReaderXML::getNode( root, "LinkedCell" );
    if ( ReaderXML::hasNodeAttr( nLC, "CellSizeFactor" ) )
      LC_coeff = T( ReaderXML::getNodeAttr_Double( nLC, "CellSizeFactor" ) );
    if ( LC_coeff < T( 1 ) ) 
        LC_coeff = T( 1 );
    cout << shiftString9 << "Cell size factor = " << LC_coeff << endl;

    // Creating linked cell
    // TODO: FIX when finalizing
    // if ( !GrainsParameters<T>::m_isGPU )
    // {
        m_linkedCell = ( LinkedCell<T>** ) malloc( sizeof( LinkedCell<T>* ) );
        *m_linkedCell = new LinkedCell<T>( 
            GrainsParameters<T>::m_origin,
            GrainsParameters<T>::m_origin + GrainsParameters<T>::m_dimension, 
            LC_coeff * T( 2 ) * linkedCellSize );
        GrainsParameters<T>::m_numCells = (*m_linkedCell)->getNumCells();
        cout << shiftString6 << "LinkedCell with "
             << GrainsParameters<T>::m_numCells <<
             " cells is created on host." << endl;
    // }
    // else
    // {
        cudaErrCheck( cudaMalloc( (void**)&m_d_linkedCell,
                                   sizeof( LinkedCell<T>* ) ) );
        int d_numCells = createLinkedCellOnDevice( 
            GrainsParameters<T>::m_origin,
            GrainsParameters<T>::m_origin + GrainsParameters<T>::m_dimension, 
            LC_coeff * T( 2 ) * linkedCellSize,
            m_d_linkedCell );

        GrainsParameters<T>::m_numCells = d_numCells;
        cout << shiftString6 << "LinkedCell with "
             << GrainsParameters<T>::m_numCells <<
             " cells is created on device." << endl;
    // }
    



    // -------------------------------------------------------------------------
    // Setting up the component managers
    GrainsParameters<T>::m_numComponents = numEachRigidBody.back();
    m_components = new ComponentManagerCPU<T>( numEachRigidBody, 
                                               0,
                                               GrainsParameters<T>::m_numCells );
    if ( GrainsParameters<T>::m_isGPU )
    {
        m_d_components = new ComponentManagerGPU<T>( GrainsParameters<T>::m_numComponents,
                                                     0,
                                                     GrainsParameters<T>::m_numCells );
        m_d_components->copy( m_components );
    }

    


    // -------------------------------------------------------------------------
    // Contact force models
    DOMNode* contacts = ReaderXML::getNode( root, "ContactForceModels" );
    if ( contacts )
    {
        cout << shiftString6 
             << "Reading the contact force model ..." 
             << endl;
        unsigned int numContactPairs = 1;
            // GrainsParameters<T>::m_materialMap.size() * 
            // ( GrainsParameters<T>::m_materialMap.size() - 1 ) / 2;
        m_contactForce = ContactForceModelBuilderFactory<T>::create( rootElement );
        if ( GrainsParameters<T>::m_isGPU )
        {
            cudaErrCheck( cudaMalloc( (void**)&m_d_contactForce,
                        numContactPairs * sizeof( ContactForceModel<T>* ) ) );
            ContactForceModelBuilderFactory<T>::
            ContactForceModelCopyHostToDevice( m_contactForce,
                                               m_d_contactForce, 
                                               numContactPairs );
        }
        cout << shiftString6 
             << "Reading the contact force model completed!" 
             << endl;
    }




    // -------------------------------------------------------------------------
    // Temporal setting and time integration
    DOMNode* tempSetting = ReaderXML::getNode( root, "TemporalSetting" );
    if ( tempSetting )
    {
        DOMNode* nTime = ReaderXML::getNode( tempSetting, "TimeInterval" );
        T tStart = ReaderXML::getNodeAttr_Double( nTime, "Start" );
        T tEnd = ReaderXML::getNodeAttr_Double( nTime, "End" );
        T tStep = ReaderXML::getNodeAttr_Double( nTime, "dt" );
        GrainsParameters<T>::m_tStart = tStart;
        GrainsParameters<T>::m_tEnd = tEnd;
        GrainsParameters<T>::m_dt = tStep;
        DOMNode* nTI = ReaderXML::getNode( tempSetting, "TimeIntegration" );
        if ( nTI )
        {
            cout << shiftString6 
                 << "Reading the time integration model ..." 
                 << endl;
            m_timeIntegrator = ( TimeIntegrator<T>** ) 
                                malloc( sizeof( TimeIntegrator<T>* ) );
            *m_timeIntegrator = TimeIntegratorBuilderFactory<T>::create( nTI, 
                                                                        tStep );
            if ( GrainsParameters<T>::m_isGPU )
            {
                cudaErrCheck( cudaMalloc( (void**)&m_d_timeIntegrator,
                                          sizeof( TimeIntegrator<T>* ) ) );
                TimeIntegratorBuilderFactory<T>::createOnDevice( 
                                                        nTI,
                                                        tStep, 
                                                        m_d_timeIntegrator );
            }
            cout << shiftString6 
                 << "Reading the time integration model completed!" 
                 << endl;
        }
    }
}




// -----------------------------------------------------------------------------
// External force definition
template <typename T>
void Grains<T>::Forces( DOMElement* rootElement )
{
    assert( rootElement != NULL );
    DOMNode* root = ReaderXML::getNode( rootElement, "Forces" );

    // Output message
    cout << shiftString6 << "Forces" << endl;


    // Read the forces
    if ( root )
    {
        // Gravity
        DOMNode* nGravity = ReaderXML::getNode( root, "Gravity" );
        if( nGravity )
        {
            GrainsParameters<T>::m_gravity[X] = 
                T( ReaderXML::getNodeAttr_Double( nGravity, "GX" ) );
            GrainsParameters<T>::m_gravity[Y] = 
                T( ReaderXML::getNodeAttr_Double( nGravity, "GY" ) );
            GrainsParameters<T>::m_gravity[Z] = 
                T( ReaderXML::getNodeAttr_Double( nGravity, "GZ" ) );
            cout << shiftString9 
                    << "Gravity = " 
                    << GrainsParameters<T>::m_gravity 
                    << endl;
        }
        else
        {
            cout << shiftString6 << "Gravity is mandatory!" << endl;
            exit( 1 );
        }
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Grains<float>;
template class Grains<double>;