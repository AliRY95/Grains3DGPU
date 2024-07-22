#include "Grains.hh"
#include "VectorMath.hh"
#include "ComponentManagerCPU.hh"
#include "ComponentManagerGPU.hh"
#include "RigidBody.hh"
#include "RigidBodyGPUWrapper.hh"
#include "LinkedCell_Kernels.hh"


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
    // Forces( rootElement );
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
    DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
    if ( !root )
    {
        cout << shiftString0 << "Construction section cannot be found!" << endl;
        exit( 1 );
    }

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


    // Particles
    DOMNode* particles = ReaderXML::getNode( root, "Particles" );
    DOMNodeList* allParticles = ReaderXML::getNodes( rootElement, "Particle" );
    int numRigidBodies = int( allParticles->getLength() );
    int numEachRigidBody[ numRigidBodies ];
    int numTotalParticles = 0;
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
            numEachRigidBody[ i ] = 
                            ReaderXML::getNodeAttr_Int( nParticle, "Number" );

            // Create the Rigid Body
            m_rigidBodyList[ i ] = new RigidBody<T, T>( nParticle );

            // Finding the max circumscribed radius among all shapes
            if ( m_rigidBodyList[i]->getCircumscribedRadius() > linkedCellSize )
                linkedCellSize = m_rigidBodyList[i]->getCircumscribedRadius();
            
            // Sum to find total number of particles
            numTotalParticles += numEachRigidBody[ i ];
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

    // Scaling coefficient of linked cell size
    cout << shiftString6 << "Constructing linked cell ..." << endl;
    T LC_coeff = T( 1 );
    DOMNode* nLC = ReaderXML::getNode( root, "LinkedCell" );
    if ( ReaderXML::hasNodeAttr( nLC, "CellSizeFactor" ) )
      LC_coeff = T( ReaderXML::getNodeAttr_Double( nLC, "CellSizeFactor" ) );
    if ( LC_coeff < T( 1 ) ) 
        LC_coeff = T( 1 );
    cout << shiftString9 << "Cell size factor = " << LC_coeff << endl;


    if ( GrainsParameters<T>::m_isGPU )
    {
        // allocating memory for only one int for the number of cells
        int* d_numCells;
        cudaMalloc( ( void** ) &d_numCells, sizeof( int ) );


        cudaErrCheck( cudaMalloc( (void**)&m_d_linkedCell,
                                    sizeof( LinkedCell<T>* ) ) );
        createLinkedCellOnGPU<<<1, 1>>>( 
            GrainsParameters<T>::m_origin[X],
            GrainsParameters<T>::m_origin[Y],
            GrainsParameters<T>::m_origin[Z],
            GrainsParameters<T>::m_origin[X] + GrainsParameters<T>::m_dimension[X], 
            GrainsParameters<T>::m_origin[Y] + GrainsParameters<T>::m_dimension[Y], 
            GrainsParameters<T>::m_origin[Z] + GrainsParameters<T>::m_dimension[Z], 
            LC_coeff * T( 2 ) * linkedCellSize,
            m_d_linkedCell,
            d_numCells );

        // copying the variable to host
        cudaMemcpy( &GrainsParameters<T>::m_numCells, 
                    d_numCells, 
                    sizeof( int ), 
                    cudaMemcpyDeviceToHost );
        cudaDeviceSynchronize();
        cout << shiftString6 << "LinkedCell with "
             << GrainsParameters<T>::m_numCells <<
             " cells is created on device." << endl;
    }
    // else
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

    // Setting the number of particles in the simulation
    GrainsParameters<T>::m_numComponents = numTotalParticles;
    m_components = new ComponentManagerCPU<T>();
    if ( GrainsParameters<T>::m_isGPU )
        m_d_components = new ComponentManagerGPU<T>( *m_components );
    // else
    //     m_components = new ComponentManagerCPU<T>();

//     // Link obstacles with the linked cell grid
//     m_collision->Link( m_allcomponents.getObstacles() );
//   }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Grains<float>;
template class Grains<double>;