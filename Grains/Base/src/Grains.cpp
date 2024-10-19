#include "Grains.hh"
#include "RigidBodyGPUWrapper.hh"
#include "LinkedCellGPUWrapper.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "TimeIntegratorBuilderFactory.hh"
#include "PostProcessingWriterBuilderFactory.hh"
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
{
    // TODO
    delete m_insertion;
}




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
    AdditionalFeatures( rootElement );
}




/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Constructs the simulation -- Reads the Construction part of the XML input to
// set the parameters
template <typename T>
void Grains<T>::Construction( DOMElement* rootElement )
{
    // Output message
    cout << shiftString3 << "Construction" << endl;
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
    if ( !root )
    {
        cout << shiftString0 << "Construction node is mandatory!" << endl;
        exit( 1 );
    }




    // -------------------------------------------------------------------------
    // Domain size: origin, max coordinates and periodicity
    DOMNode* domain = ReaderXML::getNode( root, "LinkedCell" );
    GrainsParameters<T>::m_maxCoordinate.setValue( 
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
    unsigned int numUniqueParticles = allParticles->getLength();
    // Number of each unique shape in the simulation. For simplicity, we keep it
    // accumulative. Vector [2, 5] means we have 2 id0 rigid bodies and then 
    // 5 - 2 = 3 id1 rigid bodies. it also indicates that there are 5 different
    // rigid bodies in the simulation in total.
    std::vector<unsigned int> numEachUniqueParticle( numUniqueParticles, 0 );
    // We also store the initial transformations of the rigid bodies to pass to 
    // the ComponentManager to create particles with the initial transformation
    // required.
    std::vector<Transform3<T>> particlesInitialTransform;
    particlesInitialTransform.reserve( numUniqueParticles );
    // Memory allocation for m_rigidBodyList with respect to the number of
    // shapes in the simulation.
    m_particleRigidBodyList = 
                        ( RigidBody<T, T>** ) malloc( 
                        numUniqueParticles * sizeof( RigidBody<T, T>* ) );
    if ( particles )
    {
        cout << shiftString6 << "Reading new particle types ..." << endl;
        
        // Populating the array with different kind of rigid bodies in the XML
        // file
        for ( int i = 0; i < numUniqueParticles; i++ )
        {
            DOMNode* nParticle = allParticles->item( i );
            if ( i == 0 )
                numEachUniqueParticle[ i ] =  
                            ReaderXML::getNodeAttr_Int( nParticle, "Number" );
            else
                numEachUniqueParticle[ i ] = numEachUniqueParticle[ i - 1 ] + 
                            ReaderXML::getNodeAttr_Int( nParticle, "Number" );

            // Create the Rigid Body
            m_particleRigidBodyList[ i ] = new RigidBody<T, T>( nParticle );

            // Initial transformation of the rigid body
            // One draw back is we might end up with the same rigid body shape, 
            // but with different initial transformation. 
            particlesInitialTransform.push_back( Transform3<T>( nParticle ) );
        }

        cout << shiftString6 << "Reading particle types completed!\n" << endl;
    }
    // if it is a GPU simulation and we actually have rigid bodies in the
    // simulation, we allocate memory on device as well.
    if ( GrainsParameters<T>::m_isGPU && numUniqueParticles > 0 )
    {
        cudaErrCheck( cudaMalloc( (void**) &m_d_particleRigidBodyList,
                      numUniqueParticles * sizeof( RigidBody<T, T>* ) ) );
        RigidBodyCopyHostToDevice( m_particleRigidBodyList, 
                                   m_d_particleRigidBodyList,
                                   numUniqueParticles );
        cudaDeviceSynchronize();
    }

    


    // -------------------------------------------------------------------------
    // Obstacles
    DOMNode* obstacles = ReaderXML::getNode( root, "Obstacles" );
    DOMNodeList* allObstacles = ReaderXML::getNodes( rootElement, "Obstacle" );
    // Number of unique shapes (rigid bodies) in the simulation
    unsigned int numUniqueObstacles = allObstacles->getLength();
    // Number of each unique shape in the simulation. For simplicity, we keep it
    // accumulative. Vector [2, 5] means we have 2 id0 rigid bodies and then 
    // 5 - 2 = 3 id1 rigid bodies. it also indicates that there are 5 different
    // rigid bodies in the simulation in total.
    std::vector<unsigned int> numEachUniqueObstacle( numUniqueObstacles, 0 );
    // We also store the initial transformations of the rigid bodies to pass to 
    // the ComponentManager to create particles with the initial transformation
    // required.
    std::vector<Transform3<T>> obstaclesInitialTransform;
    obstaclesInitialTransform.reserve( numUniqueObstacles );
    // Memory allocation for m_rigidBodyList with respect to the number of
    // shapes in the simulation.
    m_obstacleRigidBodyList = 
                        ( RigidBody<T, T>** ) malloc( 
                        numUniqueObstacles * sizeof( RigidBody<T, T>* ) );
    if ( obstacles )
    {
        cout << shiftString6 << "Reading new obstacle types ..." << endl;
        
        // Populating the array with different kind of rigid bodies in the XML
        // file
        for ( int i = 0; i < numUniqueObstacles; i++ )
        {
            DOMNode* nObstacle = allObstacles->item( i );
            // We only add one because the obstacles are added one by one in the
            // XML file
            if ( i == 0 )
                numEachUniqueObstacle[ i ] = 1;
            else
                numEachUniqueObstacle[ i ] = numEachUniqueObstacle[ i - 1 ] + 1;

            // Create the Rigid Body
            m_obstacleRigidBodyList[ i ] = new RigidBody<T, T>( nObstacle );

            // Initial transformation of the rigid body
            // One draw back is we might end up with the same rigid body shape, 
            // but with different initial transformation. 
            obstaclesInitialTransform.push_back( Transform3<T>( nObstacle ) );
        }

        cout << shiftString6 << "Reading obstacle types completed!\n" << endl;
    }
    // if it is a GPU simulation and we actually have rigid bodies in the
    // simulation, we allocate memory on device as well.
    if ( GrainsParameters<T>::m_isGPU && numUniqueObstacles > 0 )
    {
        cudaErrCheck( cudaMalloc( (void**) &m_d_obstacleRigidBodyList,
                      numUniqueObstacles * sizeof( RigidBody<T, T>* ) ) );
        RigidBodyCopyHostToDevice( m_obstacleRigidBodyList, 
                                   m_d_obstacleRigidBodyList,
                                   numUniqueObstacles );
        cudaDeviceSynchronize();
    }





    // -------------------------------------------------------------------------
    // LinkedCell
    cout << shiftString6 << "Constructing linked cell ..." << endl;
    // Finding the size of each cell -- max circumscribed radius among all
    // particles. Note that we loop until numParticles, because we do not care
    // about the circumscribed radius of the obstacles.
    T LCSize = T( 0 );
    for ( int i = 0; i < numUniqueParticles; i++ )
    {
        if ( m_particleRigidBodyList[i]->getCircumscribedRadius() > LCSize )
            LCSize = m_particleRigidBodyList[i]->getCircumscribedRadius();
    }
    // Scaling coefficient of linked cell size
    T LC_coeff = T( 1 );
    DOMNode* nLC = ReaderXML::getNode( root, "LinkedCell" );
    if ( ReaderXML::hasNodeAttr( nLC, "CellSizeFactor" ) )
      LC_coeff = T( ReaderXML::getNodeAttr_Double( nLC, "CellSizeFactor" ) );
    if ( LC_coeff < T( 1 ) ) 
        LC_coeff = T( 1 );
    cout << shiftString9 << "Cell size factor = " << LC_coeff << endl;

    // Creating linked cell
    m_linkedCell = ( LinkedCell<T>** ) malloc( sizeof( LinkedCell<T>* ) );
    *m_linkedCell = new LinkedCell<T>( GrainsParameters<T>::m_origin,
                                       GrainsParameters<T>::m_maxCoordinate, 
                                       LC_coeff * T( 2 ) * LCSize );
    GrainsParameters<T>::m_numCells = (*m_linkedCell)->getNumCells();
    cout << shiftString9 << "LinkedCell with "
         << GrainsParameters<T>::m_numCells
         << " cells is created on host." << endl;
    if ( GrainsParameters<T>::m_isGPU )
    {
        cudaErrCheck( cudaMalloc( (void**)&m_d_linkedCell,
                                   sizeof( LinkedCell<T>* ) ) );
        int d_numCells = createLinkedCellOnDevice( 
            GrainsParameters<T>::m_origin,
            GrainsParameters<T>::m_maxCoordinate, 
            LC_coeff * T( 2 ) * LCSize,
            m_d_linkedCell );

        GrainsParameters<T>::m_numCells = d_numCells;
        cout << shiftString9 << "LinkedCell with "
             << GrainsParameters<T>::m_numCells
             << " cells is created on device." << endl;
    }
    cout << shiftString6 << "Constructing linked cell completed!\n" << endl;
    



    // -------------------------------------------------------------------------
    // Setting up the component managers
    GrainsParameters<T>::m_numParticles = 0;
    GrainsParameters<T>::m_numObstacles = 0;
    if ( !numEachUniqueParticle.empty() )
        GrainsParameters<T>::m_numParticles = numEachUniqueParticle.back();
    if ( !numEachUniqueObstacle.empty() )
    GrainsParameters<T>::m_numObstacles = numEachUniqueObstacle.back();
    m_components =
            new ComponentManagerCPU<T>( GrainsParameters<T>::m_numParticles,
                                        GrainsParameters<T>::m_numObstacles,
                                        GrainsParameters<T>::m_numCells );
    // Initialize the particles and obstacles
    if ( !numEachUniqueParticle.empty() )
        m_components->initializeParticles( numEachUniqueParticle,
                                           particlesInitialTransform );
    if ( !numEachUniqueObstacle.empty() )
        m_components->initializeObstacles( numEachUniqueObstacle,
                                           obstaclesInitialTransform );
    // In case of GPU simulation
    if ( GrainsParameters<T>::m_isGPU )
    {
        m_d_components = 
            new ComponentManagerGPU<T>( GrainsParameters<T>::m_numParticles,
                                        GrainsParameters<T>::m_numObstacles,
                                        GrainsParameters<T>::m_numCells );
        m_d_components->copy( m_components );
    }

    


    // -------------------------------------------------------------------------
    // Contact force models
    // Calculating the proper array size for the contact force array
    // We might need to redute it by removing the obstacle-obstacle pairs,
    // but it should be fine by now
    unsigned int numMaterials = GrainsParameters<T>::m_materialMap.size();
    GrainsParameters<T>::m_numContactPairs = 
                                        numMaterials * ( numMaterials + 1 ) / 2;
    DOMNode* contacts = ReaderXML::getNode( root, "ContactForceModels" );
    if ( contacts )
    {
        cout << shiftString6 
             << "Reading the contact force model ..." 
             << endl;
        m_contactForce = ContactForceModelBuilderFactory<T>::create( rootElement );
        if ( GrainsParameters<T>::m_isGPU )
        {
            cudaErrCheck( cudaMalloc( (void**)&m_d_contactForce,
                                    GrainsParameters<T>::m_numContactPairs *
                                    sizeof( ContactForceModel<T>* ) ) );
            ContactForceModelBuilderFactory<T>::
            ContactForceModelCopyHostToDevice( m_contactForce,
                                               m_d_contactForce );
        }
        cout << shiftString6 
             << "Reading the contact force model completed!\n" 
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
        GrainsParameters<T>::m_tEnd = tEnd + HIGHEPS<T>;
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
                 << "Reading the time integration model completed!\n" 
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
    cout << shiftString3 << "Forces" << endl;


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
            cout << shiftString6 
                    << "Gravity = " 
                    << GrainsParameters<T>::m_gravity 
                    << "\n"
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
// Additional features of the simulation: insertion, post-processing
template <typename T>
void Grains<T>::AdditionalFeatures( DOMElement* rootElement )
{
    // Output message
    cout << shiftString3 << "Simulation" << endl;
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    assert( rootElement != NULL );
    DOMNode* root = ReaderXML::getNode( rootElement, "Simulation" );
    if ( !root )
    {
        cout << shiftString3 << "Simulation node is mandatory!" << endl;
        exit( 1 );
    }




    // -------------------------------------------------------------------------
    // Insertion policies
    DOMNode* nInsertion = ReaderXML::getNode( root, "ParticleInsertion" );
    cout << shiftString6 << "Reading insertion policies ..." << endl;
    if ( nInsertion )
        m_insertion = new Insertion<T>( nInsertion );
    else
    {
        cout << shiftString9 
             << "No policy found, setting the insertion policy to default" 
             << endl;
        m_insertion = new Insertion<T>();
    }
    cout << shiftString6 << "Reading insertion policies completed.\n" << endl;




    // -------------------------------------------------------------------------
    // Post-processing writers
    DOMNode* nPostProcessing = ReaderXML::getNode( root, "PostProcessing" );
    if ( nPostProcessing )
    {
        cout << shiftString6 << "Post-processing" << endl;
        // Post-processing save time
        DOMNode* nTime = ReaderXML::getNode( nPostProcessing, "TimeSave" );
        T tStart = ReaderXML::getNodeAttr_Double( nTime, "Start" );
        T tEnd = ReaderXML::getNodeAttr_Double( nTime, "End" );
        T tStep = ReaderXML::getNodeAttr_Double( nTime, "dt" );
        for ( T t = tStart; t <= tEnd; t += tStep )
            GrainsParameters<T>::m_tSave.push( t );
        // Save for tEnd as well
        GrainsParameters<T>::m_tSave.push( tEnd );
        
        // Post-processing writers
        DOMNode* nWriters = ReaderXML::getNode( nPostProcessing, "Writers" );
        if ( nWriters )
        {
            cout << shiftString6 
                 << "Reading the post processing writers ..." 
                 << endl;
            DOMNodeList* allPPW = ReaderXML::getNodes( nWriters );
            for ( XMLSize_t i = 0; i < allPPW->getLength(); i++ )
            {
                DOMNode* nPPW = allPPW->item( i );
                PostProcessingWriter<T>* ppw =
                m_postProcessor = PostProcessingWriterBuilderFactory<T>::create( nPPW );
                if ( !ppw )
                {
                    cout << shiftString6 
                         << "Unknown postprocessing writer in node <Writers>"
                         << endl;
                    exit( 1 );
                }
            }
            cout << shiftString6 
                 << "Reading the post processing writers completed!" 
                 << endl;
        }
    }
    else
        cout << shiftString6 << "No postprocessing writer!" << endl;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Grains<float>;
template class Grains<double>;