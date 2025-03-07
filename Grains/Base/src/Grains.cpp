#include "Grains.hh"
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
{
    GrainsMisc<T>::cout( std::string( 80, '=' ) );
    GrainsMisc<T>::cout( "Starting Grains3D ..." );
    GrainsMisc<T>::cout( std::string( 80, '=' ), 0, 1 );
}




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
    // Set the simulation type to default. It will be overridden later if needed
    GrainsParameters<T>::m_isGPU = false;

    // Reading different blocks of the input XML
    GrainsMisc<T>::cout( std::string( 80, '=' ) );
    GrainsMisc<T>::cout( "Reading the input file ..." );
    GrainsMisc<T>::cout( std::string( 80, '=' ) );
    Construction( rootElement );
    Forces( rootElement );
    AdditionalFeatures( rootElement );
    GrainsMisc<T>::cout( std::string( 80, '=' ) );
    GrainsMisc<T>::cout( "Reading the input file completed!" );
    GrainsMisc<T>::cout( std::string( 80, '=' ), 0, 1 );
}




// -----------------------------------------------------------------------------
// Performs post-processing
template <typename T>
void Grains<T>::postProcess( ComponentManager<T> const* cm ) const
{   
    if ( GrainsParameters<T>::m_tSave.front() - GrainsParameters<T>::m_time < 
         0.01 * GrainsParameters<T>::m_dt )
    {
        GrainsParameters<T>::m_tSave.pop();
        Grains<T>::m_postProcessor->PostProcessing( 
                                            Grains<T>::m_particleRigidBodyList,
                                            Grains<T>::m_obstacleRigidBodyList,
                                            cm,
                                            GrainsParameters<T>::m_time );
    }
    // In case we get past the saveTime, we need to remove it from the queue
    if ( GrainsParameters<T>::m_time > GrainsParameters<T>::m_tSave.front() )
    {
        GrainsParameters<T>::m_tSave.pop();
    }
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
    GrainsMisc<T>::cout( "Construction", 3 );
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
    if ( !root )
    {
        GrainsMisc<T>::cout( "Construction node is mandatory!" );
        exit( 1 );
    }




    // -------------------------------------------------------------------------
    // Domain size: origin, max coordinates and periodicity
    DOMNode* nDomain = ReaderXML::getNode( root, "LinkedCell" );
    GrainsParameters<T>::m_maxCoordinate.setValue( 
        T( ReaderXML::getNodeAttr_Double( nDomain, "MX" ) ),
        T( ReaderXML::getNodeAttr_Double( nDomain, "MY" ) ),
        T( ReaderXML::getNodeAttr_Double( nDomain, "MZ" ) ) );

    DOMNode* nOrigin = ReaderXML::getNode( root, "Origin" );
    if ( nOrigin )
        GrainsParameters<T>::m_origin.setValue( 
            T( ReaderXML::getNodeAttr_Double( nOrigin, "OX" ) ),
            T( ReaderXML::getNodeAttr_Double( nOrigin, "OY" ) ),
            T( ReaderXML::getNodeAttr_Double( nOrigin, "OZ" ) ) );
    else
        GrainsParameters<T>::m_origin.setValue( T( 0 ), T( 0 ), T( 0 ) );

    // if the simulation is periodic
    DOMNode* nPeriodicity = ReaderXML::getNode( root, "Periodicity" );
    if ( nPeriodicity )
    {
        int PX = ReaderXML::getNodeAttr_Int( nPeriodicity, "PX" );
        int PY = ReaderXML::getNodeAttr_Int( nPeriodicity, "PY" );
        int PZ = ReaderXML::getNodeAttr_Int( nPeriodicity, "PZ" );
        if ( PX * PY * PZ != 0 )
        {
            GrainsMisc<T>::cout( "Periodicity is not implemented!" );
            exit( 1 );
        }
        GrainsParameters<T>::m_isPeriodic = false;
    }




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
        GrainsMisc<T>::cout( "Reading new particle types ...", 6 );
        
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

        GrainsMisc<T>::cout( "Reading particle types completed!", 6, 1 );
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
        GrainsMisc<T>::cout( "Reading new obstacle types ...", 6 );
        
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

        GrainsMisc<T>::cout( "Reading obstacle types completed!", 6, 1 );
    }




    // -------------------------------------------------------------------------
    // LinkedCell
    GrainsMisc<T>::cout( "Constructing linked cell ...", 6 );
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
    GrainsMisc<T>::cout( "Cell size factor = " + 
                         std::to_string( LC_coeff ), 9 );

    // Creating linked cell
    GrainsParameters<T>::m_sizeLC = LC_coeff * T( 2 ) * LCSize;
    m_linkedCell = ( LinkedCell<T>** ) malloc( sizeof( LinkedCell<T>* ) );
    *m_linkedCell = new LinkedCell<T>( GrainsParameters<T>::m_origin,
                                       GrainsParameters<T>::m_maxCoordinate, 
                                       GrainsParameters<T>::m_sizeLC );
    GrainsParameters<T>::m_numCells = (*m_linkedCell)->getNumCells();
    GrainsMisc<T>::cout( "LinkedCell with " +
                         std::to_string( GrainsParameters<T>::m_numCells ) + 
                         " cells is created on host.", 9 );
    GrainsMisc<T>::cout( "Constructing linked cell completed!", 6, 1 );
    



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
        GrainsMisc<T>::cout( "Reading the contact force model ..." , 6 );
        m_contactForce = ContactForceModelBuilderFactory<T>::create( rootElement );
        GrainsMisc<T>::cout( "Reading the contact force model completed!", 6, 1 );
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
            GrainsMisc<T>::cout( "Reading the time integration model ...", 6 );
            m_timeIntegrator = ( TimeIntegrator<T>** ) 
                                malloc( sizeof( TimeIntegrator<T>* ) );
            *m_timeIntegrator = TimeIntegratorBuilderFactory<T>::create( 
                                                    nTI, 
                                                    GrainsParameters<T>::m_dt );
            GrainsMisc<T>::cout( "Reading the time integration model completed!", 6, 1 ); 
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
    GrainsMisc<T>::cout( "Forces", 3 );


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
            GrainsMisc<T>::cout( "Gravity = " +
                    std::to_string( GrainsParameters<T>::m_gravity[X] ) + " " +
                    std::to_string( GrainsParameters<T>::m_gravity[Y] ) + " " +
                    std::to_string( GrainsParameters<T>::m_gravity[Z] ), 6, 1 );
        }
        else
        {
            GrainsMisc<T>::cout( "Gravity is mandatory!", 6 );
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
    GrainsMisc<T>::cout( "Simulation", 3 );
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    assert( rootElement != NULL );
    DOMNode* root = ReaderXML::getNode( rootElement, "Simulation" );
    if ( !root )
    {
        GrainsMisc<T>::cout( "Simulation node is mandatory!" );
        exit( 1 );
    }




    // -------------------------------------------------------------------------
    // Insertion policies
    DOMNode* nInsertion = ReaderXML::getNode( root, "ParticleInsertion" );
    GrainsMisc<T>::cout( "Reading insertion policies ...", 6 );
    if ( nInsertion )
        m_insertion = new Insertion<T>( nInsertion );
    else
    {
        GrainsMisc<T>::cout( "No policy found, setting the insertion policy to default", 9 );
        m_insertion = new Insertion<T>();
    }
    GrainsMisc<T>::cout( "Reading insertion policies completed.", 6, 1 );




    // -------------------------------------------------------------------------
    // Post-processing writers
    DOMNode* nPostProcessing = ReaderXML::getNode( root, "PostProcessing" );
    if ( nPostProcessing )
    {
        GrainsMisc<T>::cout( "Post-processing", 3 );
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
            GrainsMisc<T>::cout( "Reading the post processing writers ...", 6 );
            DOMNodeList* allPPW = ReaderXML::getNodes( nWriters );
            for ( XMLSize_t i = 0; i < allPPW->getLength(); i++ )
            {
                DOMNode* nPPW = allPPW->item( i );
                PostProcessingWriter<T>* ppw =
                m_postProcessor = PostProcessingWriterBuilderFactory<T>::create( nPPW );
                if ( !ppw )
                {
                    GrainsMisc<T>::cout( "Unknown postprocessing writer in node <Writers>", 6 );
                    exit( 1 );
                }
            }
            GrainsMisc<T>::cout( "Reading the post processing writers completed!", 6 );
        }
    }
    else
        GrainsMisc<T>::cout( "No postprocessing writer!", 6 );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Grains<float>;
template class Grains<double>;