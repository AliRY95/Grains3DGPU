#include "GrainsGPU.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "RigidBodyGPUWrapper.hh"
#include "LinkedCellGPUWrapper.hh"
// #include "ConvexBuilderFactory.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "TimeIntegratorBuilderFactory.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
GrainsGPU<T>::GrainsGPU()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
GrainsGPU<T>::~GrainsGPU()
{}




// -----------------------------------------------------------------------------
// Initializes the simulation using the XML input
template <typename T>
void GrainsGPU<T>::initialize( DOMElement* rootElement )
{
    // We first read using the base class Grains<T>
    Grains<T>::initialize( rootElement );

    // Since this function is invoked, it means a GPU simulation is requested.
    GrainsParameters<T>::m_isGPU = true;

    // Reading different blocks of the input XML
    // Note that most of the reading is done at Grains<T>::initialize
    // Herein, we initialize specifically for GPU
    Construction( rootElement );
    Forces( rootElement );
    AdditionalFeatures( rootElement );
}




// -----------------------------------------------------------------------------
// Runs the simulation over the prescribed time interval
template <typename T>
void GrainsGPU<T>::simulate()
{
    cout << "\n\n\n\n\nStarting the simulation ..." << endl;
    unsigned int N = GrainsParameters<T>::m_numParticles;
    int* h_collision = new int[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
        h_collision[i] = 0;

    int* d_collision;
    cudaErrCheck( cudaMalloc( (void**)&d_collision,
                              N * sizeof( int ) ) );
    cudaErrCheck( cudaMemcpy( d_collision,
                              h_collision,
                              N * sizeof( int ), 
                              cudaMemcpyHostToDevice ) );

    // Collision detection on host
    // first, inserting partilces
    Grains<T>::m_components->insertParticles( Grains<T>::m_insertion );
    // setting up the PP
    Grains<T>::m_postProcessor->PostProcessing_start();
    
    // cout << "Time \t TO \tend \tParticles \tIn \tOut" << endl;
    auto h_start = chrono::high_resolution_clock::now();
    // for ( GrainsParameters<T>::m_time = GrainsParameters<T>::m_tStart;
    //       GrainsParameters<T>::m_time <= GrainsParameters<T>::m_tEnd;
    //       GrainsParameters<T>::m_time += GrainsParameters<T>::m_dt )
    // {
    //     // Output time
    //     ostringstream oss;
    //     oss.width( 10 );
    //     oss << left << GrainsParameters<T>::m_time;
    //     std::cout << '\r' 
    //               << oss.str() 
    //               << "  \t" 
    //               << GrainsParameters<T>::m_tEnd
    //               << std::flush;
        
    //     Grains<T>::m_components->detectCollisionAndComputeContactForces( 
    //                                         Grains<T>::m_particleRigidBodyList,
    //                                         Grains<T>::m_obstacleRigidBodyList,
    //                                         Grains<T>::m_linkedCell, 
    //                                         Grains<T>::m_contactForce,
    //                                         h_collision ); 
    //     Grains<T>::m_components->addExternalForces( 
    //                                         Grains<T>::m_particleRigidBodyList,
    //                                         GrainsParameters<T>::m_gravity );
    //     Grains<T>::m_components->moveParticles( 
    //                                         Grains<T>::m_particleRigidBodyList,
    //                                         Grains<T>::m_timeIntegrator );
        
    //     // Post-Processing
    //     if ( fabs( GrainsParameters<T>::m_time - 
    //                GrainsParameters<T>::m_tSave.front() ) < 
    //                0.01 * GrainsParameters<T>::m_dt )
    //     {
    //         GrainsParameters<T>::m_tSave.pop();
    //         Grains<T>::m_postProcessor->PostProcessing( 
    //                                         Grains<T>::m_particleRigidBodyList,
    //                                         Grains<T>::m_obstacleRigidBodyList,
    //                                         Grains<T>::m_components,
    //                                         GrainsParameters<T>::m_time );
    //     }
    // }
    auto h_end = chrono::high_resolution_clock::now();



    // Collision detection on device
    // Copying to device
    cout << "Copying the inserted particles to the device ..." << endl;
    m_d_components->copy( Grains<T>::m_components );
    cout << "Copying completed!" << endl;
    cout << "\nTime \t TO \tend \tParticles \tIn \tOut" << endl;
    auto d_start = chrono::high_resolution_clock::now();
    for ( GrainsParameters<T>::m_time = GrainsParameters<T>::m_tStart;
          GrainsParameters<T>::m_time <= GrainsParameters<T>::m_tEnd;
          GrainsParameters<T>::m_time += GrainsParameters<T>::m_dt )
    {
        // Output time
        ostringstream oss;
        oss.width( 10 );
        oss << left << GrainsParameters<T>::m_time;
        std::cout << '\r' 
                  << oss.str() 
                  << "  \t" 
                  << GrainsParameters<T>::m_tEnd
                  << std::flush;
        
        m_d_components->detectCollisionAndComputeContactForces( 
                                        m_d_particleRigidBodyList,
                                        m_d_obstacleRigidBodyList,
                                        m_d_linkedCell, 
                                        m_d_contactForce,
                                        d_collision );
        m_d_components->addExternalForces( m_d_particleRigidBodyList,
                                           GrainsParameters<T>::m_gravity );
        m_d_components->moveParticles( m_d_particleRigidBodyList,
                                       m_d_timeIntegrator );
        
        // Post-Processing
        if ( fabs( GrainsParameters<T>::m_time - 
                   GrainsParameters<T>::m_tSave.front() ) < 
                   0.01 * GrainsParameters<T>::m_dt )
        {
            GrainsParameters<T>::m_tSave.pop();
            Grains<T>::m_postProcessor->PostProcessing( 
                                            Grains<T>::m_particleRigidBodyList,
                                            Grains<T>::m_obstacleRigidBodyList,
                                            m_d_components,
                                            GrainsParameters<T>::m_time );
        }
        // In case we get past the saveTime, we need to remove it from the queue
        if ( GrainsParameters<T>::m_time > 
             GrainsParameters<T>::m_tSave.front() )
        {
            GrainsParameters<T>::m_tSave.pop();
        }
    }
    cudaDeviceSynchronize();
    auto d_end = chrono::high_resolution_clock::now();
    Grains<T>::m_postProcessor->PostProcessing_end();



    // Time comparison
    chrono::duration<double> h_time = h_end - h_start;
    chrono::duration<double> d_time = d_end - d_start;
    std::cout << "\nCPU: " << h_time.count() << endl;
    std::cout << "GPU: " << d_time.count() << endl;
    int* h_d_collision = new int[N];
    cudaErrCheck( cudaMemcpy( h_d_collision,
                              d_collision,
                              N * sizeof( int ), 
                              cudaMemcpyDeviceToHost ) );
    // accuracy
    int hCount = 0, dCount = 0;
    for( int i = 0; i < N; i++ )
    {
        hCount += h_collision[i];
        dCount += h_d_collision[i];
    }
    cout << N << " Particles, "
         << hCount << " collisions on CPU, and "
         << dCount << " collisions on GPU. " << endl;
}




/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Constructs the simulation -- Reads the Construction part of the XML input to
// set the parameters
template <typename T>
void GrainsGPU<T>::Construction( DOMElement* rootElement )
{
    // Get the Construction block. We don't check if it exists. It has been
    // already checked in the base class
    DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
    




    // -------------------------------------------------------------------------
    // Particles
    DOMNode* particles = ReaderXML::getNode( root, "Particles" );
    DOMNodeList* allParticles = ReaderXML::getNodes( rootElement, "Particle" );
    // Number of unique shapes (rigid bodies) in the simulation
    unsigned int numUniqueParticles = allParticles->getLength();
    // It is a GPU simulation, and we have already read rigid bodies on the host
    // We allocate memory on device and copy the rigid bodies over.
    cout << shiftString6 << "Copying particle types to device ..." << endl;
    if ( numUniqueParticles > 0 )
    {
        cudaErrCheck( cudaMalloc( (void**)&m_d_particleRigidBodyList,
                      numUniqueParticles * sizeof( RigidBody<T, T>* ) ) );
        RigidBodyCopyHostToDevice( Grains<T>::m_particleRigidBodyList, 
                                   m_d_particleRigidBodyList,
                                   numUniqueParticles );
        cudaDeviceSynchronize();
    }
    cout << shiftString6 
         << "Copying particle types to device completed!\n" << endl;

    


    // -------------------------------------------------------------------------
    // Obstacles
    DOMNode* obstacles = ReaderXML::getNode( root, "Obstacles" );
    DOMNodeList* allObstacles = ReaderXML::getNodes( rootElement, "Obstacle" );
    // Number of unique shapes (rigid bodies) in the simulation
    unsigned int numUniqueObstacles = allObstacles->getLength();
    // It is a GPU simulation, and we have already read obstacles on the host
    // We allocate memory on device and copy the rigid bodies over.
    cout << shiftString6 << "Copying obstacle types to device ..." << endl;
    if ( numUniqueObstacles > 0 )
    {
        cudaErrCheck( cudaMalloc( (void**)&m_d_obstacleRigidBodyList,
                      numUniqueObstacles * sizeof( RigidBody<T, T>* ) ) );
        RigidBodyCopyHostToDevice( Grains<T>::m_obstacleRigidBodyList, 
                                   m_d_obstacleRigidBodyList,
                                   numUniqueObstacles );
        cudaDeviceSynchronize();
    }
    cout << shiftString6 
         << "Copying obstacle types to device completed!\n" << endl;





    // -------------------------------------------------------------------------
    // LinkedCell
    cout << shiftString6 << "Constructing linked cell on device ..." << endl;
    cudaErrCheck( cudaMalloc( (void**)&m_d_linkedCell,
                              sizeof( LinkedCell<T>* ) ) );
    int d_numCells = createLinkedCellOnDevice( 
        GrainsParameters<T>::m_origin,
        GrainsParameters<T>::m_maxCoordinate, 
        GrainsParameters<T>::m_sizeLC,
        m_d_linkedCell );
    GrainsParameters<T>::m_numCells = d_numCells;
    cout << shiftString9 << "LinkedCell with "
         << GrainsParameters<T>::m_numCells
         << " cells is created on device." << endl;
    cout << shiftString6 
         << "Constructing linked cell on device completed!\n" << endl;
    



    // -------------------------------------------------------------------------
    // Setting up the component managers
    m_d_components = new ComponentManagerGPU<T>( 
                                        GrainsParameters<T>::m_numParticles,
                                        GrainsParameters<T>::m_numObstacles,
                                        GrainsParameters<T>::m_numCells );
    m_d_components->copy( Grains<T>::m_components );

    


    // -------------------------------------------------------------------------
    // Contact force models
    // It is a GPU simulation, and we have already read contact force models 
    // on the host. We allocate memory on device and copy the models over.
    cout << shiftString6 << "Copying contact force models to device ..." << endl;
    cudaErrCheck( cudaMalloc( (void**)&m_d_contactForce,
                              GrainsParameters<T>::m_numContactPairs *
                              sizeof( ContactForceModel<T>* ) ) );
    ContactForceModelBuilderFactory<T>::ContactForceModelCopyHostToDevice( 
                                                Grains<T>::m_contactForce,
                                                m_d_contactForce );
    cout << shiftString6 
         << "Copying contact force models to device completed!\n" << endl;




    // -------------------------------------------------------------------------
    // Temporal setting and time integration
    DOMNode* tempSetting = ReaderXML::getNode( root, "TemporalSetting" );
    DOMNode* nTI = ReaderXML::getNode( tempSetting, "TimeIntegration" );
    // It is a GPU simulation, and we have already read time integration on the 
    // host. We allocate memory on device and copy the scheme over.
    cout << shiftString6 << "Copying time integration scheme to device ..." << endl;
    cudaErrCheck( cudaMalloc( (void**)&m_d_timeIntegrator,
                              sizeof( TimeIntegrator<T>* ) ) );
    TimeIntegratorBuilderFactory<T>::createOnDevice( 
                                                nTI,
                                                GrainsParameters<T>::m_dt, 
                                                m_d_timeIntegrator );
    cout << shiftString6 
         << "Copying time integration scheme to device completed!\n" << endl;
}




// -----------------------------------------------------------------------------
// External force definition
template <typename T>
void GrainsGPU<T>::Forces( DOMElement* rootElement )
{
}




// -----------------------------------------------------------------------------
// Additional features of the simulation: insertion, post-processing
template <typename T>
void GrainsGPU<T>::AdditionalFeatures( DOMElement* rootElement )
{
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsGPU<float>;
template class GrainsGPU<double>;