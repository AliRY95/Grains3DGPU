#include "GrainsGPU.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"
#include "ConvexBuilderFactory.hh"


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




// // -----------------------------------------------------------------------------
// // Initializes the simulation using the XML input
// template <typename T>
// void GrainsGPU<T>::initialize( DOMElement* rootElement )
// {   
//     // Read the input file
//     // Construction( rootElement );
//     // Forces( rootElement );
//     // AdditionalFeatures( rootElement );
// }




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
    Grains<T>::m_d_components->copy( Grains<T>::m_components );
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
        
        Grains<T>::m_d_components->detectCollisionAndComputeContactForces( 
                                        Grains<T>::m_d_particleRigidBodyList,
                                        Grains<T>::m_d_obstacleRigidBodyList,
                                        Grains<T>::m_d_linkedCell, 
                                        Grains<T>::m_d_contactForce,
                                        d_collision );
        Grains<T>::m_d_components->addExternalForces( 
                                            Grains<T>::m_particleRigidBodyList,
                                            GrainsParameters<T>::m_gravity );
        Grains<T>::m_d_components->moveParticles(
                                        Grains<T>::m_d_particleRigidBodyList,
                                        Grains<T>::m_d_timeIntegrator );
        
        // Post-Processing
        if ( fabs( GrainsParameters<T>::m_time - 
                   GrainsParameters<T>::m_tSave.front() ) < 
                   0.01 * GrainsParameters<T>::m_dt )
        {
            GrainsParameters<T>::m_tSave.pop();
            Grains<T>::m_postProcessor->PostProcessing( 
                                            Grains<T>::m_particleRigidBodyList,
                                            Grains<T>::m_obstacleRigidBodyList,
                                            Grains<T>::m_d_components,
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




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsGPU<float>;
template class GrainsGPU<double>;