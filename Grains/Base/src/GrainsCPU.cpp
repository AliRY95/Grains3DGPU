#include "GrainsCPU.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"


#include "ConvexBuilderFactory.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
GrainsCPU<T>::GrainsCPU()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
GrainsCPU<T>::~GrainsCPU()
{}




// // -----------------------------------------------------------------------------
// // Initializes the simulation using the XML input
// template <typename T>
// void GrainsCPU<T>::initialize( DOMElement* rootElement )
// {   
//     // Read the input file
//     // Construction( rootElement );
//     // Forces( rootElement );
//     // AdditionalFeatures( rootElement );
// }




// -----------------------------------------------------------------------------
// Runs the simulation over the prescribed time interval
template <typename T>
void GrainsCPU<T>::simulate()
{
    unsigned int N = GrainsParameters<T>::m_numParticles;
    int* h_collision = new int[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
        h_collision[i] = 0;

    // Collision detection on host
    // first, inserting particles on host
    Grains<T>::m_components->insertParticles( Grains<T>::m_insertion );
    // setting up the PP
    Grains<T>::m_postProcessor->PostProcessing_start();

    auto h_start = chrono::high_resolution_clock::now();
    // time marching
    for ( GrainsParameters<T>::m_time = GrainsParameters<T>::m_tStart;
          GrainsParameters<T>::m_time < GrainsParameters<T>::m_tEnd;
          GrainsParameters<T>::m_time += GrainsParameters<T>::m_dt )
    {
        Grains<T>::m_components->detectCollisionAndComputeContactForces( 
                                                  Grains<T>::m_linkedCell, 
                                                  Grains<T>::m_rigidBodyList,
                                                  Grains<T>::m_contactForce,
                                                  h_collision ); 
        Grains<T>::m_components->moveComponents( Grains<T>::m_timeIntegrator,
                                                 Grains<T>::m_rigidBodyList );
        
        // Post-Processing
        if ( GrainsParameters<T>::m_tSave.front() - 
             GrainsParameters<T>::m_time < 
             0.01 * GrainsParameters<T>::m_dt )
        {
            GrainsParameters<T>::m_tSave.pop();
            std::vector<unsigned int> id = 
                                    Grains<T>::m_components->getRigidBodyId();
            std::vector<Transform3<T>> t = 
                                Grains<T>::m_components->getTransform();
            std::vector<Kinematics<T>> k = 
                                Grains<T>::m_components->getVelocity();
            Grains<T>::m_postProcessor->PostProcessing( 
                                                Grains<T>::m_rigidBodyList,
                                                &id,
                                                &t,
                                                &k,
                                                GrainsParameters<T>::m_time );
        }
        // In case we get past the saveTime, we need to remove it from the queue
        if ( GrainsParameters<T>::m_time > 
             GrainsParameters<T>::m_tSave.front() )
        {
            GrainsParameters<T>::m_tSave.pop();
        }
    }
    auto h_end = chrono::high_resolution_clock::now();
    std::cout << "Time: " << GrainsParameters<T>::m_time << endl;
    Grains<T>::m_postProcessor->PostProcessing_end();



    // Time comparison
    chrono::duration<double> h_time = h_end - h_start;
    std::cout << "CPU: " << h_time.count() << endl;

    // accuracy
    int trueHostCount = 0, trueDeviceCount = 0;
    for( int i = 0; i < N; i++ )
    {
        trueHostCount += h_collision[i];
    }
    cout << N << " Particles, "
         << trueHostCount << " Collision on host. " << endl;
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsCPU<float>;
template class GrainsCPU<double>;