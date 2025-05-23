#include "GrainsCPU.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "GrainsUtils.hh"
#include "VectorMath.hh"

#include "ConvexBuilderFactory.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
GrainsCPU<T>::GrainsCPU()
{
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
GrainsCPU<T>::~GrainsCPU()
{
}

// -----------------------------------------------------------------------------
// Runs the simulation over the prescribed time interval
template <typename T>
void GrainsCPU<T>::simulate()
{
    using GP = GrainsParameters<T>;

    Gout(std::string(80, '='));
    Gout("Starting the simulation on CPU");
    Gout(std::string(80, '='));
    uint N           = GP::m_numParticles;
    int* h_collision = new int[N];
    // Zeroing out
    for(int i = 0; i < N; i++)
        h_collision[i] = 0;

    // Collision detection on host
    // first, inserting particles on host
    Grains<T>::m_components->insertParticles(Grains<T>::m_insertion);

    cout << "Time \t TO \tend \tParticles \tIn \tOut" << endl;
    auto h_start = chrono::high_resolution_clock::now();
    // time marching
    for(GP::m_time = GP::m_tStart; GP::m_time <= GP::m_tEnd;
        GP::m_time += GP::m_dt)
    {
        // Output time
        ostringstream oss;
        oss.width(10);
        oss << left << GP::m_time;
        std::cout << '\r' << oss.str() << "  \t" << GP::m_tEnd << std::flush;

        Grains<T>::m_components->detectCollisionAndComputeContactForces(
            Grains<T>::m_particleRigidBodyList,
            Grains<T>::m_obstacleRigidBodyList,
            Grains<T>::m_linkedCell,
            Grains<T>::m_contactForce,
            h_collision);
        Grains<T>::m_components->addExternalForces(
            Grains<T>::m_particleRigidBodyList,
            GP::m_gravity);
        Grains<T>::m_components->moveParticles(
            Grains<T>::m_particleRigidBodyList,
            Grains<T>::m_timeIntegrator);

        // Post-Processing
        Grains<T>::postProcess(Grains<T>::m_components);
    }
    auto h_end = chrono::high_resolution_clock::now();

    // Time comparison
    chrono::duration<double> h_time = h_end - h_start;
    std::cout << "\nCPU: " << h_time.count() << endl;

    // accuracy
    int trueHostCount = 0, trueDeviceCount = 0;
    for(int i = 0; i < N; i++)
    {
        trueHostCount += h_collision[i];
    }
    cout << N << " Particles, " << trueHostCount << " Collision on host. "
         << endl;
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsCPU<float>;
template class GrainsCPU<double>;