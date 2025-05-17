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

    // first, inserting particles on host
    Grains<T>::m_components->insertParticles(Grains<T>::m_insertion);

    cout << "Time \t TO \tend \tParticles \tIn \tOut" << endl;
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
            Grains<T>::m_contactForce);
        Grains<T>::m_components->addExternalForces(
            Grains<T>::m_particleRigidBodyList);
        Grains<T>::m_components->moveParticles(
            Grains<T>::m_particleRigidBodyList,
            Grains<T>::m_timeIntegrator);

        // Post-Processing
        Grains<T>::postProcess(Grains<T>::m_components);
    }
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsCPU<float>;
template class GrainsCPU<double>;