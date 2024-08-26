#include "GrainsParameters.hh"


/* GPU */
template <typename T> bool GrainsParameters<T>::m_isGPU = false;
// template <typename T> int GrainsParameters<T>::m_numBlockGPU;
// template <typename T> int GrainsParameters<T>::m_numThreadGPU;

/* MPI */
// template <typename T> bool GrainsParameters<T>::m_isMPI = false;
// template <typename T> int GrainsParameters<T>::m_nProcs;

/* Space */
template <typename T> Vector3<T> GrainsParameters<T>::m_origin = zeroVector3T;
template <typename T> Vector3<T> GrainsParameters<T>::m_dimension = zeroVector3T;

/* Time */
template <typename T> T GrainsParameters<T>::m_tStart;
template <typename T> T GrainsParameters<T>::m_tEnd;
template <typename T> T GrainsParameters<T>::m_dt;
template <typename T> T GrainsParameters<T>::m_time;

/* Numbers */
template <typename T> unsigned int GrainsParameters<T>::m_numComponents = 0;
template <typename T> unsigned int GrainsParameters<T>::m_numCells = 0;

/* Physical */
template <typename T> Vector3<T> GrainsParameters<T>::m_gravity = Vector3<T>( 0, 0, -10 );

/* Booleans */
// template <typename T> bool GrainsParameters<T>::m_isDouble = true;
template <typename T> bool GrainsParameters<T>::m_isPeriodic = false;

// // -----------------------------------------------------------------------------
// // Default constructor
// ComponentManager::ComponentManager()
// {}




// // -----------------------------------------------------------------------------
// // Destructor
// ComponentManager::~ComponentManager()
// {}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsParameters<float>;
template class GrainsParameters<double>;