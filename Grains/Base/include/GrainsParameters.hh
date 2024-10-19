#ifndef _GRAINSPARAMETERS_HH_
#define _GRAINSPARAMETERS_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief Parameters needed for Grains.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class GrainsParameters
{
    public:
        /** @name Parameters */
        //@{
        /* Spatial */
        /** @brief Global domain origin */
        static Vector3<T> m_origin;
        /** @brief Global domain dimension */
        static Vector3<T> m_maxCoordinate;
        /** @brief Is simulation periodic? */
        static bool m_isPeriodic;

        /* Temporal */
        /** @brief Initial simulation time */
        static T m_tStart;
        /** @brief End simulation time */
        static T m_tEnd;
        /** @brief Simulation time step */
        static T m_dt;
        /** @brief Physical time */
        static T m_time;
        
        /* Numbers */
        /** @brief Number of particles in simulation */
        static unsigned int m_numParticles;
        /** @brief Number of obstacles in simulation */
        static unsigned int m_numObstacles;
        /** @brief Number of cells in simulation */
        static unsigned int m_numCells;

        /* Simulation */
        /** \brief Gravity vector */
        static Vector3<T> m_gravity;

        /* Material */
        /** \brief Map from material name to an unsigned int ID */
        static std::unordered_map<std::string, unsigned int> m_materialMap;
        /** \brief Number of different possible contact pairs (incl. obs-obs) */
        static unsigned int m_numContactPairs;

        /* Post-Processing */
        /** \brief Queue of simulation time to write PP */
        static std::queue<T> m_tSave;
        
        /* GPU */
        /** \brief is simulation on GPU? */
        static bool m_isGPU;
        //@}
};


typedef GrainsParameters<float> GrainsParamatersF;
typedef GrainsParameters<double> GrainsParamatersD;


#endif