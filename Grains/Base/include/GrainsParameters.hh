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
        static Vector3<T> m_origin; /**< global domain origin */
        static Vector3<T> m_dimension; /**< global domain dimension */
        static bool m_isPeriodic; /**< is simulation periodic? */

        /* Temporal */
        static T m_tStart; /**< initial simulation time */  
        static T m_tEnd; /**< end simulation time */  
        static T m_dt; /**< simulation time step */
        static T m_time; /**< physical time */
        
        /* Numbers */
        static unsigned int m_numComponents; /**< number of components in 
                                                  simulation */
        static unsigned int m_numCells; /**< number of cells in simulation */

        /* Simulation */
        /** \brief Gravity vector */
        static Vector3<T> m_gravity;

        /* Material */
        /** \brief Map from material name to an unsigned int ID */
        static std::unordered_map<std::string, unsigned int> m_materialMap;
        /** \brief Number of different possible contact pairs (incl. obs-obs) */
        static unsigned int m_numContactPairs;

        /* GPU */
        static bool m_isGPU; /**< is simulation on GPU? */
        //@}
};


typedef GrainsParameters<float> GrainsParamatersF;
typedef GrainsParameters<double> GrainsParamatersD;


#endif