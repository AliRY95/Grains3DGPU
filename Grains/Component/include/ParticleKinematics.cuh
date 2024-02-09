#ifndef _PARTICLEKINEMATICS_CUH_
#define _PARTICLEKINEMATICS_CUH_


#include "Vector3.cuh"


/** @brief The class ParticleKinematics.

    Manages the particle kinematics.

    @author A.Yazdani - 2023 - Construction */
// ============================================================================
class ParticleKinematics
{  
    public:
        /**@name Constructors & Destructor */
        //@{
        /** @brief Default constructor */
        __host__ __device__ ParticleKinematics();
        //@}
          


    protected:
        /**@name Parameters */
        //@{
        
        Vector3 m_translationalVelocity; /**< Translational velocity */
        Vector3 m_angularVelocity; /**< Angular velocity */
        //@}
};

#endif