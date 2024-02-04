#ifndef _PARTICLEFEATURES_CUH_
#define _PARTICLEFEATURES_CUH_


#include "Vector3.cuh"


/** @brief The class ParticleFeatures.

    Several features related to particles such as shape, density, inertia, and
    inverse inertia.

    @author A.Yazdani - 2023 - Construction */
// ============================================================================
class ParticleFeatures
{  
    public:
        /**@name Constructors & Destructor */
        //@{
        /** @brief Default constructor */
        __host__ __device__ ParticleFeatures();

        /** @brief Constructor with radius */
        __host__ __device__ ParticleFeatures( double radius );

        /** @brief Default destructor */
        __host__ __device__ ~ParticleFeatures();
        //@}
          


    protected:
        /**@name Parameters */
        //@{
        double m_radius; /** Particle radius */
        // double m_density; /** Particle density */
        // double m_inertia[6]; /**< Inertia tensor I={I(1,1), I(1,2), I(1,3),
  	    //                           I(2,2), I(2,3), I(3,3)} */
        // double m_inertia_1[6]; /**< Inverse inertia tensor */
        //@}
};

#endif