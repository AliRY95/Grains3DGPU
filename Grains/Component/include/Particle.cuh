#ifndef _PARTICLES_CUH_
#define _PARTICLES_CUH_


#include "Vector3.cuh"
#include "ParticleFeatures.cuh"
#include "ParticleKinematics.cuh"


/** @brief The class Particles.

    A freely moving particle.

    @author A.Yazdani - 2023 - Construction */
// ============================================================================
class Particles
{  
    public:
        /**@name Constructors & Destructor */
        //@{
        /** @brief Default constructor */
        __host__ __device__ Particles();

        /** @brief Host constructor with No. particles */
        Particles( int num );

        /** @brief Device constructor with No. particles */
        __device__ Particles( int num );

        /** @brief Host destructor */
        ~Particles();

        /** @brief Device destructor */
        __device__ ~Particles();
        //@}

        /**@name Initialize? */
        //@{
        /** @brief Host function to assign random positions */
        void setupParticles();

        /** @brief Device function to assign random positions */
        __device__ void setupParticles();


        //@}


    protected:
        /** @name Parameters */
        //@{
        int m_num; /**< No. particles */
        Vector3* m_position; /**< Array of position vectors */
        // ParticleFeatures* m_features; /**< Features such as shape, density, 
        //                                    inertia, and inverse inertia */
        // ParticleKinematics* m_kinematics; /**< Particle kinematics */
        //@}
};

#endif