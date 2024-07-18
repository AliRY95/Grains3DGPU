#ifndef _GRAINSGPU_HH_
#define _GRAINSGPU_HH_


#include "Grains.hh"
#include "ComponentManagerGPU.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class GrainsGPU.

    Standard Grains3D application on GPU.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class GrainsGPU : public Grains<T>
{
    public:
        /** @name Contructors & Destructor */
        //@{
        /** @brief Default constructor */
        GrainsGPU();

        /** @brief Destructor */
        ~GrainsGPU();
        //@}


        /** @name High-level methods */
        //@{
        // /** @brief Tasks to perform before time-stepping 
        // // @param rootElement XML root */
        // void initialize( DOMElement* rootElement );

        /** @brief Runs the simulation over the prescribed time interval */
        void simulate() final; 

        // /** @brief Tasks to perform after time-stepping */
        // virtual void finalize();
        //@}


        /**@name Low-level methods */
        //@{
        // /** @brief Construction of the simulation: linked cell, particles &
        // obstacles, domain decomposition 
        // @param rootElement XML root */
        // void Construction( DOMElement* rootElement );
        //@}
};

#endif
  
