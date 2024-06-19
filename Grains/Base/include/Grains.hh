#ifndef _GRAINS_HH_
#define _GRAINS_HH_

#include "ComponentManager.hh"
#include "LinkedCell.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class Grains.

    Standard Grains3D application.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Grains
{
    protected:
        /** @name Parameters */
        //@{
        ComponentManager<T>* m_allcomponents; /**< all components */
        LinkedCell<T>* m_linkedCell; /**< linked cell for broad-phase */
        // TimeIntegration<T>* m_timeIntegration; /**< time integration model */
        //@}
        

    public:
        /** @name Contructors & Destructor */
        //@{
        /** @brief Default constructor */
        Grains();

        /** @brief Destructor */
        virtual ~Grains();
        //@}


        /** @name High-level methods */
        //@{
        /** @brief Tasks to perform before time-stepping 
        @param rootElement XML root */
        virtual void initialize( DOMElement* rootElement );

        // /** @brief Runs the simulation over the prescribed time interval */
        // virtual void Simulation() ; 

        // /** @brief Tasks to perform after time-stepping */
        // virtual void finalize();
        //@}


        /**@name Low-level methods */
        //@{
        /** @brief Construction of the simulation: linked cell, particles &
        obstacles, domain decomposition 
        @param rootElement XML root */
        virtual void Construction( DOMElement* rootElement );
        //@}
};

#endif
  
