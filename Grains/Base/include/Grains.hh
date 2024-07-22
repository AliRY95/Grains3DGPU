#ifndef _GRAINS_HH_
#define _GRAINS_HH_


#include "GrainsParameters.hh"
#include "RigidBody.hh"
#include "ComponentManager.hh"
#include "ComponentManagerCPU.hh"
#include "ComponentManagerGPU.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class Grains.

    Standard Grains3D application as an abstract class.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class Grains
{
    protected:
        /** @name Parameters */
        //@{
        /** \brief Parameters used in the simulation on the host memory. */
        GrainsParameters<T> m_parameters;
        /** \brief List of rigid bodies as a double pointer. The first ptr is
        for the list of rigid bodies as we might have different rigid bodies in
        the simulation, and the second pointer is for dynamically allocating
        the memory. */
        RigidBody<T, T>** m_rigidBodyList;
        /** \brief Same thing as above but on device if using GPU. Note that we
        force to use single precision for bounding volume. */
        RigidBody<T, T>** m_d_rigidBodyList;
        /** \brief Manager of the components in the simulation on the host mem. 
        We use a pointer here as we want to use runtime polymorphism for
        switching between ComponentManagerCPU and ComponentManagerGPU. */
        ComponentManagerCPU<T>* m_components;
        /** \brief Manager of the components in the simulation on the device 
        memory. We use a pointer here as we want to use runtime polymorphism for
        switching between ComponentManagerCPU and ComponentManagerGPU. */
        ComponentManagerGPU<T>* m_d_components;
        /** \brief Linked cell for broad-phase. We use a pointer because
        the linkedCell is directly instantiated on device in the case that we
        run Grains on GPU. */
        LinkedCell<T>** m_linkedCell;
        /** \brief Linked cell for broad-phase. We use a pointer because
        the linkedCell is directly instantiated on device in the case that we
        run Grains on GPU. */
        LinkedCell<T>** m_d_linkedCell;
        /** \brief Time integration. We use a pointer because the linkedCell is 
        directly instantiated on device in the case that we
        run Grains on GPU. */
        // TimeIntegration<T>* m_timeIntegration;
        //@}


    public:
        /** @name Contructors & Destructor */
        //@{
        /** @brief Default constructor */
        Grains();

        /** @brief Destructor */
        // TODO: cudaFree!!!
        virtual ~Grains();
        //@}


        /** @name High-level methods */
        //@{
        /** @brief Tasks to perform before time-stepping, mostly reading setting
        variables in GrainsParameters.
        @param rootElement XML root */
        virtual void initialize( DOMElement* rootElement );

        /** @brief Runs the simulation over the prescribed time interval */
        virtual void simulate() = 0; 

        // /** @brief Tasks to perform after time-stepping */
        // virtual void finalize();
        //@}


        /**@name Low-level methods */
        //@{
        /** @brief Construction of the simulation: linked cell, particles &
        obstacles, domain decomposition 
        @param rootElement XML root */
        void Construction( DOMElement* rootElement );
        //@}
};

#endif
  
