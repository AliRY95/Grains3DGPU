#ifndef _GRAINSGPU_HH_
#define _GRAINSGPU_HH_

#include "ComponentManagerGPU.hh"
#include "Grains.hh"
#include "ReaderXML.hh"

// =============================================================================
/** @brief The class GrainsGPU.

    Standard Grains3D application on GPU.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class GrainsGPU : public Grains<T>
{
protected:
    /** \brief Same thing as above but on device if using GPU. Note that we
        force to use single precision for bounding volume. */
    RigidBody<T, T>** m_d_particleRigidBodyList;
    /** \brief Same thing as above but on device if using GPU. Note that we
        force to use single precision for bounding volume. */
    RigidBody<T, T>** m_d_obstacleRigidBodyList;
    /** \brief Manager of the components in the simulation on the device 
        memory. We use a pointer here as we want to use runtime polymorphism for
        switching between ComponentManagerCPU and ComponentManagerGPU. */
    ComponentManager<T>* m_d_components;
    /** \brief Linked cell for broad-phase. We use a pointer because
        the linkedCell is directly instantiated on device in the case that we
        run Grains on GPU. */
    LinkedCell<T>** m_d_linkedCell;
    /** \brief Linked cell for broad-phase. We use a pointer because
        the linkedCell is directly instantiated on device in the case that we
        run Grains on GPU. */
    ContactForceModel<T>** m_d_contactForce;
    /** \brief Linked cell for broad-phase. We use a pointer because
        the linkedCell is directly instantiated on device in the case that we
        run Grains on GPU. */
    TimeIntegrator<T>** m_d_timeIntegrator;
    //@}

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
    /** @brief Tasks to perform before time-stepping 
        // @param rootElement XML root */
    void initialize(DOMElement* rootElement) final;

    /** @brief Runs the simulation over the prescribed time interval */
    void simulate() final;

    // /** @brief Tasks to perform after time-stepping */
    // virtual void finalize();
    //@}

    /**@name Low-level methods */
    //@{
    /** @brief Construction of the simulation: linked cell, particles &
        obstacles, domain decomposition 
        @param rootElement XML root */
    void Construction(DOMElement* rootElement);

    /** @brief External force definition
        @param rootElement XML root */
    void Forces(DOMElement* rootElement);

    /** @brief Additional features of the simulation: insertion, 
        post-processing
        @param rootElement XML root */
    void AdditionalFeatures(DOMElement* rootElement);
    //@}
};

#endif
