#include <numeric>

#include "ComponentManagerCPU.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "Grains.hh"
#include "GrainsUtils.hh"
#include "PostProcessingWriterBuilderFactory.hh"
#include "TimeIntegratorBuilderFactory.hh"
#include "VectorMath.hh"

/* ========================================================================== */
/*                             High-Level Methods                             */
/* ========================================================================== */
// Default constructor
template <typename T>
Grains<T>::Grains()
{
    Gout(std::string(80, '='));
    Gout("Starting Grains3D ...");
    Gout(std::string(80, '='));
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
Grains<T>::~Grains()
{
    // TODO
    for(auto ptr : m_postProcessor)
        delete ptr;
    delete m_insertion;
}

// -----------------------------------------------------------------------------
// Initializes the simulation using the XML input
template <typename T>
void Grains<T>::initialize(DOMElement* rootElement)
{
    // Reading different blocks of the input XML
    Gout(std::string(80, '='));
    Gout("Reading the input file ...");
    Gout(std::string(80, '='));
    Construction(rootElement);
    Forces(rootElement);
    AdditionalFeatures(rootElement);
    Gout(std::string(80, '='));
    Gout("Reading the input file completed!");
    Gout(std::string(80, '='));

    // Post-processing start
    for(auto pp : m_postProcessor)
        pp->PostProcessing_start();
}

// -----------------------------------------------------------------------------
// Performs post-processing
template <typename T>
void Grains<T>::postProcess(ComponentManager<T> const* cm) const
{
    using GP = GrainsParameters<T>;

    if(GP::m_tSave.front() - GP::m_time < 0.01 * GP::m_dt)
    {
        GP::m_tSave.pop();
        for(auto pp : m_postProcessor)
            pp->PostProcessing(m_particleRigidBodyList,
                               m_obstacleRigidBodyList,
                               cm,
                               GP::m_time);
    }
    // In case we get past the saveTime, we need to remove it from the queue
    if(GP::m_time > GP::m_tSave.front())
        GP::m_tSave.pop();
}

// -----------------------------------------------------------------------------
// Performs tasks after time-stepping
template <typename T>
void Grains<T>::finalize()
{
    for(auto pp : m_postProcessor)
        pp->PostProcessing_end();
}

/* ========================================================================== */
/*                            Low-Level Methods                               */
/* ========================================================================== */
// Constructs the simulation -- Reads the Construction part of the XML input to
// set the parameters
template <typename T>
void Grains<T>::Construction(DOMElement* rootElement)
{
    using GP = GrainsParameters<T>;

    // Output message
    GoutWI(3, "Construction");
    // -------------------------------------------------------------------------
    // Checking if Construction node is available
    DOMNode* root = ReaderXML::getNode(rootElement, "Construction");
    if(!root)
    {
        Gout("Construction node is mandatory!");
        exit(1);
    }

    // -------------------------------------------------------------------------
    // Domain size: origin, max coordinates and periodicity
    DOMNode* nDomain = ReaderXML::getNode(root, "LinkedCell");
    GP::m_maxCoordinate.setValue(
        T(ReaderXML::getNodeAttr_Double(nDomain, "MX")),
        T(ReaderXML::getNodeAttr_Double(nDomain, "MY")),
        T(ReaderXML::getNodeAttr_Double(nDomain, "MZ")));

    DOMNode* nOrigin = ReaderXML::getNode(root, "Origin");
    if(nOrigin)
        GP::m_origin.setValue(T(ReaderXML::getNodeAttr_Double(nOrigin, "OX")),
                              T(ReaderXML::getNodeAttr_Double(nOrigin, "OY")),
                              T(ReaderXML::getNodeAttr_Double(nOrigin, "OZ")));
    else
        GP::m_origin.setValue(T(0), T(0), T(0));

    // if the simulation is periodic
    DOMNode* nPeriodicity = ReaderXML::getNode(root, "Periodicity");
    if(nPeriodicity)
    {
        int PX = ReaderXML::getNodeAttr_Int(nPeriodicity, "PX");
        int PY = ReaderXML::getNodeAttr_Int(nPeriodicity, "PY");
        int PZ = ReaderXML::getNodeAttr_Int(nPeriodicity, "PZ");
        if(PX * PY * PZ != 0)
        {
            Gout("Periodicity is not implemented!");
            exit(1);
        }
        GP::m_isPeriodic = false;
    }

    // -------------------------------------------------------------------------
    // Particles
    DOMNode*     particles    = ReaderXML::getNode(root, "Particles");
    DOMNodeList* allParticles = ReaderXML::getNodes(rootElement, "Particle");
    // Number of unique shapes (rigid bodies) in the simulation
    uint numUniqueParticles = allParticles->getLength();
    // In the first traverse, we find the total number of particles
    std::vector<uint> numEachUniqueParticle(numUniqueParticles);
    if(particles)
    {
        for(int i = 0; i < numUniqueParticles; i++)
        {
            DOMNode* nParticle       = allParticles->item(i);
            numEachUniqueParticle[i] = static_cast<uint>(
                ReaderXML::getNodeAttr_Int(nParticle, "Number"));
        }
    }
    // Total number of particles in the simulation
    uint numParticles = std::accumulate(numEachUniqueParticle.begin(),
                                        numEachUniqueParticle.end(),
                                        0);
    // We also store the initial transformations of the rigid bodies to pass to
    // the ComponentManager to create particles with the initial transformation
    // required.
    std::vector<Transform3<T>> particlesInitialTransform(numParticles);
    // Memory allocation for m_rigidBodyList with respect to the number of
    // shapes in the simulation.
    m_particleRigidBodyList
        = (RigidBody<T, T>**)malloc(numParticles * sizeof(RigidBody<T, T>*));
    if(numParticles)
    {
        GoutWI(6, "Reading new particle types ...");
        // Populating the array with different kind of rigid bodies in the XML
        // file
        uint offset = 0;
        for(uint i = 0; i < numUniqueParticles; i++)
        {
            DOMNode* nParticle = allParticles->item(i);
            for(uint j = 0; j < numEachUniqueParticle[i]; j++)
            {
                // Create the Rigid Body
                m_particleRigidBodyList[offset + j]
                    = new RigidBody<T, T>(nParticle);
                // Initial transformation of the rigid body
                particlesInitialTransform[offset + j]
                    = Transform3<T>(nParticle);
            }
            // Increment the starting position
            offset += numEachUniqueParticle[i];
        }
        GoutWI(6, "Reading particle types completed!");
    }

    // -------------------------------------------------------------------------
    // Obstacles
    DOMNode*     obstacles    = ReaderXML::getNode(root, "Obstacles");
    DOMNodeList* allObstacles = ReaderXML::getNodes(rootElement, "Obstacle");
    // Number of unique obstacles in the simulation
    uint numObstacles = allObstacles->getLength();
    // We also store the initial transformations of the rigid bodies to pass to
    // the ComponentManager to create particles with the initial transformation
    // required.
    std::vector<Transform3<T>> obstaclesInitialTransform(numObstacles);
    // Memory allocation for m_rigidBodyList with respect to the number of
    // shapes in the simulation.
    m_obstacleRigidBodyList
        = (RigidBody<T, T>**)malloc(numObstacles * sizeof(RigidBody<T, T>*));
    if(numObstacles)
    {
        GoutWI(6, "Reading obstacles types ...");
        for(uint i = 0; i < numObstacles; i++)
        {
            DOMNode* nObstacle = allObstacles->item(i);
            // Create the Rigid Body
            m_obstacleRigidBodyList[i] = new RigidBody<T, T>(nObstacle);
            // Initial transformation of the rigid body
            // One draw back is we might end up with the same rigid body shape,
            // but with different initial transformation.
            obstaclesInitialTransform[i] = Transform3<T>(nObstacle);
        }
        GoutWI(6, "Reading obstacles types completed!");
    }

    // -------------------------------------------------------------------------
    // LinkedCell
    GoutWI(6, "Constructing linked cell ...");
    // Finding the size of each cell -- max circumscribed radius among all
    // particles. Note that we loop until numParticles, because we do not care
    // about the circumscribed radius of the obstacles.
    T LCSize = T(0);
    for(int i = 0; i < numParticles; i++)
    {
        if(m_particleRigidBodyList[i]->getCircumscribedRadius() > LCSize)
            LCSize = m_particleRigidBodyList[i]->getCircumscribedRadius();
    }
    // Scaling coefficient of linked cell size
    T        LC_coeff = T(1);
    DOMNode* nLC      = ReaderXML::getNode(root, "LinkedCell");
    if(ReaderXML::hasNodeAttr(nLC, "CellSizeFactor"))
        LC_coeff = T(ReaderXML::getNodeAttr_Double(nLC, "CellSizeFactor"));
    if(LC_coeff < T(1))
        LC_coeff = T(1);
    GoutWI(9, "Cell size factor =", std::to_string(LC_coeff));

    // Creating linked cell
    GP::m_sizeLC = LC_coeff * T(2) * LCSize;
    m_linkedCell = (LinkedCell<T>**)malloc(sizeof(LinkedCell<T>*));
    *m_linkedCell
        = new LinkedCell<T>(GP::m_origin, GP::m_maxCoordinate, GP::m_sizeLC);
    GP::m_numCells = (*m_linkedCell)->getNumCells();
    GoutWI(9, "LinkedCell with", GP::m_numCells, "cells is created on host.");
    GoutWI(6, "Constructing linked cell completed!");

    // -------------------------------------------------------------------------
    // Setting up the component managers
    GP::m_numParticles = numParticles;
    GP::m_numObstacles = numObstacles;
    m_components       = new ComponentManagerCPU<T>(GP::m_numParticles,
                                              GP::m_numObstacles,
                                              GP::m_numCells);
    // Initialize the particles and obstacles
    m_components->initializeParticles(particlesInitialTransform);
    m_components->initializeObstacles(obstaclesInitialTransform);

    // -------------------------------------------------------------------------
    // Contact force models
    // Calculating the proper array size for the contact force array
    // We might need to redute it by removing the obstacle-obstacle pairs,
    // but it should be fine by now
    uint numMaterials     = GP::m_materialMap.size();
    GP::m_numContactPairs = numMaterials * (numMaterials + 1) / 2;
    DOMNode* contacts     = ReaderXML::getNode(root, "ContactForceModels");
    if(contacts)
    {
        GoutWI(6, "Reading the contact force model ...");
        m_contactForce
            = ContactForceModelBuilderFactory<T>::create(rootElement);
        GoutWI(6, "Reading the contact force model completed!");
    }

    // -------------------------------------------------------------------------
    // Temporal setting and time integration
    DOMNode* tempSetting = ReaderXML::getNode(root, "TemporalSetting");
    if(tempSetting)
    {
        DOMNode* nTime  = ReaderXML::getNode(tempSetting, "TimeInterval");
        T        tStart = ReaderXML::getNodeAttr_Double(nTime, "Start");
        T        tEnd   = ReaderXML::getNodeAttr_Double(nTime, "End");
        T        tStep  = ReaderXML::getNodeAttr_Double(nTime, "dt");
        GP::m_tStart    = tStart;
        GP::m_tEnd      = tEnd + HIGHEPS<T>;
        GP::m_dt        = tStep;
        DOMNode* nTI    = ReaderXML::getNode(tempSetting, "TimeIntegration");
        if(nTI)
        {
            GoutWI(6, "Reading the time integration model ...");
            m_timeIntegrator
                = (TimeIntegrator<T>**)malloc(sizeof(TimeIntegrator<T>*));
            *m_timeIntegrator
                = TimeIntegratorBuilderFactory<T>::create(nTI, GP::m_dt);
            GoutWI(6, "Reading the time integration model completed!");
        }
    }
}

// -----------------------------------------------------------------------------
// External force definition
template <typename T>
void Grains<T>::Forces(DOMElement* rootElement)
{
    assert(rootElement != NULL);
    DOMNode* root = ReaderXML::getNode(rootElement, "Forces");

    // Output message
    GoutWI(3, "Forces");

    // Read the forces
    if(root)
    {
        // Gravity
        DOMNode* nGravity = ReaderXML::getNode(root, "Gravity");
        if(nGravity)
        {
            GrainsParameters<T>::m_gravity[X]
                = T(ReaderXML::getNodeAttr_Double(nGravity, "GX"));
            GrainsParameters<T>::m_gravity[Y]
                = T(ReaderXML::getNodeAttr_Double(nGravity, "GY"));
            GrainsParameters<T>::m_gravity[Z]
                = T(ReaderXML::getNodeAttr_Double(nGravity, "GZ"));
            GoutWI(6,
                   "Gravity =",
                   Vector3ToString(GrainsParameters<T>::m_gravity));
        }
        else
        {
            GoutWI(6, "Gravity is mandatory!");
            exit(1);
        }
    }
}

// -----------------------------------------------------------------------------
// Additional features of the simulation: insertion, post-processing
template <typename T>
void Grains<T>::AdditionalFeatures(DOMElement* rootElement)
{
    // Output message
    GoutWI(3, "Simulation");
    // -------------------------------------------------------------------------
    // Checking if Simulation node is available
    assert(rootElement != NULL);
    DOMNode* root = ReaderXML::getNode(rootElement, "Simulation");
    if(!root)
    {
        GoutWI(6, "Simulation node is mandatory!");
        exit(1);
    }

    // -------------------------------------------------------------------------
    // Insertion policies
    DOMNode* nInsertion = ReaderXML::getNode(root, "ParticleInsertion");
    GoutWI(6, "Reading insertion policies ...");
    if(nInsertion)
        m_insertion = new Insertion<T>(nInsertion);
    else
    {
        GoutWI(9, "No policy found, setting the insertion policy to default");
        m_insertion = new Insertion<T>();
    }
    GoutWI(6, "Reading insertion policies completed.");

    // -------------------------------------------------------------------------
    // Post-processing writers
    DOMNode* nPostProcessing = ReaderXML::getNode(root, "PostProcessing");
    if(nPostProcessing)
    {
        GoutWI(3, "Post-processing");
        // Post-processing save time
        DOMNode* nTime  = ReaderXML::getNode(nPostProcessing, "TimeSave");
        T        tStart = ReaderXML::getNodeAttr_Double(nTime, "Start");
        T        tEnd   = ReaderXML::getNodeAttr_Double(nTime, "End");
        T        tStep  = ReaderXML::getNodeAttr_Double(nTime, "dt");
        for(T t = tStart; t <= tEnd; t += tStep)
            GrainsParameters<T>::m_tSave.push(t);
        // Save for tEnd as well
        GrainsParameters<T>::m_tSave.push(tEnd);

        // Post-processing writers
        DOMNode* nWriters = ReaderXML::getNode(nPostProcessing, "Writers");
        if(nWriters)
        {
            GoutWI(6, "Reading the post processing writers ...");
            DOMNodeList* allPPW = ReaderXML::getNodes(nWriters);
            for(uint i = 0; i < allPPW->getLength(); ++i)
            {
                DOMNode* nPPW = allPPW->item(i);
                m_postProcessor.push_back(
                    PostProcessingWriterBuilderFactory<T>::create(nPPW));
            }
            GoutWI(6, "Reading the post processing writers completed!");
        }
    }
    else
        GoutWI(6, "No postprocessing writer!");
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class Grains<float>;
template class Grains<double>;