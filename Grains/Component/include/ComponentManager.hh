#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_


// =============================================================================
/** @brief The class ComponentManager.

    Components in the simulation.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class ComponentManager
{
    protected:
        /** @name Parameters */
        //@{
        Mat3d* orientation; /**< array of components orientation */
        Vec3d* position; /**< array of components position */
        // Vec3d* force; /**< array of components force */
        // Vec3d* torque; /**< array of components torque */
        // Vec3d* translationalVelocity; /**< array of components velocity */
        // Vec3d* angularVelocity; /**< array of components angular velocity */
        unsigned int* neighborsID; /**< array of components neighbor ID */ 
        unsigned int* rigidBodyId; /**< array of components rigid body ID */
        unsigned int* compId; /**< array of component IDs */
        unsigned int* neighborsCount; /**< array of components neighbor count */
        // bool* isActive; /**< array of components activity in the simulation */
        // bool* isObstacle; /**< array of components flag for being obstacle */
        //@}

    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor */
        ComponentManager();

        /** @brief Constructor with the number of particles randomly positioned 
        in the computational domain */
        ComponentManager( unsigned int numParticles );

        /** @brief Destructor */
        ~ComponentManager();
        //@}


        /** @name Methods */
        //@{
        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Detects collision between particles */
        void detectCollision();

        // /** @brief Computes impact forces */
        // void computeForces();

        // /** @brief Updates the position and velocities of particles */
        // void updateParticles();
        //@}
};


#endif