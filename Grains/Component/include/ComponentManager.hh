#ifndef _COMPONENTMANAGER_HH_
#define _COMPONENTMANAGER_HH_


#include "Transform3.hh"
#include "Kinematics.hh"
#include "Torce.hh"
#include "RigidBody.hh"
#include "LinkedCell.hh"
#include "TimeIntegrator.hh"
#include "ContactForceModel.hh"
#include "ContactForceModelBuilderFactory.hh"
#include "CollisionDetection.hh"
#include "Insertion.hh"
#include "GrainsParameters.hh"


// =============================================================================
/** @brief The class ComponentManager.

    This is just an abstract class to make sure all derived classess follow the
    same set of methods.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ComponentManager
{
    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor (forbidden except in derived classes) */
        ComponentManager();

        /** @brief Destructor */
        virtual ~ComponentManager();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the number of particles in manager */
        virtual unsigned int getNumberOfParticles() const = 0;

        /** @brief Gets the number of obstacles in manager */
        virtual unsigned int getNumberOfObstacles() const = 0;

        /** @brief Gets the number of cells in manager */
        virtual unsigned int getNumberOfCells() const = 0;

        /** @brief Gets components rigid body Id */
        virtual std::vector<unsigned int> getRigidBodyId() const = 0;

        /** @brief Gets components transformation */
        virtual std::vector<Transform3<T>> getTransform() const = 0;

        /** @brief Gets components velocities */
        virtual std::vector<Kinematics<T>> getVelocity() const = 0;

        /** @brief Gets components torce */
        virtual std::vector<Torce<T>> getTorce() const = 0;

        /** @brief Gets the array of component Ids */
        virtual std::vector<int> getComponentId() const = 0;

        // /** @brief Gets the array of components neighbor Id */
        // std::vector<unsigned int> getNeighborsId() const;

        // /** @brief Gets the array of components neighbor count */
        // std::vector<unsigned int> getNeighborsCount() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the array of components rigid body Id */
        virtual void setRigidBodyId( std::vector<unsigned int> const& id ) = 0;

        /** @brief Sets components transformation */
        virtual void setTransform( std::vector<Transform3<T>> const& t ) = 0;

        /** @brief Sets components velocities */
        virtual void setVelocity( std::vector<Kinematics<T>> const& v ) = 0;

        /** @brief Sets components torce */
        virtual void setTorce( std::vector<Torce<T>> const& t ) = 0;

        /** @brief Sets the array of component Ids */
        virtual void setComponentId( std::vector<int> const& id ) = 0;

        // /** @brief Sets the array of components neighbor Id */
        // void setNeighborsId( std::vector<unsigned int> const& id );
        
        // /** @brief Sets the array of components neighbor count */
        // void setNeighborsCount( std::vector<unsigned int> const& count );
        //@}


        /** @name Methods */
        //@{
        /** @brief Copies data from another ComponentManager object.
        @param cm component manager on device */
        void copy( ComponentManager<T> const* cm );

        /** @brief Initializes the RigidBody IDs and transformations for the 
        simulation
        @param numEachRigidBody accumulating vector for number of different RB 
        @param initTr initial transformation of components */
        virtual void initialize( std::vector<unsigned int> numEachRigidBody,
                                 std::vector<Transform3<T>> initTr );

        /** @brief Inserts particles according to a given insertion policy
        @param ins insertion policy */
        virtual void insertParticles( Insertion<T>* ins );

        // /** @brief Sorts particles based on a Z-curve */
        // void sortParticles();

        // /** @brief Creates a neighbor list */
        // void createNeighborList();

        /** @brief Updates links between components and linked cell */
        // template <typename U>
        // TODO: @param
        virtual void updateLinks( LinkedCell<T> const* const* LC ) = 0;

        /** @brief Detects collision between components and computes forces */
        // template <typename U>
        // TODO: @param
        virtual void detectCollisionAndComputeContactForces( 
                                        LinkedCell<T> const* const* LC,
                                        RigidBody<T, T> const* const* RB, 
                                        ContactForceModel<T> const* const* CF,
                                        int* result ) = 0;

        /** @brief Updates the position and velocities of components */
        // TODO: @param
        virtual void moveComponents( TimeIntegrator<T> const* const* TI,
                                     RigidBody<T, T> const* const* RB ) = 0;
        //@}
};

typedef ComponentManager<float> ComponentManager_d;
typedef ComponentManager<double> ComponentManager_f;

#endif