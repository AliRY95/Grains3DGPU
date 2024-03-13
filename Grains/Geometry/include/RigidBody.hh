#ifndef _RIGIDBODY_HH_
#define _RIGIDBODY_HH_


#include "BoundingBox.hh"
#include "Convex.hh"
#include "ContactInfo.hh"


// =============================================================================
/** @brief The class RigidBody.

    Rigid bodies.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
class RigidBody
{
    protected:
        /**@name Parameters */
        //@{
        Convex* m_convex; /**< Convex shape */
        double m_crustThickness; /**< Rigid body's crust thickness */
        double m_volume; /**< Rigid body's volume */
        double* m_inertia; /**< Rigid body's inertia */
        double* m_inertia_1; /**< Rigid body's inversed inertia */
        BoundingBox* m_boundingBox; /** Bounding box of the convex body **/
        float m_circumscribedRadius; /**< Circumscribed radius */
        //@}

    public:
        /**@name Constructeurs */
        //@{
        /** @brief Default constructor */
        __host__ __device__
        RigidBody();

        /** @brief Constructor with a convex
        @param convex convex
        @param ct crust thickness of the rigid body */
        __host__ __device__
        RigidBody( Convex* convex, double ct );

        /** @brief Copy constructor
        @param rb RigidBody object to be copied */
        __host__ __device__
        RigidBody( RigidBody const& rb );

        /** @brief Destructor */
        __host__ __device__
        ~RigidBody();
        //@}


        /**@name Get methods */
        //@{
        /** @brief Gets the rigid body's convex */
        __host__ __device__ 
        Convex* getConvex() const;

        /** @brief Gets the rigid body's crust thickness */
        __host__ __device__ 
        double getCrustThickness() const;
        
        /** @brief Gets the rigid body's volume */
        __host__ __device__ 
        double getVolume() const;

        /** @brief Gets the rigid body's inertia */
        __host__ __device__ 
        double* getInertia() const;

        /** @brief Gets the inverse of rigid body's inertia */
        __host__ __device__ 
        double* getInertia_1() const;

        /** @brief Gets the rigid body's bounding box */
        __host__ __device__ 
        BoundingBox* getBoundingBox() const;

        /** @brief Gets the rigid body's circumscribed radius */
        __host__ __device__
        float getCircumscribedRadius() const;
        //@}

            
        /**@name Set methods */
        //@{
        // /** @brief Sets the rigid body's convex
        // @param convex convex object */
        // __host__ __device__ void setConvex( Convex const* convex );
        
        // /** @brief Sets the rigid body's curst thickness
        // @param ct crust thickness */
        // __host__ __device__ void setCrustThickness( double ct );

        // /** @brief Sets the rigid body's volume
        // @param v volume */
        // __host__ __device__ void setVolume( double v );

        // /** @brief Sets the rigid body's inertia
        // @param inertia inertia tensor */
        // __host__ __device__ void setInertia( double* inertia );

        // /** @brief Sets the inverse of rigid body's inertia
        // @param inertia_1 inverse of the inertia tensor */
        // __host__ __device__ void setInertia_1( double* inertia_1 );

        // /** @brief Sets the rigid body's bounding box
        // @param r circumscribed radius */
        // __host__ __device__ void setOBB( OBB const* bb );

        // /** @brief Sets the rigid body's circumscribed radius
        // @param r circumscribed radius */
        // __host__ __device__ void setCircumscribedRadius( float r );
        //@}
};


/** @name RigidBody : External methods collision detection */
//@{
/** @brief Returns whether 2 rigid bodies intersect
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
__host__ __device__ 
bool intersectRigidBodies( RigidBody const& rbA,
                           RigidBody const& rbB,
                           Transform3d const& a2w,
                           Transform3d const& b2w );

/** @brief Returns whether 2 rigid bodies intersect - relative transformation
 @param rbA first rigid body
 @param rbB second rigid body
 @param b2a geometric tramsformation describing convex B in the A's reference
 frame */
__host__ __device__
bool intersectRigidBodies( RigidBody const& rbA,
                           RigidBody const& rbB,
                           Transform3d const& b2a );

/** @brief Returns the contact information (if any) for 2 rigid bodies
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
__host__ __device__
ContactInfo closestPointsRigidBodies( RigidBody const& rbA,
                                      RigidBody const& rbB,
                                      Transform3d const& a2w,
                                      Transform3d const& b2w );

// TODO: LATER
// /** @brief Returns the contact information (if any) for 2 rigid bodies - 
// relative transformation
//  @param rbA first rigid body
//  @param rbB second rigid body
//  @param b2a geometric tramsformation describing convex B in the A's reference
//  frame */
// __host__ __device__
// ContactInfo closestPointsRigidBodies( RigidBody const& rbA,
//                                       RigidBody const& rbB,
//                                       Transform3d const& b2a );
//@}


#endif
