#ifndef _RIGIDBODY_HH_
#define _RIGIDBODY_HH_


#include "BoundingBox.hh"
#include "Convex.hh"
#include "ContactInfo.hh"


// =============================================================================
/** @brief The class RigidBody.

    Rigid bodies comprising their shapes and physical attributes.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T, typename U>
class RigidBody
{
    protected:
        /**@name Parameters */
        //@{
        Convex<T>* m_convex; /**< Convex shape */
        T m_crustThickness; /**< Rigid body's crust thickness */
        T m_volume; /**< Rigid body's volume */
        T* m_inertia; /**< Rigid body's inertia */
        T* m_inertia_1; /**< Rigid body's inversed inertia */
        BoundingBox<U>* m_boundingBox; /** Bounding box of the convex body **/
        U m_circumscribedRadius; /**< Circumscribed radius */
        //@}

    public:
        /**@name Constructeurs */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        RigidBody();

        /** @brief Constructor with a convex
        @param convex convex
        @param ct crust thickness of the rigid body */
        __HOSTDEVICE__
        RigidBody( Convex<T>* convex, T ct );

        /** @brief Copy constructor
        @param rb RigidBody object to be copied */
        __HOSTDEVICE__
        RigidBody( RigidBody<T, U> const& rb );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~RigidBody();
        //@}


        /**@name Get methods */
        //@{
        /** @brief Gets the rigid body's convex */
        __HOSTDEVICE__ 
        Convex<T>* getConvex() const;

        /** @brief Gets the rigid body's crust thickness */
        __HOSTDEVICE__ 
        T getCrustThickness() const;
        
        /** @brief Gets the rigid body's volume */
        __HOSTDEVICE__ 
        T getVolume() const;

        /** @brief Gets the rigid body's inertia */
        __HOSTDEVICE__ 
        T* getInertia() const;

        /** @brief Gets the inverse of rigid body's inertia */
        __HOSTDEVICE__ 
        T* getInertia_1() const;

        /** @brief Gets the rigid body's bounding box */
        __HOSTDEVICE__ 
        BoundingBox<U>* getBoundingBox() const;

        /** @brief Gets the rigid body's circumscribed radius */
        __HOSTDEVICE__
        U getCircumscribedRadius() const;
        //@}

        
        /**@name Set methods */
        //@{
        // TODO: IMPLEMENT SET METHODS
        // /** @brief Sets the rigid body's convex
        // @param convex convex object */
        // __HOSTDEVICE__ void setConvex( Convex const* convex );
        
        // /** @brief Sets the rigid body's curst thickness
        // @param ct crust thickness */
        // __HOSTDEVICE__ void setCrustThickness( double ct );

        // /** @brief Sets the rigid body's volume
        // @param v volume */
        // __HOSTDEVICE__ void setVolume( double v );

        // /** @brief Sets the rigid body's inertia
        // @param inertia inertia tensor */
        // __HOSTDEVICE__ void setInertia( double* inertia );

        // /** @brief Sets the inverse of rigid body's inertia
        // @param inertia_1 inverse of the inertia tensor */
        // __HOSTDEVICE__ void setInertia_1( double* inertia_1 );

        // /** @brief Sets the rigid body's bounding box
        // @param r circumscribed radius */
        // __HOSTDEVICE__ void setOBB( OBB const* bb );

        // /** @brief Sets the rigid body's circumscribed radius
        // @param r circumscribed radius */
        // __HOSTDEVICE__ void setCircumscribedRadius( float r );
        //@}
};


typedef RigidBody<float, float> RigidBodyF;
typedef RigidBody<double, float> RigidBodyDF;
typedef RigidBody<double, double> RigidBodyD;


/** @name RigidBody : External methods collision detection */
//@{
/** @brief Returns whether 2 rigid bodies intersect
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__ 
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
                           Transform3<T> const& a2w,
                           Transform3<T> const& b2w );

/** @brief Returns whether 2 rigid bodies intersect - relative transformation
 @param rbA first rigid body
 @param rbB second rigid body
 @param b2a geometric tramsformation describing convex B in the A's reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
                           Transform3<T> const& b2a );

/** @brief Returns the contact information (if any) for 2 rigid bodies
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
 template <typename T, typename U>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, U> const& rbA,
                                         RigidBody<T, U> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w );

/** @brief Returns the contact information (if any) for 2 rigid bodies
 @param rbA first rigid body
 @param rbB second rigid body
 @param a2w geometric tramsformation describing convex A in the world reference
 frame
 @param b2w geometric tramsformation describing convex B in the world reference
 frame */
 template <typename T>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, T> const& rbA,
                                         RigidBody<T, T> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w );
                                         
// TODO: LATER
// /** @brief Returns the contact information (if any) for 2 rigid bodies - 
// relative transformation
//  @param rbA first rigid body
//  @param rbB second rigid body
//  @param b2a geometric tramsformation describing convex B in the A's reference
//  frame */
// __HOSTDEVICE__
// ContactInfo closestPointsRigidBodies( RigidBody const& rbA,
//                                       RigidBody const& rbB,
//                                       Transform3d const& b2a );
//@}


#endif
