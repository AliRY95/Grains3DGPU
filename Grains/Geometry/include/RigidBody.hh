#ifndef _RIGIDBODY_HH_
#define _RIGIDBODY_HH_


#include "BoundingBox.hh"
#include "Convex.hh"
#include "Kinematics.hh"
#include "Quaternion.hh"
#include "Torce.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class RigidBody.

    Rigid bodies comprising their shapes and physical attributes. The precision
    is managed by two typenames "T" and "U". "T" corresponds to the precision of
    the rigid body, and "U" represents the precision of the bounding volume
    encapsulating the rigid body. We explicitly instantiate three classes out of
    this template; (T, U) = (double, double), (double, float), (float, float).

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
        Vector3<T> m_scaling; /**< Scaling vector related to crust thickness */
        unsigned int m_material; /**< Rigid body's material ID */
        T m_volume; /**< Rigid body's volume */
        T m_mass; /**< Rigid body's mass */
        T m_inertia[6]; /**< Rigid body's inertia */
        T m_inertia_1[6]; /**< Rigid body's inversed inertia */
        BoundingBox<U>* m_boundingBox; /** Bounding box of the convex body **/
        U m_circumscribedRadius; /**< Circumscribed radius */
        //@}


    public:
        /**@name Constructeurs */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        RigidBody();

        /** @brief Constructor with a convex, crust thickness, material, and 
        density
        @param convex convex
        @param ct crust thickness of the rigid body 
        @param material material ID
        @param density density */
        __HOSTDEVICE__
        RigidBody( Convex<T>* convex, 
                   T ct,
                   unsigned int material,
                   T density );

        /** @brief Constructor with an XML input
        @param root XML input */
        __HOST__
        RigidBody( DOMNode* root );

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

        /** @brief Gets the scaling vector related to crust thickness */
        __HOSTDEVICE__ 
        Vector3<T> getScalingVector() const;

        /** @brief Gets the rigid body's material ID */
        __HOSTDEVICE__ 
        unsigned int getMaterial() const;
        
        /** @brief Gets the rigid body's volume */
        __HOSTDEVICE__ 
        T getVolume() const;

        /** @brief Gets the rigid body's mass */
        __HOSTDEVICE__ 
        T getMass() const;

        /** @brief Gets the rigid body's inertia
        @param inertia the destination for inertia */
        __HOSTDEVICE__ 
        void getInertia( T (&inertia)[6] ) const;

        /** @brief Gets the inverse of rigid body's inertia
        @param inertia_1 the destination for the inverse inertia */
        __HOSTDEVICE__ 
        void getInertia_1( T (&inertia_1)[6] ) const;

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


        /**@name Methods */
        //@{
        /** @brief Computes the acceleration of the rigid body as a kinematics
        object after imposing a torce (Torque + Force). The assumption is that
        the torce is given in the body-fixed coordinate system, hence, there is
        no need to have the quaternion
        @param omega angular velocity in the body-fixed coordinate system
        @param t imposed torce in the body-fixed coordinate system */
        __HOSTDEVICE__
        Kinematics<T> computeMomentum( Vector3<T> const& omega,
                                       Torce<T> const& t ) const;

        /** @brief Computes the acceleration of the rigid body as a kinematics
        object after imposing a torce (Torque + Force). The assumption is that
        the torce is given in the space-fixed coordinate system.
        @param omega angular velocity in the space-fixed coordinate system
        @param t imposed torce in the space-fixed coordinate system
        @param q quaternion of rotation from space to body coordinate systems */
        __HOSTDEVICE__
        Kinematics<T> computeMomentum( Vector3<T> const& omega,
                                       Torce<T> const& t,
                                       Quaternion<T> const& q ) const;
        //@}    
};


typedef RigidBody<float, float> RigidBodyF;
typedef RigidBody<double, float> RigidBodyDF;
typedef RigidBody<double, double> RigidBodyD;


#endif
