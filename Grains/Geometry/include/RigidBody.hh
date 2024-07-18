#ifndef _RIGIDBODY_HH_
#define _RIGIDBODY_HH_


#include "BoundingBox.hh"
#include "Convex.hh"
#include "ReaderXML.hh"


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

        /** @brief Constructor with a convex and the crust thickness
        @param convex convex
        @param ct crust thickness of the rigid body */
        __HOSTDEVICE__
        RigidBody( Convex<T>* convex, T ct );

        /** @brief Constructor with an XML input
        @param convex convex
        @param ct crust thickness of the rigid body */
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


#endif
