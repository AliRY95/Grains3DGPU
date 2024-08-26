#ifndef _KINEMATICS_HH_
#define _KINEMATICS_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief The class Kinematics.

    Manages the kinematics (velocities or accelerations) of components.

    @author A.YAZDANI - 2024 - Construction */
// ============================================================================
template <typename T>
class Kinematics
{
    protected:
        /** @name Parameters */
        //@{
        Vector3<T> m_translational; /**< Translational component */
        Vector3<T> m_angular; /**< Angular component */  
        //@}
    
    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        Kinematics();

        /** @brief Constructor with two vectors as input parameters
        @param translational translational components
        @param angular angular component */
        __HOSTDEVICE__
        Kinematics( Vector3<T> const& translational,
                    Vector3<T> const& angular );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~Kinematics();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the translational component of the kinematics */
        __HOSTDEVICE__
        Vector3<T> getTranslationalComponent() const;
        
        /** @brief Gets the angular component of the kinematics */
        __HOSTDEVICE__
        Vector3<T> getAngularComponent() const;
        //@}
    

        /** @name Set methods */
        //@{
        /** @brief Sets the translational component of the kinematics
        @param translational translational component */
        __HOSTDEVICE__
        void setTranslationalComponent( Vector3<T> const& translational );

        /** @brief Sets the angular component of the kinematics
        @param angular angular component */
        __HOSTDEVICE__
        void setAngularComponent( Vector3<T> const& angular );
        //@}


        /** @name Methods */
        //@{
        /** @brief Adds a vector to the translational component
        @param translational vector to add to the translational component */
        __HOSTDEVICE__
        void addToTranslationalComponent( Vector3<T> const& translational );

        /** @brief Adds a angular velocity to the angular velocity
        @param angular vector to add to the angular component */
        __HOSTDEVICE__
        void addToAngularComponent( Vector3<T> const& angular );

        /** @brief Returns the total velocity U + om x R given R 
        @param R arm vector */
        __HOSTDEVICE__
        Vector3<T> Velocity( Vector3<T> const& R ) const;
        //@}    
};


typedef Kinematics<float> KinematicsF;
typedef Kinematics<double> KinematicsD;


#endif
