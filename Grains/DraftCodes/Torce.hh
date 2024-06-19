#ifndef _TORCE_HH_
#define _TORCE_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief The class Torce.

    A torque and a force (Torque + Force = Torce) imposed on the center of mass 
    of a component.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T = double>
class Torce
{
    private:
        /**@name Parameter */
        //@{
        Vector3<T> m_torque; /**< torques exerted at the center of mass */
        Vector3<T> m_force; /**< force exerted on the component */
        //@}
    
    public:
        /**@name Constructors */
        //@{
        /** @brief Default constructor */
        __host__ __device__
        Torce();

        /** @brief Constructor with a torque and a force as input parameters
        @param t torque
        @param f force */
        __host__ __device__
        Torce( Vector3<T> const& t, 
               Vector3<T> const& f );

        /** @brief Destructor */
        __host__ __device__
        ~Torce();
        //@}


        /**@name Get methods */
        //@{
        /** @brief Gets the total torque of the torce */
        __host__ __device__
        Vector3<T> getTorque() const;

        /** @brief Gets the total force of the torce */
        __host__ __device__
        Vector3<T> getForce() const;
        //@}  


        /**@name Set methods */
        //@{
        /** @brief Sets the torque of the torce
        @param t torque */
        __host__ __device__
        void setTorque( Vector3<T> const& t );

        /** @brief Sets the force of the torce
        @param f force */
        __host__ __device__
        void setForce( Vector3<T> const& f );
        //@}


        /**@name Methods */
        //@{
        /** @brief Adds a torque to the torce
        @param t the added torque */
        __host__ __device__
        void addTorque( Vector3<T> const& t );

        /** @brief Adds a force to the torce
        @param f the added force */
        __host__ __device__
        void addForce( Vector3<T> const& f ); 
        //@}    
};

#endif
