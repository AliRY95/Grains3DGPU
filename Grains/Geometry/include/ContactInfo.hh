#ifndef _CONTACTINFO_HH_
#define _CONTACTINFO_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief The class ContactInfo.

    Contains all the features of a contact point.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ContactInfo
{
    protected:
        /** @name Parameters */
        //@{
        Vector3<T> m_contactPoint; /**< contact point */
        Vector3<T> m_contactVector; /**< contact vector */
        T m_overlapDistance; /**< overlap distance */
        // int m_nbIterGJK; /**< number of iterations of GJK for convergence */
        //@}

    public:
        /** @name Constructors */
        //@{
        /** @brief Default constructor */
        __HOSTDEVICE__
        ContactInfo();

        /** @brief Constructor with contact point location in the world reference
        frame, overlap vector, overlap distance and number of iterations of GJK as 
        input parameters
        @param pt contact point
        @param vec contact vector
        @param distance_ overlap distance */
        __HOSTDEVICE__
        ContactInfo( Vector3<T> const& pt, 
                     Vector3<T> const& vec, 
                     T overlap );

        /** @brief Destructor */
        __HOSTDEVICE__
        ~ContactInfo();
        //@}


        /** @name Get methods */
        //@{
        /** @brief Gets the contact point */
        __HOSTDEVICE__
        Vector3<T> getContactPoint() const;
    
        /** @brief Gets the contact vector */
        __HOSTDEVICE__
        Vector3<T> getContactVector() const;

        /** @brief Gets the overlap distance */
        __HOSTDEVICE__
        T getOverlapDistance() const;
        //@}


        /** @name Set methods */
        //@{
        /** @brief Sets the contact point
        @param p contact point */
        __HOSTDEVICE__
        void setContactPoint( Vector3<T> const& p );
    
        /** @brief Sets the contact vector
        @param v overlap vector */
        __HOSTDEVICE__
        void setContactVector( Vector3<T> const& v );

        /** @brief Sets the overlap distance
        @param d overlap distance  */
        __HOSTDEVICE__
        void setOverlapDistance( T d );
        //@}
};


typedef ContactInfo<double> ContactInfoD;
typedef ContactInfo<float> ContactInfoF;


// __HOSTDEVICE__
// static ContactInfoD noContact( Vec3d( 0., 0., 0. ), 
//                               Vec3d( 0., 0., 0. ),
//                               1.e20 );
// __HOSTDEVICE__
// static ContactInfoF noContactF( Vec3f( 0., 0., 0. ), 
//                                 Vec3f( 0., 0., 0. ),
//                                 1.e20 );	

#endif