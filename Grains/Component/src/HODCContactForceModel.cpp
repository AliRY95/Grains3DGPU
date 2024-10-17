#include "HODCContactForceModel.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
HODCContactForceModel<T>::HODCContactForceModel()
{}




// -----------------------------------------------------------------------------
// Constructor with an XML node
template <typename T>
__HOST__
HODCContactForceModel<T>::HODCContactForceModel( DOMNode* root )
{
    DOMNode* parameter;
    parameter = ReaderXML::getNode( root, "stiff" );
    m_stiff = T( ReaderXML::getNodeValue_Double( parameter ) );

    parameter = ReaderXML::getNode( root, "muc" );
    m_muec = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    parameter = ReaderXML::getNode( root, "en" );
    m_en = T( ReaderXML::getNodeValue_Double( parameter ) );
    m_muen = log( m_en ) / sqrt( PI<T> * PI<T> + log( m_en ) * log( m_en ) );

    parameter = ReaderXML::getNode( root, "mut" );
    m_muet = T( ReaderXML::getNodeValue_Double( parameter ) ); 

    parameter = ReaderXML::getNode( root, "kms" );
    m_kms = T( ReaderXML::getNodeValue_Double( parameter ) ); 
}




// -----------------------------------------------------------------------------
// Constructor with five values as contact parameters
template <typename T>
__HOSTDEVICE__
HODCContactForceModel<T>::HODCContactForceModel( T stiff,
                                                 T en, 
                                                 T muet, 
                                                 T muec, 
                                                 T kms )
: m_stiff( stiff )
, m_en( en )
, m_muet( muet )
, m_muec( muec )
, m_kms( kms )                                     
{
    m_muen = log( m_en ) / sqrt( PI<T> * PI<T> + log( m_en ) * log( m_en ) );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
HODCContactForceModel<T>::~HODCContactForceModel()
{}




// -----------------------------------------------------------------------------
// Gets the ContactForceModel type
template <typename T>
__HOSTDEVICE__
ContactForceModelType HODCContactForceModel<T>::getContactForceModelType() const
{
    return( HODC );
}




// -----------------------------------------------------------------------------
// Gets the parameters of the HODC contact force model
template <typename T>
__HOSTDEVICE__
void HODCContactForceModel<T>::getContactForceModelParameters( T& stiff,
                                                               T& en, 
                                                               T& muet, 
                                                               T& muec, 
                                                               T& kms ) const
{
    stiff = m_stiff;
    en = m_en;
    muet = m_muet;
    muec = m_muec;
    kms = m_kms;
}




// -----------------------------------------------------------------------------
// Performs forces & torques computation
template <typename T>
__HOSTDEVICE__
void HODCContactForceModel<T>::performForcesCalculus( 
                                        ContactInfo<T> const& contactInfos,
                                        Vector3<T> const& relVelocityAtContact,
                                        Vector3<T> const& relAngVelocity,
                                        T m1,
                                        T m2,
                                        Vector3<T>& delFN,
                                        Vector3<T>& delFT,
                                        Vector3<T>& delM ) const
{
    Vector3<T> geometricPointOfContact = contactInfos.getContactPoint();
    Vector3<T> penetration = contactInfos.getContactVector();
    
    // Normal linear elastic force
    // We do this here as we want to modify the penetration vector later
    delFN = m_stiff * penetration;

    // Unit normal vector at contact point
    penetration /= norm( penetration );
    penetration.round();

    Vector3<T> v_n = ( relVelocityAtContact * penetration ) * penetration;
    Vector3<T> v_t = relVelocityAtContact - v_n;

    // Unit tangential vector along relative velocity at contact point 
    T normv_t = norm( v_t );
    Vector3<T> tangent( zeroVector3T );
    if ( normv_t > EPS<T> )
        tangent = v_t / normv_t;
  
    // Normal dissipative force  
    T avmass = m1 * m2 / ( m1 + m2 );
    T omega0 = sqrt( m_stiff / avmass );
    if ( avmass == T( 0 ) ) 
    {
        avmass = m2 == T( 0 ) ? T( 0.5 ) * m1 : T( 0.5 ) * m2;
        omega0 = T( 2 ) * sqrt( m_stiff / avmass );
    }
    T muen = - omega0 * m_muen;
    delFN += - T( 2 ) * muen * avmass * v_n;
    T normFN = norm( delFN );
  
    // Tangential dissipative force
    delFT = ( -m_muet * T( 2 ) * avmass ) * v_t;  

    // Tangential Coulomb saturation
    T fn = m_muec * normFN;
    T ft = norm( delFT );
    if ( fn < ft ) 
        delFT = ( -fn ) * tangent;
  
    // Rolling resistance moment
    if ( m_kms )
    {
        // Relative angular velocity at contact point
        Vector3<T> wn = ( relAngVelocity * penetration ) * penetration;
        Vector3<T> wt = relAngVelocity - wn;
        T normwt = norm( wt );

        // Anti-spinning effect along the normal wn
        delM = - m_kms * normFN * T( 0.001 ) * wn;
        
        // Classical rolling resistance moment
        if ( normwt > EPS<T> )
            delM -= m_kms * normFN * wt;
    }
}




// -----------------------------------------------------------------------------
// Returns a torce based on the contact information
template <typename T>
__HOSTDEVICE__
void HODCContactForceModel<T>::computeForces( 
                                        ContactInfo<T> const& contactInfos,
                                        Vector3<T> const& relVelocityAtContact,
                                        Vector3<T> const& relAngVelocity,
                                        T m1,
                                        T m2,
                                        Vector3<T> const& trOrigin,
                                        Torce<T>& torce ) const
{
    // Compute contact force and torque
    Vector3<T> delFN, delFT, delM;
    performForcesCalculus( contactInfos,
                           relVelocityAtContact,
                           relAngVelocity,
                           m1,
                           m2,
                           delFN,
                           delFT, 
                           delM );

    Vector3<T> geometricPointOfContact = contactInfos.getContactPoint();
    torce.addForce( delFN + delFT, geometricPointOfContact - trOrigin );
    if ( m_kms )
        torce.addTorque( delM );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class HODCContactForceModel<float>;
template class HODCContactForceModel<double>;