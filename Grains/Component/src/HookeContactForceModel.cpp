#include "HookeContactForceModel.hh"
#include "GrainsUtils.hh"
#include "VectorMath.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__ HookeContactForceModel<T>::HookeContactForceModel()
{
}

// -----------------------------------------------------------------------------
// Constructor with an XML node
template <typename T>
__HOST__ HookeContactForceModel<T>::HookeContactForceModel(DOMNode* root)
{
    DOMNode* parameter;
    parameter = ReaderXML::getNode(root, "kn");
    if(!parameter)
        Gout(0, 9, "kn not defined!", "Aborting Grains!");
    m_kn = T(ReaderXML::getNodeValue_Double(parameter));

    parameter = ReaderXML::getNode(root, "en");
    if(!parameter)
        Gout(0, 9, "en not defined!", "Aborting Grains!");
    m_en   = T(ReaderXML::getNodeValue_Double(parameter));
    m_muen = log(m_en) / sqrt(PI<T> * PI<T> + log(m_en) * log(m_en));

    parameter = ReaderXML::getNode(root, "etat");
    if(!parameter)
        Gout(0, 9, "etat not defined!", "Aborting Grains!");
    m_etat = T(ReaderXML::getNodeValue_Double(parameter));

    parameter = ReaderXML::getNode(root, "muc");
    if(!parameter)
        Gout(0, 9, "muc not defined!", "Aborting Grains!");
    m_muc = T(ReaderXML::getNodeValue_Double(parameter));

    parameter = ReaderXML::getNode(root, "kr");
    if(!parameter)
        Gout(0, 9, "kr not defined!", "Aborting Grains!");
    m_kr = T(ReaderXML::getNodeValue_Double(parameter));
}

// -----------------------------------------------------------------------------
// Constructor with five values as contact parameters
template <typename T>
__HOSTDEVICE__ HookeContactForceModel<T>::HookeContactForceModel(T kn, T en, T etat, T muc, T kr)
    : m_kn(kn)
    , m_en(en)
    , m_etat(etat)
    , m_muc(muc)
    , m_kr(kr)
{
    m_muen = log(m_en) / sqrt(PI<T> * PI<T> + log(m_en) * log(m_en));
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ HookeContactForceModel<T>::~HookeContactForceModel()
{
}

// -----------------------------------------------------------------------------
// Gets the ContactForceModel type
template <typename T>
__HOSTDEVICE__ ContactForceModelType HookeContactForceModel<T>::getContactForceModelType() const
{
    return (HOOKE);
}

// -----------------------------------------------------------------------------
// Gets the parameters of the Hooke contact force model
template <typename T>
__HOSTDEVICE__ void HookeContactForceModel<T>::getContactForceModelParameters(
    T& kn, T& en, T& etat, T& muc, T& kr) const
{
    kn   = m_kn;
    en   = m_en;
    etat = m_etat;
    muc  = m_muc;
    kr   = m_kr;
}

// -----------------------------------------------------------------------------
// Performs forces & torques computation
template <typename T>
__HOSTDEVICE__ void
    HookeContactForceModel<T>::performForcesCalculus(ContactInfo<T> const& contactInfos,
                                                     Vector3<T> const&     relVelocityAtContact,
                                                     Vector3<T> const&     relAngVelocity,
                                                     T                     m1,
                                                     T                     m2,
                                                     Vector3<T>&           delFN,
                                                     Vector3<T>&           delFT,
                                                     Vector3<T>&           delM) const
{
    Vector3<T> geometricPointOfContact = contactInfos.getContactPoint();
    Vector3<T> penetration             = contactInfos.getContactVector();

    // Normal linear elastic force
    // We do this here as we want to modify the penetration vector later
    delFN = m_kn * penetration;

    // Unit normal vector at contact point
    penetration /= norm(penetration);
    penetration.round();

    Vector3<T> v_n = (relVelocityAtContact * penetration) * penetration;
    Vector3<T> v_t = relVelocityAtContact - v_n;

    // Unit tangential vector along relative velocity at contact point
    T          normv_t = norm(v_t);
    Vector3<T> tangent(zeroVector3T);
    if(normv_t > EPS<T>)
        tangent = v_t / normv_t;

    // Normal dissipative force
    T avmass = m1 * m2 / (m1 + m2);
    T omega0 = sqrt(m_kn / avmass);
    if(avmass == T(0))
    {
        avmass = m2 == T(0) ? T(0.5) * m1 : T(0.5) * m2;
        omega0 = T(2) * sqrt(m_kn / avmass);
    }
    T muen = -omega0 * m_muen;
    delFN += -T(2) * muen * avmass * v_n;
    T normFN = norm(delFN);

    // Tangential dissipative force
    delFT = (-m_etat * T(2) * avmass) * v_t;

    // Tangential Coulomb saturation
    T fn = m_muc * normFN;
    T ft = norm(delFT);
    if(fn < ft)
        delFT = (-fn) * tangent;

    // Rolling resistance moment
    if(m_kr)
    {
        // Relative angular velocity at contact point
        Vector3<T> wn     = (relAngVelocity * penetration) * penetration;
        Vector3<T> wt     = relAngVelocity - wn;
        T          normwt = norm(wt);

        // Anti-spinning effect along the normal wn
        delM = -m_kr * normFN * T(0.001) * wn;

        // Classical rolling resistance moment
        if(normwt > EPS<T>)
            delM -= m_kr * normFN * wt;
    }
}

// -----------------------------------------------------------------------------
// Returns a torce based on the contact information
template <typename T>
__HOSTDEVICE__ void HookeContactForceModel<T>::computeForces(ContactInfo<T> const& contactInfos,
                                                             Vector3<T> const& relVelocityAtContact,
                                                             Vector3<T> const& relAngVelocity,
                                                             T                 m1,
                                                             T                 m2,
                                                             Vector3<T> const& trOrigin,
                                                             Torce<T>&         torce) const
{
    // Compute contact force and torque
    Vector3<T> delFN, delFT, delM;
    performForcesCalculus(
        contactInfos, relVelocityAtContact, relAngVelocity, m1, m2, delFN, delFT, delM);

    Vector3<T> geometricPointOfContact = contactInfos.getContactPoint();
    torce.addForce(delFN + delFT, geometricPointOfContact - trOrigin);
    if(m_kr)
        torce.addTorque(delM);
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class HookeContactForceModel<float>;
template class HookeContactForceModel<double>;