#include <ctime>
#include "Insertion.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Reads if the root is of type Random
template <typename T>
__HOST__
static INLINE
std::pair<InsertionType, InsertionInfo<T>> readTypeAndDataRand( DOMNode* root )
{
    // Random generator seed. We pass it to InsertionWindow directly.
    // We also set the seed with srand. We use it for to randomly pick an 
    // insertion window
    RandomGeneratorSeed rgs;
    std::string seedString = ReaderXML::getNodeAttr_String( root, "Seed" );
    if ( seedString == "UserDefined" )
    {
        unsigned int val = ReaderXML::getNodeAttr_Int( root, "Value" );
        if ( val )
            rgs = RGS_UDEF;
        else
        {
            std::cout << "Seed value is not provided. Aborting Grains!" 
                        << std::endl;
            exit( 1 );
        }

        // srand
        srand( static_cast<unsigned>( val ) );
    }
    else if ( seedString == "Random" )
    {
        rgs = RGS_RANDOM;

        // srand
        srand( static_cast<unsigned>( time( NULL ) ) );
    }
    // if ( seedString == "Default" )
    else
    {
        rgs = RGS_DEFAULT;

        // srand
        srand( static_cast<unsigned>( 0 ) );
    }
    std::cout << shiftString12
              << "Random initialization with "
              << rgs
              << " seed." << std::endl;

    // Insertion window
    DOMNode* nWindows = ReaderXML::getNode( root, "Windows" );
    std::vector<InsertionWindow<T>> insertionWindows;
    if ( nWindows )
    {
        // DOMNodeList* allWindows = ReaderXML::getNodes( nWindows, "Window" );
        DOMNodeList* allWindows = ReaderXML::getNodes( nWindows );
        for ( int i = 0; i < allWindows->getLength(); i++ )
        {
            DOMNode* nWindow = allWindows->item( i );
            insertionWindows.push_back( InsertionWindow<T>( nWindow, rgs ) );
        }
    }

    return ( std::make_pair( RANDOMINSERTION, insertionWindows ) );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type File
template <typename T>
__HOST__
static INLINE
std::pair<InsertionType, InsertionInfo<T>> readTypeAndDataFile( DOMNode* root )
{
    std::string fileName = ReaderXML::getNodeAttr_String( root, "Name" );
    std::ifstream file( fileName );
    // Check whether the file exists
    if ( file.good() )
    {
        std::cout << shiftString12
                  << "File initialization with path "
                  << fileName
                  << "." << std::endl;
    }
    else
    {
        std::cout << shiftString12
                  << "File does not exist. Aborting Grains!" << std::endl;
        exit( 1 );
    }

    return ( std::make_pair( FILEINSERTION, file ) );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type Constant
template <typename T>
__HOST__
static INLINE
std::pair<InsertionType, InsertionInfo<T>> readTypeAndDataCons( DOMNode* root )
{
    T xVal = T( ReaderXML::getNodeAttr_Double( root, "X" ) );
    T yVal = T( ReaderXML::getNodeAttr_Double( root, "Y" ) );
    T zVal = T( ReaderXML::getNodeAttr_Double( root, "Z" ) );
    Vector3<T> vec( xVal, yVal, zVal );
    std::cout << shiftString12
              << "Constant initialization with ["
              << vec
              << "]." << std::endl;
    
    return ( std::make_pair( CONSTANTINSERTION, vec ) );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type Zero
template <typename T>
__HOST__
static INLINE
std::pair<InsertionType, InsertionInfo<T>> readTypeAndDataZero( DOMNode* root )
{
    Vector3<T> vec( T( 0 ), T( 0 ), T( 0 ) );
    std::cout << shiftString12
              << "Zero initialization."
              << std::endl;
    
    return ( std::make_pair( DEFAULTINSERTION, vec ) );
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Default constructor
template <typename T>
__HOST__
Insertion<T>::Insertion()
: m_positionType( DEFAULTINSERTION )
, m_orientationType( DEFAULTINSERTION )
, m_translationalVelType( DEFAULTINSERTION )
, m_angularVelType( DEFAULTINSERTION )
, m_positionInsertionInfo( 0 )
, m_orientationInsertionInfo( 0 )
, m_translationalVelInsertionInfo( 0 )
, m_angularVelInsertionInfo( 0 )
{}




// -----------------------------------------------------------------------------
// Constructor with XML node
template <typename T>
__HOST__
Insertion<T>::Insertion( DOMNode* dn )
{
    DOMNode* nIP = ReaderXML::getNode( dn, "InitialPosition" );
    std::cout << shiftString9
              << "Reading PositionInsertion Policy ..." 
              << std::endl;
    std::tie( m_positionType, m_positionInsertionInfo ) = 
                                                        readTypeAndData( nIP );
    
    std::cout << shiftString9 
              << "Reading OrientationInsertion Policy ..." 
              << std::endl;
    DOMNode* nIO = ReaderXML::getNode( dn, "InitialOrientation" );
    std::tie( m_orientationType, m_orientationInsertionInfo ) = 
                                                        readTypeAndData( nIO );
    
    std::cout << shiftString9
              << "Reading VeclocityInsertion Policy ..." 
              << std::endl;
    DOMNode* nIV = ReaderXML::getNode( dn, "InitialVelocity" );
    std::tie( m_translationalVelType, m_translationalVelInsertionInfo ) = 
                                                        readTypeAndData( nIV );
    
    std::cout << shiftString9
              << "Reading AngularVeclocityInsertion Policy ..." 
              << std::endl;
    DOMNode* nIA = ReaderXML::getNode( dn, "InitialAngularVelocity" );
    std::tie( m_angularVelType, m_angularVelInsertionInfo ) = 
                                                        readTypeAndData( nIA );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
Insertion<T>::~Insertion()
{
    if ( std::holds_alternative<std::ifstream>( m_positionInsertionInfo ) )
        ( std::get<std::ifstream>( m_positionInsertionInfo ) ).close();
    if ( std::holds_alternative<std::ifstream>( m_orientationInsertionInfo ) )
        ( std::get<std::ifstream>( m_orientationInsertionInfo ) ).close();
    if ( std::holds_alternative<std::ifstream>( m_translationalVelInsertionInfo ) )
        ( std::get<std::ifstream>( m_translationalVelInsertionInfo ) ).close();
    if ( std::holds_alternative<std::ifstream>( m_angularVelInsertionInfo ) )
        ( std::get<std::ifstream>( m_angularVelInsertionInfo ) ).close();
}




// -----------------------------------------------------------------------------
// Reads an XML node to set the insertion type and related info
template <typename T>
__HOST__
std::pair<InsertionType, InsertionInfo<T>> 
Insertion<T>::readTypeAndData( DOMNode* root ) const
{
    std::string nType = ReaderXML::getNodeAttr_String( root, "Type" );
    if( nType == "Random" )
        return ( readTypeAndDataRand<T>( root ) );
    else if ( nType == "File" )
        return ( readTypeAndDataFile<T>( root ) );
    else if ( nType == "Constant" )
        return ( readTypeAndDataCons<T>( root ) );
    else if( nType == "Zero" )
        return ( readTypeAndDataZero<T>( root ) );
    else
    {
        std::cout << "Unknown Type in ParticleInsertion! "
                  << "Aborting Grains!"
                  << std::endl;
        exit( 1 );
    }
}




// -----------------------------------------------------------------------------
// Returns a vector of Vector3 accroding to type and data
template <typename T>
__HOST__
Vector3<T> Insertion<T>::fetchInsertionDataForEach( 
                                                InsertionType const type,
                                                InsertionInfo<T>& data )
{
    // We only return a vector3. It is clear how it works for position, and 
    // kinematics. However, for orientation, it returns the vector3 of rotation
    // angles. We later construct a rotation matrix.
    Vector3<T> output;

    if ( type == RANDOMINSERTION )
    {
        // Randomly choose between the available insertion windows
        auto IWs = std::get<std::vector<InsertionWindow<T>>>( data );
        int random_IW = rand() % ( IWs.size() + 1 );
        // Generates a random point within the chosen window
        output = IWs[random_IW].generateRandomPoint();
    }
    else if ( type == FILEINSERTION )
        std::get<std::ifstream>( data ) >> output;
    else if ( type == CONSTANTINSERTION )
        output = std::get<Vector3<T>>( data );
    else if ( type == DEFAULTINSERTION )
        output = Vector3<T>();
    
    return( output );
}




// -----------------------------------------------------------------------------
// Returns all required data members to insert components as a vector
template <typename T>
__HOST__
std::pair<Transform3<T>, Kinematics<T>> Insertion<T>::fetchInsertionData()
{
    // vector of positions
    Vector3<T> pos = fetchInsertionDataForEach( m_positionType, 
                                                m_positionInsertionInfo );
    
    // vector of orientation angles. These are not matrices, so we have to
    // compute the rotation matrices.
    Vector3<T> ori = fetchInsertionDataForEach( m_orientationType, 
                                                m_orientationInsertionInfo );

    // vector of velocities
    Vector3<T> vel = fetchInsertionDataForEach( m_translationalVelType, 
                                                m_translationalVelInsertionInfo );

    // vector of angular velocities
    Vector3<T> ang = fetchInsertionDataForEach( m_angularVelType, 
                                                m_angularVelInsertionInfo );


    // Transformation
    Transform3<T> tr;
    tr.setBasis( ori[X], ori[Y], ori[Z] );
    tr.setOrigin( pos );
    
    // Kinematics
    Kinematics<T> k( vel, ang );

    return( std::make_pair( tr, k ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Insertion<float>;
template class Insertion<double>;