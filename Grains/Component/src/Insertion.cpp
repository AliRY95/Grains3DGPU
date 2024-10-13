#include <ctime>
#include "Insertion.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Reads if the root is of type Random
template <typename T>
__HOST__
static INLINE
InsertionInfo<T> readDataRand( DOMNode* root )
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
        std::cout << shiftString12
                  << "Random initialization with "
                  << val
                  << " seed." 
                  << std::endl;
    }
    else if ( seedString == "Random" )
    {
        rgs = RGS_RANDOM;
        std::cout << shiftString12
                  << "Random initialization with random seed." 
                  << std::endl;
    }
    // if ( seedString == "Default" )
    else
    {
        rgs = RGS_DEFAULT;
        std::cout << shiftString12
                  << "Random initialization with default seed." 
                  << std::endl;
    }
    // srand
    // srand( static_cast<unsigned>( time( NULL ) ) );

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

    return ( insertionWindows );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type File
template <typename T>
__HOST__
static INLINE
InsertionInfo<T> readDataFile( DOMNode* root )
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

    return ( file );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type Constant
template <typename T>
__HOST__
static INLINE
InsertionInfo<T> readDataCons( DOMNode* root )
{
    T xVal = T( ReaderXML::getNodeAttr_Double( root, "X" ) );
    T yVal = T( ReaderXML::getNodeAttr_Double( root, "Y" ) );
    T zVal = T( ReaderXML::getNodeAttr_Double( root, "Z" ) );
    Vector3<T> vec( xVal, yVal, zVal );
    std::cout << shiftString12
              << "Constant initialization with ["
              << vec
              << "]." << std::endl;
    
    return ( vec );
}




// -----------------------------------------------------------------------------
// Reads if the root is of type Zero
template <typename T>
__HOST__
static INLINE
InsertionInfo<T> readDataZero( DOMNode* root )
{
    Vector3<T> vec( T( 0 ), T( 0 ), T( 0 ) );
    std::cout << shiftString12
              << "Zero initialization."
              << std::endl;
    
    return ( vec );
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
, m_positionInsertionInfo( Vector3<T>() )
, m_orientationInsertionInfo( Vector3<T>() )
, m_translationalVelInsertionInfo( Vector3<T>() )
, m_angularVelInsertionInfo( Vector3<T>() )
{}




// -----------------------------------------------------------------------------
// Constructor with XML node
template <typename T>
__HOST__
Insertion<T>::Insertion( DOMNode* dn )
{
    // We define a lambda function to read the XML node
    auto read = []( DOMNode* root, 
                    InsertionType& type, 
                    InsertionInfo<T>& data )
    {
        std::string nType = ReaderXML::getNodeAttr_String( root, "Type" );
        if( nType == "Random" )
        {
            type = RANDOMINSERTION;
            data = readDataRand<T>( root );
        }
        else if ( nType == "File" )
        {
            type = FILEINSERTION;
            data = readDataFile<T>( root );
        }
        else if ( nType == "Constant" )
        {
            type = CONSTANTINSERTION;
            data = readDataCons<T>( root );
        }
        else if( nType == "Zero" )
        {
            type = DEFAULTINSERTION;
            data = readDataZero<T>( root );
        }
        else
        {
            std::cout << "Unknown Type in ParticleInsertion! "
                      << "Aborting Grains!"
                      << std::endl;
            exit( 1 );
        }
    };


    DOMNode* nIP = ReaderXML::getNode( dn, "InitialPosition" );
    std::cout << shiftString9
              << "Reading PositionInsertion Policy ..." 
              << std::endl;
    read( nIP, m_positionType, m_positionInsertionInfo );
    
    std::cout << shiftString9 
              << "Reading OrientationInsertion Policy ..." 
              << std::endl;
    DOMNode* nIO = ReaderXML::getNode( dn, "InitialOrientation" );
    read( nIO, m_orientationType, m_orientationInsertionInfo );
    
    std::cout << shiftString9
              << "Reading VeclocityInsertion Policy ..." 
              << std::endl;
    DOMNode* nIV = ReaderXML::getNode( dn, "InitialVelocity" );
    read( nIV, m_translationalVelType, m_translationalVelInsertionInfo );
    
    std::cout << shiftString9
              << "Reading AngularVeclocityInsertion Policy ..." 
              << std::endl;
    DOMNode* nIA = ReaderXML::getNode( dn, "InitialAngularVelocity" );
    read( nIA, m_angularVelType, m_angularVelInsertionInfo );
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
// Returns a vector of Vector3 accroding to type and data
template <typename T>
__HOST__
Vector3<T> Insertion<T>::fetchInsertionDataForEach( InsertionType const type,
                                                    InsertionInfo<T>& data )
{
    // We only return a vector3. It is clear how it works for position, and 
    // kinematics. However, for orientation, it returns the vector3 of rotation
    // angles. We later construct a rotation matrix.
    if ( type == RANDOMINSERTION )
    {
        // TODO: support multiple InsertionWindow
        // Randomly choose between the available insertion windows
        // std::vector<InsertionWindow<T>> IWs = 
        //                     std::get<std::vector<InsertionWindow<T>>>( data );
        // int random_IW = rand() % IWs.size();
        // Generates a random point within the chosen window
        // output = IWs[random_IW].generateRandomPoint();
        // output = IWs[0].generateRandomPoint();
        Vector3<T> output;
        output = std::get<std::vector<InsertionWindow<T>>>( data )[0].generateRandomPoint();
        return( output );
    }
    else if ( type == FILEINSERTION )
    {
        Vector3<T> output;
        std::get<std::ifstream>( data ) >> output;
        return( output );
    }
    else if ( type == CONSTANTINSERTION )
        return( std::get<Vector3<T>>( data ) );
    else
        return( Vector3<T>() );
}




// -----------------------------------------------------------------------------
// Returns all required data members to insert components as a vector
template <typename T>
__HOST__
std::pair<Transform3<T>, Kinematics<T>> Insertion<T>::fetchInsertionData()
{
    // Position
    Vector3<T> pos = fetchInsertionDataForEach( m_positionType, 
                                                m_positionInsertionInfo );
    
    // Orientation angles. These are not matrices, so we have to compute the
    // rotation matrices.
    Vector3<T> ori = fetchInsertionDataForEach( m_orientationType, 
                                                m_orientationInsertionInfo );

    // Velocity
    Vector3<T> vel = fetchInsertionDataForEach( m_translationalVelType, 
                                                m_translationalVelInsertionInfo );

    // Angular velocity
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