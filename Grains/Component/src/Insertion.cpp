#include <ctime>
#include "Insertion.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__
Insertion<T>::Insertion( unsigned int numToInsert )
: m_positionType( DEFAULTINSERTION )
, m_orientationType( DEFAULTINSERTION )
, m_translationalVelType( DEFAULTINSERTION )
, m_angularVelType( DEFAULTINSERTION )
, m_positionInsertionInfo( 0 )
, m_orientationInsertionInfo( 0 )
, m_translationalVelInsertionInfo( 0 )
, m_angularVelInsertionInfo( 0 )
, m_numToInsert( numToInsert )
{}




// -----------------------------------------------------------------------------
// Constructor with XML node
template <typename T>
__HOST__
Insertion<T>::Insertion( DOMNode* dn,
                         unsigned int numToInsert )
: m_numToInsert( numToInsert )                         
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
// Constructor with InsertionType and InsertionInfo for each component.
template <typename T>
__HOST__
Insertion<T>::Insertion( InsertionType pos,
                         InsertionType ori,
                         InsertionType vel,
                         InsertionType ome,
                         InsertionInfo<T> const& posData,
                         InsertionInfo<T> const& oriData,
                         InsertionInfo<T> const& velData,
                         InsertionInfo<T> const& omeData,
                         unsigned int numToInsert )
: m_positionType( pos )
, m_orientationType( ori )
, m_translationalVelType( vel )
, m_angularVelType( ome )
, m_positionInsertionInfo( posData )
, m_orientationInsertionInfo( oriData )
, m_translationalVelInsertionInfo( velData )
, m_angularVelInsertionInfo( omeData )
, m_numToInsert( numToInsert )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
Insertion<T>::~Insertion()
{}




// -----------------------------------------------------------------------------
// Reads an XML node to set the insertion type and related info
template <typename T>
__HOST__
std::pair<InsertionType, InsertionInfo<T>> 
Insertion<T>::readTypeAndData( DOMNode* root ) const
{
    InsertionType type;
    InsertionInfo<T> data;
    std::string nType = ReaderXML::getNodeAttr_String( root, "Type" );
    if( nType == "Random" )
    {
        type = RANDOMINSERTION;
        std::string seedString = ReaderXML::getNodeAttr_String( root, "Seed" );
        if ( seedString == "Default" )
            data = 0;
        else if ( seedString == "UserDefined" )
        {
            unsigned int val = ReaderXML::getNodeAttr_Int( root, "Value" );
            if ( val )
                data = val;
            else
            {
                std::cout << "Seed value is not provided. Default is used!" 
                          << std::endl;
                data = 0;
            }
        }
        else if ( seedString == "Random" )
        {
            data = time( NULL );
        }
    }
    else if( nType == "File" )
    {
        type = FILEINSERTION;
        data = ReaderXML::getNodeAttr_String( root, "Name" );
    }
    else if( nType == "Constant" )
    {
        type = CONSTANTINSERTION;
        T xVal = T( ReaderXML::getNodeAttr_Double( root, "X" ) );
        T yVal = T( ReaderXML::getNodeAttr_Double( root, "Y" ) );
        T zVal = T( ReaderXML::getNodeAttr_Double( root, "Z" ) );
        data = Vector3<T>( xVal, yVal, zVal );
    }
    else if( nType == "Zero" )
    {
        type = DEFAULTINSERTION;
        data = 0;
    }
    else
    {
        std::cout << "Unknown Type in ParticleInsertion! "
                  << "Aborting Grains!"
                  << std::endl;
        exit( 1 );
    }
    return ( std::make_pair( type, data ) );
}




// -----------------------------------------------------------------------------
// Returns a vector of Vector3 accroding to type and data
template <typename T>
__HOST__
std::vector<Vector3<T>> Insertion<T>::fetchInsertionDataForEach( 
                                            InsertionType const type,
                                            InsertionInfo<T> const& data ) const
{
    std::vector<Vector3<T>> output( m_numToInsert, Vector3<T>() );
    Vector3<T> temp;

    if ( type == RANDOMINSERTION )
    {
        // TODO
    }
    else if ( type == FILEINSERTION )
    {
        // retrieivng the insertion file name
        std::string fileName = std::get<std::string>( data );
        // Create a unique pointer to ifstream
        std::ifstream fileIn( fileName );
        if ( !fileIn.is_open() )
        {
            std::cout << "Error in opening the insertion file "
                      << fileName
                      << ". Aborting Grains!"
                      << std::endl;
            exit( 1 );
        }
        for ( unsigned int i = 0; i < m_numToInsert; i++ )
            fileIn >> output[i];
        fileIn.close();
    }
    else if ( type == CONSTANTINSERTION )
    {
        Vector3<T> constant = std::get<Vector3<T>>( data );
        for ( unsigned int i = 0; i < m_numToInsert; i++ )
            output[i] = constant;
    }
    // else
    return( output );
}




// -----------------------------------------------------------------------------
// Returns all required data members to insert components as a vector
template <typename T>
__HOST__
std::vector<std::pair<Transform3<T>, Kinematics<T>>> 
Insertion<T>::fetchInsertionData() const
{
    // vector of positions
    vector<Vector3<T>> pos = 
    fetchInsertionDataForEach( m_positionType, m_positionInsertionInfo );
    
    // vector of orientation angles. These are not matrices, so we have to
    // compute the rotation matrices.
    vector<Vector3<T>> ori = 
    fetchInsertionDataForEach( m_orientationType, m_orientationInsertionInfo );

    // vector of velocities
    vector<Vector3<T>> vel = 
    fetchInsertionDataForEach( m_translationalVelType, m_translationalVelInsertionInfo );

    // vector of angular velocities
    vector<Vector3<T>> ang = 
    fetchInsertionDataForEach( m_angularVelType, m_angularVelInsertionInfo );


    Transform3<T> tr;
    Kinematics<T> k;
    std::vector<std::pair<Transform3<T>, Kinematics<T>>> output;
    // preparing the output
    for ( unsigned int i = 0; i < m_numToInsert; i++ )
    {
        // transform
        tr.setBasis( ori[i][X], ori[i][Y], ori[i][Z] );
        tr.setOrigin( pos[i] );

        // kinematics
        k.setTranslationalComponent( vel[i] );
        k.setAngularComponent( ang[i] );

        output.push_back( std::make_pair( tr, k ) );
    }

    return( output );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Insertion<float>;
template class Insertion<double>;