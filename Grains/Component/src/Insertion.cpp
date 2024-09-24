#include "Insertion.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__
Insertion<T>::Insertion()
: m_positionType( DEFAULT )
, m_orientationType( DEFAULT )
, m_translationalVelType( DEFAULT )
, m_angularVelType( DEFAULT )
, m_positionInsertionInfo( NULL )
, m_orientationInsertionInfo( NULL )
, m_translationalVelInsertionInfo( NULL )
, m_angularVelInsertionInfo( NULL )
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
    readXML( nIP, m_positionType, m_positionInsertionInfo );
    DOMNode* nIO = ReaderXML::getNode( dn, "InitialOrientation" );
    readXML( nIO, m_orientationType, m_orientationInsertionInfo );
    DOMNode* nIV = ReaderXML::getNode( dn, "InitialVelocity" );
    readXML( nIV, m_translationalVelType, m_translationalVelInsertionInfo );
    DOMNode* nIA = ReaderXML::getNode( dn, "InitialAngularVelocity" );
    readXML( nIA, m_angularVelType, m_angularVelInsertionInfo );
}




// -----------------------------------------------------------------------------
// Constructor with InsertionType and InsertionInfo for each component.
template <typename T>
__HOST__
Insertion<T>::Insertion( InsertionType pos,
                         InsertionType ori,
                         InsertionType vel,
                         InsertionType ome,
                         InsertionInfo const& posData,
                         InsertionInfo const& oriData,
                         InsertionInfo const& velData,
                         InsertionInfo const& omeData,
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
// Reads an XML node to figure out the insertion type and related info
template <typename T>
__HOST__
void Insertion<T>::readXML( DOMNode* root,
                            InsertionType& type,
                            InsertionInfo& data )
{
    std::string nType = ReaderXML::getNodeAttr_String( root, "Type" );
    if( nType == "Random" )
    {
        type = Random;
        data = 0;
        // TODO: fix the seed here
        // std::string seedString = ReaderXML::getNodeAttr_String( root, "Seed" );
        // if ( seedString == "Default" )
    }
    else if( nType == "File" )
    {
        type = FILE;
        data = ReaderXML::getNodeAttr_String( root, "Name" );
    }
    else if( nType == "Constant" )
    {
        type = CONSTANT;
        T xVal = T( ReaderXML::getNodeAttr_Double( root, "X" ) );
        T yVal = T( ReaderXML::getNodeAttr_Double( root, "Y" ) );
        T zVal = T( ReaderXML::getNodeAttr_Double( root, "Z" ) );
        data = Vector3<T>( xVal, yVal, zVal );
    }
    else if( nType == "Constant" )
    {
        type = CONSTANT;
        data = NULL;
    }
    else
    {
        std::cout << "Unknown Type in ParticleInsertion! "
                  << "Aborting Grains!"
                  << std::endl;
        exit( 1 );
    }
}




// -----------------------------------------------------------------------------
// return all required data members to insert components as a vector
template <typename T>
__HOST__
std::vector<std::pair<Transform<T>, Kinematics<T>>> 
Insertion<T>::fetchInsertionData() const
{

    // First, positions
    if ( m_positionType == FILE )
    {
         // Create a unique pointer to ifstream
        std::ifstream fileStream = 
            std::make_unique<std::ifstream>( std::get<std::string> m_positionInsertion );
    }
}




// -----------------------------------------------------------------------------
// return all required data members to insert components as a vector
template <typename T>
__HOST__
std::vector<Vector3<T>> 
Insertion<T>::fetchInsertionData( InsertionType const type,
                                  InsertionInfo const& data,
                                  unsigned int numToInsert ) const
{
    std::vector<Vector3<T>> output( numToInsert, Vector3<T>() )
    Vector3<T> temp;
    // // TODO:
    // if ( type == RANDOM )
    // {

    // }
    // else if ( type == FILE )
    if ( type == FILE )
    {
        // retrieivng the insertion file name
        std::string fileName = std::get<std::string> data;
        // Create a unique pointer to ifstream
        std::ifstream fileIn = std::make_unique<std::ifstream>( fileName );
        if ( !fileIn.is_open() )
        {
            std::cout << "Error in opening the insertion file "
                      << fileName
                      << ". Aborting Grains!"
                      << std::endl;
            exit( 1 );
        }
        for ( unsigned int i = 0; i < numToInsert; i++ )
            fileIn >> output[i];
    }
    else if ( type == CONSTANT )
    {
        Vector3<T> constant = std::get<Vector3<T>> data;
        for ( unsigned int i = 0; i < numToInsert; i++ )
            output[i] = constant;
    }
    // else
    return( output );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Insertion<float>;
template class Insertion<double>;