#include <ctime>
#include "VectorMath.hh"
#include "InsertionWindow.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__
InsertionWindow<T>::InsertionWindow()
{}




// -----------------------------------------------------------------------------
// Constructor with XML node and the type of the seed
template <typename T>
__HOST__
InsertionWindow<T>::InsertionWindow( DOMNode* dn,
                                     RandomGeneratorSeed seed )
{
    std::cout << shiftString12
              << "Reading insertion window ..." 
              << std::endl;

    // Setting up the random generator engine with the given seed
    if ( seed == RGS_DEFAULT )
        m_randGenerator.seed( 0 );
    else if ( seed == RGS_RANDOM )
        m_randGenerator.seed( time( NULL ) );
    
    std::string nType = ReaderXML::getNodeAttr_String( dn, "Type" );
    if ( nType == "Box" )
    {
        m_type = BOXWINDOW;
        DOMNode* nP1 = ReaderXML::getNode( dn, "MinPoint" );
        T xVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "X" ) );
        T yVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "Y" ) );
        T zVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "Z" ) );
        m_v1 = Vector3<T>( xVal1, yVal1, zVal1 );
        DOMNode* nP2 = ReaderXML::getNode( dn, "MaxPoint" );
        T xVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "X" ) );
        T yVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "Y" ) );
        T zVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "Z" ) );
        m_v2 = Vector3<T>( xVal2, yVal2, zVal2 );
        std::cout << shiftString15
                  << "Box insertion window with min and max points ["
                  << m_v1
                  << "] and ["
                  << m_v2
                  << "]." 
                  << std::endl;
        std::cout << shiftString12
                  << "Reading insertion window completed!" 
                  << std::endl;
    }
    else if ( nType == "Annulus" )
    {
        m_type = ANNULUSWINDOW;
        DOMNode* nP1 = ReaderXML::getNode( dn, "BottomPoint" );
        T xVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "X" ) );
        T yVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "Y" ) );
        T zVal1 = T( ReaderXML::getNodeAttr_Double( nP1, "Z" ) );
        m_v1 = Vector3<T>( xVal1, yVal1, zVal1 );
        DOMNode* nP2 = ReaderXML::getNode( dn, "TopPoint" );
        T xVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "X" ) );
        T yVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "Y" ) );
        T zVal2 = T( ReaderXML::getNodeAttr_Double( nP2, "Z" ) );
        m_v2 = Vector3<T>( xVal2, yVal2, zVal2 );
        DOMNode* nR = ReaderXML::getNode( dn, "Radius" );
        m_iRad = T( ReaderXML::getNodeAttr_Double( nR, "Inner" ) );
        m_oRad = T( ReaderXML::getNodeAttr_Double( nR, "Outter" ) );
        std::cout << shiftString15
                  << "Annulus insertion window with bottom point ["
                  << m_v1
                  << "], direction ["
                  << m_v2
                  << "], outter radius "
                  << m_oRad 
                  << " , and inner radius "
                  << m_iRad
                  << "."
                  << std::endl;
        std::cout << shiftString12
                  << "Reading insertion window completed!" 
                  << std::endl;
    }
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
InsertionWindow<T>::~InsertionWindow()
{}




// -----------------------------------------------------------------------------
// Generates a random number with uniform distribution in window
template <typename T>
__HOST__
Vector3<T> InsertionWindow<T>::generateRandomPoint()
{
    Vector3<T> out;
    if ( m_type == BOXWINDOW )
    {
        out = Vector3<T>( 
                m_v1[X] + m_dist( m_randGenerator ) * ( m_v2[X] - m_v1[X] ),
                m_v1[Y] + m_dist( m_randGenerator ) * ( m_v2[Y] - m_v1[Y] ),
                m_v1[Z] + m_dist( m_randGenerator ) * ( m_v2[Z] - m_v1[Z] ) );
    }
    else if ( m_type == ANNULUSWINDOW )
    {
        // Step 1: Sample random angle theta between 0 and 2*pi
        T theta = m_dist( m_randGenerator ) * TWO_PI<T>;

        // Step 2: Sample random radius r
        T u = m_dist( m_randGenerator );
        T r = sqrt( ( T( 1 ) - u ) * m_iRad * m_iRad + u * m_oRad * m_oRad);

        // Step 3: Sample random height
        Vector3<T> h = m_v1 + m_dist( m_randGenerator ) * ( m_v2 - m_v1 );

        // Assemble everything together:
        // TODO: NOT CORRECT - FIX LATER
        out = Vector3<T>( h + 
                    Vector3<T>( r * cos( theta ), r * sin( theta ), T( 0 ) ) );
    }
    return ( out );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class InsertionWindow<float>;
template class InsertionWindow<double>;