#include "Sphere.hh"
#include "Box.hh"
#include "Superquadric.hh"
#include "ConvexBuilderFactory.hh"


// -----------------------------------------------------------------------------
// Construct a convex with an XML node as an input parameter
__HOST__
Convex* ConvexBuilderFactory::create( DOMNode* root )
{
    assert( root != NULL );
    Convex* convex = NULL;
    DOMNode* element = ReaderXML::getNodeNext( root );
    string   type    = ReaderXML::getNodeName( element );

    if ( type == "Sphere" ) 
        convex = new Sphere( element );
    else if ( type == "Box" ) 
        convex = new Box( element );
    else if ( type == "Superquadric" ) 
        convex = new Superquadric( element );
    else
    {
        cout << "Invalid convex type : " << type.c_str() << endl;
        exit(1);
    }

    return ( convex );
}




// -----------------------------------------------------------------------------
// Construct a convex with a type and a input stream as input parameters
__HOST__
Convex* ConvexBuilderFactory::create( string& type, 
                                      istream& fileIn )
{
    Convex *convex = NULL;

    if ( type == "Sphere" ) 
        convex = new Sphere( fileIn );
    else if ( type == "Box" ) 
        convex = new Box( fileIn );
    else if ( type == "Superquadric" ) 
        convex = new Superquadric( fileIn );
    else
    {
        cout << "Invalid convex type : " << type.c_str() << endl;
        exit(1);
    }

  return ( convex );
}
