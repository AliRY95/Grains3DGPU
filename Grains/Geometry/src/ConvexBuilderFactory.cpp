#include "ConvexBuilderFactory.hh"
#include "Convex.hh"
#include "Sphere.hh"
#include "Box.hh"
#include "Cylinder.hh"
#include "Cone.hh"
#include "Superquadric.hh"
#include "Rectangle.hh"


// -----------------------------------------------------------------------------
// Construct a convex with an XML node as an input parameter
template <typename T>
Convex<T>* ConvexBuilderFactory<T>::create( DOMNode* root )
{
    Convex<T>* convex;
    DOMNode* element = ReaderXML::getNodeNext( root );
    string type = ReaderXML::getNodeName( element );

    if ( type == "Sphere" ) 
        convex = new Sphere<T>( element );
    else if ( type == "Box" ) 
        convex = new Box<T>( element );
    else if ( type == "Cylinder" ) 
        convex = new Cylinder<T>( element );
    else if ( type == "Cone" ) 
        convex = new Cone<T>( element );
    else if ( type == "Superquadric" )
        convex = new Superquadric<T>( element );
    else if ( type == "Rectangle" ) 
        convex = new Rectangle<T>( element );
    else
    {
        cout << "Invalid convex type: " << type.c_str() << endl;
        exit( 1 );
    }

    return ( convex ); // returns the host-side pointer to convex object
}




// -----------------------------------------------------------------------------
// Construct a convex with a type and a input stream as input parameters
template <typename T>
Convex<T>* ConvexBuilderFactory<T>::create( string& type, 
                                            istream& fileIn )
{
    Convex<T>* convex = NULL;
    if ( type == "Sphere" ) 
        convex = new Sphere<T>( fileIn );
    else if ( type == "Box" ) 
        convex = new Box<T>( fileIn );
    else if ( type == "Cylinder" ) 
        convex = new Cylinder<T>( fileIn );
    else if ( type == "Cone" ) 
        convex = new Cone<T>( fileIn );
    else if ( type == "Superquadric" ) 
        convex = new Superquadric<T>( fileIn );
    else if ( type == "Rectangle" ) 
        convex = new Rectangle<T>( fileIn );
    else
    {
        cout << "Invalid convex type: " << type.c_str() << endl;
        exit( 1 );
    }

    return ( convex );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class ConvexBuilderFactory<float>;
template class ConvexBuilderFactory<double>;