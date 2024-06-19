#ifndef _CONVEXBUILDERFACTORY_HH_
#define _CONVEXBUILDERFACTORY_HH_

class Convex;

#include <iostream>
#include <string>
#include "Convex.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class ConvexBuilderFactory.

    Static class that constructs a convex using input data from an XML node or a
    stream. Create a vonvex of the derived type and returns a pointer to the
    mother class Convex.

    @author G.FERRER - Institut Francais du Petrole - 2003 - Creation
    @author D. RAKOTONIRINA - IFP Energies nouvelles - Oct. 2014 - Modification 
    @author A.WACHS - 2019 - Major cleaning & refactoring
    @author A.Yazdani - 2024 - Modification */
// =============================================================================
class ConvexBuilderFactory
{
    public:
        /** @name Methods Static */
        //@{
        /** @brief Construct a convex with an XML node as an input parameter
        @param root XML node */
        __HOST__
        static Convex* create( DOMNode *root );

        /** @brief Construct a convex with a type and a input stream as input 
        parameters
        @param type convex type
        @param fileIn input stream */
        __HOST__
        static Convex* create( string& type, 
                               std::istream& fileIn );
        //@}
};

#endif
