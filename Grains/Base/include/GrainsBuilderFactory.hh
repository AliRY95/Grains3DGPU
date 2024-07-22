#ifndef _GRAINSBUILDERFACTORY_HH_
#define _GRAINSBUILDERFACTORY_HH_


#include "Grains.hh"
#include "ReaderXML.hh"


// =============================================================================
/** @brief The class GrainsBuilderFactory.

    Creates the appropriate Grains application depending on options.
    
    @author A.WACHS - Institut Francais du Petrole - 2009 - Creation 
    @author A.WACHS - 2019 - Major cleaning & refactoring
    @author A.Yazdani - 2024 - Modification for GPU */
// =============================================================================
template <typename T>
class GrainsBuilderFactory
{
  public:
    /**@name Constructors */
    //@{
    /** @brief Default constructor (forbidden) */
    GrainsBuilderFactory() {}

    /** @brief Destructor (forbidden) */
    ~GrainsBuilderFactory() {}
    //@}

    /**@name Static methods */
    //@{
    /** @brief Adds the path to the dtd files using the GRAINS_HOME variable to
    a copy of the input file. Returns the name of this copy.
    @param filename input file name */
    static string init( string const& filename );  

    /** @brief Creates and returns a standard Grains application
    @param root XML root ("<Grains3D>" or "<Graind2D>") */
    static Grains<T>* create( DOMElement* root );
    //@}
};

#endif

