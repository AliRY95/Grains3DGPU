#ifndef _POSTPROCESSINGWRITERBUILDERFACTORY_HH_
#define _POSTPROCESSINGWRITERBUILDERFACTORY_HH_

#include "PostProcessingWriter.hh"
#include "ReaderXML.hh"
#include <string>

// =============================================================================
/** @brief The class PostProcessingWriterBuilderFactory.

    Creates the appropriate post-processing writer depending on options.

    @author A.WACHS - Institut Francais du Petrole - 2011 - Creation 
    @author A.WACHS - 2019 - Major cleaning & refactoring
    @author A.Yazdani - 2024 - Porting to GPU */
// =============================================================================
template <typename T>
class PostProcessingWriterBuilderFactory
{
private:
    /**@name Constructors & Destructor */
    //@{
    /** @brief Default constructor (forbidden) */
    __HOST__
    PostProcessingWriterBuilderFactory();

    /** @brief Destructor (forbidden) */
    __HOST__
    ~PostProcessingWriterBuilderFactory();
    //@}

public:
    /** @name Methods */
    //@{
    /** @brief Creates a post-processing writer from an XML node
		@param nPPW XMl node */
    __HOST__
    static PostProcessingWriter<T>* create(DOMNode* nPPW);
    //@}
};

#endif
