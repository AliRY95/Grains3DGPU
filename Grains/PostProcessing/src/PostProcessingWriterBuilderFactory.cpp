#include "PostProcessingWriterBuilderFactory.hh"
#include "GrainsUtils.hh"
#include "ParaviewPostProcessingWriter.hh"
#include "RawDataPostProcessingWriter.hh"

// -----------------------------------------------------------------------------
// Creates a post-processing writer from an XML node
template <typename T>
__HOST__ PostProcessingWriter<T>*
         PostProcessingWriterBuilderFactory<T>::create(DOMNode* nPPW)
{
    std::string              PPWName = ReaderXML::getNodeName(nPPW);
    PostProcessingWriter<T>* ppw     = NULL;

    if(PPWName == "RawData")
        ppw = new RawDataPostProcessingWriter<T>(nPPW);
    else if(PPWName == "Paraview")
        ppw = new ParaviewPostProcessingWriter<T>(nPPW);
    else
    {
        GoutWI(6, "Unknown postprocessing writer in node <Writers>");
        exit(1);
    }

    return (ppw);
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class PostProcessingWriterBuilderFactory<float>;
template class PostProcessingWriterBuilderFactory<double>;