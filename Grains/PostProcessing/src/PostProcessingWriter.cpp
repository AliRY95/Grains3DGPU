#include "PostProcessingWriter.hh"


// ----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__
PostProcessingWriter<T>::PostProcessingWriter()
{}




// ----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
PostProcessingWriter<T>::~PostProcessingWriter()
{}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class PostProcessingWriter<float>;
template class PostProcessingWriter<double>;
