#ifndef _POSTPROCESSINGWRITER_HH_
#define _POSTPROCESSINGWRITER_HH_

#include <filesystem>
#include <iostream>
#include <regex>

#include "Basic.hh"
#include "ComponentManager.hh"
#include "GrainsMisc.hh"
#include "ReaderXML.hh"

// PostProcessingWriter types
enum PostProcessingWriterType
{
    PARAVIEW,
    RAW
};

// =============================================================================
/** @brief The class PostProcessingWriter.

    Writes results in files for post-processing by an external software.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class PostProcessingWriter
{
protected:
    /** @name Constructors */
    //@{
    /** @brief Default constructor */
    __HOST__
    PostProcessingWriter();
    //@}

public:
    /** @name Constructors */
    //@{
    /** @brief Destructor */
    __HOST__
    virtual ~PostProcessingWriter();
    //@}

    /** @name Get methods */
    //@{
    __HOST__
    virtual PostProcessingWriterType getPostProcessingWriterType() const = 0;
    //@}

    /** @name Methods */
    //@{
    /** @brief Removes post-processing files already in the directory
     @param directory directory of the files to be removed
     @param patterns Regex patterns of the files to be removed */
    __HOST__
    void
        clearPostProcessingFiles(const std::filesystem::path&   directory,
                                 const std::vector<std::regex>& patterns) const;

    /** @brief Initializes the post-processing writer */
    __HOST__
    virtual void PostProcessing_start() = 0;

    /** @brief Writes post-processing data
     @param particleRB Arrays of particles rigid bodies
     @param obstacleRB Arrays of obstacles rigid bodies
     @param cm component manager
     @param currentTime Current simulation time */
    __HOST__
    virtual void PostProcessing(RigidBody<T, T> const* const* particleRB,
                                RigidBody<T, T> const* const* obstacleRB,
                                const ComponentManager<T>*    cm,
                                const T                       currentTime)
        = 0;

    /** @brief Finalizes writing data */
    __HOST__
    virtual void PostProcessing_end() = 0;
    //@}
};

#endif
