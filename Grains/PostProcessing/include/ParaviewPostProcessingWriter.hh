#ifndef _PARAVIEWPOSTPROCESSINGWRITER_HH_
#define _PARAVIEWPOSTPROCESSINGWRITER_HH_

#include "Kinematics.hh"
#include "PostProcessingWriter.hh"
#include "Transform3.hh"

// =============================================================================
/** @brief The class ParaviewPostProcessingWriter.

    Writes data in files for post-processing with Paraview.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class ParaviewPostProcessingWriter : public PostProcessingWriter<T>
{
protected:
    /** @name Parameters */
    //@{
    /** \brief Particles output stream */
    vector<ostringstream*> m_Paraview_saveParticles_pvd;
    /** \brief Obstacles output stream */
    ostringstream m_Paraview_saveObstacles_pvd;
    /** \brief Output directory name */
    std::string m_directory;
    /** \brief files root name */
    std::string m_rootName;
    /** \brief Writing in binary */
    bool m_binary;
    /** \brief Writing separately for each type of particle */
    bool m_pertype;
    /** \brief Cycle number */
    uint m_ParaviewCycleNumber;
    //@}

public:
    /** @name Constructors */
    //@{
    /** @brief Default constructor */
    __HOST__
    ParaviewPostProcessingWriter();

    /** @brief Constructor with an XML node */
    __HOST__
    ParaviewPostProcessingWriter(DOMNode* dn);

    /** @brief Destructor */
    __HOST__
    ~ParaviewPostProcessingWriter();
    //@}

    /** @name Get methods */
    //@{
    __HOST__
    PostProcessingWriterType getPostProcessingWriterType() const;
    //@}

    /** @name Methods */
    //@{
    /** @brief Removes post-processing files already in the directory */
    __HOST__
    void clearPostProcessingFiles() const;

    /** @brief Initializes the post-processing writer */
    __HOST__
    void PostProcessing_start() final;

    /** @brief Writes post-processing data
     @param particleRB Arrays of particles rigid bodies
     @param obstacleRB Arrays of obstacles rigid bodies
     @param cm component manager
     @param currentTime Current simulation time */
    __HOST__
    void PostProcessing(RigidBody<T, T> const* const* particleRB,
                        RigidBody<T, T> const* const* obstacleRB,
                        const ComponentManager<T>*    cm,
                        const T                       currentTime) final;

    /** @brief Finalizes writing data */
    void PostProcessing_end() final;
    //@}
};

#endif