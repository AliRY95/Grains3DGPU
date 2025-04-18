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
    /** \brief Output directory name */
    std::string m_ParaviewFilename_dir;
    /** \brief Output file name */
    std::string m_ParaviewFilename;
    /** \brief Particles output stream */
    vector<ostringstream*> m_Paraview_saveParticles_pvd;
    /** \brief Obstacles output stream */
    ostringstream m_Paraview_saveObstacles_pvd;
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
    /** @brief Initializes the post-processing writer */
    __HOST__
    void PostProcessing_start() final;

    /** @brief Writes data */
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