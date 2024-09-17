#ifndef _PARAVIEWPOSTPROCESSINGWRITER_HH_
#define _PARAVIEWPOSTPROCESSINGWRITER_HH_


#include "PostProcessingWriter.hh"
#include "Transform3.hh"
#include "Kinematics.hh"


// =============================================================================
/** @brief The class ParaviewPostProcessingWriter.

    Writes data in files for post-processing with Paraview.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typedef T>
class ParaviewPostProcessingWriter : public PostProcessingWriter<T>
{
	protected:
		/** @name Parameters */
        //@{
        /** \brief output directory name */
		std::string m_ParaviewFilename_dir;
		/** \brief output file name */
		std::string m_ParaviewFilename;
		/** \brief particles output stream */
		vector<ostringstream*> m_Paraview_saveParticles_pvd;
		/** \brief writing in binary */
		bool m_binary;
		/** \brief writing separately for each type of particle */
		bool m_pertype;
		//@}


	public:
		/** @name Constructors */
		//@{
		/** @brief Default constructor */
		__HOST__
		ParaviewPostProcessingWriter();

		/** @brief Constructor with an XML node */
		__HOST__
		ParaviewPostProcessingWriter( DOMNode* dn );

		/** @brief Destructor */
		__HOST__
		~ParaviewPostProcessingWriter();
		//@}


		/** @name Get methods */
		//@{
		__HOST__
		PostProcessingWriterType getPostProcessingWriterType();
		//@}


		/** @name Methods */
		//@{
		/** @brief Initializes the post-processing writer */
		__HOST__
		void PostProcessing_start() final;

		/** @brief Writes data */
		__HOST__
		void PostProcessing( std::vector<RigidBody<T, T>> const* rb,
							 std::vector<unsigined int> const* RigidBodyID,
							 std::vector<Transform3<T>> const* t,
							 std::vector<Kinematics<T>> const* k ) final;

		/** @brief Finalizes writing data */
		void PostProcessing_end() final;
		//@}
};

#endif