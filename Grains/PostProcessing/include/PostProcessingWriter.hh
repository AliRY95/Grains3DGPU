#ifndef _POSTPROCESSINGWRITER_HH_
#define _POSTPROCESSINGWRITER_HH_


#include "Basic.hh"
#include "RigidBody.hh"
#include "Transform3.hh"
#include "Kinematics.hh"
#include "GrainsMisc.hh"
#include "ReaderXML.hh"


// PostProcessingWriter types
enum PostProcessingWriterType {
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
		/** @brief Initializes the post-processing writer */
		__HOST__
		virtual void PostProcessing_start() = 0;

		/** @brief Writes data */
		__HOST__
		virtual void PostProcessing( 
								std::vector<RigidBody<T, T>> const* rb,
								std::vector<unsigned int> const* rigidBodyID,
								std::vector<Transform3<T>> const* t,
								std::vector<Kinematics<T>> const* k,
								T currentTime ) = 0;

		/** @brief Finalizes writing data */
		__HOST__
		virtual void PostProcessing_end() = 0;
		//@}
};

#endif
  
