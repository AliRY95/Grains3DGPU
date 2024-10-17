#ifndef _RAWDATAPOSTPROCESSINGWRITER_HH_
#define _RAWDATAPOSTPROCESSINGWRITER_HH_


#include <iostream>
#include <fstream>
#include "PostProcessingWriter.hh"


// =============================================================================
/** @brief The class RawPostProcessingWriter.

    Writes particle data in raw format for post-processing.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class RawDataPostProcessingWriter : public PostProcessingWriter<T>
{
	private:
		/**@name Parameters */
		//@{
		/** \brief center of mass x-coordinate output stream */
		std::ofstream m_gc_coordinates_x;
		/** \brief center of mass y-coordinate output stream */
		std::ofstream m_gc_coordinates_y;
		/** \brief center of mass z-coordinate output stream */
		std::ofstream m_gc_coordinates_z;
		/** \brief x-component of translational velocity along output stream */
		std::ofstream m_translational_velocity_x;
		/** \brief y-component of translational velocity along output stream */
		std::ofstream m_translational_velocity_y;
		/** \brief z-component of translational velocity along output stream */
		std::ofstream m_translational_velocity_z;
		/** \brief x-component of angular velocity along output stream */
		std::ofstream m_angular_velocity_x;
		/** \brief y-component of angular velocity along output stream */
		std::ofstream m_angular_velocity_y;
		/** \brief z-component of angular velocity along output stream */
		std::ofstream m_angular_velocity_z;
		/** \brief coordination number output stream */
		std::ofstream m_coordination_number;
		/** \brief particle class output stream */
		std::ofstream m_particle_class;
		/** \brief files root name */
		std::string m_filerootname;
		/** \brief No. digits after the decimal in the scientific format */
		int m_ndigits;
		//@}


	public:
		/** @name Constructors */
		//@{
		/** @brief Default constructor */
		__HOST__
		RawDataPostProcessingWriter();

		/** @brief Constructor with XML node
		@param dn XML node */
		__HOST__
		RawDataPostProcessingWriter( DOMNode* dn );

		/** @brief Destructor */
		__HOST__
		~RawDataPostProcessingWriter();
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
		void PostProcessing_start();

		/** @brief Writes data TODO:PARAMS*/
		__HOST__
		void PostProcessing( RigidBody<T, T> const* const* particleRB,
							 RigidBody<T, T> const* const* obstacleRB,
							 ComponentManager<T> const* cm,
							 T currentTime );

		/** @brief Finalizes writing data */
		__HOST__
		void PostProcessing_end();

		/** @brief Creates output files and open streams */
		__HOST__
		void prepareResultFiles( ios_base::openmode mode );
		//@}
    //@}
};

#endif
  
