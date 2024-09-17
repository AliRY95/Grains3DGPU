#ifndef _RAWDATAPOSTPROCESSINGWRITER_HH_
#define _RAWDATAPOSTPROCESSINGWRITER_HH_


#include "PostProcessingWriter.hh"
#include <iostream>
#include <fstream>
using std::ofstream;


// =============================================================================
/** @brief The class RawPostProcessingWriter.

    Writes particle data in raw format for post-processing.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typedef T>
class RawDataPostProcessingWriter : public PostProcessingWriter<T>
{
	private:
		/**@name Parameters */
		//@{
		/** \brief center of mass x-coordinate output stream */
		ofstream m_gc_coordinates_x;
		/** \brief center of mass y-coordinate output stream */
		ofstream m_gc_coordinates_y;
		/** \brief center of mass z-coordinate output stream */
		ofstream m_gc_coordinates_z;
		/** \brief x-component of translational velocity along output stream */
		ofstream m_translational_velocity_x;
		/** \brief y-component of translational velocity along output stream */
		ofstream m_translational_velocity_y;
		/** \brief z-component of translational velocity along output stream */
		ofstream m_translational_velocity_z;
		/** \brief x-component of angular velocity along output stream */
		ofstream m_angular_velocity_x;
		/** \brief y-component of angular velocity along output stream */
		ofstream m_angular_velocity_y;
		/** \brief z-component of angular velocity along output stream */
		ofstream m_angular_velocity_z;
		ofstream m_coordination_number; /**< coordination number output stream */
		ofstream m_particle_class; /**< particle class output stream */
		/** \brief files root name */
		string m_filerootname;
		/** \brief whether to write data in binary */
		bool m_binary;
		/** \brief No. digits after the decimal in the scientific format */
		int m_ndigits;


	public:
		/** @name Constructors */
		//@{
		/** @brief Constructor with XML node
		@param dn XML node */
		RawDataPostProcessingWriter( DOMNode* dn );

		/** @brief Destructor */
		~RawDataPostProcessingWriter();
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
    
    
  private:
    /** @name Methods */
    //@{
    /** @brief Writes data in parallel mode at one physical time
    @param time physical time
    @param nb_total_part total number of particles
    @param types_Global vector containing particle type
    @param data_Global vector containing particle data  */
    void one_output_MPI( double const& time, size_t const& nb_total_part,
  	vector<int>* types_Global,
	vector< vector<double> > const* data_Global );
	
    /** @brief Writes data in serial mode at one physical time
    @param time physical time
    @param nb_total_part total number of particles     
    @param particles active particles */
    void one_output_Standard( double const& time, size_t const& nb_total_part,
  	list<Particle*> const* particles );
	
    /** @brief Delete all result files */
    void clearResultFiles() const ; 	  
  
    /** @brief Creates output files and open streams
    @param mode File opening mode (here : ios::app) */
    void prepareResultFiles( ios_base::openmode mode ) ;
        
    /** @brief Writes particle type file 
    @param nb_total_part total number of particles     
    @param particles active particles */
    void writeParticleTypeFile( size_t const& nb_total_part,
  	list<Particle*> const* particles ) ;    
    //@}   
};

#endif
  
