#ifndef _POSTPROCESSINGWRITER_HH_
#define _POSTPROCESSINGWRITER_HH_

#include "Basic.hh"
#include <list>
#include <string>
using std::list;
using std::string;
#include "ReaderXML.hh"
#include "LinkedCell.hh"

class Particle;
class Obstacle;
class Component;


// =============================================================================
/** @brief The class PostProcessingWriter.

    Writes results in files for post-processing by an external software.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typedef T>
class PostProcessingWriter
{
	protected:
		/** @name Constructors */
		//@{
		/** @brief Default constructor */
		__HOST__
		PostProcessingWriter();

		/** @brief Constructor with XML node, rank and number of processes 
		as input parameters
		@param dn XML node
		@param rank_ process rank 
		@param nbranks_ number of processes */
		__HOST__
		virtual PostProcessingWriter( DOMNode* root ) = 0;
		//@}

	public:
		/** @name Constructors */
		//@{
		/** @brief Destructor */
		virtual ~PostProcessingWriter();
		//@}


		/** @name Methods */
		//@{
		/** @brief Initializes the post-processing writer
		@param time physical time
		@param dt time step magnitude
		@param particles active particles
		@param inactiveparticles inactive particles
		@param periodic_clones periodic particles
		@param referenceParticles reference particles
		@param obstacle obstacles 
		@param LC linked-cell grid
		@param insert_windows insertion windows */
		virtual void PostProcessing_start( double const& time,
		double const& dt,
		list<Particle*> const* particles,
		list<Particle*> const* inactiveparticles,
		list<Particle*> const* periodic_clones,	
		vector<Particle*> const* referenceParticles,
		Obstacle* obstacle,
		LinkedCell const* LC,
		vector<Window> const& insert_windows ) = 0;

		/** @brief Writes data
		@param time physical time
		@param dt time step magnitude
		@param particles active particles
		@param inactiveparticles inactive particles
		@param periodic_clones periodic particles
		@param referenceParticles reference particles
		@param obstacle obstacles 
		@param LC linked-cell grid */
		virtual void PostProcessing( double const& time,
		double const& dt,
		list<Particle*> const* particles,
		list<Particle*> const* inactiveparticles,
		list<Particle*> const* periodic_clones,		
		vector<Particle*> const* referenceParticles,
		Obstacle* obstacle,
		LinkedCell const* LC ) = 0;

		/** @brief Finalizes writing data */
		virtual void PostProcessing_end() = 0;
		//@}
};

#endif
  
