#include "RawDataPostProcessingWriter.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Delete all raw result files
__HOST__
void clearResultFiles()
{
	// std::string cmd = "bash " + 
	// 		   		  GrainsExecParameters<T>::m_GRAINS_HOME +
	// 				  "/Tools/ExecScripts/Text_clear.exec " + 
	// 				  m_filerootname;
	// GrainsExec::m_return_syscmd = system( cmd.c_str() );
}




/* ========================================================================== */
/*                             High-Level Methods                             */
/* ========================================================================== */
// Default constructor
template <typename T>
__HOST__
RawDataPostProcessingWriter<T>::RawDataPostProcessingWriter()
{}




// -----------------------------------------------------------------------------
// Constructor with XML node
template <typename T>
__HOST__
RawDataPostProcessingWriter<T>::RawDataPostProcessingWriter( DOMNode* dn )
: m_ndigits( 6 )  
{ 
	m_filerootname = ReaderXML::getNodeAttr_String( dn, "Name" );
	cout << shiftString9 << "Type = RawData" << endl;
	cout << shiftString12 << "Output file name = " 
		 << m_filerootname << endl;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
RawDataPostProcessingWriter<T>::~RawDataPostProcessingWriter()
{}




// -----------------------------------------------------------------------------
// Gets the post-processing writer type
template <typename T>
__HOST__
PostProcessingWriterType 
RawDataPostProcessingWriter<T>::getPostProcessingWriterType() const
{
  return ( RAW );
}




// -----------------------------------------------------------------------------
// Initializes the post-processing writer
template <typename T>
__HOST__
void RawDataPostProcessingWriter<T>::PostProcessing_start()
{
	// Open files
	ios_base::openmode mode = ios::app;
	mode = ios::out;
	// clearResultFiles();
	prepareResultFiles( mode );
}




// -----------------------------------------------------------------------------
// Writes data -- Particles come first, followed by obtacles
template <typename T>
__HOST__
void RawDataPostProcessingWriter<T>::PostProcessing( 
								RigidBody<T, T> const* const* particleRB,
								RigidBody<T, T> const* const* obstacleRB,
								ComponentManager<T> const* cm,
								T currentTime )
{
	// Particles
	unsigned int numParticles = cm->getNumberOfParticles();
	std::vector<unsigned int> rbParticle = cm->getRigidBodyId();
	std::vector<Transform3<T>> tParticle = cm->getTransform();
	std::vector<Kinematics<T>> kParticle = cm->getVelocity();
	// Obstacles
	unsigned int numObstacles = cm->getNumberOfObstacles();
	std::vector<unsigned int> rbObstacle = cm->getRigidBodyIdObstacles();
	std::vector<Transform3<T>> tObstacle = cm->getTransformObstacles();
	// TODO:
	std::vector<Kinematics<T>> kObstacle( numObstacles );
    // Aux. variables
	Vector3<T> centre;
	Vector3<T> velT;
	Vector3<T> velR;
	unsigned int type;
	m_particle_class.open( ( m_filerootname + "_particleType.dat" ).c_str(), 
							ios::out );


	// Writing current time at the beginning of each line
	std::string stime = GrainsMisc<T>::realToString( 
											ios::scientific, 6, currentTime );
	m_gc_coordinates_x << stime;
	m_gc_coordinates_y << stime;
	m_gc_coordinates_z << stime;
	m_translational_velocity_x << stime;
	m_translational_velocity_y << stime;
	m_translational_velocity_z << stime;
	m_angular_velocity_x << stime;
	m_angular_velocity_y << stime;
	m_angular_velocity_z << stime;

  
	// Writing particles data
	for ( size_t i = 0; i < numParticles; i++ )
	{
    	// Center of mass position
        centre = tParticle[i].getOrigin();
        m_gc_coordinates_x << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, centre[X] );
        m_gc_coordinates_y << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, centre[Y] );
        m_gc_coordinates_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, centre[Z] );

        // Translational velocity
        velT = kParticle[i].getTranslationalComponent();
        m_translational_velocity_x << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, velT[X] );
        m_translational_velocity_y << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, velT[Y] );
        m_translational_velocity_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velT[Z] );
    
        // Angular velocity
        velR = kParticle[i].getAngularComponent();
        m_angular_velocity_x << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[X] );
        m_angular_velocity_y << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[Y] );
        m_angular_velocity_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[Z] );
    
        // // Number of contacts
        // m_coordination_number << " " << pp->getCoordinationNumber();
	
		// Particle type
		type = particleRB[ rbParticle[i] ]->getConvex()->getConvexType();
		// m_particle_class << type << " " ;	 
	}


	// Writing obstacles data
	for ( size_t i = 0; i < numObstacles; i++ )
	{
    	// Center of mass position
        centre = tObstacle[i].getOrigin();
        m_gc_coordinates_x << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, centre[X] );
        m_gc_coordinates_y << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, centre[Y] );
        m_gc_coordinates_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, centre[Z] );

        // Translational velocity
        velT = kObstacle[i].getTranslationalComponent();
        m_translational_velocity_x << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, velT[X] );
        m_translational_velocity_y << " " << GrainsMisc<T>::realToString( 
										ios::scientific, m_ndigits, velT[Y] );
        m_translational_velocity_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velT[Z] );
    
        // Angular velocity
        velR = kObstacle[i].getAngularComponent();
        m_angular_velocity_x << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[X] );
        m_angular_velocity_y << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[Y] );
        m_angular_velocity_z << " " << GrainsMisc<T>::realToString(
										ios::scientific, m_ndigits, velR[Z] );
    
        // // Number of contacts
        // m_coordination_number << " " << pp->getCoordinationNumber();
	
		// Particle type
		type = obstacleRB[ rbObstacle[i] ]->getConvex()->getConvexType();
		// m_particle_class << type << " " ;
	}  
    

	// Closing
    m_gc_coordinates_x << endl;
    m_gc_coordinates_y << endl;  
    m_gc_coordinates_z << endl; 
    m_translational_velocity_x << endl;
    m_translational_velocity_y << endl;   
    m_translational_velocity_z << endl;
    m_angular_velocity_x << endl;
    m_angular_velocity_y << endl;   
    m_angular_velocity_z << endl;  
    m_coordination_number << endl;
    m_particle_class << endl;
	m_particle_class.close();
}




// -----------------------------------------------------------------------------
// Finalizes writing data
template <typename T>
__HOST__
void RawDataPostProcessingWriter<T>::PostProcessing_end()
{
    m_gc_coordinates_x.close();
    m_gc_coordinates_y.close();  
    m_gc_coordinates_z.close(); 
    m_translational_velocity_x.close();
    m_translational_velocity_y.close();   
    m_translational_velocity_z.close();
    m_angular_velocity_x.close();
    m_angular_velocity_y.close();   
    m_angular_velocity_z.close();
    m_coordination_number.close();
    m_particle_class.close();
}




// -----------------------------------------------------------------------------
// Creates output files and open streams
template <typename T>
__HOST__
void 
RawDataPostProcessingWriter<T>::prepareResultFiles( ios_base::openmode mode )
{
	string file;
	file = m_filerootname+"_position_x.dat";
	m_gc_coordinates_x.open( file.c_str(), mode );
	file = m_filerootname+"_position_y.dat";
	m_gc_coordinates_y.open( file.c_str(), mode );
	file = m_filerootname+"_position_z.dat";
	m_gc_coordinates_z.open( file.c_str(), mode );

	file = m_filerootname+"_translational_velocity_x.dat";
	m_translational_velocity_x.open( file.c_str(), mode );
	file = m_filerootname+"_translational_velocity_y.dat";
	m_translational_velocity_y.open( file.c_str(), mode );
	file = m_filerootname+"_translational_velocity_z.dat";
	m_translational_velocity_z.open( file.c_str(), mode );

	file = m_filerootname+"_angular_velocity_x.dat";
	m_angular_velocity_x.open( file.c_str(), mode );
	file = m_filerootname+"_angular_velocity_y.dat";
	m_angular_velocity_y.open( file.c_str(), mode );
	file = m_filerootname+"_angular_velocity_z.dat";
	m_angular_velocity_z.open( file.c_str(), mode );
	
	file = m_filerootname+"_coordinationNumber.dat";
	m_coordination_number.open( file.c_str(), mode );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class RawDataPostProcessingWriter<float>;
template class RawDataPostProcessingWriter<double>;