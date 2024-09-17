#include "RawDataPostProcessingWriter.hh"


// -----------------------------------------------------------------------------
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
: m_binary( false )
, m_ndigits( 6 )  
{ 
	m_filerootname = ReaderXML::getNodeAttr_String( dn, "Name" );
	if ( ReaderXML::hasNodeAttr( dn, "WritingMode" ) )
	{ 
		string sm_binary = ReaderXML::getNodeAttr_String( dn, "WritingMode" );
		if ( sm_binary == "Binary" ) 
			m_binary = true;
	}  
	
	cout << shiftString9 << "Type = RawData" << endl;
	cout << shiftString12 << "Output file name = " 
		 << m_filerootname << endl;
	cout << shiftString12 << "Writing mode = " 
		 << ( m_binary ? "Binary" : "Text" ) << endl;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
RawDataPostProcessingWriter<T>::~RawDataPostProcessingWriter()
{}




// -----------------------------------------------------------------------------
// Initializes the post-processing writer
template <typename T>
__HOST__
void RawDataPostProcessingWriter<T>::PostProcessing_start()
{
	// Open files
	ios_base::openmode mode = ios::app;
	if ( GrainsExec::m_ReloadType == "new" ) 
	{
	mode = ios::out;
	clearResultFiles();
	}
	prepareResultFiles( mode );

	// Write data
	PostProcessing( time, dt, particles, inactiveparticles, periodic_clones,
		referenceParticles, obstacle, LC );
}




// ----------------------------------------------------------------------------
// Writes data
template <typename T>
__HOST__
void RawDataPostProcessingWriter::PostProcessing( double const& time, 
    double const& dt,
    list<Particle*> const* particles,
    list<Particle*> const* inactiveparticles,
    list<Particle*> const* periodic_clones,
    vector<Particle*> const* referenceParticles,
    Obstacle* obstacle,
    LinkedCell const* LC )
{
  GrainsMPIWrapper const* wrapper = GrainsExec::getComm() ;
  size_t nb_total_part = GrainsExec::getTotalNumberPhysicalParticles() ;

  // Particle data and type
  // In parallel mode
  if ( wrapper )
  {
    // Gather particles class from every proc on master proc
    vector<int>* types_Global = NULL;
    types_Global = wrapper->GatherParticlesClass_PostProcessing( *particles,
            nb_total_part );

    // Gather particle data ordered by particle ID on the master proc    
    vector< vector<double> >* data_Global = NULL;
    data_Global = wrapper->GatherParticleData_PostProcessing( *particles,
        nb_total_part );

    // Write particle data
    if ( m_rank == 0 )
      one_output_MPI( time, nb_total_part, types_Global, data_Global ) ;

    if ( data_Global ) delete data_Global ;
    if ( types_Global ) delete types_Global ;    
  }
  // In serial mode
  else
    one_output_Standard( time, nb_total_part, particles );
}




// ----------------------------------------------------------------------------
// Finalizes writing data
void RawDataPostProcessingWriter::PostProcessing_end()
{
  if ( m_rank == 0 )
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
}




// ----------------------------------------------------------------------------
// Writes data in parallel mode at one physical time
void RawDataPostProcessingWriter::one_output_MPI( double const& time, 
    size_t const& nb_total_part, vector<int>* types_Global, 
    vector< vector<double> > const* data_Global )
{
  m_particle_class.open( ( m_filerootname + "_particleType.dat" ).c_str(), 
  	ios::out | ( m_binary ? ios::binary : ios::out ) );

  if ( m_binary )
  {
    double tt = time;
    m_gc_coordinates_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_gc_coordinates_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_gc_coordinates_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );					  
  }
  else
  {
    string stime = GrainsExec::doubleToString( ios::scientific, 6, time );
    m_gc_coordinates_x << stime;
    m_gc_coordinates_y << stime;
    m_gc_coordinates_z << stime;
    m_translational_velocity_x << stime;
    m_translational_velocity_y << stime;
    m_translational_velocity_z << stime;
    m_angular_velocity_x << stime;
    m_angular_velocity_y << stime;
    m_angular_velocity_z << stime;
    m_angular_velocity_z << stime;
  }

  // Write in files: inactive particles are assigned 0 values and values are
  // written in increasing order of particle ID from 1 to nb_total_part (but
  // from 0 to nb_total_part - 1 in the data_Global vector
  if ( m_binary )
  {
    int coord = 0, type = 0;
    double val = 0.;
    
    for (size_t i=0; i<nb_total_part; i++)
    {
      // Center of mass position
      val = (*data_Global)[i][0];
      m_gc_coordinates_x.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) ); 
      val = (*data_Global)[i][1];	
      m_gc_coordinates_y.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );
      val = (*data_Global)[i][2];       
      m_gc_coordinates_z.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) ); 

      // Translational velocity
      val = (*data_Global)[i][3];      
      m_translational_velocity_x.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );
      val = (*data_Global)[i][4];      
      m_translational_velocity_y.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );	
      val = (*data_Global)[i][5];      
      m_translational_velocity_z.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );	 
    
      // Angular velocity
      val = (*data_Global)[i][6];       
      m_angular_velocity_x.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );
      val = (*data_Global)[i][7];       
      m_angular_velocity_y.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );	
      val = (*data_Global)[i][8];       
      m_angular_velocity_z.write( reinterpret_cast<char*>( &val ), 
      	sizeof(double) );	

      // Number of contacts
      coord = int((*data_Global)[i][9]) ;
      m_coordination_number.write( reinterpret_cast<char*>( 
      	&coord ), sizeof(int) );
      
      // Particle type          
      type = (*types_Global)[i];
      m_particle_class.write( reinterpret_cast<char*>( &type ), 
	  	sizeof(int) );	
    }  
  }
  else
  { 
    for (size_t i=0; i<nb_total_part; i++)
    {
      // Center of mass position
      m_gc_coordinates_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][0] ) ;
      m_gc_coordinates_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][1] ) ;
      m_gc_coordinates_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][2] ) ;

      // Translational velocity
      m_translational_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][3] ) ;
      m_translational_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][4] ) ;
      m_translational_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][5] ) ;
    
      // Angular velocity
      m_angular_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][6] ) ;
      m_angular_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][7] ) ;
      m_angular_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*data_Global)[i][8] ) ;

      // Number of contacts
      m_coordination_number << " " << int((*data_Global)[i][9] ) ;
      
      // Particle type  
      m_particle_class << (*types_Global)[i] << " " ;             
    }
  }
  
  if ( !m_binary )
  {
    m_gc_coordinates_x << endl ;
    m_gc_coordinates_y << endl ;
    m_gc_coordinates_z << endl ;
    m_translational_velocity_x << endl ;
    m_translational_velocity_y << endl ;
    m_translational_velocity_z << endl ;
    m_angular_velocity_x << endl ;
    m_angular_velocity_y << endl ;
    m_angular_velocity_z << endl ;
    m_coordination_number << endl ;
    m_particle_class << endl ;    
  }  
  
  m_particle_class.close();
}




// ----------------------------------------------------------------------------
// Delete all result files
void RawDataPostProcessingWriter::clearResultFiles() const
{
  if ( m_rank == 0 ) 
  {
    string cmd = "bash " + GrainsExec::m_GRAINS_HOME 
        + "/Tools/ExecScripts/Text_clear.exec " + m_filerootname;
    GrainsExec::m_return_syscmd = system( cmd.c_str() );
  }
}




// ----------------------------------------------------------------------------
// Creates output files and open streams
void RawDataPostProcessingWriter::prepareResultFiles( ios_base::openmode mode )
{
  string file;
  file = m_filerootname+"_position_x.dat";
  m_gc_coordinates_x.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_position_y.dat";
  m_gc_coordinates_y.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_position_z.dat";
  m_gc_coordinates_z.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );

  file = m_filerootname+"_translational_velocity_x.dat";
  m_translational_velocity_x.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_translational_velocity_y.dat";
  m_translational_velocity_y.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_translational_velocity_z.dat";
  m_translational_velocity_z.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) ); 

  file = m_filerootname+"_angular_velocity_x.dat";
  m_angular_velocity_x.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_angular_velocity_y.dat";
  m_angular_velocity_y.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  file = m_filerootname+"_angular_velocity_z.dat";
  m_angular_velocity_z.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
  
  file = m_filerootname+"_coordinationNumber.dat";
  m_coordination_number.open( file.c_str(), mode | ( m_binary ? 
  	ios::binary : mode ) );
}




// ----------------------------------------------------------------------------
// Writes data in serial mode at one physical time
void RawDataPostProcessingWriter::one_output_Standard( double const& time,
    size_t const& nb_total_part, list<Particle*> const* particles )
{ 
  Point3* centre = NULL;
  Vector3* velT = NULL; 
  Vector3* velR = NULL; 
  double zero = 0., tt = time; 
  int coord = 0, izero = 0, type = 0;
  m_particle_class.open( ( m_filerootname + "_particleType.dat" ).c_str(), 
  	ios::out | ( m_binary ? ios::binary : ios::out ) );

  if ( m_binary )
  {
    m_gc_coordinates_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_gc_coordinates_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_gc_coordinates_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_translational_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_x.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_y.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );
    m_angular_velocity_z.write( reinterpret_cast<char*>( &tt ), 
    	sizeof(double) );					  
  }
  else
  {
    string stime = GrainsExec::doubleToString( ios::scientific, 6, time );
    m_gc_coordinates_x << stime;
    m_gc_coordinates_y << stime;
    m_gc_coordinates_z << stime;
    m_translational_velocity_x << stime;
    m_translational_velocity_y << stime;
    m_translational_velocity_z << stime;
    m_angular_velocity_x << stime;
    m_angular_velocity_y << stime;
    m_angular_velocity_z << stime;
    m_angular_velocity_z << stime;
  }
  
  // Extract the active particles that are not periodic clones
  // and create a map part ID - particle pointer such that we can write data
  // in increasing order of particle ID from 1 to nb_total_part
  map<size_t,Particle*> IDtoPart;
  list<Particle*>::const_iterator il;
  Particle* pp = NULL;
  for (il=particles->begin(); il!=particles->end();il++)
    if ( (*il)->getTag() != 2 )
       IDtoPart.insert( pair<int,Particle*>( size_t((*il)->getID()), *il ) ); 
       
  // Write in files: inactive particles are assigned 0 values and -1 type 
  // and values are written in increasing order of particle ID from 1 to 
  // nb_total_part
  for (size_t i=1;i<nb_total_part+1;i++)
    if ( IDtoPart.count( i ) )
    {
      pp = IDtoPart[i];
      type = pp->getGeometricType();
      
      if ( m_binary )
      {
        // Center of mass position
        centre = const_cast<Point3*>(pp->getPosition());
	m_gc_coordinates_x.write( reinterpret_cast<char*>( &(*centre)[X] ), 
    		sizeof(double) );
	m_gc_coordinates_y.write( reinterpret_cast<char*>( &(*centre)[Y] ), 
    		sizeof(double) );
	m_gc_coordinates_z.write( reinterpret_cast<char*>( &(*centre)[Z] ), 
    		sizeof(double) );
		
        // Translational velocity
        velT = const_cast<Vector3*>(pp->getTranslationalVelocity());
        m_translational_velocity_x.write( reinterpret_cast<char*>( 
		&(*velT)[X] ), sizeof(double) );
        m_translational_velocity_y.write( reinterpret_cast<char*>( 
		&(*velT)[Y] ), sizeof(double) );
        m_translational_velocity_z.write( reinterpret_cast<char*>( 
		&(*velT)[Z] ), sizeof(double) );
		    
        // Angular velocity
        velR = const_cast<Vector3*>(pp->getAngularVelocity());
        m_angular_velocity_x.write( reinterpret_cast<char*>( 
		&(*velR)[X] ), sizeof(double) );
        m_angular_velocity_y.write( reinterpret_cast<char*>( 
		&(*velR)[Y] ), sizeof(double) );
        m_angular_velocity_z.write( reinterpret_cast<char*>( 
		&(*velR)[Z] ), sizeof(double) );
    
        // Number of contacts
        coord = pp->getCoordinationNumber(); 
	m_coordination_number.write( reinterpret_cast<char*>( 
		&coord ), sizeof(int) );
		
	// Particle type
	m_particle_class.write( reinterpret_cast<char*>( &type ), sizeof(int) );
      }
      else
      {
        // Center of mass position
        centre = const_cast<Point3*>(pp->getPosition());
        m_gc_coordinates_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*centre)[X] ) ;
        m_gc_coordinates_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*centre)[Y] ) ;
        m_gc_coordinates_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*centre)[Z] ) ;

        // Translational velocity
        velT = const_cast<Vector3*>(pp->getTranslationalVelocity());
        m_translational_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velT)[X] ) ;
        m_translational_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velT)[Y] ) ;
        m_translational_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velT)[Z] ) ;
    
        // Angular velocity
        velR = const_cast<Vector3*>(pp->getAngularVelocity());
        m_angular_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velR)[X] ) ;
        m_angular_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velR)[Y] ) ;
        m_angular_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, (*velR)[Z] ) ;
    
        // Number of contacts
        m_coordination_number << " " << pp->getCoordinationNumber();
	
	// Particle type        
	m_particle_class << type << " " ;	 
      }         
    }
    else
    {
      type = - 1;
      
      if ( m_binary )
      {
        // Center of mass position
	m_gc_coordinates_x.write( reinterpret_cast<char*>( 
		&GrainsExec::m_defaultInactivePos[X] ), 
    		sizeof(double) );
	m_gc_coordinates_y.write( reinterpret_cast<char*>( 
		&GrainsExec::m_defaultInactivePos[Y] ), 
    		sizeof(double) );
	m_gc_coordinates_z.write( reinterpret_cast<char*>( &
		GrainsExec::m_defaultInactivePos[Z] ), 
    		sizeof(double) );
		
        // Translational velocity
        m_translational_velocity_x.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
        m_translational_velocity_y.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
        m_translational_velocity_z.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
		    
        // Angular velocity
        m_angular_velocity_x.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
        m_angular_velocity_y.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
        m_angular_velocity_z.write( reinterpret_cast<char*>( 
		&zero ), sizeof(double) );
    
        // Number of contacts
	m_coordination_number.write( reinterpret_cast<char*>( 
		&izero ), sizeof(int) );
		
	// Particle type
	m_particle_class.write( reinterpret_cast<char*>( &type ), sizeof(int) );
      }
      else
      {      
        // Center of mass position
        m_gc_coordinates_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, GrainsExec::m_defaultInactivePos[X] ) ;
        m_gc_coordinates_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, GrainsExec::m_defaultInactivePos[Y] ) ;
        m_gc_coordinates_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, GrainsExec::m_defaultInactivePos[Z] ) ;

        // Translational velocity
        m_translational_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
        m_translational_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
        m_translational_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
    
        // Angular velocity
        m_angular_velocity_x << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
        m_angular_velocity_y << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
        m_angular_velocity_z << " " << GrainsExec::doubleToString( 
    	ios::scientific, m_ndigits, zero ) ;
    
        // Number of contacts
        m_coordination_number << " 0";
	
	// Particle type        
	m_particle_class << type << " " ;	
      }    
    }   
  
  if ( !m_binary )
  {
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
    m_particle_class << endl ;
  }
  
  m_particle_class.close();
}




// ----------------------------------------------------------------------------
// Gets the post-processing writer type
string RawDataPostProcessingWriter::getPostProcessingWriterType() const
{
  return ( "RawData" );
}




// ----------------------------------------------------------------------------
// Writes particle type file */
void RawDataPostProcessingWriter::writeParticleTypeFile( 
	size_t const& nb_total_part, list<Particle*> const* particles )
{
  GrainsMPIWrapper const* wrapper = GrainsExec::getComm() ;
  string file = m_filerootname+"_particleType.dat";
  int type = 0;
  if ( m_rank == 0 ) m_particle_class.open( file.c_str(), ios::out | 
  	( m_binary ? ios::binary : ios::out ) );

  // In parallel mode
  if ( wrapper )
  {
    vector<int>* types_Global = NULL;
    
    // Gather particles class from every proc on master proc
    types_Global = 
        wrapper->GatherParticlesClass_PostProcessing( *particles,
            nb_total_part );

    // Write down particles class only once at the begining
    if ( m_rank == 0 )
    {
      if ( m_binary )
      { 
        for (size_t i=0; i<nb_total_part; i++)
	{
	  type = (*types_Global)[i];
	  m_particle_class.write( reinterpret_cast<char*>( &type ), 
	  	sizeof(int) );
	}      
      }
      else
      {     
        for (size_t i=0; i<nb_total_part; i++)
          m_particle_class << (*types_Global)[i] << " " ;
        m_particle_class << endl ;
      }
    }
    if ( types_Global ) delete types_Global ;
  }
  // In serial mode
  else
  {  
    // Extract the active particles that are not periodic clones
    // and create a map part ID - particle pointer such that we can write data
    // in increasing order of particle ID from 0 to nb_total_part-1
    map<size_t,Particle*> IDtoPart;
    list<Particle*>::const_iterator il;
    for (il=particles->cbegin(); il!=particles->cend();il++)
      if ( (*il)->getTag() != 2 )
         IDtoPart.insert( pair<int,Particle*>( size_t((*il)->getID()), *il ) ); 
       
    // Write in files: inactive particles are assigned -1 type and values are
    // written in increasing order of particle ID from 0 to nb_total_part-1
    for (size_t i=0;i<nb_total_part;i++)
    {
      if ( IDtoPart.count( i ) ) type = IDtoPart[i]->getGeometricType();
      else type = -1;
	
      if ( m_binary )
        m_particle_class.write( reinterpret_cast<char*>( &type ), sizeof(int) );
      else
        m_particle_class << type << " " ;
    }
    if ( !m_binary ) m_particle_class << endl ;     
  }
  
  if ( m_rank == 0 ) m_particle_class.close();  
}
