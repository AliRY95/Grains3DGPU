#include "ParaviewPostProcessingWriter.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Writes particles data
template <typename T>
__HOST__
void writeParticles_Paraview( std::vector<RigidBody<T, T>> const* rb,
							  std::vector<unsigned int> const* rigidBodyID,
							  std::vector<Transform3<T>> const* t,
							  std::vector<Kinematics<T>> const* k )
{
	list<Particle*>::const_iterator particle;
	unsigned int numParticles = rigidBodyID->size();

	std::ofstream f( ( m_ParaviewFilename_dir + 
						"/" + 
						partFilename ).c_str(), ios::out );
	
	f << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" "
	  << "byte_order=\"LittleEndian\" ";
	if ( m_binary ) 
		f << "compressor=\"vtkZLibDataCompressor\"";
	f << ">" << endl;
	f << "<UnstructuredGrid>" << endl;
	int nbpts = 0, nbcells = 0, i;

	for ( unsigned int i = 0; i < numParticles; i++ )
	{
		nbpts += rb[i].getConvex()->numberOfPoints_PARAVIEW();
		nbcells += rb[i].getConvex()->numberOfCells_PARAVIEW();
	}
	f << "<Piece NumberOfPoints=\"" << nbpts << "\""
	  << " NumberOfCells=\"" << nbcells << "\">" << endl;

	f << "<Points>" << endl;
	f << "<DataArray type=\"Float32\" NumberOfComponents=\"3\" ";
	f << "offset=\"" << OFFSET << "\" format=\"appended\">"; 
	start_output_binary( sizeof_Float32, 3*nbpts ) ;
	for ( unsigned int i = 0; i < numParticles; i++ )
	{
		ppp = (*particle)->get_polygonsPts_PARAVIEW( PPTranslation );
		for (ilpp=ppp.begin();ilpp!=ppp.end();ilpp++)
		for (int comp=0;comp<3;++comp)
		write_double_binary( (*ilpp)[comp] ) ;
	}
	flush_binary( f, "writeParticles_Paraview/Points" );
	f << "</DataArray>" << endl;
	f << "</Points>" << endl;

	list<int> connectivity, offsets, cellstype;
	list<int>::iterator ii;
	int firstpoint_globalnumber = 0, last_offset = 0;
	for (particle=particles->begin();particle!=particles->end();particle++)
		if ( (*particle)->getActivity() == COMPUTE &&
		( (*particle)->getTag() != 2 || forceForAllTag ) )    
		(*particle)->write_polygonsStr_PARAVIEW( connectivity,
			offsets, cellstype, firstpoint_globalnumber, last_offset );
	f << "<Cells>" << endl;
	f << "<DataArray type=\"Int32\" Name=\"connectivity\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">";
	else f << "format=\"ascii\">" << endl;
	if ( m_binary )
	{
		start_output_binary( sizeof_Int32, int(connectivity.size()) ) ;
		for (ii=connectivity.begin();ii!=connectivity.end();ii++)
		write_int_binary( *ii );
		flush_binary( f, "writeParticles_Paraview/connectivity" );
	}
	else 
	{ 
		for (ii=connectivity.begin();ii!=connectivity.end();ii++)
		f << *ii << " ";	
		f << endl; 
	}     
	f << "</DataArray>" << endl;
	f << "<DataArray type=\"Int32\" Name=\"offsets\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">";
	else f << "format=\"ascii\">" << endl;
	if ( m_binary )
	{
		start_output_binary( sizeof_Int32, int(offsets.size()) ) ;
		for (ii=offsets.begin();ii!=offsets.end();ii++)
		write_int_binary( *ii );
		flush_binary( f, "writeParticles_Paraview/offsets" );
	}
	else
	{  
		for (ii=offsets.begin();ii!=offsets.end();ii++)
		f << *ii << " ";	
		f << endl;
	} 
	f << "</DataArray>" << endl;
	f << "<DataArray type=\"Int32\" Name=\"types\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">";
	else f << "format=\"ascii\">" << endl;
	if ( m_binary )
	{
		start_output_binary( sizeof_Int32, int(cellstype.size()) ) ;
		for (ii=cellstype.begin();ii!=cellstype.end();ii++)
		write_int_binary( *ii );
		flush_binary( f, "writeParticles_Paraview/types" );
	}
	else 
	{ 
		for (ii=cellstype.begin();ii!=cellstype.end();ii++)
		f << *ii << " ";	
		f << endl;
	}
	f << "</DataArray>" << endl;
	f << "</Cells>" << endl;

	f << "<CellData Scalars=\"NormU,NormOm,CoordNumb\">" << endl;

	f << "<DataArray type=\"Float32\" Name=\"NormU\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">"; 
	else f << "format=\"ascii\">" << endl;
	if ( m_binary ) start_output_binary( sizeof_Float32, int(cellstype.size()) );
	for (particle=particles->begin();particle!=particles->end();particle++)
		if ( (*particle)->getActivity() == COMPUTE && 
		( (*particle)->getTag() != 2 || forceForAllTag ) )
		{
		double normU = Norm( *(*particle)->getTranslationalVelocity() );
		int nc = (*particle)->numberOfCells_PARAVIEW();
		if ( m_binary ) for (i=0;i<nc;++i) write_double_binary( normU );
		else for (i=0;i<nc;++i) f << normU << " ";
		}
	if ( m_binary ) flush_binary( f, 
		"writeParticles_Paraview/NormU" ); 
	else f << endl;
	f << "</DataArray>" << endl;      

	f << "<DataArray type=\"Float32\" Name=\"NormOm\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">"; 
	else f << "format=\"ascii\">" << endl;
	if ( m_binary ) start_output_binary( sizeof_Float32, int(cellstype.size()) );
	for (particle=particles->begin();particle!=particles->end();particle++)
		if ( (*particle)->getActivity() == COMPUTE && 
		( (*particle)->getTag() != 2 || forceForAllTag ) )
		{
		double normOm = Norm( *(*particle)->getAngularVelocity() );
		int nc = (*particle)->numberOfCells_PARAVIEW();
		if ( m_binary ) for (i=0;i<nc;++i) write_double_binary( normOm );
		else for (i=0;i<nc;++i) f << normOm << " ";
		}
	if( m_binary )
		flush_binary( f, "writeParticles_Paraview/NormOm" ); 
	else f << endl;
	f << "</DataArray>" << endl; 

	f << "<DataArray type=\"Float32\" Name=\"CoordNumb\" ";
	if ( m_binary ) f << "offset=\"" << OFFSET << "\" format=\"appended\">"; 
	else f << "format=\"ascii\">" << endl;
	if ( m_binary ) start_output_binary( sizeof_Float32, int(cellstype.size()) );
	for (particle=particles->begin();particle!=particles->end();particle++)
		if ( (*particle)->getActivity() == COMPUTE && 
		( (*particle)->getTag() != 2 || forceForAllTag ) )
		{
		double coordNum = double((*particle)->getCoordinationNumber());
		int nc = (*particle)->numberOfCells_PARAVIEW();
		if ( m_binary ) for (i=0;i<nc;++i) write_double_binary( coordNum );
		else for (i=0;i<nc;++i) f << coordNum << " ";
		}
	if ( m_binary ) flush_binary( f, 
		"writeParticles_Paraview/CoordNumb" ); 
	else f << endl;
	f << "</DataArray>" << endl;
	f << "</CellData>" << endl;
	f << "</Piece>" << endl;
	
	f << "</UnstructuredGrid>" << endl;
	if ( m_binary )
	{
		f << "<AppendedData encoding=\"raw\">" << endl << "    _" ;
		f.write( BUFFER, OFFSET ) ;
		delete [] BUFFER ; BUFFER = 0 ;
		ALLOCATED = 0 ;
		OFFSET = 0 ;
		f << endl << "</AppendedData>" << endl;    
	}  
	f << "</VTKFile>" << endl;	
	f.close();	    
}	




// -----------------------------------------------------------------------------
// Writes data at one physical time
template <typename T>
__HOST__
void one_output( std::vector<RigidBody<T, T>> const* rb,
				 std::vector<unsigned int> rigidBodyID,
				 std::vector<Transform3<T>> const* t,
				 std::vector<Kinematics<T>> const* k )
{
	// No. different rigid bodies for writing.
	size_t numRigidBodyTypes = m_pertype ? rb->size() : 1;

	list<string> Scalars;
	Scalars.push_back("NormU");
	Scalars.push_back("NormOm");
	Scalars.push_back("CoordNumb");
   
	// Particles
	if ( numRigidBodyTypes == 1 )
	{ 
		string partFilename = m_ParaviewFilename + "_Particles_T";
		*m_Paraview_saveParticles_pvd[0] 
			<< "<DataSet timestep=\"" 
			<< time 
			<< "\" " 
			<< "group=\"\" part=\"0\" file=\"" 
			<< partFilename 
			<< ".vtu\"/>" 
			<< endl;             
       
		ofstream g( ( m_ParaviewFilename_dir + 
					  "/" + 
					  m_ParaviewFilename +
					  "_Particles.pvd" ).c_str(), ios::out );
		g << m_Paraview_saveParticles_pvd[0]->str();
		g << "</Collection>" << endl;
		g << "</VTKFile>" << endl;
		g.close();
    }
    
    // VTU files
	writeParticles_Paraview( particles, partFilename 
	  + ( m_nprocs > 1 ? "_" + ossRK.str() : "" ) + ".vtu", false,
	  PostProcessingWriter::m_bPPWindow[m_rank] );
}




// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__
ParaviewPostProcessingWriter<T>::ParaviewPostProcessingWriter()
{}




// -----------------------------------------------------------------------------
// Constructor with XML node, rank and number of processes as input parameters
template <typename T>
__HOST__
ParaviewPostProcessingWriter<T>::ParaviewPostProcessingWriter( DOMNode* dn )
{
	m_ParaviewFilename = ReaderXML::getNodeAttr_String( dn, "RootName" );
	m_ParaviewFilename_dir = ReaderXML::getNodeAttr_String( dn, "Directory" );
  
	cout << shiftString9 << "Type = Paraview" << endl;
	cout << shiftString12 << "Output file root name = " 
		 << m_ParaviewFilename << endl;
	cout << shiftString12 << "Output file directory name = " 
		 << m_ParaviewFilename_dir << endl;
	cout << shiftString12 << "Writing mode = " 
		 << ( m_binary ? "Binary" : "Text" ) << endl;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__
ParaviewPostProcessingWriter<T>::~ParaviewPostProcessingWriter()
{}




// ----------------------------------------------------------------------------
// Gets the post-processing writer type
template <typename T>
__HOST__
PostProcessingWriterType 
ParaviewPostProcessingWriter<T>::getPostProcessingWriterType() const
{
  return ( PARAVIEW );
}




// -----------------------------------------------------------------------------
// Initializes the post-processing writer
template <typename T>
__HOST__
void ParaviewPostProcessingWriter<T>::PostProcessing_start( 
										std::vector<RigidBody<T, T>> const* rb )
{
	// No. different rigid bodies for writing.
	size_t numRigidBodyTypes = m_pertype ? rb->size() : 1;

	clearResultFiles();
    
	// Particles
	ostringstream *ossNULL = NULL;
	m_Paraview_saveParticles_pvd.reserve( numRigidBodyTypes );
	for ( size_t i = 0; i < numRigidBodyTypes; ++i )
        m_Paraview_saveParticles_pvd.push_back( ossNULL );
	for ( size_t i = 0; i < numRigidBodyTypes; ++i )
        m_Paraview_saveParticles_pvd[i] = new ostringstream;      
    
	for ( size_t i = 0; i < numRigidBodyTypes ; ++i )
	{
        *m_Paraview_saveParticles_pvd[i] 
			<< "<?xml version=\"1.0\"?>" 
			<< endl;
        *m_Paraview_saveParticles_pvd[i] 
			<< "<VTKFile type=\"Collection\" version=\"0.1\""
       		<< " byte_order=\"LittleEndian\"";
        
		if ( m_binary ) 
		{
			*m_Paraview_saveParticles_pvd[i] 
				<< " compressor=\"vtkZLibDataCompressor\"";
		}
			
        *m_Paraview_saveParticles_pvd[i] << ">" << endl;
        *m_Paraview_saveParticles_pvd[i] << "<Collection>" << endl;
	}
}




// -----------------------------------------------------------------------------
// Writes data
template <typename T>
__HOST__
void ParaviewPostProcessingWriter<T>::PostProcessing( 
										std::vector<RigidBody<T, T>> const* rb,
										std::vector<unsigned int> rigidBodyID,
										std::vector<Transform3<T>> const* t,
										std::vector<Kinematics<T>> const* k )
{
  one_output( time, dt, particles, periodic_clones, referenceParticles,
  	obstacle, LC );
}




// ----------------------------------------------------------------------------
// Finalizes writing data
template <typename T>
__HOST__
void ParaviewPostProcessingWriter<T>::PostProcessing_end()
{}




// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Writes a pvtu file
void ParaviewPostProcessingWriter::writePVTU_Paraview( string const& filename,
  	list<string> const* pointVector,
	list<string> const* pointScalar,
	list<string> const* cellScalar )
{
  list<string>::const_iterator il;

  ofstream f( ( m_ParaviewFilename_dir + "/" + filename + ".pvtu" ).c_str(),
  	ios::out );
  f << "<?xml version=\"1.0\"?>" << endl; 
  f << "<VTKFile type=\"PUnstructuredGrid\" version=\"0.1\" "
    	<< "byte_order=\"LittleEndian\" ";
  if ( m_binary ) f << "compressor=\"vtkZLibDataCompressor\"";
  f << ">" << endl;
  f << "<PUnstructuredGrid GhostLevel=\"0\">" << endl;  
  f << "<PPoints>" << endl;
  f << "<PDataArray NumberOfComponents=\"3\" type=\"Float32\" format=\"ascii\">"
  	<< endl;
  f << "</PDataArray>" << endl;
  f << "</PPoints>" << endl;  
  f << "<PCells>" << endl;
  f << "<PDataArray Name=\"connectivity\" type=\"Int32\" format=\"ascii\">"
  	<< endl;
  f << "</PDataArray>" << endl;
  f << "<PDataArray Name=\"offsets\" type=\"Int32\" format=\"ascii\">" << endl;
  f << "</PDataArray>" << endl;
  f << "<PDataArray Name=\"types\" type=\"Int32\" format=\"ascii\">" << endl;
  f << "</PDataArray>" << endl;
  f << "</PCells>" << endl;    
  if ( pointVector->size() || pointScalar->size() )
  {
    f << "<PPointData";
    if ( pointVector->size() )
    {
      il = pointVector->begin();
      f << " Vectors=\"" << *il;
      il++;
      for ( ;il!=pointVector->end();il++) f << "," << *il;
      f << "\"";      
    }
    if ( pointScalar->size() )
    {
      il = pointScalar->begin();
      f << " Scalars=\"" << *il;
      il++;
      for ( ;il!=pointScalar->end();il++) f << "," << *il;
      f << "\"";      
    }
    f << ">" << endl;
    for (il = pointVector->begin();il!=pointVector->end();il++)
    {    
      f << "<PDataArray Name=\"" << *il 
    	<< "\" NumberOfComponents=\"3\" type=\"Float32\""
    	<< " format=\"ascii\">" << endl;
      f << "</PDataArray>" << endl;
    }
    for (il = pointScalar->begin();il!=pointScalar->end();il++)
    {    
      f << "<PDataArray Name=\"" << *il 
    	<< "\" type=\"Float32\" format=\"ascii\">" << endl;
      f << "</PDataArray>" << endl;
    }    
    f << "</PPointData>" << endl;    
  }      
  if ( cellScalar->size() )
  {        
    f << "<PCellData Scalars=\"";
    il = cellScalar->begin();
    f << *il;
    il++;
    for ( ;il!=cellScalar->end();il++) f << "," << *il;
    f << "\">"; 
    for (il = cellScalar->begin();il!=cellScalar->end();il++)
    {     
      f << "<PDataArray Name=\"" << *il << "\" type=\"Float32\"" 
    	<< " format=\"ascii\">" << endl;
      f << "</PDataArray>" << endl;
    }
    f << "</PCellData>" << endl;  
  }  
  for (int i=0;i<m_nprocs;++i)
  {
    // Does this processor have to write down outputs ?
    if( PostProcessingWriter::m_bPPWindow[i] )
    {
      f << "<Piece Source=\"" << filename << "_" << i << ".vtu\">" << endl;
      f << "</Piece>" << endl; 
    }
  }
  f << "</PUnstructuredGrid>" << endl;
  f << "</VTKFile>" << endl;  
  f.close();  
}




// // ----------------------------------------------------------------------------
// // Delete all result files
// void ParaviewPostProcessingWriter::clearResultFiles() const
// {
// 	string cmd = "bash " + GrainsExec::m_GRAINS_HOME 
//      	+ "/Tools/ExecScripts/Paraview_clear.exec " + m_ParaviewFilename_dir +
// 	" " + m_ParaviewFilename;
//     GrainsExec::m_return_syscmd = system( cmd.c_str() );
//   }   
// }




// ----------------------------------------------------------------------------
// Methods to write binary data
void ParaviewPostProcessingWriter:: start_output_binary( int size, int number )
{
  int current_output_size = size*number ;
//   unsigned long ncomp = current_output_size + (current_output_size+999)/1000 
//   	+ 12 + sizeof_Int32 ;
  int ncomp = current_output_size + (current_output_size+999)/1000 
  	+ 12 + sizeof_Int32 ;	
  check_allocated_binary( ncomp ) ;
  CURRENT_LENGTH = store_int_binary( current_output_size ) ;
}




void ParaviewPostProcessingWriter:: write_double_binary( double val )  
{
  *((float*)&(BUFFER[OFFSET])) = (float)val ;
  OFFSET += sizeof_Float32  ;
}




void ParaviewPostProcessingWriter:: write_int_binary( int val )  
{
//  store_int_binary(val) ;
  *((int*)&(BUFFER[OFFSET])) = val ;
  OFFSET += sizeof_Int32  ;
}




int ParaviewPostProcessingWriter:: store_int_binary( int val )  
{
  int result = OFFSET ;
  *((int*)&(BUFFER[OFFSET])) = val ;
  OFFSET += sizeof_Int32  ;
  return result ;
}




void ParaviewPostProcessingWriter:: check_allocated_binary( int size )  
{
  if ( OFFSET + size >= ALLOCATED ) 
  {
    int new_size = max( 2*ALLOCATED, (int)1024 ) ;
    new_size = max( new_size, 2*(OFFSET+size) ) ;
    new_size = 4 * ( new_size/4 +1 ) ; // alignment on 4 bytes
      
    char * new_buffer = new char [ new_size ] ;
    for( int i=0 ;i<OFFSET ;i++ ) new_buffer[i] = BUFFER[i] ;
    if ( BUFFER!=0 ) delete [] BUFFER ;
    BUFFER = new_buffer ;
    ALLOCATED = new_size ;      
  }
}




void ParaviewPostProcessingWriter:: flush_binary( std::ofstream& file, 
	string const& calling )  
{
  compress_segment_binary( CURRENT_LENGTH, calling ) ;         
//  file << endl ;
}




void ParaviewPostProcessingWriter:: compress_segment_binary( int seg,
	string const& calling )  
{
   static int BlockSize = 32768 ;
   int size = (int)(*((int*)&BUFFER[seg])) ;
   
   int numFullBlocks = size / BlockSize;
   int lastBlockSize = size % BlockSize;
   int numBlocks = numFullBlocks + (lastBlockSize?1:0);

   int headerLength = numBlocks+3;

   int * CompressionHeader = new int[headerLength];
   CompressionHeader[0] = numBlocks;
   CompressionHeader[1] = BlockSize;
   CompressionHeader[2] = lastBlockSize;

   unsigned long encoded_buff_size = max(BlockSize,size)  ;
   unsigned char* encoded_buff = new unsigned char [ encoded_buff_size ] ;
   int encoded_offset = 0 ;
   for( int block=0 ; block<numBlocks ; block++ )
   {
      int buffer_start = seg + sizeof_Int32 + block*BlockSize ;
      int length = ( block+1<numBlocks || !lastBlockSize ? 
      	BlockSize : lastBlockSize ) ;
      unsigned char* to_encode = (unsigned char *)(&BUFFER[buffer_start]) ;
      unsigned char* encoded = &encoded_buff[encoded_offset] ;
      unsigned long ncomp = encoded_buff_size - encoded_offset ;

      if ( compress2( (Bytef*)encoded,
                   &ncomp,
                   (const Bytef*)to_encode,
                   length,
                   Z_DEFAULT_COMPRESSION) != Z_OK )
      {
         cout << "Zlib error while compressing data." << endl;
	 cout << "from " << calling << endl;
	 cout << "Details : block = " << block << "  numBlocks = " <<
	 	numBlocks << "  length = " << length << "  ncomp = "
		<< ncomp << endl;
	 exit(0);
      }
      CompressionHeader[3+block] = int(ncomp) ;
      encoded_offset += int(ncomp) ;      
   }
   
   OFFSET = seg ;
   check_allocated_binary( headerLength * sizeof_Int32 + encoded_offset ) ;
   
   for(int i=0 ; i<headerLength ; i++ )
      store_int_binary( CompressionHeader[i] ) ;     

   for(int i=0 ; i<encoded_offset ; i++ )
      BUFFER[OFFSET++] = encoded_buff[i] ;

   if( OFFSET%4 != 0 )
      OFFSET = 4*( OFFSET/4 +1 ) ; // Re-alignment
   
   delete [] CompressionHeader ;
   delete [] encoded_buff ;
}
// End of Methods to write binary data
// ----------------------------------------------------------------------------
