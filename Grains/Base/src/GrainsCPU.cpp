#include "GrainsCPU.hh"
#include "Grains.hh"
#include "GrainsParameters.hh"
#include "VectorMath.hh"


#include "ConvexBuilderFactory.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
GrainsCPU<T>::GrainsCPU()
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
GrainsCPU<T>::~GrainsCPU()
{}




// // -----------------------------------------------------------------------------
// // Initializes the simulation using the XML input
// template <typename T>
// void GrainsCPU<T>::initialize( DOMElement* rootElement )
// {   
//     // Read the input file
//     // Construction( rootElement );
//     // Forces( rootElement );
//     // AdditionalFeatures( rootElement );
// }




// -----------------------------------------------------------------------------
// Runs the simulation over the prescribed time interval
template <typename T>
void GrainsCPU<T>::simulate()
{
    unsigned int N = GrainsParameters<T>::m_numComponents;
    int* h_collision = new int[N];
    // Zeroing out
    for( int i = 0; i < N; i++ )
        h_collision[i] = 0;

    // Collision detection on host
    Grains<T>::m_postProcessor->PostProcessing_start();
    auto h_start = chrono::high_resolution_clock::now();
    for ( GrainsParameters<T>::m_time = GrainsParameters<T>::m_tStart;
          GrainsParameters<T>::m_time < GrainsParameters<T>::m_tEnd;
          GrainsParameters<T>::m_time += GrainsParameters<T>::m_dt )
    {
        Grains<T>::m_components->detectCollisionAndComputeForces( 
                                                  Grains<T>::m_linkedCell, 
                                                  Grains<T>::m_rigidBodyList,
                                                  Grains<T>::m_contactForce,
                                                  h_collision ); 
        Grains<T>::m_components->moveComponents( Grains<T>::m_timeIntegrator,
                                                 Grains<T>::m_rigidBodyList );
        
        // Post-Processing
        std::vector<unsigned int> id = Grains<T>::m_components->getRigidBodyId();
        std::vector<Transform3<T>> t = Grains<T>::m_components->getTransform();
        std::vector<Kinematics<T>> k = Grains<T>::m_components->getVelocity();
        Grains<T>::m_postProcessor->PostProcessing( Grains<T>::m_rigidBodyList,
                                                    &id,
                                                    &t,
                                                    &k,
                                                    GrainsParameters<T>::m_time );
    }
    auto h_end = chrono::high_resolution_clock::now();
    Grains<T>::m_postProcessor->PostProcessing_end();



    // Time comparison
    chrono::duration<double> h_time = h_end - h_start;
    std::cout << "CPU: " << h_time.count() << endl;

    // accuracy
    int trueHostCount = 0, trueDeviceCount = 0;
    for( int i = 0; i < N; i++ )
    {
        trueHostCount += h_collision[i];
    }
    cout << N << " Particles, "
         << trueHostCount << " Collision on host. " << endl;
}




// -----------------------------------------------------------------------------
// // Constructs the simulation -- Reads the Construction part of the XML input
// // linked cell, particles, obstacles, domain decomposition
// template <typename T>
// void GrainsCPU<T>::Construction( DOMElement* rootElement )
// {
//     DOMNode* root = ReaderXML::getNode( rootElement, "Construction" );
//     if ( !root )
//     {
//         cout << shiftString0 << "Construction section cannot be found!" << endl;
//         exit( 1 );
//     }

//     // Domain size: origin, max coordinates and periodicity
//     DOMNode* domain = ReaderXML::getNode( root, "LinkedCell" );
//     GrainsParameters<T>::m_dimension.setValue( 
//         T( ReaderXML::getNodeAttr_Double( domain, "MX" ) ),
//         T( ReaderXML::getNodeAttr_Double( domain, "MY" ) ),
//         T( ReaderXML::getNodeAttr_Double( domain, "MZ" ) ) );

//     DOMNode* domain_origin = ReaderXML::getNode( root, "Origin" );
//     if ( domain_origin )
//         GrainsParameters<T>::m_origin.setValue( 
//             T( ReaderXML::getNodeAttr_Double( domain_origin, "OX" ) ),
//             T( ReaderXML::getNodeAttr_Double( domain_origin, "OY" ) ),
//             T( ReaderXML::getNodeAttr_Double( domain_origin, "OZ" ) ) );
//     else
//         GrainsParameters<T>::m_origin.setValue( T( 0 ), T( 0 ), T( 0 ) );

//     GrainsParameters<T>::m_isPeriodic = false;


//     // Particles
//     DOMNode* particles = ReaderXML::getNode( root, "Particles" );
//     DOMNodeList* allParticles = ReaderXML::getNodes( rootElement, "Particle" );
//     int numRigidBodies = int( allParticles->getLength() );
//     int numEachRigidBody[ numRigidBodies ];
//     int numTotalParticles = 0;
//     T linkedCellSize = T( 0 );
//     if ( particles )
//     {
//         cout << shiftString6 << "Reading new particle types:" << endl;

//         // Memory allocation for m_rigidBodyList with respect to the number of 
//         // shapes in the simulation
//         m_rigidBodyList = ( RigidBody<T, T>** ) 
//                         malloc( numRigidBodies * sizeof( RigidBody<T, T>* ) );
//         for ( int i = 0; i < numRigidBodies; i++ )
//         {
//             DOMNode* nParticle = allParticles->item( i );
//             numEachRigidBody[ i ] = 
//                             ReaderXML::getNodeAttr_Int( nParticle, "Number" );

//             // Create the Rigid Body
//             m_rigidBodyList[ i ] = new RigidBody<T, T>( nParticle );

//             // Finding the max circumscribed radius among all shapes
//             if ( m_rigidBodyList[i]->getCircumscribedRadius() > linkedCellSize )
//                 linkedCellSize = m_rigidBodyList[i]->getCircumscribedRadius();
            
//             // Sum to find total number of particles
//             numTotalParticles += numEachRigidBody[ i ];
//         }

//         cout << shiftString6 << "Reading particle types completed." << endl;
//     }


//     // Create the LinkedCell
//     // m_linkedCell = ( LinkedCell<T>** ) 
//     //                  malloc( sizeof( LinkedCell<T>* ) );
//     // *m_linkedCell = new LinkedCell<T>( 
//     //     GrainsParameters<T>::m_origin,
//     //     GrainsParameters<T>::m_origin + GrainsParameters<T>::m_dimension, 
//     //     T( 2 ) * linkedCellSize );
//     // GrainsParameters<T>::m_numCells = (*m_linkedCell)->getNumCells();
//     m_linkedCell = LinkedCell<T>( 
//         GrainsParameters<T>::m_origin,
//         GrainsParameters<T>::m_origin + GrainsParameters<T>::m_dimension, 
//         T( 2 ) * linkedCellSize );
//     GrainsParameters<T>::m_numCells = m_linkedCell.getNumCells();
//     cout << shiftString6 << "LinkedCell created." << endl;

//     // Setting the number of particles in the simulation
//     GrainsParameters<T>::m_numComponents = numTotalParticles;
//     // m_components = ( ComponentManagerCPU<T>* ) 
//     //                  malloc( sizeof( ComponentManagerCPU<T> ) );
//     // m_components = new ComponentManagerCPU<T>();
//     // m_components = ComponentManagerCPU<T>();
//     m_components = ComponentManagerCPU<T>();

// //     // Scaling coefficient of linked cell size
// //     double LC_coef = 1.;
// //     DOMNode* nLC = ReaderXML::getNode( root, "LinkedCell" );
// //     if ( ReaderXML::hasNodeAttr( nLC, "CellSizeFactor" ) )
// //       LC_coef = ReaderXML::getNodeAttr_Double( nLC, "CellSizeFactor" );
// //     if ( LC_coef < 1. ) LC_coef = 1.;
// //     else if ( m_rank == 0 )
// //       cout << GrainsExec::m_shift9 << "Cell size factor = " << LC_coef << endl;


// //     // Link obstacles with the linked cell grid
// //     m_collision->Link( m_allcomponents.getObstacles() );
// //   }
// }




// // ----------------------------------------------------------------------------
// // External force definition
// void Grains::Forces( DOMElement* rootElement )
// {
//   if ( m_processorIsActive )
//   {
//     assert( rootElement != NULL );
//     DOMNode* root = ReaderXML::getNode( rootElement, "Forces" );

//     // Output message
//     if ( m_rank == 0 ) cout << GrainsExec::m_shift3 << "Forces" << endl;


//     // Read the forces
//     if ( root )
//     {
//       // Gravity
//       DOMNode* nGravity = ReaderXML::getNode( root, "Gravity" );
//       if( nGravity )
//       {
//         GrainsExec::m_vgravity[X] = ReaderXML::getNodeAttr_Double(
//       		nGravity, "GX" );
//         GrainsExec::m_vgravity[Y] = ReaderXML::getNodeAttr_Double(
//       		nGravity, "GY" );
//         GrainsExec::m_vgravity[Z] = ReaderXML::getNodeAttr_Double(
//       		nGravity, "GZ" );
//         if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "Gravity = " <<
// 		GrainsExec::m_vgravity << endl;
//       }
//       else
//       {
//         if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Gravity is mandatory !!" << endl;
//         grainsAbort();
//       }
//     }
//     else
//     {
//       if ( m_rank == 0 )
//       {
//         cout << GrainsExec::m_shift6 << "No force specified"
//       		<< endl;
//         cout << GrainsExec::m_shift6 << "At least gravity is mandatory !!"
//       		<< endl;
//         grainsAbort();
//       }
//     }


//     // Computes particle weight
//     m_allcomponents.computeWeight( 0., 0. );
//   }
// }




// // ----------------------------------------------------------------------------
// // Additional features of the simulation: time features, insertion,
// // post-processing
// void Grains::AdditionalFeatures( DOMElement* rootElement )
// {
//   if ( m_processorIsActive )
//   {
//     assert( rootElement != NULL );
//     DOMNode* root = ReaderXML::getNode( rootElement, "Simulation" );


//     // Output message
//     if ( m_rank == 0 ) cout << GrainsExec::m_shift3 << "Simulation" << endl;


//     // Check that Simulation node exists
//     if ( !root )
//     {
//       cout << GrainsExec::m_shift6 << "<Simulation> node is mandatory !!"
//       		<< endl;
//       grainsAbort();
//     }


//     // Time interval
//     DOMNode* nTimeInterval = ReaderXML::getNode( root, "TimeInterval" );
//     if ( nTimeInterval )
//     {
//       m_tstart = ReaderXML::getNodeAttr_Double( nTimeInterval, "Start" );
//       m_tend = ReaderXML::getNodeAttr_Double( nTimeInterval, "End" );
//       if ( GrainsExec::m_ReloadType == "same" ) m_tstart = m_time;
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "Time interval = ["
//       	<< m_tstart << "," << m_tend << "]" << endl;
//     }
//     else
//     {
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Time Interval is mandatory !!" << endl;
//       grainsAbort();
//     }

//     // Time step
//     DOMNode* nTimeStep = ReaderXML::getNode( root, "TimeStep" );
//     if ( nTimeStep )
//     {
//       m_dt = ReaderXML::getNodeAttr_Double( nTimeStep, "dt" );
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
//       	"Time step magnitude = " << m_dt << endl;
//     }
//     else
//     {
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Time step magnitude is mandatory !!" << endl;
//       grainsAbort();
//     }

//     // Time integrator
//     DOMNode* nTimeIntegration = ReaderXML::getNode( root, "TimeIntegration" );
//     if ( nTimeIntegration )
//       GrainsExec::m_TIScheme = ReaderXML::getNodeAttr_String( nTimeIntegration,
//     		"Type" );
//     if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
//       	"Time integration scheme = " << GrainsExec::m_TIScheme << endl;

//     // Restart file and writing mode
//     DOMNode* nRestartFile = ReaderXML::getNode( root, "RestartFile" );
//     if ( nRestartFile )
//     {
//       m_fileSave = ReaderXML::getNodeAttr_String( nRestartFile, "Name" );
//       if ( GrainsExec::m_ReloadType == "new" ) clearResultXmlFiles();
//       GrainsExec::m_SaveDirectory = GrainsExec::extractRoot( m_fileSave );
//       string wmode = ReaderXML::getNodeAttr_String( nRestartFile,
//       	"WritingMode" );
//       if ( wmode == "Hybrid" ) GrainsExec::m_writingModeHybrid = true ;
//       if ( m_rank == 0 )
//       {
//         cout << GrainsExec::m_shift6 << "Restart file" << endl;
// 	cout << GrainsExec::m_shift9 << "File name = " << m_fileSave << endl;
//         cout << GrainsExec::m_shift9 << "Directory = " <<
// 		GrainsExec::m_SaveDirectory << endl;
//         cout << GrainsExec::m_shift9 << "Writing mode = " <<
// 		( GrainsExec::m_writingModeHybrid ? "Hybrid" : "Text" ) << endl;
//       }
//     }
//     else
//     {
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"RestartFile features are mandatory !!" << endl;
//       grainsAbort();
//     }


//     // Output data frequency
//     DOMNode* nTimeSave = ReaderXML::getNode( root, "TimeSave" );
//     if ( nTimeSave )
//     {
//       double startSave = ReaderXML::getNodeAttr_Double( nTimeSave, "Start" );
//       double endSave = ReaderXML::getNodeAttr_Double( nTimeSave, "End" );
//       double dtSave = ReaderXML::getNodeAttr_Double( nTimeSave, "Every" );
//       if ( dtSave < m_dt ) dtSave = m_dt;
//       for (double t=startSave; t-endSave < 0.01 * m_dt; t+=dtSave)
//         m_save.push_back(t);
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "Output data every "
//       	<< dtSave << " from " << startSave << " to " << endSave << endl;
//     }
//     else
//     {
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Output data time features are mandatory !!" << endl;
//       grainsAbort();
//     }


//     // Moving obstacles
//     DOMNode* nMovingObstacles = ReaderXML::getNode( root,
//     	"MovingObstacles" );
//     int ObstacleUpdateFreq = 1;
//     bool displaceObstacles = true;
//     if ( nMovingObstacles )
//     {
//       // Linked cell grid update frequency
//       if ( ReaderXML::hasNodeAttr( nMovingObstacles, "LinkUpdateEvery" ) )
//         ObstacleUpdateFreq = ReaderXML::getNodeAttr_Int( nMovingObstacles,
//       		"LinkUpdateEvery" );

//       // Whether moving obstacles are geometrically displaced
//       if ( ReaderXML::hasNodeAttr( nMovingObstacles, "GeometricallyDisplace" ) )
//       {
//         string disp = ReaderXML::getNodeAttr_String( nMovingObstacles,
//      		"GeometricallyDisplace" );
//         if ( disp == "False" )
// 	{
// 	  displaceObstacles = false;
// 	  Obstacle::setMoveObstacle( false );
// 	}
//       }
//     }

//     if ( m_rank == 0 )
//     {
//       cout << GrainsExec::m_shift6 << "Moving obstacles (if any)" << endl;
//       cout << GrainsExec::m_shift9 <<
// 	"Moving obstacle - linked cell grid update every " <<
// 	ObstacleUpdateFreq << " time step" <<
// 	( ObstacleUpdateFreq > 1 ? "s" : "" ) << endl;
//       cout << GrainsExec::m_shift9 <<
//       	"Displace moving obstacles geometrically = " <<
//       	( displaceObstacles ? "True" : "False" ) << endl;
//     }


//     // Particle insertion
//     DOMNode* nInsertion = ReaderXML::getNode( root, "ParticleInsertion" );
//     if ( nInsertion )
//     {
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "Particle insertion"
//       	<< endl;

//       // Insertion mode
//       DOMNode* nMode = ReaderXML::getNode( nInsertion, "Mode" );
//       if ( nMode )
//       {
//         string type = ReaderXML::getNodeAttr_String( nMode, "Type" );
// 	if ( type == "InitialTime" ) m_insertion_mode = IM_INITIALTIME;
// 	else if ( type == "OverTime" ) m_insertion_mode = IM_OVERTIME;
//       }
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Mode = " <<
//       	( m_insertion_mode == IM_INITIALTIME ? "At initial time" :
// 		"Over time" ) << endl;


//       // Insertion order
//       DOMNode* nOrder = ReaderXML::getNode( nInsertion, "Order" );
//       if ( nOrder )
//       {
//         string type = ReaderXML::getNodeAttr_String( nOrder, "Type" );
// 	if ( type == "Random" ) m_insertion_order = PM_RANDOM;
//       }
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Order = " <<
//       	( m_insertion_order == PM_ORDERED ? "Ordered as defined in input file" :
// 		"Random" ) << endl;


//       // Initial angular position
//       DOMNode* nInitAngPos = ReaderXML::getNode( nInsertion,
//       	"InitialAngularPosition" );
//       if ( nInitAngPos )
//       {
//         string type = ReaderXML::getNodeAttr_String( nInitAngPos, "Type" );
// 	if ( type == "Random" ) m_init_angpos = IAP_RANDOM;
//       }
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Initial angular"
//       	" position = " << ( m_init_angpos == IAP_FIXED ? "Fixed as defined in "
// 		"input file particle class" : "Random" ) << endl;


//       // Random generator seed
//       DOMNode* nRGS = ReaderXML::getNode( nInsertion, "RandomGeneratorSeed" );
//       if ( nRGS )
//       {
//         string type = ReaderXML::getNodeAttr_String( nRGS, "Type" );
// 	if ( type == "Random" )
// 	{
// 	  m_randomseed = RGS_RANDOM;
// 	  srand( (unsigned int)( time(NULL)) );
// 	}
//       }
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Random generator"
//       	" seed = " << ( m_randomseed == RGS_DEFAULT ? "Default to 1 "
// 	"(infinitely reproducible)" : "Initialized with running day/time "
// 	"(non-reproducible)" ) << endl;


//       // Insertion attempt frequency
//       DOMNode* nFrequency = ReaderXML::getNode( nInsertion,
//       	"Frequency" );
//       if ( nFrequency )
//         m_insertion_frequency = size_t(
// 		ReaderXML::getNodeAttr_Int( nFrequency, "TryEvery" ));
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Insertion "
//       	"frequency = " << m_insertion_frequency << endl;


//       // Force insertion
//       DOMNode* nForceInsertion = ReaderXML::getNode( nInsertion,
//       	"ForceInsertion" );
//       if ( nForceInsertion )
//       {
//         string value = ReaderXML::getNodeAttr_String( nForceInsertion,
// 		"Value" );
// 	if ( value == "True" ) m_force_insertion = true;
//       }
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "Force insertion = " <<
//       	( m_force_insertion  ? "True" : "False" ) << endl;


//       // Particle positions via an external file OR a structured array OR a
//       // collection of insertion windows, in this order of priority
//       // Remark: these 3 modes cannot be combined
//       DOMNode* nPosition = ReaderXML::getNode( nInsertion, "ParticlePosition" );
//       if ( nPosition )
//       {
//         if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "Particle positions"
//       		<< endl;

//         // Fixed particle positions via an external file
//         DOMNode* nFile = ReaderXML::getNode( nPosition, "File" );
//         if ( nFile )
//         {
//           m_position = ReaderXML::getNodeAttr_String( nFile, "Name" );
//           if ( m_rank == 0 ) cout << GrainsExec::m_shift9 << "External file = "
// 		<< m_position << endl;
//         }
//         else
//         {
//           // Fixed particle positions via a structured array
// 	  DOMNode* nStruct = ReaderXML::getNode( nPosition, "StructuredArray" );
// 	  if ( nStruct )
// 	  {
// 	    m_InsertionArray = new struct StructArrayInsertion;
//             m_InsertionArray->box.ftype = WINDOW_BOX;
//             m_InsertionArray->box.radius = m_InsertionArray->box.height = 0. ;
//             m_InsertionArray->box.axisdir = NONE ;

//             DOMNode* nBox = ReaderXML::getNode( nStruct, "Box" );
//             if ( nBox )
//             {
//               DOMNodeList* points = ReaderXML::getNodes( nBox );
//               DOMNode* pointA = points->item( 0 );
//               DOMNode* pointB = points->item( 1 );
//               m_InsertionArray->box.ptA[X] =
//             	ReaderXML::getNodeAttr_Double( pointA, "X" );
//               m_InsertionArray->box.ptA[Y] =
//             	ReaderXML::getNodeAttr_Double( pointA, "Y" );
//               m_InsertionArray->box.ptA[Z] =
//             	ReaderXML::getNodeAttr_Double( pointA, "Z" );
//               m_InsertionArray->box.ptB[X] =
//             	ReaderXML::getNodeAttr_Double( pointB, "X" );
//               m_InsertionArray->box.ptB[Y] =
//             	ReaderXML::getNodeAttr_Double( pointB, "Y" );
//               m_InsertionArray->box.ptB[Z] =
//             	ReaderXML::getNodeAttr_Double( pointB, "Z" );
//             }
// 	    else
// 	    {
//               if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Node Box is required in <StructuredArray> !!" << endl;
//               grainsAbort();
// 	    }

//             DOMNode* nNumber = ReaderXML::getNode( nStruct, "Number" );
// 	    if ( nNumber )
// 	    {
//               m_InsertionArray->NX = ReaderXML::getNodeAttr_Int( nNumber,
// 	      	"NX" );
//               m_InsertionArray->NY = ReaderXML::getNodeAttr_Int( nNumber,
// 	      	"NY" );
//               m_InsertionArray->NZ = ReaderXML::getNodeAttr_Int( nNumber,
// 	      	"NZ" );
// 	    }
// 	    else
// 	    {
//               if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Node Number is required in <StructuredArray> !!" << endl;
//               grainsAbort();
// 	    }

//             m_position = "STRUCTURED";
// 	    if ( m_rank == 0 )
// 	    {
// 	      cout << GrainsExec::m_shift9 << "Structured array" << endl;
//               cout << GrainsExec::m_shift12 << "Point3 min = " <<
// 	    	m_InsertionArray->box.ptA[X] << " " <<
// 		m_InsertionArray->box.ptA[Y] << " " <<
//                 m_InsertionArray->box.ptA[Z] << endl;
//               cout << GrainsExec::m_shift12 << "Point3 max = " <<
// 	    	m_InsertionArray->box.ptB[X] << " " <<
// 		m_InsertionArray->box.ptB[Y] << " " <<
//                 m_InsertionArray->box.ptB[Z] << endl;
//               cout << GrainsExec::m_shift12 << "Array = " <<
// 	    	m_InsertionArray->NX << " x " <<
// 		m_InsertionArray->NY << " x " <<
//             	m_InsertionArray->NZ << endl;
// 	    }
// 	  }
// 	  else
// 	  {
// 	    // Random particle positions from a collection of insertion windows
// 	    DOMNode* nWindows = ReaderXML::getNode( nPosition, "Windows" );
//             if ( nWindows )
// 	    {
// 	      cout << GrainsExec::m_shift9 << "Insertion windows" << endl;
// 	      DOMNodeList* allWindows = ReaderXML::getNodes( nWindows );
//               for (XMLSize_t i=0; i<allWindows->getLength(); i++)
// 	      {
// 	        DOMNode* nWindow = allWindows->item( i );
//                 Window iwindow;
// 		      readWindow( nWindow, iwindow, GrainsExec::m_shift12 );
// 	        m_insertion_windows.insert( m_insertion_windows.begin(), 
// 			iwindow );
//               }
// 	    }
//             else
// 	    {
//               if ( m_insertion_mode != IM_NOINSERT )
//               {
//                 if ( m_rank == 0 )
//                   cout << GrainsExec::m_shift6 <<
//             "Insertion positions or windows are mandatory !!" << endl;
//                 grainsAbort();
//               }
// 	    }
// 	  }
//   }
//       }
//       else
//       {
//         if ( m_insertion_mode != IM_NOINSERT )
//         {
//           if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Insertion positions/windows are mandatory !!" << endl;
//           grainsAbort();
//         }
//       }


//       // Initialization of particle velocity
//       if ( m_rank == 0 )
//         cout << GrainsExec::m_shift6 << "Particle initial velocity: ";

//       DOMNode* nInitVit = ReaderXML::getNode( nInsertion, "InitialVelocity" );
//       if ( nInitVit )
//       {
//         string sInitVitmode =
// 	    ReaderXML::getNodeAttr_String( nInitVit, "Mode" );

//         if ( sInitVitmode == "Constant" )
//         {
// 	  m_initvit_mode = IV_CONSTANT;

//           DOMNode* nVitTransInit = ReaderXML::getNode( nInitVit,
// 	  	"TranslationalVelocity" );
//           if ( nVitTransInit )
//           {
//             m_InitVtrans[X] = ReaderXML::getNodeAttr_Double( nVitTransInit,
// 	      	"VX" );
//             m_InitVtrans[Y] = ReaderXML::getNodeAttr_Double( nVitTransInit,
// 	      	"VY" );
//             m_InitVtrans[Z] = ReaderXML::getNodeAttr_Double( nVitTransInit,
// 	      	"VZ" );
//           }

//           DOMNode* nVitRotInit = ReaderXML::getNode( nInitVit,
// 	    	"AngularVelocity" );
//           if ( nVitRotInit )
//           {
//             m_InitVrot[X] = ReaderXML::getNodeAttr_Double( nVitRotInit,
// 	      	"RX" );
//             m_InitVrot[Y] = ReaderXML::getNodeAttr_Double( nVitRotInit,
// 	      	"RY" );
//             m_InitVrot[Z] = ReaderXML::getNodeAttr_Double( nVitRotInit,
// 	      	"RZ" );
//           }
//         }
//         else if ( sInitVitmode == "Random" )
//         {
// 	  m_initvit_mode = IV_RANDOM;

// 	  DOMNode* nRandomTrans = ReaderXML::getNode( nInitVit,
// 		"Translational" );
// 	  if ( nRandomTrans )
// 	    m_RandomMotionCoefTrans =
// 		ReaderXML::getNodeAttr_Double( nRandomTrans, "Amplitude" );

// 	  DOMNode* nRandomRot = ReaderXML::getNode( nInitVit,
// 	    	"Angular" );
// 	  if ( nRandomRot )
// 	    m_RandomMotionCoefRot =
// 	 	ReaderXML::getNodeAttr_Double( nRandomRot, "Amplitude" ) ;
//         }
//         else m_initvit_mode = IV_ZERO;
//       }

//       if ( m_rank == 0 )
//       {
//         switch( m_initvit_mode )
//         {
//           case IV_CONSTANT :
//             cout << "constant" << endl;
// 	    cout << GrainsExec::m_shift9 << "translational = ( " <<
// 	   	 m_InitVtrans[X] << ", " << m_InitVtrans[Y] << ", "
// 		 << m_InitVtrans[Z] << " )" << endl;
// 	    cout << GrainsExec::m_shift9 << "angular = ( " <<
// 	   	 m_InitVrot[X] << ", " << m_InitVrot[Y] << ", "
// 		 << m_InitVrot[Z] << " )" << endl;
//             break;

//           case IV_RANDOM :
//             cout << "random" << endl;
// 	    cout << GrainsExec::m_shift9 << "translational amplitude = " <<
// 	   	m_RandomMotionCoefTrans << endl;
// 	    cout << GrainsExec::m_shift9 << "angular amplitude = " <<
// 	   	m_RandomMotionCoefRot << endl;
//             break;

//           default :
//             cout << "uniformly zero" << endl;
//             break;
//         }
//       }
//     }
//     else
//     {
//       m_insertion_mode = IM_NOINSERT;
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6 << "No insertion" << endl;
//     }


//     // Obstacle loadings
//     DOMNode* nObstacleLoadings = ReaderXML::getNode( root, "ObstacleLoadings" );
//     size_t error = 0;
//     if ( nObstacleLoadings )
//     {
//       if ( m_rank == 0 )
//         cout << GrainsExec::m_shift6 << "Obstacle loadings" << endl;

//       DOMNodeList* allOLs = ReaderXML::getNodes( nObstacleLoadings );
//       for (XMLSize_t i=0; i<allOLs->getLength(); i++)
//       {
//         DOMNode* nOL = allOLs->item( i );
// 	string type = ReaderXML::getNodeAttr_String( nOL, "Type" );
// 	if ( m_rank == 0 )
//           cout << GrainsExec::m_shift9 << "Type = " << type << endl;

// 	// Chargements en Force
// 	if ( type == "Force" )
// 	{
// 	  ObstacleImposedForce* load = new ObstacleImposedForce(
// 	      nOL, m_dt, m_rank, error );
// 	  if ( error != 0 ) grainsAbort();
// 	  else m_allcomponents.LinkImposedMotion( load );
// 	}
// 	else if ( type == "Velocity" )
// 	{
// 	  ObstacleImposedVelocity* load = new ObstacleImposedVelocity(
// 	  	nOL, m_dt, m_rank, error );
// 	  if ( error != 0 ) grainsAbort();
// 	  else m_allcomponents.LinkImposedMotion( load );
// 	}
// 	else
//         {
// 	  if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Unknown obstacle loading type; values: Force or Velocity"
// 		<< endl;
//           grainsAbort();
// 	}
//       }
//     }


//     // Post-processing writers
//     DOMNode* nPostProcessing = ReaderXML::getNode( root,
//     	"PostProcessing" );
//     if ( nPostProcessing )
//     {
//       if ( m_rank == 0 )
//         cout << GrainsExec::m_shift6 << "Postprocessing" << endl;

//       // Post-processing subdomain
//       PostProcessingWriter::allocate_PostProcessingWindow( m_nprocs );

//       DOMNode* nPostProcessingDomain = ReaderXML::getNode( nPostProcessing,
//       	"Domain" );
//       if ( nPostProcessingDomain )
//       {
//         DOMNodeList* nWindowPoints = ReaderXML::getNodes(
// 		nPostProcessingDomain );

//  	DOMNode* pointA = nWindowPoints->item( 0 );
//  	DOMNode* pointB = nWindowPoints->item( 1 );

//  	Window PPWindow;
// 	PPWindow.ftype = WINDOW_BOX;
// 	PPWindow.radius = PPWindow.radius_int = PPWindow.height = 0. ;
// 	PPWindow.axisdir = NONE ;
//  	PPWindow.ptA[X] = ReaderXML::getNodeAttr_Double( pointA, "X" );
// 	PPWindow.ptA[Y] = ReaderXML::getNodeAttr_Double( pointA, "Y" );
// 	PPWindow.ptA[Z] = ReaderXML::getNodeAttr_Double( pointA, "Z" );
// 	PPWindow.ptB[X] = ReaderXML::getNodeAttr_Double( pointB, "X" );
// 	PPWindow.ptB[Y] = ReaderXML::getNodeAttr_Double( pointB, "Y" );
// 	PPWindow.ptB[Z] = ReaderXML::getNodeAttr_Double( pointB, "Z" );

// 	double Ox, Oy, Oz, lx, ly, lz;
// 	bool b_X = false, b_Y = false, b_Z = false, b_PPWindow = false;
// 	App::get_local_domain_origin( Ox, Oy, Oz );
// 	App::get_local_domain_size( lx, ly, lz );

// 	if ( ( PPWindow.ptA[X] >= Ox && PPWindow.ptA[X] < Ox + lx )
// 		|| ( PPWindow.ptB[X] >= Ox && PPWindow.ptB[X] < Ox + lx )
// 		|| ( Ox > PPWindow.ptA[X] && Ox < PPWindow.ptB[X] )
// 		|| ( Ox > PPWindow.ptB[X] && Ox < PPWindow.ptA[X] ) )
// 	  b_X = true;
// 	if ( ( PPWindow.ptA[Y] >= Oy && PPWindow.ptA[Y] < Oy + ly )
// 		|| ( PPWindow.ptB[Y] >= Oy && PPWindow.ptB[Y] < Oy + ly )
// 		|| ( Oy > PPWindow.ptA[Y] && Oy < PPWindow.ptB[Y] )
// 		|| ( Oy > PPWindow.ptB[Y] && Oy < PPWindow.ptA[Y] ) )
// 	  b_Y = true;
// 	if ( ( PPWindow.ptA[Z] >= Oz && PPWindow.ptA[Z] < Oz + lz )
// 		|| ( PPWindow.ptB[Z] >= Oz && PPWindow.ptB[Z] < Oz + lz )
// 		|| ( Oz > PPWindow.ptA[Z] && Oz < PPWindow.ptB[Z] )
// 		|| ( Oz > PPWindow.ptB[Z] && Oz < PPWindow.ptA[Z] ) )
// 	  b_Z = true;

// 	if ( b_X && b_Y && b_Z ) b_PPWindow = true;

// 	PostProcessingWriter::set_PostProcessingWindow( m_rank, b_PPWindow );

// 	synchronize_PPWindow();

// 	if ( m_rank == 0 )
// 	{
// 	  cout << GrainsExec::m_shift9 << "Domain" << endl;
//           cout << GrainsExec::m_shift12 << "Point3 A = " <<
// 		PPWindow.ptA[X] << " " << PPWindow.ptA[Y] << " " <<
// 		PPWindow.ptA[Z] << endl;
//           cout << GrainsExec::m_shift12 << "Point3 B = " <<
// 		PPWindow.ptB[X] << " " << PPWindow.ptB[Y] << " " <<
// 		PPWindow.ptB[Z] << endl;
// 	}
//       }
//       else
// 	if ( m_rank == 0 )
// 	  cout << GrainsExec::m_shift9 << "Domain = linked cell grid"
// 	  	<< endl;


//       // Post-processing writers
//       DOMNode* nWriters = ReaderXML::getNode( nPostProcessing, "Writers" );
//       if ( nWriters )
//       {
//         DOMNodeList* allPPW = ReaderXML::getNodes( nWriters );
//         for (XMLSize_t i=0; i<allPPW->getLength(); i++)
//         {
//           DOMNode* nPPW = allPPW->item( i );
//           PostProcessingWriter* ppw =
// 	  	PostProcessingWriterBuilderFactory::create(
//       		nPPW, m_rank, m_nprocs );
//           if ( ppw ) m_allcomponents.addPostProcessingWriter( ppw );
// 	  else
//           {
// 	    if ( m_rank == 0 ) cout << GrainsExec::m_shift6 <<
// 		"Unknown postprocessing writer in node <Writers>"
// 		<< endl;
//             grainsAbort();
// 	  }
//         }
//       }
//       else
//         if ( m_rank == 0 ) cout << GrainsExec::m_shift6
//       		<< "No postprocessing writers" << endl;


//       // Total Force & torque on obstacles
//       DOMNode* nForceTorqueObstacles = ReaderXML::getNode( nPostProcessing,
//       	"ForceTorqueObstacles" );
//       if ( nForceTorqueObstacles )
//       {
//         int FToutputFreq = ReaderXML::getNodeAttr_Int( nForceTorqueObstacles,
// 		"Every" );
//         string ppObsdir = ReaderXML::getNodeAttr_String( nForceTorqueObstacles,
// 		"Directory" );
//         list<string> allppObsName;
//         DOMNodeList* allppObs = ReaderXML::getNodes( nForceTorqueObstacles );
//         for (XMLSize_t i=0; i<allppObs->getLength(); i++)
//         {
//           DOMNode* nppObs = allppObs->item( i );
//           allppObsName.push_back(
// 		ReaderXML::getNodeAttr_String( nppObs, "ObstacleName" ) );
//         }
//         m_allcomponents.setOutputObstaclesLoadParameters( ppObsdir,
//         	FToutputFreq, allppObsName );

// 	if ( m_rank == 0 )
// 	{
// 	  cout << GrainsExec::m_shift9 << "Force & torque on obstacles" << endl;
//           cout << GrainsExec::m_shift12 << "Write values in file every " <<
// 		FToutputFreq << " time step" <<
// 		( FToutputFreq > 1 ? "s" : "" ) << endl;
//           cout << GrainsExec::m_shift12 << "Output file directory name = "
//     		<< ppObsdir << endl;
//           cout << GrainsExec::m_shift12 << "Obstacle names" << endl;
// 	  for (list<string>::const_iterator il=allppObsName.begin();
// 	  	il!=allppObsName.end();il++)
// 	    cout << GrainsExec::m_shift15 << *il << endl;
// 	}
//       }
//     }
//     else
//       if ( m_rank == 0 ) cout << GrainsExec::m_shift6
//       	<< "No postprocessing" << endl;
//   }
// }




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsCPU<float>;
template class GrainsCPU<double>;