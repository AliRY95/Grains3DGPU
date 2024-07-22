#include "GrainsBuilderFactory.hh"
#include "Grains.hh"
#include "GrainsCPU.hh"
#include "GrainsGPU.hh"


// -----------------------------------------------------------------------------
// Adds the path to the dtd files using the GRAINS_HOME variable to
// a copy of the input file. Returns the name of this copy.
// TODO: CLEAN
template <typename T>
string GrainsBuilderFactory<T>::init( string const& filename )
{
    // Get the GRAINS_HOME from the shell
    char* grainshome = getenv( "GRAINS_HOME" );
    string str_grainshome( grainshome );
        
    // Creates the copy file
    string tline, buffer, 
    header1 = "<?xml version=\"1.0\" encoding=\"iso-8859-1\"?>", 
    header2 = "<!DOCTYPE Grains3D SYSTEM \"", option, dtd_file;
    list<string> inputFile_linelist;
    int dimension = 0;

    header2 += str_grainshome + "/Main/dtd/";

    ifstream fileIN( filename.c_str(), ios::in );
    ofstream fileOUT( ( filename + ".tmp" ).c_str(), ios::out );   

    while ( !fileIN.eof() ) 
    { 
        buffer.clear();
        getline( fileIN, tline, '\n' );
        istringstream iss(tline);
        iss >> buffer;
        if ( buffer != "<?xml" && buffer != "<!DOCTYPE" )  
        {
            inputFile_linelist.push_back( tline );
            if ( buffer == "<Grains3D" || buffer == "<GrainsGeomTest" ) 
            {
                dimension = 3;
                buffer.clear();
                iss >> buffer;
                size_t pos = buffer.find( "\"" );
                string sub = buffer.substr( pos + 1 );
                pos = sub.rfind( "\"" );
                option = sub.erase( pos );
            }	
            else if ( buffer == "<Grains2D" ) 
            {
                dimension = 2;
                buffer.clear();
                iss >> buffer;
                size_t pos = buffer.find( "\"" );
                string sub = buffer.substr( pos + 1 );
                pos = sub.rfind( "\"" );
                option = sub.erase( pos );	
            }
            else if ( buffer == "<Grains3D>" || buffer == "<GrainsGeomTest>" )
                dimension = 3;
            else if ( buffer == "<Grains2D>" )
                dimension = 2;	      		
        }	
    }
  
    if ( dimension == 2 )
    {
        if ( option == "CoupledFluid" || option == "CoupledFluidMPI" )
            header2 += "Grains2D_InFluid.dtd\">";
        else
            header2 += "Grains2D.dtd\">";  
    }
    else
    {
        if ( option == "CoupledFluid" || option == "CoupledFluidMPI" )
            header2 += "Grains3D_InFluid.dtd\">";
        else
            header2 += "Grains3D.dtd\">";   
    }

    fileOUT << header1 << endl;
    fileOUT << header2 << endl;  
    for ( list<string>::iterator il = inputFile_linelist.begin();
        il != inputFile_linelist.end(); il++ )
        fileOUT << *il << endl;

    fileIN.close();
    fileOUT.close();

    return ( filename + ".tmp" );
}




// -----------------------------------------------------------------------------
// Creates and returns a standard Grains application
template <typename T>
Grains<T>* GrainsBuilderFactory<T>::create( DOMElement* root )
{
    // Preconditions
    if ( !root )
    {
        cout << "Invalid XML file! Aborting Grains!" << endl;
        exit( 1 );
    }

    Grains<T>* grains = NULL;

    string type   = ReaderXML::getNodeName( root );
    string option = ReaderXML::getNodeAttr_String( root, "Type" );
    
    if ( option == "Standard" ) 
        grains = new GrainsCPU<T>();
    else if ( option == "GPU" )
        grains = new GrainsGPU<T>();
        
    // Postconditions
    if ( !grains )
    {
        cout << "Invalid Mode! Aborting Grains!" << endl;
        exit( 1 );
    }

    return ( grains );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsBuilderFactory<float>;
template class GrainsBuilderFactory<double>;