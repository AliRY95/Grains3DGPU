/* ========================================================================== */
/*                Discrete Element Method Using NVIDIA CUDA                   */
/*                      Alireza Yazdani, July 2023                            */
/* ========================================================================== */
#include <cuda.h>
#include <cuda_runtime.h>

#include "Grains.hh"
#include "GrainsMisc.hh"
#include "GrainsBuilderFactory.hh"
#include "ReaderXML.hh"


using namespace std;

/* ========================================================================== */
/*                                    Main                                    */
/* ========================================================================== */
/* Main Function */
int main(int argc, char* argv[])
{
    // Input file
    string filename = argv[1], filename_exe;
    size_t error = 0;
    size_t pos = filename.find(".xml");
    if ( pos == string::npos )
    {
        cout << "ERROR: input file needs the .xml extension" << endl;
        error = true;
    }

    // Execute the Grains application
    if ( !error )
    {
        // Create a temporary input file with the proper XML header
        // We use the double version of GrainsBuilderFactory, but it doesn't
        // matter.
        filename_exe = GrainsBuilderFactory<double>::init( filename );

        // Creates the Grains application
        ReaderXML::initialize();
        DOMElement* rootNode = ReaderXML::getRoot( filename_exe );
        string prc = ReaderXML::getNodeAttr_String( rootNode, "Precision" );
        if ( prc == "Single" )
        {
            Grains<float>* grains = nullptr;
            grains = GrainsBuilderFactory<float>::create( rootNode );
            
            // Initial output message
            // grains->initialOutputMessage();

            // Tasks to perform before time-stepping
            grains->initialize( rootNode );
            ReaderXML::terminate();

            // Delete the temporary input file
            std::string cmd = "/bin/rm " + filename_exe;
            GrainsMisc<float>::m_returnSysCmd = system( cmd.c_str() );

            // Run the simulation
            grains->simulate();

            // Tasks to perform after time-stepping
            // grains->do_after_time_stepping();

            // Delete the Grains application
            delete grains;
        }
        else
        {
            if ( prc != "Double" )
            {
                std::cout << "Invalid precision! Creating Grains in double "
                          << "precision!" << endl;
            }

            Grains<double>* grains = nullptr;
            grains = GrainsBuilderFactory<double>::create( rootNode );
            
            // Initial output message
            // grains->initialOutputMessage();

            // Tasks to perform before time-stepping
            grains->initialize( rootNode );
            ReaderXML::terminate();

            // Delete the temporary input file
            std::string cmd = "/bin/rm " + filename_exe;
            GrainsMisc<double>::m_returnSysCmd = system( cmd.c_str() );

            // Run the simulation
            grains->simulate();

            // Tasks to perform after time-stepping
            // grains->do_after_time_stepping();

            // Delete the Grains application
            delete grains;
        }
    }

    return ( 0 );
}