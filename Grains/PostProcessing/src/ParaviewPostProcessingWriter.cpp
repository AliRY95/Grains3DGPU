#include "ParaviewPostProcessingWriter.hh"
#include "GrainsUtils.hh"
#include "VectorMath.hh"

/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// // Delete all result files
// __HOST__
// void clearResultFiles()
// {
// 	string cmd = "bash " + GrainsExec::m_GRAINS_HOME
//      	+ "/Tools/ExecScripts/Paraview_clear.exec " + m_ParaviewFilename_dir +
// 	" " + m_ParaviewFilename;
//     GrainsExec::m_return_syscmd = system( cmd.c_str() );
//   }
// }

// -----------------------------------------------------------------------------
// Writes obstacles data
template <typename T>
__HOST__ void writeObstacles_Paraview(RigidBody<T, T> const* const* obstacleRB,
                                      const ComponentManager<T>*    cm,
                                      const std::string&            obsFileName)
{
    ofstream                         f((obsFileName).c_str(), ios::out);
    const uint                       numObstacles = cm->getNumberOfObstacles();
    const std::vector<Transform3<T>> tr           = cm->getTransformObstacles();

    f << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" "
      << "byte_order=\"LittleEndian\" ";
    f << ">" << endl;
    f << "<UnstructuredGrid>" << endl;
    uint nbpts = 0, nbcells = 0;
    for(uint i = 0; i < numObstacles; ++i)
    {
        nbpts += obstacleRB[i]->getConvex()->numberOfPoints_PARAVIEW();
        nbcells += obstacleRB[i]->getConvex()->numberOfCells_PARAVIEW();
    }
    f << "<Piece NumberOfPoints=\"" << nbpts << "\"" << " NumberOfCells=\""
      << nbcells << "\">" << endl;
    f << "<Points>" << endl;
    f << "<DataArray type=\"Float32\" NumberOfComponents=\"3\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(uint i = 0; i < numObstacles; ++i)
        obstacleRB[i]->getConvex()->writePoints_PARAVIEW(f, tr[i]);
    f << "</DataArray>" << endl;
    f << "</Points>" << endl;

    list<uint>           connectivity, offsets, cellstype;
    list<uint>::iterator ii;
    uint                 firstpoint_globalnumber = 0, last_offset = 0;
    for(uint i = 0; i < numObstacles; ++i)
        obstacleRB[i]->getConvex()->writeConnection_PARAVIEW(
            connectivity,
            offsets,
            cellstype,
            firstpoint_globalnumber,
            last_offset);
    f << "<Cells>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"connectivity\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(ii = connectivity.begin(); ii != connectivity.end(); ii++)
        f << *ii << " ";
    f << endl;
    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"offsets\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(ii = offsets.begin(); ii != offsets.end(); ii++)
        f << *ii << " ";
    f << endl;
    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"types\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(ii = cellstype.begin(); ii != cellstype.end(); ii++)
        f << *ii << " ";
    f << endl;
    f << "</DataArray>" << endl;
    f << "</Cells>" << endl;

    f << "<CellData Scalars=\"Indicator\">" << endl;
    f << "<DataArray type=\"Float32\" Name=\"Indicator\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(uint i = 0; i < numObstacles; ++i)
    {
        // double indic = obstacleRB[i]->getIndicator();
        double indic = 0;
        int    nc    = obstacleRB[i]->getConvex()->numberOfCells_PARAVIEW();
        for(uint j = 0; j < nc; ++j)
            f << indic << " ";
    }
    f << endl;
    f << "</DataArray>" << endl;
    f << "</CellData>" << endl;

    f << "</Piece>" << endl;
    f << "</UnstructuredGrid>" << endl;
    f << "</VTKFile>" << endl;
    f.close();
}

// -----------------------------------------------------------------------------
// Writes particles data
template <typename T>
__HOST__ void writeParticles_Paraview(RigidBody<T, T> const* const* particleRB,
                                      const ComponentManager<T>*    cm,
                                      const std::string&            parFileName)
{
    ofstream                         f((parFileName).c_str(), ios::out);
    const uint                       numParticles = cm->getNumberOfParticles();
    const std::vector<Transform3<T>> tr           = cm->getTransform();
    const std::vector<Kinematics<T>> kin          = cm->getVelocity();

    f << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" "
      << "byte_order=\"LittleEndian\" ";
    f << ">" << endl;
    f << "<UnstructuredGrid>" << endl;
    uint nbpts = 0, nbcells = 0;
    for(uint i = 0; i < numParticles; ++i)
    {
        nbpts += particleRB[i]->getConvex()->numberOfPoints_PARAVIEW();
        nbcells += particleRB[i]->getConvex()->numberOfCells_PARAVIEW();
    }

    f << "<Piece NumberOfPoints=\"" << nbpts << "\"" << " NumberOfCells=\""
      << nbcells << "\">" << endl;

    f << "<Points>" << endl;
    f << "<DataArray type=\"Float32\" NumberOfComponents=\"3\" ";
    f << "format=\"ascii\">" << endl;

    for(uint i = 0; i < numParticles; ++i)
    {
        particleRB[i]->getConvex()->writePoints_PARAVIEW(f, tr[i]);
    }
    f << "</DataArray>" << endl;
    f << "</Points>" << endl;

    list<uint>           connectivity, offsets, cellstype;
    list<uint>::iterator ii;
    uint                 firstpoint_globalnumber = 0, last_offset = 0;
    for(uint i = 0; i < numParticles; ++i)
        particleRB[i]->getConvex()->writeConnection_PARAVIEW(
            connectivity,
            offsets,
            cellstype,
            firstpoint_globalnumber,
            last_offset);
    f << "<Cells>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"connectivity\" ";
    f << "format=\"ascii\">" << endl;
    for(ii = connectivity.begin(); ii != connectivity.end(); ii++)
        f << *ii << " ";
    f << endl;
    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"offsets\" ";
    f << "format=\"ascii\">" << endl;

    for(ii = offsets.begin(); ii != offsets.end(); ii++)
        f << *ii << " ";
    f << endl;

    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"types\" ";
    f << "format=\"ascii\">" << endl;

    for(ii = cellstype.begin(); ii != cellstype.end(); ii++)
        f << *ii << " ";
    f << endl;

    f << "</DataArray>" << endl;
    f << "</Cells>" << endl;

    f << "<CellData Scalars=\"NormU,NormOm,CoordNumb\">" << endl;

    f << "<DataArray type=\"Float32\" Name=\"NormU\" ";
    f << "format=\"ascii\">" << endl;

    for(uint i = 0; i < numParticles; ++i)
    {
        T    normU = norm(kin[i].getTranslationalComponent());
        uint nc    = particleRB[i]->getConvex()->numberOfCells_PARAVIEW();
        for(uint j = 0; j < nc; ++j)
            f << normU << " ";
    }
    f << endl;
    f << "</DataArray>" << endl;

    f << "<DataArray type=\"Float32\" Name=\"NormOm\" ";
    f << "format=\"ascii\">" << endl;

    for(uint i = 0; i < numParticles; ++i)
    {
        T    normOm = norm(kin[i].getAngularComponent());
        uint nc     = particleRB[i]->getConvex()->numberOfCells_PARAVIEW();
        for(uint j = 0; j < nc; ++j)
            f << normOm << " ";
    }
    f << endl;
    f << "</DataArray>" << endl;

    f << "<DataArray type=\"Float32\" Name=\"CoordNumb\" ";
    f << "format=\"ascii\">" << endl;

    for(uint i = 0; i < numParticles; ++i)
    {
        T    coordNum = 0;
        uint nc       = particleRB[i]->getConvex()->numberOfCells_PARAVIEW();
        for(uint j = 0; j < nc; ++j)
            f << coordNum << " ";
    }
    f << endl;
    f << "</DataArray>" << endl;
    f << "</CellData>" << endl;
    f << "</Piece>" << endl;

    f << "</UnstructuredGrid>" << endl;
    f << "</VTKFile>" << endl;
    f.close();
}

/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Default constructor
template <typename T>
__HOST__ ParaviewPostProcessingWriter<T>::ParaviewPostProcessingWriter()
{
}

// -----------------------------------------------------------------------------
// Constructor with XML node, rank and number of processes as input parameters
template <typename T>
__HOST__
    ParaviewPostProcessingWriter<T>::ParaviewPostProcessingWriter(DOMNode* dn)
{
    m_ParaviewFilename     = ReaderXML::getNodeAttr_String(dn, "RootName");
    m_ParaviewFilename_dir = ReaderXML::getNodeAttr_String(dn, "Directory");

    GoutWI(9, "Type = Paraview");
    GoutWI(12, "Output file root name =", m_ParaviewFilename);
    GoutWI(12, "Output file directory name =", m_ParaviewFilename_dir);
    // GoutWI(12, "Writing mode =", (m_binary ? "Binary" : "Text"));
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__ ParaviewPostProcessingWriter<T>::~ParaviewPostProcessingWriter()
{
}

// ----------------------------------------------------------------------------
// Gets the post-processing writer type
template <typename T>
__HOST__ PostProcessingWriterType
    ParaviewPostProcessingWriter<T>::getPostProcessingWriterType() const
{
    return (PARAVIEW);
}

// -----------------------------------------------------------------------------
template <typename T>
__HOST__ void ParaviewPostProcessingWriter<T>::PostProcessing_start()
{
    // clearResultFiles();
    // Obstacles
    m_Paraview_saveObstacles_pvd << "<?xml version=\"1.0\"?>" << endl;
    m_Paraview_saveObstacles_pvd
        << "<VTKFile type=\"Collection\" version=\"0.1\""
        << " byte_order=\"LittleEndian\"";
    m_Paraview_saveObstacles_pvd << ">" << endl;
    m_Paraview_saveObstacles_pvd << "<Collection>" << endl;

    // Particles
    ostringstream* ossNULL = NULL;
    m_Paraview_saveParticles_pvd.reserve(1);
    m_Paraview_saveParticles_pvd.push_back(ossNULL);
    m_Paraview_saveParticles_pvd[0] = new ostringstream;
    *m_Paraview_saveParticles_pvd[0] << "<?xml version=\"1.0\"?>" << endl;
    *m_Paraview_saveParticles_pvd[0]
        << "<VTKFile type=\"Collection\" version=\"0.1\""
        << " byte_order=\"LittleEndian\"";
    *m_Paraview_saveParticles_pvd[0] << ">" << endl;
    *m_Paraview_saveParticles_pvd[0] << "<Collection>" << endl;
}

// -----------------------------------------------------------------------------
// Writes data
template <typename T>
__HOST__ void ParaviewPostProcessingWriter<T>::PostProcessing(
    RigidBody<T, T> const* const* particleRB,
    RigidBody<T, T> const* const* obstacleRB,
    const ComponentManager<T>*    cm,
    const T                       currentTime)
{
    // list<string> Scalars;
    // Scalars.push_back("NormU");
    // Scalars.push_back("NormOm");
    // Scalars.push_back("CoordNumb");
    std::ostringstream ossCN;
    ossCN << m_ParaviewCycleNumber;

    // Obstacles
    std::string obsFileName
        = m_ParaviewFilename + "_Obstacles_T" + ossCN.str() + ".vtu";
    std::string obsFileNamePath = m_ParaviewFilename_dir + "/" + obsFileName;
    m_Paraview_saveObstacles_pvd << "<DataSet timestep=\"" << currentTime
                                 << "\" " << "group=\"\" part=\"0\" file=\""
                                 << obsFileName << "\"/>\n";

    ofstream f(
        (m_ParaviewFilename_dir + "/" + m_ParaviewFilename + "_Obstacles.pvd")
            .c_str(),
        ios::out);
    f << m_Paraview_saveObstacles_pvd.str();
    f << "</Collection>" << endl;
    f << "</VTKFile>" << endl;
    f.close();
    writeObstacles_Paraview(obstacleRB, cm, obsFileNamePath);

    // Particles
    std::string parFileName
        = m_ParaviewFilename + "_Particles_T" + ossCN.str() + ".vtu";
    std::string parFileNamePath = m_ParaviewFilename_dir + "/" + parFileName;
    *m_Paraview_saveParticles_pvd[0] << "<DataSet timestep=\"" << currentTime
                                     << "\" " << "group=\"\" part=\"0\" file=\""
                                     << parFileName << "\"/>\n";

    ofstream g(
        (m_ParaviewFilename_dir + "/" + m_ParaviewFilename + "_Particles.pvd")
            .c_str(),
        ios::out);
    g << m_Paraview_saveParticles_pvd[0]->str();
    g << "</Collection>" << endl;
    g << "</VTKFile>" << endl;
    g.close();

    writeParticles_Paraview(particleRB, cm, parFileNamePath);
    m_ParaviewCycleNumber++;
}

// ----------------------------------------------------------------------------
// Finalizes writing data
template <typename T>
__HOST__ void ParaviewPostProcessingWriter<T>::PostProcessing_end()
{
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class ParaviewPostProcessingWriter<float>;
template class ParaviewPostProcessingWriter<double>;