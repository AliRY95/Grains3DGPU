#include "ParaviewPostProcessingWriter.hh"
#include "GrainsUtils.hh"

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
                                      const uint         numObstacles,
                                      const std::string& obsFilename)
{
    ofstream f((m_ParaviewFilename_dir + "/" + obsFilename).c_str(), ios::out);
    list<SimpleObstacle*>::const_iterator il;

    f << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" "
      << "byte_order=\"LittleEndian\" ";
    f << ">" << endl;
    f << "<UnstructuredGrid>" << endl;
    int nbpts = 0, nbcells = 0, i;
    for(uint i = 0; i < numObstacles; ++i)
    {
        nbpts += obstacleRB[i]->numberOfPoints_PARAVIEW();
        nbcells += obstacleRB[i]->numberOfCells_PARAVIEW();
    }
    f << "<Piece NumberOfPoints=\"" << nbpts << "\"" << " NumberOfCells=\""
      << nbcells << "\">" << endl;
    f << "<Points>" << endl;
    f << "<DataArray type=\"Float32\" NumberOfComponents=\"3\" ";
    f << "format=\"ascii\">";
    f << endl;
    for(uint i = 0; i < numObstacles; ++i)
        obstacleRB[i]->write_polygonsPts_PARAVIEW(f);
    f << "</DataArray>" << endl;
    f << "</Points>" << endl;

    list<int>           connectivity, offsets, cellstype;
    list<int>::iterator ii;
    int                 firstpoint_globalnumber = 0, last_offset = 0;
    for(uint i = 0; i < numObstacles; ++i)
        obstacleRB[i]->writeConnection_PARAVIEW(connectivity,
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
        double indic = obstacleRB[i]->getIndicator();
        int    nc    = obstacleRB[i]->numberOfCells_PARAVIEW();
        for(i = 0; i < nc; ++i)
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
__HOST__ void writeParticles_Paraview(std::vector<RigidBody<T, T>> const* rb,
                                      std::vector<uint> const* rigidBodyID,
                                      std::vector<Transform3<T>> const* t,
                                      std::vector<Kinematics<T>> const* k)
{
    list<Particle*>::const_iterator particle;
    uint                            numParticles = rigidBodyID->size();

    std::ofstream f((m_ParaviewFilename_dir + "/" + partFilename).c_str(),
                    ios::out);

    f << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" "
      << "byte_order=\"LittleEndian\" ";
    if(m_binary)
        f << "compressor=\"vtkZLibDataCompressor\"";
    f << ">" << endl;
    f << "<UnstructuredGrid>" << endl;
    int nbpts = 0, nbcells = 0, i;

    for(uint i = 0; i < numParticles; i++)
    {
        nbpts += rb[i].getConvex()->numberOfPoints_PARAVIEW();
        nbcells += rb[i].getConvex()->numberOfCells_PARAVIEW();
    }
    f << "<Piece NumberOfPoints=\"" << nbpts << "\"" << " NumberOfCells=\""
      << nbcells << "\">" << endl;

    f << "<Points>" << endl;
    f << "<DataArray type=\"Float32\" NumberOfComponents=\"3\" ";
    f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    start_output_binary(sizeof_Float32, 3 * nbpts);
    for(uint i = 0; i < numParticles; i++)
    {
        ppp = (*particle)->get_polygonsPts_PARAVIEW(PPTranslation);
        for(ilpp = ppp.begin(); ilpp != ppp.end(); ilpp++)
            for(int comp = 0; comp < 3; ++comp)
                write_double_binary((*ilpp)[comp]);
    }
    flush_binary(f, "writeParticles_Paraview/Points");
    f << "</DataArray>" << endl;
    f << "</Points>" << endl;

    list<int>           connectivity, offsets, cellstype;
    list<int>::iterator ii;
    int                 firstpoint_globalnumber = 0, last_offset = 0;
    for(particle = particles->begin(); particle != particles->end(); particle++)
        if((*particle)->getActivity() == COMPUTE
           && ((*particle)->getTag() != 2 || forceForAllTag))
            (*particle)->write_polygonsStr_PARAVIEW(connectivity,
                                                    offsets,
                                                    cellstype,
                                                    firstpoint_globalnumber,
                                                    last_offset);
    f << "<Cells>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"connectivity\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
    {
        start_output_binary(sizeof_Int32, int(connectivity.size()));
        for(ii = connectivity.begin(); ii != connectivity.end(); ii++)
            write_int_binary(*ii);
        flush_binary(f, "writeParticles_Paraview/connectivity");
    }
    else
    {
        for(ii = connectivity.begin(); ii != connectivity.end(); ii++)
            f << *ii << " ";
        f << endl;
    }
    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"offsets\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
    {
        start_output_binary(sizeof_Int32, int(offsets.size()));
        for(ii = offsets.begin(); ii != offsets.end(); ii++)
            write_int_binary(*ii);
        flush_binary(f, "writeParticles_Paraview/offsets");
    }
    else
    {
        for(ii = offsets.begin(); ii != offsets.end(); ii++)
            f << *ii << " ";
        f << endl;
    }
    f << "</DataArray>" << endl;
    f << "<DataArray type=\"Int32\" Name=\"types\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
    {
        start_output_binary(sizeof_Int32, int(cellstype.size()));
        for(ii = cellstype.begin(); ii != cellstype.end(); ii++)
            write_int_binary(*ii);
        flush_binary(f, "writeParticles_Paraview/types");
    }
    else
    {
        for(ii = cellstype.begin(); ii != cellstype.end(); ii++)
            f << *ii << " ";
        f << endl;
    }
    f << "</DataArray>" << endl;
    f << "</Cells>" << endl;

    f << "<CellData Scalars=\"NormU,NormOm,CoordNumb\">" << endl;

    f << "<DataArray type=\"Float32\" Name=\"NormU\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
        start_output_binary(sizeof_Float32, int(cellstype.size()));
    for(particle = particles->begin(); particle != particles->end(); particle++)
        if((*particle)->getActivity() == COMPUTE
           && ((*particle)->getTag() != 2 || forceForAllTag))
        {
            double normU = Norm(*(*particle)->getTranslationalVelocity());
            int    nc    = (*particle)->numberOfCells_PARAVIEW();
            if(m_binary)
                for(i = 0; i < nc; ++i)
                    write_double_binary(normU);
            else
                for(i = 0; i < nc; ++i)
                    f << normU << " ";
        }
    if(m_binary)
        flush_binary(f, "writeParticles_Paraview/NormU");
    else
        f << endl;
    f << "</DataArray>" << endl;

    f << "<DataArray type=\"Float32\" Name=\"NormOm\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
        start_output_binary(sizeof_Float32, int(cellstype.size()));
    for(particle = particles->begin(); particle != particles->end(); particle++)
        if((*particle)->getActivity() == COMPUTE
           && ((*particle)->getTag() != 2 || forceForAllTag))
        {
            double normOm = Norm(*(*particle)->getAngularVelocity());
            int    nc     = (*particle)->numberOfCells_PARAVIEW();
            if(m_binary)
                for(i = 0; i < nc; ++i)
                    write_double_binary(normOm);
            else
                for(i = 0; i < nc; ++i)
                    f << normOm << " ";
        }
    if(m_binary)
        flush_binary(f, "writeParticles_Paraview/NormOm");
    else
        f << endl;
    f << "</DataArray>" << endl;

    f << "<DataArray type=\"Float32\" Name=\"CoordNumb\" ";
    if(m_binary)
        f << "offset=\"" << OFFSET << "\" format=\"appended\">";
    else
        f << "format=\"ascii\">" << endl;
    if(m_binary)
        start_output_binary(sizeof_Float32, int(cellstype.size()));
    for(particle = particles->begin(); particle != particles->end(); particle++)
        if((*particle)->getActivity() == COMPUTE
           && ((*particle)->getTag() != 2 || forceForAllTag))
        {
            double coordNum = double((*particle)->getCoordinationNumber());
            int    nc       = (*particle)->numberOfCells_PARAVIEW();
            if(m_binary)
                for(i = 0; i < nc; ++i)
                    write_double_binary(coordNum);
            else
                for(i = 0; i < nc; ++i)
                    f << coordNum << " ";
        }
    if(m_binary)
        flush_binary(f, "writeParticles_Paraview/CoordNumb");
    else
        f << endl;
    f << "</DataArray>" << endl;
    f << "</CellData>" << endl;
    f << "</Piece>" << endl;

    f << "</UnstructuredGrid>" << endl;
    if(m_binary)
    {
        f << "<AppendedData encoding=\"raw\">" << endl << "    _";
        f.write(BUFFER, OFFSET);
        delete[] BUFFER;
        BUFFER    = 0;
        ALLOCATED = 0;
        OFFSET    = 0;
        f << endl << "</AppendedData>" << endl;
    }
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
    GoutWI(12, "Writing mode =", (m_binary ? "Binary" : "Text"));
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
    ComponentManager<T> const*    cm,
    T                             currentTime)
{
    const uint   numObstacles = cm->getNumberOfObstacles();
    const uint   numParticles = cm->getNumberOfParticles();
    list<string> Scalars;
    Scalars.push_back("NormU");
    Scalars.push_back("NormOm");
    Scalars.push_back("CoordNumb");
    std::ostringstream ossCN, ossRK;
    ossCN << m_ParaviewCycleNumber;
    ossRK << m_rank;

    // Obstacles
    updateObstaclesIndicator(time, dt, obstacle);
    list<SimpleObstacle*> allObstacles = obstacle->getObstacles();
    string                obsFilename
        = m_ParaviewFilename + "_Obstacles_T" + ossCN.str() + ".vtu";
    m_Paraview_saveObstacles_pvd << "<DataSet timestep=\"" << currentTime
                                 << "\" " << "group=\"\" part=\"0\" file=\""
                                 << obsFilename << "\"/>\n";

    ofstream f(
        (m_ParaviewFilename_dir + "/" + m_ParaviewFilename + "_Obstacles.pvd")
            .c_str(),
        ios::out);
    f << m_Paraview_saveObstacles_pvd.str();
    f << "</Collection>" << endl;
    f << "</VTKFile>" << endl;
    f.close();
    writeObstacles_Paraview(obstacleRB, numObstacles, obsFilename);

    // Particles
    string partFilename = m_ParaviewFilename + "_Particles_T" + ossCN.str();
    *m_Paraview_saveParticles_pvd[0]
        << "<DataSet timestep=\"" << time << "\" "
        << "group=\"\" part=\"0\" file=\"" << partFilename
        << (m_nprocs > 1 && !m_mpiio_singlefile ? ".pvtu\"/>" : ".vtu\"/>")
        << endl;

    ofstream g(
        (m_ParaviewFilename_dir + "/" + m_ParaviewFilename + "_Particles.pvd")
            .c_str(),
        ios::out);
    g << m_Paraview_saveParticles_pvd[0]->str();
    g << "</Collection>" << endl;
    g << "</VTKFile>" << endl;
    g.close();

    if(m_nprocs > 1 && !m_mpiio_singlefile)
    {
        if(nbParticleTypes == 1
           && (*referenceParticles)[0]->getRigidBody()->getConvex()->isSphere()
           && !GrainsExec::m_SphereAsPolyParaview)
        {
            list<string> ptVec;
            ptVec.push_back("Orientation");
            writePVTU_Paraview(partFilename,
                               &ptVec,
                               &Scalars,
                               &empty_string_list);
        }
        else
            writePVTU_Paraview(partFilename,
                               &empty_string_list,
                               &empty_string_list,
                               &Scalars);
    }

    // VTU files
    if(nbParticleTypes == 1
       && (*referenceParticles)[0]->getRigidBody()->getConvex()->isSphere()
       && !GrainsExec::m_SphereAsPolyParaview)
    {
        if(m_mpiio_singlefile && m_nprocs > 1)
        {
            if(m_binary)
                writeSpheres_Paraview_MPIIO_binary(
                    particles,
                    partFilename + ".vtu",
                    false,
                    PostProcessingWriter::m_bPPWindow[m_rank]);
            else
                writeSpheres_Paraview_MPIIO_text(
                    particles,
                    partFilename + ".vtu",
                    false,
                    PostProcessingWriter::m_bPPWindow[m_rank]);
        }
        else
            writeSpheres_Paraview(
                particles,
                partFilename + (m_nprocs > 1 ? "_" + ossRK.str() : "") + ".vtu",
                false,
                PostProcessingWriter::m_bPPWindow[m_rank]);
    }
    else
    {
        writeParticles_Paraview(
            particles,
            partFilename + (m_nprocs > 1 ? "_" + ossRK.str() : "") + ".vtu",
            false,
            PostProcessingWriter::m_bPPWindow[m_rank]);
    }
    m_ParaviewCycleNumber++;
}

// ----------------------------------------------------------------------------
// Finalizes writing data
template <typename T>
__HOST__ void ParaviewPostProcessingWriter<T>::PostProcessing_end()
{
}