#include "RawDataPostProcessingWriter.hh"
#include "GrainsUtils.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOST__ RawDataPostProcessingWriter<T>::RawDataPostProcessingWriter()
{
}

// -----------------------------------------------------------------------------
// Constructor with XML node
template <typename T>
__HOST__
    RawDataPostProcessingWriter<T>::RawDataPostProcessingWriter(DOMNode* dn)
    : m_ndigits(6)
{
    m_directory = ReaderXML::getNodeAttr_String(dn, "Directory");
    m_rootName  = ReaderXML::getNodeAttr_String(dn, "RootName");
    GoutWI(9, "Type = RawData");
    GoutWI(12, "Output file directory name =", m_directory);
    GoutWI(12, "Output file root name =", m_rootName);
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__ RawDataPostProcessingWriter<T>::~RawDataPostProcessingWriter()
{
}

// -----------------------------------------------------------------------------
// Gets the post-processing writer type
template <typename T>
__HOST__ PostProcessingWriterType
    RawDataPostProcessingWriter<T>::getPostProcessingWriterType() const
{
    return (RAW);
}

// ----------------------------------------------------------------------------
// Removes post-processing files already in the directory
template <typename T>
__HOST__ void RawDataPostProcessingWriter<T>::clearPostProcessingFiles() const
{
    std::string              directory = m_directory;
    std::vector<std::string> patternsStr
        = {"^" + m_rootName + R"(_position_.*\.dat$)",
           "^" + m_rootName + R"(_translational_velocity_.*\.dat$)",
           "^" + m_rootName + R"(_angular_velocity_.*\.dat$)",
           "^" + m_rootName + R"(_coordinationNumber\.dat$)",
           "^" + m_rootName + R"(_particleType\.dat$)"};
    std::vector<std::regex> patternsReg;
    for(const auto& pattern : patternsStr)
        patternsReg.push_back(std::regex(pattern));
    PostProcessingWriter<T>::clearPostProcessingFiles(directory, patternsReg);
}

// -----------------------------------------------------------------------------
// Initializes the post-processing writer
template <typename T>
__HOST__ void RawDataPostProcessingWriter<T>::PostProcessing_start()
{
    // Open files
    ios_base::openmode mode = ios::app;
    mode                    = ios::out;
    clearPostProcessingFiles();
    prepareResultFiles(mode);
}

// -----------------------------------------------------------------------------
// Writes data -- Particles come first, followed by obtacles
template <typename T>
__HOST__ void RawDataPostProcessingWriter<T>::PostProcessing(
    RigidBody<T, T> const* const* particleRB,
    RigidBody<T, T> const* const* obstacleRB,
    const ComponentManager<T>*    cm,
    const T                       currentTime)
{
    // Particles
    uint                       numParticles = cm->getNumberOfParticles();
    std::vector<uint>          rbParticle   = cm->getRigidBodyId();
    std::vector<Transform3<T>> tParticle    = cm->getTransform();
    std::vector<Kinematics<T>> kParticle    = cm->getVelocity();
    // Obstacles
    uint                       numObstacles = cm->getNumberOfObstacles();
    std::vector<uint>          rbObstacle   = cm->getRigidBodyIdObstacles();
    std::vector<Transform3<T>> tObstacle    = cm->getTransformObstacles();
    // TODO:
    std::vector<Kinematics<T>> kObstacle(numObstacles);
    // Aux. variables
    Vector3<T>  centre;
    Vector3<T>  velT;
    Vector3<T>  velR;
    uint        type;
    std::string fileName(m_directory + "/" + m_rootName);
    m_particle_class.open((fileName + "_particleType.dat").c_str(), ios::out);

    // Writing current time at the beginning of each line
    std::string stime = realToString(ios::scientific, 6, currentTime);
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
    for(size_t i = 0; i < numParticles; i++)
    {
        // Center of mass position
        centre = tParticle[i].getOrigin();
        m_gc_coordinates_x << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[X]);
        m_gc_coordinates_y << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[Y]);
        m_gc_coordinates_z << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[Z]);

        // Translational velocity
        velT = kParticle[i].getTranslationalComponent();
        m_translational_velocity_x
            << " " << realToString(ios::scientific, m_ndigits, velT[X]);
        m_translational_velocity_y
            << " " << realToString(ios::scientific, m_ndigits, velT[Y]);
        m_translational_velocity_z
            << " " << realToString(ios::scientific, m_ndigits, velT[Z]);

        // Angular velocity
        velR = kParticle[i].getAngularComponent();
        m_angular_velocity_x
            << " " << realToString(ios::scientific, m_ndigits, velR[X]);
        m_angular_velocity_y
            << " " << realToString(ios::scientific, m_ndigits, velR[Y]);
        m_angular_velocity_z
            << " " << realToString(ios::scientific, m_ndigits, velR[Z]);

        // // Number of contacts
        // m_coordination_number << " " << pp->getCoordinationNumber();

        // Particle type
        type = particleRB[rbParticle[i]]->getConvex()->getConvexType();
        // m_particle_class << type << " " ;
    }

    // Writing obstacles data
    for(size_t i = 0; i < numObstacles; i++)
    {
        // Center of mass position
        centre = tObstacle[i].getOrigin();
        m_gc_coordinates_x << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[X]);
        m_gc_coordinates_y << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[Y]);
        m_gc_coordinates_z << " "
                           << realToString(ios::scientific,
                                           m_ndigits,
                                           centre[Z]);

        // Translational velocity
        velT = kObstacle[i].getTranslationalComponent();
        m_translational_velocity_x
            << " " << realToString(ios::scientific, m_ndigits, velT[X]);
        m_translational_velocity_y
            << " " << realToString(ios::scientific, m_ndigits, velT[Y]);
        m_translational_velocity_z
            << " " << realToString(ios::scientific, m_ndigits, velT[Z]);

        // Angular velocity
        velR = kObstacle[i].getAngularComponent();
        m_angular_velocity_x
            << " " << realToString(ios::scientific, m_ndigits, velR[X]);
        m_angular_velocity_y
            << " " << realToString(ios::scientific, m_ndigits, velR[Y]);
        m_angular_velocity_z
            << " " << realToString(ios::scientific, m_ndigits, velR[Z]);

        // // Number of contacts
        // m_coordination_number << " " << pp->getCoordinationNumber();

        // Particle type
        type = obstacleRB[rbObstacle[i]]->getConvex()->getConvexType();
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
__HOST__ void RawDataPostProcessingWriter<T>::PostProcessing_end()
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
__HOST__ void
    RawDataPostProcessingWriter<T>::prepareResultFiles(ios_base::openmode mode)
{
    std::string fileName(m_directory + "/" + m_rootName);
    string      file;
    file = fileName + "_position_x.dat";
    m_gc_coordinates_x.open(file.c_str(), mode);
    file = fileName + "_position_y.dat";
    m_gc_coordinates_y.open(file.c_str(), mode);
    file = fileName + "_position_z.dat";
    m_gc_coordinates_z.open(file.c_str(), mode);

    file = fileName + "_translational_velocity_x.dat";
    m_translational_velocity_x.open(file.c_str(), mode);
    file = fileName + "_translational_velocity_y.dat";
    m_translational_velocity_y.open(file.c_str(), mode);
    file = fileName + "_translational_velocity_z.dat";
    m_translational_velocity_z.open(file.c_str(), mode);

    file = fileName + "_angular_velocity_x.dat";
    m_angular_velocity_x.open(file.c_str(), mode);
    file = fileName + "_angular_velocity_y.dat";
    m_angular_velocity_y.open(file.c_str(), mode);
    file = fileName + "_angular_velocity_z.dat";
    m_angular_velocity_z.open(file.c_str(), mode);

    file = fileName + "_coordinationNumber.dat";
    m_coordination_number.open(file.c_str(), mode);
}

// -----------------------------------------------------------------------------
// Explicit instantiation
template class RawDataPostProcessingWriter<float>;
template class RawDataPostProcessingWriter<double>;