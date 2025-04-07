#ifndef _GRAINSMISC_HH_
#define _GRAINSMISC_HH_

#include "Basic.hh"
#include "Vector3.hh"

// =============================================================================
/** @brief Miscellaneous functionalities (mostly low-level) for Grains.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
template <typename T>
class GrainsMisc
{
public:
    /** @name Members */
    //@{
    /** \brief main Grains directory */
    static std::string m_grainsHome;
    /** \brief output of system command */
    static int m_returnSysCmd;
    //@}

    /** @name Methods */
    //@{
    /** @brief Writes a real number with a prescribed number of digits in a
        string
        @param figure the float number
        @param size number of digits */
    __HOST__
    static std::string realToString(T const& figure, int const& size);

    /** @brief Writes a float number with a prescribed format and a 
        prescribed number of digits after the decimal point in a string
        @param format the format
        @param digits number of digits after the decimal point
        @param number the float number */
    __HOST__
    static std::string realToString(std::ios_base::fmtflags format, int digits, T const& number);

    /** @brief Writes a vector3 object in a string
        @param vec the vector3 object */
    __HOST__
    static std::string Vector3ToString(Vector3<T> const& vec);

    /** @brief Writes a message to stdout
        @param message the output message
        @param numShift the number of shift characters at the beginning
        @param nextLine if going to the next line is required */
    __HOST__
    static void cout(std::string message, int numShift = 0, bool nextLine = false);
    //@}
};

typedef GrainsMisc<float>  GrainsMiscF;
typedef GrainsMisc<double> GrainsMiscD;

#endif