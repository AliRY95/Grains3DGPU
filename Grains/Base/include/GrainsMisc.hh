#ifndef _GRAINSMISC_HH_
#define _GRAINSMISC_HH_

#include "Basic.hh"


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
        static std::string realToString( T const& figure, 
                                         int const& size );

        /** @brief Writes a float number with a prescribed format and a 
        prescribed number of digits after the decimal point in a string
        @param format the format
        @param digits number of digits after the decimal point
        @param number the float number */
        static std::string realToString( std::ios_base::fmtflags format, 
                                         int digits,
                                         T const& number );
        //@}
};


typedef GrainsMisc<float> GrainsMiscF;
typedef GrainsMisc<double> GrainsMiscD;


#endif