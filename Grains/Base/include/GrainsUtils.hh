#ifndef _GRAINSUTILS_HH_
#define _GRAINSUTILS_HH_

#include "Vector3.hh"

// =============================================================================
/** @brief Miscellaneous functionalities (mostly low-level) for Grains.

    @author A.Yazdani - 2025 - Construction */
// =============================================================================
/** @name Miscellaneous functions and utilities for Grains */
//@{
/** @brief Writes a real number with a prescribed number of digits in a string
@param figure the float number
@param size number of digits */
template <typename T>
__HOST__ static constexpr INLINE std::string realToString(const T&  figure,
                                                          const int size)
{
    std::ostringstream oss;
    oss.width(size);
    oss << std::left << figure;
    return (oss.str());
}

// -----------------------------------------------------------------------------
/** @brief Writes a float number with a prescribed format and a prescribed 
number of digits after the decimal point in a string
@param format the format
@param digits number of digits after the decimal point
@param number the float number */
template <typename T>
__HOST__ static constexpr INLINE std::string realToString(
    std::ios_base::fmtflags format, const int digits, const T& number)
{
    std::ostringstream oss;
    if(number != T(0))
    {
        oss.setf(format, std::ios::floatfield);
        oss.precision(digits);
    }
    oss << number;
    return (oss.str());
}

// -----------------------------------------------------------------------------
/** @brief Writes a vector3 object in a string
@param vec the vector3 object */
template <typename T>
__HOST__ static constexpr INLINE std::string
                                 Vector3ToString(const Vector3<T>& vec)
{
    std::ostringstream oss;
    oss << vec;
    return ("[" + oss.str() + "]");
}

// -----------------------------------------------------------------------------
/** @brief Writes a message to stdout
@param args the output messages */
template <typename... Args>
__HOST__ static constexpr INLINE void Gout(const Args&... args)
{
    ((std::cout << args << " "), ...);
    std::cout << std::endl;
}

// -----------------------------------------------------------------------------
/** @brief Writes a message to stdout with Indent (WI)
 @param numShift the number of shift characters at the beginning
@param nextLine if going to the next line is required
@param args the output messages */
template <typename... Args>
__HOST__ INLINE void GoutWI(const int numShift, const Args&... args)
{
    // indent
    auto shift = [](int n) { return std::string(n, ' '); };

    std::cout << shift(numShift);
    ((std::cout << args << " "), ...);
    std::cout << std::endl;
}

#endif