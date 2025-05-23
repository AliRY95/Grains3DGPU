#include "Convex.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__ Convex<T>::Convex()
{
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ Convex<T>::~Convex()
{
}

// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOST__ void
    Convex<T>::writePoints_PARAVIEW(std::ostream&        f,
                                    const Transform3<T>& transform,
                                    const Vector3<T>*    translation) const
{
    std::list<Vector3<T>> points = writePoints_PARAVIEW(transform, translation);

    for(auto& point : points)
    {
        f << point[X] << " " << point[Y] << " " << point[Z] << std::endl;
    }
}

// // ----------------------------------------------------------------------------
// // Input operator
// template <typename T>
// __HOST__
// std::istream& Convex<T>::operator >> ( std::istream& fileIn,
//                                        Convex<T>& convex )
// {
//   convex.readShape( fileIn );
//   return ( fileIn );
// }

// // ---------------------------------------------------------------------
// // Output operator
// template <typename T>
// __HOST__
// std::ostream& Convex<T>::operator << ( std::ostream& fileOut,
//                                        Convex<T> const& convex )
// {
//   convex.writeShape( fileOut );
//   return ( fileOut );
// }

// -----------------------------------------------------------------------------
// Explicit instantiation
template class Convex<float>;
template class Convex<double>;