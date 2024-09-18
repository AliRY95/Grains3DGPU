#include <sys/types.h>
#include <unistd.h>
#include "GrainsMisc.hh"


// -----------------------------------------------------------------------------
// Static variables
template <typename T> 
std::string GrainsMisc<T>::m_grainsHome = ".";
template <typename T>
int GrainsMisc<T>::m_returnSysCmd = 0;




// -----------------------------------------------------------------------------
// Writes a real number with a given number of digits
template <typename T>
__HOST__
std::string GrainsMisc<T>::realToString( T const& figure, 
                                   		 int const& size )
{
	std::ostringstream oss;
	oss.width( size );
	oss << std::left << figure;
	return ( oss.str() );
}




// -----------------------------------------------------------------------------
// Writes a float number with a prescribed format and a prescribed
// number of digits after the decimal point
template <typename T>
__HOST__
std::string GrainsMisc<T>::realToString( std::ios_base::fmtflags format, 
										 int digits,
      									 T const& number )
{
	std::ostringstream oss;
	if ( number != T( 0 ) )
	{
		oss.setf( format, std::ios::floatfield );
		oss.precision( digits );
	}
	oss << number;
	return ( oss.str() );
}




// // -----------------------------------------------------------------------------
// // Writes an integer in a string
// string GrainsExec::intToString( int const& figure )
// {
//   ostringstream oss;
//   oss << figure;

//   return ( oss.str() );
// }




// // ----------------------------------------------------------------------------
// // Returns memory used by this process
// template <typename T>
// __HOST__
// size_t GrainsMisc<T>::used_memory( void )
// {
//   ostringstream os ;
//   string word ;
//   size_t result = 0 ;

//   os << "/proc/" << getpid() << "/status" ;

//   ifstream in( os.str().c_str() ) ;
//   if( !in )
//     cout << "GrainsExec::used_memory : unable to open " << os.str() << endl ;
//   else
//   {
//     while( !in.eof() )
//     {
//       in >> word ;
//       if( word == "VmSize:" )
//       {
//         in >> result ;
//         in >> word ;
//         if( !( word == "kB" ) )
//           cout << "GrainsExec::used_memory : Unit is " << word << endl ;
//         result *= 1000 ;
//         break ;
//       }
//     }
//     in.close() ;
//   }

//   return ( result ) ;
// }




// // ----------------------------------------------------------------------------
// // Writes memory used by this process in a stream
// void GrainsExec::display_memory( ostream& os, size_t memory )
// {
//   static size_t const mo = 1024*1024 ;
//   static size_t const go = 1024*1024*1024 ;

//   if( memory > go )
//     os << ( (double) memory )/go << " Go" << std::flush;
//   else if( memory > mo )
//     os << ( (double) memory )/mo << " Mo" << std::flush ;
//   else os << memory << " octets" << std::flush ;
// }




// // ----------------------------------------------------------------------------
// // Returns a random rotation matrix
// Matrix GrainsExec::RandomRotationMatrix( size_t dim )
// {
//   Matrix rotation;

//   double angleZ = 2. * PI * (double)rand() / RAND_MAX;
//   Matrix rZ( cos(angleZ), -sin(angleZ), 0.,
//   	sin(angleZ), cos(angleZ), 0.,
// 	0., 0., 1. );

//   if ( dim == 3 )
//   {
//     double angleX = 2. * PI * (double)rand() / RAND_MAX;
//     Matrix rX( 1., 0., 0.,
//    	0., cos(angleX), -sin(angleX),
// 	0., sin(angleX), cos(angleX) );

//     double angleY = 2. * PI * (double)rand() / RAND_MAX;
//     Matrix rY( cos(angleY), 0., sin(angleY),
//    	0., 1., 0.,
// 	-sin(angleY), 0., cos(angleY) );
//     Matrix tmp = rY * rZ;
//     rotation = rX * tmp;
//   }
//   else rotation = rZ;

//   return ( rotation );
// }




// // ----------------------------------------------------------------------------
// // Returns a random unit vector
// Vector3 GrainsExec::RandomUnitVector( size_t dim )
// {
//   Vector3 vec;

//   vec[X] = 2. * (double)rand() / RAND_MAX - 1.;
//   vec[Y] = 2. * (double)rand() / RAND_MAX - 1.;
//   if ( dim == 3 )
//     vec[Z] = 2. * (double)rand() / RAND_MAX - 1.;
//   else
//     vec[Z] = 0.;

//   vec.normalize();

//   return( vec );
// }




// -----------------------------------------------------------------------------
// Explicit instantiation
template class GrainsMisc<float>;
template class GrainsMisc<double>;
