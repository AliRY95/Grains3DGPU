#include "RigidBody.hh"
#include "Sphere.hh"
#include "Box.hh"
#include "Cylinder.hh"
#include "Cone.hh"
#include "Superquadric.hh"
#include "Rectangle.hh"
#include "Convex.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// GPU kernel to construct the rigidbody on device.
// This is mandatory as we cannot access device memory addresses on the host
// So, we pass a device memory address to a kernel.
// Memory address is then populated within the kernel.
// This kernel is not declared in any header file since we directly use it below
// It helps to NOT explicitly instantiate it.
template <typename T, typename U, typename... Arguments>
__GLOBAL__
void createRigidBodyKernel( RigidBody<T, U>** rb, 
                            unsigned int index,
                            T crustThickness,
                            unsigned int material,
                            T density,
                            ConvexType convexType,
                            Arguments... args )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if ( tid > 0 ) 
        return;
    
    Convex<T>* convex = nullptr;
    if constexpr ( sizeof...( args ) == 1 )
    {
        if ( convexType == SPHERE ) 
            convex = new Sphere<T>( args... );
    }
    else if constexpr ( sizeof...( args ) == 2 ) 
    {
        if ( convexType == CYLINDER ) 
            convex = new Cylinder<T>( args... );
        if ( convexType == CONE ) 
            convex = new Cone<T>( args... );
        if ( convexType == RECTANGLE ) 
            convex = new Rectangle<T>( args... );
    }
    else if constexpr ( sizeof...( args ) == 3 ) 
    {
        if ( convexType == BOX ) 
            convex = new Box<T>( args... );
    }
    else if constexpr ( sizeof...( args ) == 5 )
    {
        if ( convexType == SUPERQUADRIC ) 
            convex = new Superquadric<T>( args... );
    }

    if ( !convex )
    {
        printf( "Convex is not created! Aborting Grains!\n" );
    }

    rb[index] = new RigidBody<T, U>( convex, 
                                     crustThickness, 
                                     material, 
                                     density );
}




/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
// Constructs a RigidBody object on device identical to a RigidBody object on 
// the host memory.
// It is assumed that appropriate memory is allocated to d_rb.
template <typename T, typename U>
__HOST__
void RigidBodyCopyHostToDevice( RigidBody<T, U>** h_rb,
                                RigidBody<T, U>** d_rb,
                                int numRigidBodies )
{
    for ( int index = 0; index < numRigidBodies; index++ )
    {
        // Extracting info from the host side object
        Convex<T>* convex = h_rb[index]->getConvex();
        ConvexType cvxType = convex->getConvexType();
        T ct = h_rb[index]->getCrustThickness();
        unsigned int material = h_rb[index]->getMaterial();
        // We also need the density to calculate the mass of the rigid body.
        // However, it is not available here. So, we manually compute it:
        T density = h_rb[index]->getMass() / h_rb[index]->getVolume();

        if ( cvxType == SPHERE )
        {
            Sphere<T>* c = dynamic_cast<Sphere<T>*>( convex );
            T r = c->getRadius();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             SPHERE, 
                                             r );
        }
        else if ( cvxType == BOX )
        {
            Box<T>* c = dynamic_cast<Box<T>*>( convex );
            Vector3<T> L = c->getExtent();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             BOX, 
                                             L[X], L[Y], L[Z] );
        }
        else if ( cvxType == CYLINDER )
        {
            Cylinder<T>* c = dynamic_cast<Cylinder<T>*>( convex );
            T r = c->getRadius();
            T h = c->getHeight();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             CYLINDER,
                                             r, h );
        }
        else if ( cvxType == CONE )
        {
            Cone<T>* c = dynamic_cast<Cone<T>*>( convex );
            T r = c->getRadius();
            T h = c->getHeight();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             CONE, 
                                             r, h );
        }
        else if ( cvxType == SUPERQUADRIC )
        {
            Superquadric<T>* c = dynamic_cast<Superquadric<T>*>( convex );
            Vector3<T> L = c->getExtent();
            Vector3<T> N = c->getExponent();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             SUPERQUADRIC, 
                                             L[X], L[Y], L[Z], N[X], N[Y] );
        }
        else if ( cvxType == RECTANGLE )
        {
            Rectangle<T>* c = dynamic_cast<Rectangle<T>*>( convex );
            Vector3<T> L = c->getExtent();
            createRigidBodyKernel<<<1, 1>>>( d_rb, index, ct, material, density, 
                                             RECTANGLE, 
                                             L[X], L[Y] );
        }
        else
        {
            cout << "Convex type is not implemented for GPU! Aborting Grains!" 
                << endl;
            exit( 1 );
        }
    }
    cudaDeviceSynchronize();
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T, U ) \
template                                                                       \
__HOST__                                                                       \
void RigidBodyCopyHostToDevice( RigidBody<T, U>** h_rb,                        \
                                RigidBody<T, U>** d_rb,                        \
                                int numRigidBodies );
X( float, float )
X( double, float )
X( double, double )
#undef X