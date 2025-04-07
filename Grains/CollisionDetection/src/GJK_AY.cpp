#include "GJK_AY.hh"
#include "MatrixMath.hh"

/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
template <typename T>
__HOSTDEVICE__ static INLINE unsigned int compareSigns(T a, T b)
{
    // Maybe there's a faster way to deal with this set of operations?
    return static_cast<unsigned int>(!((a > 0) ^ (b > 0)));
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void s1d(Vector3<T> const y[4], unsigned int& bits, T (&lambdas)[4])
{
    // Identify the appropriate indices
    bool         s1_set = false;
    unsigned int i1 = 0xffffffff, i2 = 0xffffffff;
    for(unsigned int i = 0; i < 4; ++i)
    {
        if(bits & (1 << i))
        {
            if(s1_set)
            {
                i2 = i;
                break;
            }
            else
            {
                i1     = i;
                s1_set = true;
            }
        }
    }

    // Calculate the signed volume of the simplex.
    Vector3<T>   t      = y[i2] - y[i1];
    unsigned int I      = 0;
    T            neg_tI = -t[0];

    if(fabs(t[1]) > fabs(neg_tI))
    {
        I      = 1;
        neg_tI = -t[1];
    }

    if(fabs(t[2]) > fabs(neg_tI))
    {
        I      = 2;
        neg_tI = -t[2];
    }

    T pI = (y[i2] * t) / norm2(t) * neg_tI + y[i2][I];

    // Identify the signed volume resulting from replacing each point by the
    // origin.
    T            C[2]                = {-y[i2][I] + pI, y[i1][I] - pI};
    unsigned int sign_comparisons[2] = {compareSigns(neg_tI, C[0]), compareSigns(neg_tI, C[1])};

    // If all signed volumes are identical, the origin lies inside the simplex.
    if(sign_comparisons[0] + sign_comparisons[1] == 2)
    {
        lambdas[i1] = C[0] / neg_tI;
        lambdas[i2] = C[1] / neg_tI;
    }
    else
    {
        // The point to retain is the one whose sign matches. In the
        // first case, the origin lies past the first point.
        if(sign_comparisons[0])
        {
            bits &= ~(1 << i2);
            lambdas[i1] = T(1);
        }
        else
        {
            bits &= ~(1 << i1);
            lambdas[i2] = T(1);
        }
    }
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void s2d(Vector3<T> const y[4], unsigned int& bits, T (&lambdas)[4])
{
    unsigned int counter = 0, point0_idx = 0, point1_idx = 0, point2_idx = 0;
    for(unsigned int i = 0; i < 4; ++i)
    {
        if(bits & (1 << i))
        {
            if(counter == 0)
                point0_idx = i;
            else if(counter == 1)
                point1_idx = i;
            else
                point2_idx = i;
            counter += 1;
        }
    }

    Vector3<T> n  = (y[point1_idx] - y[point0_idx]) ^ (y[point2_idx] - y[point0_idx]);
    Vector3<T> p0 = (y[point0_idx] * n / norm2(n)) * n;

    // Choose maximum area plane to project onto.
    // Make sure to store the *signed* area of the plane.
    // This loop is unrolled to save a few extra ops (assigning
    // an initial area of zero, an extra abs, etc)
    unsigned int idx_x  = 1;
    unsigned int idx_y  = 2;
    T            mu_max = (y[point1_idx][1] * y[point2_idx][2] + y[point0_idx][1] * y[point1_idx][2]
                + y[point2_idx][1] * y[point0_idx][2] - y[point1_idx][1] * y[point0_idx][2]
                - y[point2_idx][1] * y[point1_idx][2] - y[point0_idx][1] * y[point2_idx][2]);

    // This term is multiplied by -1.
    T mu = (y[point1_idx][2] * y[point0_idx][0] + y[point2_idx][2] * y[point1_idx][0]
            + y[point0_idx][2] * y[point2_idx][0] - y[point1_idx][2] * y[point2_idx][0]
            - y[point0_idx][2] * y[point1_idx][0] - y[point2_idx][2] * y[point0_idx][0]);
    if(fabs(mu) > fabs(mu_max))
    {
        mu_max = mu;
        idx_x  = 0;
    }

    mu = (y[point1_idx][0] * y[point2_idx][1] + y[point0_idx][0] * y[point1_idx][1]
          + y[point2_idx][0] * y[point0_idx][1] - y[point1_idx][0] * y[point0_idx][1]
          - y[point2_idx][0] * y[point1_idx][1] - y[point0_idx][0] * y[point2_idx][1]);
    if(fabs(mu) > fabs(mu_max))
    {
        mu_max = mu;
        idx_x  = 0;
        idx_y  = 1;
    }

    // Compute the signed areas of each of the simplices formed by replacing an
    // index with a projection of the origin onto the area in this plane
    T    C[3]                = {T(0)};
    bool sign_comparisons[3] = {false};

    C[0]                = (p0[idx_x] * y[point1_idx][idx_y] + p0[idx_y] * y[point2_idx][idx_x]
            + y[point1_idx][idx_x] * y[point2_idx][idx_y] - p0[idx_x] * y[point2_idx][idx_y]
            - p0[idx_y] * y[point1_idx][idx_x] - y[point2_idx][idx_x] * y[point1_idx][idx_y]);
    sign_comparisons[0] = compareSigns(mu_max, C[0]);

    C[1]                = (p0[idx_x] * y[point2_idx][idx_y] + p0[idx_y] * y[point0_idx][idx_x]
            + y[point2_idx][idx_x] * y[point0_idx][idx_y] - p0[idx_x] * y[point0_idx][idx_y]
            - p0[idx_y] * y[point2_idx][idx_x] - y[point0_idx][idx_x] * y[point2_idx][idx_y]);
    sign_comparisons[1] = compareSigns(mu_max, C[1]);

    C[2]                = (p0[idx_x] * y[point0_idx][idx_y] + p0[idx_y] * y[point1_idx][idx_x]
            + y[point0_idx][idx_x] * y[point1_idx][idx_y] - p0[idx_x] * y[point1_idx][idx_y]
            - p0[idx_y] * y[point0_idx][idx_x] - y[point1_idx][idx_x] * y[point0_idx][idx_y]);
    sign_comparisons[2] = compareSigns(mu_max, C[2]);

    if(sign_comparisons[0] + sign_comparisons[1] + sign_comparisons[2] == 3)
    {
        lambdas[point0_idx] = C[0] / mu_max;
        lambdas[point1_idx] = C[1] / mu_max;
        lambdas[point2_idx] = C[2] / mu_max;
    }
    else
    {
        T            d = T(100000);
        Vector3<T>   new_point;
        unsigned int new_bits = 0;
        for(unsigned int j = 0; j < 3; ++j)
        {
            if(!sign_comparisons[j])
            {
                unsigned int new_used = bits;
                // Test removal of the current point.
                if(j == 0)
                    new_used &= ~(1 << point0_idx);
                else if(j == 1)
                    new_used &= ~(1 << point1_idx);
                else
                    new_used &= ~(1 << point2_idx);

                T new_lambdas[4] = {T(0)};

                s1d(y, new_used, new_lambdas);
                // Consider resetting in place if possible.
                new_point[0] = 0;
                new_point[1] = 0;
                new_point[2] = 0;
                for(unsigned int i = 0; i < 4; ++i)
                {
                    if(new_used & (1 << i))
                        new_point += new_lambdas[i] * y[i];
                }
                T d_star = new_point * new_point;
                if(d_star < d)
                {
                    new_bits = new_used;
                    d        = d_star;
                    for(unsigned int i = 0; i < 4; ++i)
                        lambdas[i] = new_lambdas[i];
                }
            }
        }
        bits = new_bits;
    }
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void s3d(Vector3<T> const y[4], unsigned int& bits, T (&lambdas)[4])
{
    T C[4] = {0.};

    // Compute all minors and the total determinant of the matrix M,
    // which is the transpose of the y matrix with an extra row of
    // ones at the bottom. Since the indexing is nontrivial and the
    // array is small (and we can save on some negation), all the
    // computations are done directly rather than with a loop.
    // C[0] and C[2] are negated due to the (-1)^(i+j+1) prefactor,
    // where i is always 4 because we're expanding about the 4th row.
    C[0] = y[3][0] * y[2][1] * y[1][2] + y[2][0] * y[1][1] * y[3][2] + y[1][0] * y[3][1] * y[2][2]
           - y[1][0] * y[2][1] * y[3][2] - y[2][0] * y[3][1] * y[1][2]
           - y[3][0] * y[1][1] * y[2][2];
    C[1] = y[0][0] * y[2][1] * y[3][2] + y[2][0] * y[3][1] * y[0][2] + y[3][0] * y[0][1] * y[2][2]
           - y[3][0] * y[2][1] * y[0][2] - y[2][0] * y[0][1] * y[3][2]
           - y[0][0] * y[3][1] * y[2][2];
    C[2] = y[3][0] * y[1][1] * y[0][2] + y[1][0] * y[0][1] * y[3][2] + y[0][0] * y[3][1] * y[1][2]
           - y[0][0] * y[1][1] * y[3][2] - y[1][0] * y[3][1] * y[0][2]
           - y[3][0] * y[0][1] * y[1][2];
    C[3] = y[0][0] * y[1][1] * y[2][2] + y[1][0] * y[2][1] * y[0][2] + y[2][0] * y[0][1] * y[1][2]
           - y[2][0] * y[1][1] * y[0][2] - y[1][0] * y[0][1] * y[2][2]
           - y[0][0] * y[2][1] * y[1][2];
    T dM = C[0] + C[1] + C[2] + C[3];

    unsigned int sign_comparisons[4] = {0};
    sign_comparisons[0]              = compareSigns(dM, C[0]);
    sign_comparisons[1]              = compareSigns(dM, C[1]);
    sign_comparisons[2]              = compareSigns(dM, C[2]);
    sign_comparisons[3]              = compareSigns(dM, C[3]);

    if((sign_comparisons[0] + sign_comparisons[1] + sign_comparisons[2] + sign_comparisons[3]) == 4)
    {
        for(unsigned int i = 0; i < 4; ++i)
            lambdas[i] = C[i] / dM;
    }
    else
    {
        T            d = T(100000), d_star = T(0);
        Vector3<T>   new_point;
        unsigned int new_bits = 0;
        for(unsigned int j = 0; j < 4; ++j)
        {
            if(!sign_comparisons[j])
            {
                // Test removal of the current point.
                unsigned int new_used = bits;
                new_used &= ~(1 << j);
                T new_lambdas[4] = {T(0)};

                s2d(y, new_used, new_lambdas);

                new_point = Vector3<T>();
                for(unsigned int i = 0; i < 4; ++i)
                {
                    if(new_used & (1 << i))
                        new_point += new_lambdas[i] * y[i];
                }
                d_star = new_point * new_point;
                if(d_star < d)
                {
                    new_bits = new_used;
                    d        = d_star;
                    for(unsigned int i = 0; i < 4; ++i)
                        lambdas[i] = new_lambdas[i];
                }
            }
        }
        bits = new_bits;
    }
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void
    computeVector(unsigned int const bits, Vector3<T> const y[4], T const lambdas[4], Vector3<T>& v)
{
    v.setValue(T(0), T(0), T(0));
    for(unsigned int i = 0; i < 4; ++i)
    {
        if(bits & (1 << i))
            v += lambdas[i] * y[i];
    }
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void computePoints(unsigned int const bits,
                                                Vector3<T> const   p[4],
                                                Vector3<T> const   q[4],
                                                T const            lambdas[4],
                                                Vector3<T>&        p1,
                                                Vector3<T>&        p2)
{
    p1.setValue(T(0), T(0), T(0));
    p2.setValue(T(0), T(0), T(0));
    for(unsigned int i = 0; i < 4; ++i)
    {
        if(bits & (1 << i))
        {
            p1 += lambdas[i] * p[i];
            p2 += lambdas[i] * q[i];
        }
    }
}

// -----------------------------------------------------------------------------
template <typename T>
__HOSTDEVICE__ static INLINE void
    sv_subalgorithm(Vector3<T> const y[4], unsigned int& bits, T (&lambdas)[4], Vector3<T>& v)
{
    // The y array is never modified by this function.  The bits may be
    // modified if necessary, and the lambdas will be updated.  All the other
    // functions (if they need to make deeper calls e.g. s3d->s2d) will have to
    // make copies of bits to avoid overwriting that data incorrectly.
    unsigned int num_used = 0;
    for(unsigned int i = 0; i < 4; ++i)
        num_used += (bits >> i) & 1;

    // Start with the most common cases.
    if(num_used == 1)
    {
        for(unsigned int i = 0; i < 4; ++i)
        {
            if(bits & (1 << i))
                lambdas[i] = T(1);
        }
    }
    else if(num_used == 2)
        s1d(y, bits, lambdas);
    else if(num_used == 3)
        s2d(y, bits, lambdas);
    else
        s3d(y, bits, lambdas);

    computeVector(bits, y, lambdas, v);
}

// -----------------------------------------------------------------------------
// The next function is used for detecting degenerate cases that cause
// termination problems due to rounding errors.
template <typename T>
__HOSTDEVICE__ static INLINE bool
    degenerate(unsigned int const bits, Vector3<T> const y[4], Vector3<T> const& w)
{
    for(unsigned int i = 0, bit = 1; i < 4; ++i, bit <<= 1)
    {
        if((bits & bit) && y[i] == w)
            return (true);
    }
    return (false);
}

/* ========================================================================== */
/*                            High-Level Methods                              */
/* ========================================================================== */
template <typename T>
__HOSTDEVICE__ T computeClosestPoints_GJK_AY(Convex<T> const&     a,
                                             Convex<T> const&     b,
                                             Transform3<T> const& a2w,
                                             Transform3<T> const& b2w,
                                             Vector3<T>&          pa,
                                             Vector3<T>&          pb,
                                             int&                 nbIter)
{
    // GJK variables
    unsigned int bits = 0; // identifies current simplex
    unsigned int last = 0; // identifies last found support point
    Vector3<T>   p[4]; // support points of A in local
    Vector3<T>   q[4]; // support points of B in local
    Vector3<T>   y[4]; // support points of A-B in world
    T            mu            = 0.; // optimality gap
    int          numIterations = 0; // No. iterations
    T            lambdas[4]    = {T(0)}; // Weights

    // Misc variables, e.g. tolerance, ...
    // T relError = GrainsExec::m_colDetTolerance;             // rel error for opt gap
    // T absError = T( 1.e-4 ) * relError;                     // abs error for optimality gap
    // relative tolerance
    constexpr T relError = LOWEPS<T>;
    // absolute tolerance
    constexpr T absError = 1.e-4 * relError;
    // bool acceleration = GrainsExec::m_colDetAcceleration;   // isAcceleration?
    // T momentum = T( 0 ), oneMinusMomentum = T( 1 );         // in case we use acceleration

    // compute b2a transformation and store in register
    Transform3<T> b2a(a2w, b2w);

    // Initializing vectors
    Vector3<T> v(a.support(zeroVector3T) - b2a(b.support(zeroVector3T)));
    Vector3<T> w;
    // Vector3<T> d( v );
    T dist = v.norm();

    while(bits < 15 && dist > HIGHEPS<T> && numIterations < 1000)
    {
        // Updating the bits, ...
        for(unsigned int new_index = 0; new_index < 4; ++new_index)
        {
            // At least one of these must be empty, otherwise we have an overlap.
            if(!(bits & (1 << new_index)))
            {
                last = new_index;
                break;
            }
        }

        // Finding the suitable direction using either Nesterov or original
        // The number 8 is hard-coded. Emprically, it shows the best convergence
        // for superquadrics. For the rest of shapes, we really do not need to
        // use Nesterov as the improvemenet is marginal.
        p[last] = a.support((-v));
        q[last] = b.support((v)*b2a.getBasis());
        w       = p[last] - b2a(q[last]);

        // termination criteria -- optimiality gap
        mu = dist - v * w / dist;
        if(mu < dist * relError || mu < absError)
            break;
        // termination criteria -- degenerate case
        if(degenerate(bits, y, w))
            break;

        // if not terminated, get ready for the next iteration
        y[last] = w;
        bits |= (1 << last);
        ++numIterations;
        sv_subalgorithm(y, bits, lambdas, v);
        dist = v.norm();
    }
    // compute witness points
    computePoints(bits, p, q, lambdas, pa, pb);
    nbIter = numIterations;
    return (dist);
}

// -----------------------------------------------------------------------------
// Explicit instantiation
#define X(T)                                                                        \
    template __HOSTDEVICE__ T computeClosestPoints_GJK_AY(Convex<T> const&     a,   \
                                                          Convex<T> const&     b,   \
                                                          Transform3<T> const& a2w, \
                                                          Transform3<T> const& b2w, \
                                                          Vector3<T>&          pa,  \
                                                          Vector3<T>&          pb,  \
                                                          int&                 nbIter);
X(float)
X(double)
#undef X
