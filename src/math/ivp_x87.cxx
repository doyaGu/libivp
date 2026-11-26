#include "math/ivp_x87.h"

#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_fast_math.h>
#include <libx87/x87.hxx>
#include <fenv.h>
#include <string.h>

static uint16_t ivp_x87_soft_control_word()
{
    unsigned rounding = 0;
    switch (fegetround())
    {
#ifdef FE_DOWNWARD
        case FE_DOWNWARD: rounding = 1; break;
#endif
#ifdef FE_UPWARD
        case FE_UPWARD: rounding = 2; break;
#endif
#ifdef FE_TOWARDZERO
        case FE_TOWARDZERO: rounding = 3; break;
#endif
        default: break;
    }
    return static_cast<uint16_t>((rounding << 10) | 0x037f);
}

static uint16_t ivp_x87_native_control_word()
{
#if defined(_MSC_VER) && defined(_M_IX86)
    uint16_t control;
    __asm fnstcw control
    return control;
#elif defined(__i386__)
    uint16_t word;
    __asm__ __volatile__("fnstcw %0" : "=m"(word));
    return word;
#else
    return ivp_x87_soft_control_word();
#endif
}

static void ivp_x87_set_precision(uint16_t word, uint16_t precision)
{
    if ((word & 0x0300) == precision)
        return;
#if defined(_MSC_VER) && defined(_M_IX86)
    const uint16_t selected = static_cast<uint16_t>((word & 0xfcff) | precision);
    __asm fldcw selected
#elif defined(__i386__)
    const uint16_t selected = static_cast<uint16_t>((word & 0xfcff) | precision);
    __asm__ __volatile__("fldcw %0" : : "m"(selected));
#endif
}

uint16_t ivp_x87_enter_step(void)
{
    const uint16_t word = ivp_x87_native_control_word();
    ivp_x87_set_precision(word, 0x0300);
    return word & 0x0300;
}

void ivp_x87_leave_step(uint16_t saved_precision)
{
    ivp_x87_set_precision(ivp_x87_native_control_word(), saved_precision);
}

template<int Mode>
struct ivp_x87_arithmetic : X87_Arithmetic
{
    ivp_x87_arithmetic() : ivp_x87_arithmetic(
        Mode == IVP_X87_HARD ? ivp_x87_native_control_word() : ivp_x87_soft_control_word())
    {}

    double scalar_result(const X87_Float80 &value)
    {
        // A C double return is a binary64 write-back on all targets.
        // The hardware mode also supports the original Win32 ST0 convention.
        return Mode == IVP_X87_HARD ? return_double(value) : store_double(value);
    }

private:
    explicit ivp_x87_arithmetic(uint16_t control)
        : X87_Arithmetic(precision(control),
            static_cast<Rounding>((control >> 10) & 3),
            Mode == IVP_X87_HARD && IVP_X87_ARITHMETIC == IVP_X87_HARD,
            Mode == IVP_X87_HARD && IVP_X87_ARITHMETIC_TRANSCENDENTAL == IVP_X87_HARD)
    {}

    static Precision precision(uint16_t control)
    {
        const unsigned bits = control & 0x0300;
        return bits == 0 ? PRECISION_24 : bits == 0x0200 ? PRECISION_53 : PRECISION_64;
    }
};

#if IVP_X87_ARITHMETIC
static X87_Float80 ivp_x87_damping_exponential(X87_Arithmetic &fp, const X87_Float80 &value)
{
#if IVP_X87_ARITHMETIC_TRANSCENDENTAL
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> transcendental;
    return transcendental.exponential(fp.negate(value));
#else
    return fp.load(ivp_expd(-fp.store_double(value)));
#endif
}

void ivp_x87_gyroscopic_coefficients(const ivp_vec3f_t *inertia, const ivp_vec3f_t *inverse_inertia,
                                     double coefficients[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    for (int axis = 0; axis < 3; ++axis)
    {
        const int first = (axis + 1) % 3;
        const int second = (axis + 2) % 3;
        const X87_Float80 difference = fp.subtract(fp.load(inertia->k[first]), fp.load(inertia->k[second]));
        const X87_Float80 coefficient = fp.multiply(difference, fp.load(inverse_inertia->k[axis]));
#ifdef IVP_REAL_DOUBLE
        coefficients[axis] = fp.store_double(coefficient);
#else
        coefficients[axis] = static_cast<double>(fp.store_float(coefficient));
#endif
    }
}

void ivp_x87_add_gyroscopic_velocity(ivp_vec3f_t *angular_velocity, const double coefficients[3],
                                     double step)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 velocity[3] = {
        fp.load(angular_velocity->k[0]), fp.load(angular_velocity->k[1]), fp.load(angular_velocity->k[2])
    };
    const X87_Float80 dt = fp.load(step);
    for (int axis = 0; axis < 3; ++axis)
    {
        const int first = (axis + 1) % 3;
        const int second = (axis + 2) % 3;
        const X87_Float80 acceleration = fp.multiply(fp.multiply(velocity[first], velocity[second]),
                                                     fp.load(coefficients[axis]));
        const float stored_acceleration = fp.store_float(acceleration);
        X87_Float80 updated = fp.add(fp.multiply(fp.load(stored_acceleration), dt), velocity[axis]);
        // The last component spills to binary64 before the angular velocity is written.
        if (axis == 2)
            updated = fp.load(fp.store_double(updated));
        angular_velocity->k[axis] = fp.store_float(updated);
    }
}

void ivp_x87_damp_velocity(ivp_vec3f_t *linear, ivp_vec3f_t *angular,
                           const ivp_vec3f_t *angular_damping,
                           double linear_damping, double step)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 dt = fp.load(step);
    const X87_Float80 one = fp.load(1.0);
    X87_Float80 angle[3];
    for (int axis = 0; axis < 3; ++axis)
        angle[axis] = fp.multiply(fp.load(angular_damping->k[axis]), dt);
    const X87_Float80 square = fp.add(fp.add(fp.multiply(angle[2], angle[2]),
        fp.multiply(angle[1], angle[1])), fp.multiply(angle[0], angle[0]));
    // The squared angle uses the unrounded factors. The Y and Z factors
    // have float write-backs before they are used to damp angular velocity.
    angle[1] = fp.load(fp.store_float(angle[1]));
    angle[2] = fp.load(fp.store_float(angle[2]));
    const X87_Arithmetic::Relation angle_relation = fp.compare(square, fp.load(0.5));
    const bool linear_angle = angle_relation == X87_Arithmetic::LESS || angle_relation == X87_Arithmetic::UNORDERED;
    const X87_Float80 distance = fp.multiply(dt, fp.load(linear_damping));
    const X87_Arithmetic::Relation distance_relation = fp.compare(distance, fp.load(0.25));
    const bool linear_distance = distance_relation == X87_Arithmetic::LESS ||
                                 distance_relation == X87_Arithmetic::UNORDERED;
    const X87_Float80 linear_scale = linear_distance ? fp.subtract(one, distance) :
        ivp_x87_damping_exponential(fp, distance);
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 scale = linear_angle ? fp.subtract(one, angle[axis]) :
            ivp_x87_damping_exponential(fp, angle[axis]);
        const X87_Float80 factor = fp.load(fp.store_float(scale));
        angular->k[axis] = fp.store_float(fp.multiply(fp.load(angular->k[axis]), factor));
        linear->k[axis] = fp.store_float(fp.multiply(fp.load(linear->k[axis]), linear_scale));
    }
}

/* Keep the Newton corrections in registers and narrow only the seed input. */
static X87_Float80 ivp_x87_isqrt(X87_Arithmetic &fp, const X87_Float80 &square, int steps)
{
    const double stored_square = fp.store_double(square);
    uint64_t bits;
    memcpy(&bits, &stored_square, sizeof(bits));
    const uint32_t high = static_cast<uint32_t>(bits >> 32);
    const uint64_t seed = uint64_t(((0x7ff00000u - high) >> 1) + 0x1ff00000u) << 32;
    double initial;
    memcpy(&initial, &seed, sizeof(initial));

    const X87_Float80 half = fp.load(0.5);
    const X87_Float80 one = fp.load(1.0);
    const X87_Float80 square_half = fp.multiply(square, half);
    X87_Float80 inverse = fp.load(initial);
    X87_Float80 correction = fp.subtract(half,
        fp.multiply(fp.multiply(square_half, inverse), inverse));
    inverse = fp.multiply(inverse, fp.add(one, correction));
    const int corrections = 1 + (steps > 0) + (steps > 1) + (steps > 2);
    for (int i = 0; i < corrections; ++i)
    {
        correction = fp.subtract(half, fp.multiply(fp.multiply(inverse, inverse), square_half));
        inverse = fp.multiply(inverse, fp.add(one, correction));
    }
    return inverse;
}

double ivp_isqrt_double(double square, int resolution_steps)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(ivp_x87_isqrt(fp, fp.load(square), resolution_steps));
}

template<class Scalar>
static X87_Float80 ivp_x87_point_square(X87_Arithmetic &fp, const Scalar point[3])
{
    const X87_Float80 x = fp.load(point[0]);
    const X87_Float80 y = fp.load(point[1]);
    const X87_Float80 z = fp.load(point[2]);
    return fp.add(fp.add(fp.multiply(x, x), fp.multiply(y, y)), fp.multiply(z, z));
}

static void ivp_x87_store(X87_Arithmetic &fp, float &out, const X87_Float80 &value)
{
    out = fp.store_float(value);
}

static void ivp_x87_store(X87_Arithmetic &fp, double &out, const X87_Float80 &value)
{
    out = fp.store_double(value);
}

template<class Scalar>
static bool ivp_x87_normalize_point(Scalar point[3], int steps, double epsilon, double *length)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 square = ivp_x87_point_square(fp, point);
    const X87_Arithmetic::Relation relation = fp.compare(square, fp.load(epsilon));
    if (relation == X87_Arithmetic::LESS || relation == X87_Arithmetic::UNORDERED)
        return false;
    const X87_Float80 factor = ivp_x87_isqrt(fp, square, steps);
    for (int i = 0; i < 3; ++i)
        ivp_x87_store(fp, point[i], fp.multiply(fp.load(point[i]), factor));
    if (length)
        *length = fp.store_double(fp.multiply(factor, square));
    return true;
}

bool ivp_x87_solve_constraint_system(int count, double matrix[6][6], double desired[6],
                                     double result[6], double epsilon)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 tolerance = fp.load(epsilon);
    const X87_Float80 residual_tolerance = fp.multiply(tolerance, fp.load(1000.0f));

    for (int column = 0; column < count; ++column)
    {
        int pivot = column;
        for (int row = count - 1; row > column; --row)
        {
            if (fabs(matrix[row][column]) > fabs(matrix[pivot][column]))
                pivot = row;
        }
        if (pivot != column)
        {
            for (int i = column; i < count; ++i)
            {
                const double coefficient = matrix[column][i];
                matrix[column][i] = matrix[pivot][i];
                matrix[pivot][i] = coefficient;
            }
            const double value = desired[column];
            desired[column] = desired[pivot];
            desired[pivot] = value;
        }

        const double diagonal = matrix[column][column];
        if (fabs(diagonal) < epsilon)
            continue;

        const double inverse = fp.store_double(fp.divide(fp.load(-1.0), fp.load(diagonal)));
        for (int row = column + 1; row < count; ++row)
        {
            const double coefficient = matrix[row][column];
            if (!(fabs(coefficient) > epsilon))
                continue;
            const X87_Float80 factor = fp.load(fp.store_double(fp.multiply(fp.load(coefficient), fp.load(inverse))));
            for (int i = column; i < count; ++i)
            {
                const X87_Float80 product = fp.multiply(fp.load(matrix[column][i]), factor);
                matrix[row][i] = fp.store_double(fp.add(fp.load(matrix[row][i]), product));
            }
            const X87_Float80 product = fp.multiply(fp.load(desired[column]), factor);
            desired[row] = fp.store_double(fp.add(fp.load(desired[row]), product));
        }
    }

    for (int row = count - 1; row >= 0; --row)
    {
        X87_Float80 residual = fp.load(desired[row]);
        for (int column = count - 1; column > row; --column)
        {
            const X87_Float80 product = fp.multiply(fp.load(matrix[row][column]), fp.load(result[column]));
            residual = fp.subtract(residual, product);
        }
        const double diagonal = matrix[row][row];
        if (fabs(diagonal) < epsilon)
        {
            if (fp.compare(fp.absolute(residual), residual_tolerance) != X87_Arithmetic::LESS)
            {
                for (int i = 0; i < count; ++i)
                    result[i] = 0.0;
                return false;
            }
            result[row] = 0.0;
        }
        else
        {
            result[row] = fp.store_double(fp.divide(residual, fp.load(diagonal)));
        }
    }
    return true;
}

double ivp_x87_collision_speed_bound(float first_linear, float second_linear,
                                     float first_surface, float second_surface)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 surface_speed = fp.add(fp.load(first_surface), fp.load(second_surface));
    const X87_Float80 linear_speed = fp.add(fp.load(second_linear), fp.load(first_linear));
    return fp.store_double(fp.add(linear_speed, surface_speed));
}

double ivp_x87_collision_time(double first, double last, double target,
                              double first_distance, double last_distance, bool adjust_midpoint)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 start = fp.load(first);
    const X87_Float80 end = fp.load(last);
    const X87_Float80 distance = fp.load(first_distance);
    const X87_Float80 time_offset = fp.divide(
        fp.multiply(fp.subtract(end, start), fp.subtract(fp.load(target), distance)),
        fp.subtract(fp.load(last_distance), distance));
    double selected = fp.store_double(fp.add(time_offset, start));
    if (adjust_midpoint) {
        const X87_Float80 estimate = fp.load(selected);
        const X87_Float80 offset = fp.add(fp.subtract(start, estimate), fp.subtract(end, estimate));
        selected = fp.store_double(fp.add(fp.multiply(offset, fp.load(0.375)), estimate));
    }
    return selected;
}

/* Compare each neighboring face's normalized slope before the gradient is
 * stored. The edge and normal squared lengths enter the reciprocal square
 * root as binary32 values. */
bool ivp_x87_better_edge_face_gradient(const ivp_vec3r_t *edge, const ivp_vec3r_t *face_normal,
                                       ivp_real_t minimum, ivp_real_t *gradient)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.load(edge->k[0]), y = fp.load(edge->k[1]), z = fp.load(edge->k[2]);
    const X87_Float80 nx = fp.load(face_normal->k[0]);
    const X87_Float80 ny = fp.load(face_normal->k[1]);
    const X87_Float80 nz = fp.load(face_normal->k[2]);
    const X87_Float80 delta = fp.add(fp.add(fp.multiply(y, ny), fp.multiply(z, nz)), fp.multiply(x, nx));
    const X87_Arithmetic::Relation sign = fp.compare(delta, fp.load(0.0));
    if (sign != X87_Arithmetic::LESS && sign != X87_Arithmetic::UNORDERED)
        return false;

    const float normal_square = fp.store_float(fp.add(fp.add(fp.multiply(nx, nx), fp.multiply(nz, nz)),
                                                      fp.multiply(ny, ny)));
    const float edge_square = fp.store_float(fp.add(fp.add(fp.multiply(z, z), fp.multiply(y, y)),
                                                    fp.multiply(x, x)));
    const double inverse_normal = fp.store_double(ivp_x87_isqrt(fp, fp.load(normal_square), 2));
    const X87_Float80 inverse_edge = ivp_x87_isqrt(fp, fp.load(edge_square), 2);
    const X87_Float80 value = fp.multiply(fp.multiply(inverse_edge, fp.load(fp.store_double(delta))),
                                          fp.load(inverse_normal));
    const X87_Arithmetic::Relation relation = fp.compare(value, fp.load(minimum));
    if (relation != X87_Arithmetic::LESS && relation != X87_Arithmetic::UNORDERED)
        return false;
    ivp_x87_store(fp, *gradient, value);
    return true;
}

#ifdef IVP_REAL_DOUBLE
double ivp_x87_vertex_walk_projection(const ivp_vec3r_t *offset, const ivp_vec3f_t *vertex)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.multiply(fp.load(offset->k[0]), fp.load(vertex->k[0]));
    const X87_Float80 y = fp.multiply(fp.load(offset->k[1]), fp.load(vertex->k[1]));
    const X87_Float80 z = fp.multiply(fp.load(offset->k[2]), fp.load(vertex->k[2]));
    return fp.store_double(fp.add(fp.add(z, y), x));
}

bool ivp_x87_better_vertex_walk_gradient(const ivp_vec3r_t *offset, const ivp_vec3f_t *vertex,
                                         const ivp_vec3f_t *neighbor, double projection,
                                         double best_gradient, double *gradient)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.multiply(fp.load(offset->k[0]), fp.load(neighbor->k[0]));
    const X87_Float80 y = fp.multiply(fp.load(offset->k[1]), fp.load(neighbor->k[1]));
    const X87_Float80 z = fp.multiply(fp.load(offset->k[2]), fp.load(neighbor->k[2]));
    const X87_Float80 difference = fp.subtract(fp.add(fp.add(x, y), z), fp.load(projection));
    if (fp.compare(difference, fp.load(0.0)) != X87_Arithmetic::GREATER)
        return false;

    const X87_Float80 dx = fp.subtract(fp.load(neighbor->k[0]), fp.load(vertex->k[0]));
    const X87_Float80 dy = fp.subtract(fp.load(neighbor->k[1]), fp.load(vertex->k[1]));
    const X87_Float80 dz = fp.subtract(fp.load(neighbor->k[2]), fp.load(vertex->k[2]));
    const float edge_square = fp.store_float(fp.add(fp.add(fp.multiply(dz, dz), fp.multiply(dy, dy)),
                                                    fp.multiply(dx, dx)));
    const X87_Float80 inverse_length = ivp_x87_isqrt(fp, fp.load(edge_square), 2);
    const X87_Float80 scaled = fp.multiply(inverse_length, fp.load(fp.store_double(difference)));
    // The selected gradient is stored as a double. The candidate stays extended for the comparison.
    if (fp.compare(scaled, fp.load(best_gradient)) != X87_Arithmetic::GREATER)
        return false;
    *gradient = fp.store_double(scaled);
    return true;
}
#endif

static X87_Float80 ivp_x87_determinant2(X87_Arithmetic &fp,
                                        double a, double b, double c, double d)
{
    return fp.subtract(fp.multiply(fp.load(a), fp.load(d)), fp.multiply(fp.load(b), fp.load(c)));
}

static X87_Float80 ivp_x87_hull_determinant_extended(X87_Arithmetic &fp,
                                                     double *const rows[], int dimension)
{
    const double *a = rows[0];
    const double *b = rows[1];
    if (dimension == 2)
        return ivp_x87_determinant2(fp, a[0], a[1], b[0], b[1]);

    const double *c = rows[2];
    const X87_Float80 x = fp.multiply(fp.load(a[0]), ivp_x87_determinant2(fp, b[1], b[2], c[1], c[2]));
    const X87_Float80 y = fp.multiply(fp.load(b[0]), ivp_x87_determinant2(fp, a[1], a[2], c[1], c[2]));
    const X87_Float80 z = fp.multiply(fp.load(c[0]), ivp_x87_determinant2(fp, a[1], a[2], b[1], b[2]));
    return fp.add(fp.subtract(x, y), z);
}

double ivp_x87_hull_determinant(double *const rows[], int dimension)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(ivp_x87_hull_determinant_extended(fp, rows, dimension));
}

int ivp_x87_hull_determinant_exceeds(double *const rows[], int dimension, double maximum)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 determinant = fp.absolute(ivp_x87_hull_determinant_extended(fp, rows, dimension));
    return fp.compare(determinant, fp.load(maximum)) == X87_Arithmetic::GREATER;
}

static X87_Float80 ivp_x87_hull_plane_dot(X87_Arithmetic &fp, const double point[3],
                                          const double normal[3])
{
    const X87_Float80 x = fp.multiply(fp.load(point[0]), fp.load(normal[0]));
    const X87_Float80 y = fp.multiply(fp.load(point[1]), fp.load(normal[1]));
    const X87_Float80 z = fp.multiply(fp.load(point[2]), fp.load(normal[2]));
    return fp.add(fp.add(x, y), z);
}

bool ivp_x87_set_hull_plane_offset(double *const vertices[3], const double origin[3],
                                   const double normal[3], double tolerance, double *offset)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 plane_offset = fp.negate(ivp_x87_hull_plane_dot(fp, origin, normal));
    const X87_Float80 maximum = fp.load(tolerance);
    *offset = fp.store_double(plane_offset);

    // Keep the unrounded offset while checking that the three vertices lie on the plane.
    for (int vertex = 2; vertex >= 0; --vertex)
    {
        if (vertices[vertex] == origin)
            continue;
        const X87_Float80 projection = ivp_x87_hull_plane_dot(fp, vertices[vertex], normal);
        const X87_Float80 distance = fp.absolute(fp.add(plane_offset, projection));
        if (fp.compare(distance, maximum) == X87_Arithmetic::GREATER)
            return true;
    }
    return false;
}

double ivp_x87_hull_plane_gauss_offset(const double *point, const double *normal, int dimension)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    double offset = fp.store_double(fp.negate(fp.multiply(fp.load(point[0]), fp.load(normal[0]))));
    for (int i = 1; i < dimension; ++i)
    {
        const X87_Float80 product = fp.multiply(fp.load(point[i]), fp.load(normal[i]));
        offset = fp.store_double(fp.subtract(fp.load(offset), product));
    }
    return offset;
}

double ivp_x87_hull_plane_distance(const double point[3], const double normal[3], double offset)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.multiply(fp.load(point[0]), fp.load(normal[0]));
    const X87_Float80 y = fp.multiply(fp.load(point[1]), fp.load(normal[1]));
    const X87_Float80 z = fp.multiply(fp.load(point[2]), fp.load(normal[2]));
    return fp.store_double(fp.add(fp.add(fp.add(z, y), x), fp.load(offset)));
}

int ivp_x87_normalize_hull_plane(double normal[3], unsigned orientation,
                                 double min_denominator, const double *minimum,
                                 unsigned *below_minimum, double *length)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.load(normal[0]);
    const X87_Float80 y = fp.load(normal[1]);
    const X87_Float80 z = fp.load(normal[2]);
    const X87_Float80 magnitude = fp.square_root(
        fp.add(fp.add(fp.multiply(z, z), fp.multiply(y, y)), fp.multiply(x, x)));
    if (fp.compare(magnitude, fp.load(min_denominator)) != X87_Arithmetic::GREATER)
        return 0;

    if (minimum && below_minimum)
    {
        const X87_Arithmetic::Relation relation = fp.compare(magnitude, fp.load(*minimum));
        *below_minimum = relation == X87_Arithmetic::LESS || relation == X87_Arithmetic::UNORDERED;
    }
    *length = fp.store_double(magnitude);
    const X87_Float80 divisor = orientation ? magnitude : fp.negate(magnitude);
    for (int axis = 0; axis < 3; ++axis)
        normal[axis] = fp.store_double(fp.divide(fp.load(normal[axis]), divisor));
    return 1;
}

double ivp_x87_length_squared3f(const float point[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(ivp_x87_point_square(fp, point));
}

float ivp_x87_minimum_rotational_inertia(const ivp_vec3f_t *inertia, float factor)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 length = fp.square_root(ivp_x87_point_square(fp, inertia->k));
    return fp.store_float(fp.multiply(length, fp.load(factor)));
}

double ivp_x87_length3f(const float point[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(fp.square_root(ivp_x87_point_square(fp, point)));
}

double ivp_x87_length3d(const double point[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(fp.square_root(ivp_x87_point_square(fp, point)));
}

bool ivp_x87_normalize3f(float point[3], int steps, double epsilon, double *length)
{
    return ivp_x87_normalize_point(point, steps, epsilon, length);
}

bool ivp_x87_normalize3d(double point[3], int steps, double epsilon, double *length)
{
    return ivp_x87_normalize_point(point, steps, epsilon, length);
}

#ifdef IVP_REAL_DOUBLE
void ivp_x87_quat_interpolate(ivp_quatr_t *out, const ivp_quatr_t *from,
                              const ivp_quatr_t *to, ivp_real_t time)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 a[4] = {fp.load(from->x), fp.load(from->y), fp.load(from->z), fp.load(from->w)};
    const X87_Float80 b[4] = {fp.load(to->x), fp.load(to->y), fp.load(to->z), fp.load(to->w)};
    const X87_Float80 one = fp.load(1.0);
    const X87_Float80 step = fp.load(time);
    X87_Float80 cosine = fp.add(fp.add(fp.add(fp.multiply(a[2], b[2]), fp.multiply(a[1], b[1])),
                                        fp.multiply(b[3], a[3])), fp.multiply(a[0], b[0]));
    const bool same_hemisphere = fp.compare(cosine, fp.load(0.0)) == X87_Arithmetic::GREATER;
    const X87_Float80 sign = fp.load(same_hemisphere ? 1.0 : -1.0);
    if (!same_hemisphere)
        cosine = fp.negate(cosine);
    const X87_Arithmetic::Relation relation = fp.compare(cosine, fp.load(1.0 - static_cast<double>(0.001f)));
    ivp_real_t result[4];
    if (relation == X87_Arithmetic::LESS || relation == X87_Arithmetic::UNORDERED) {
        const double stored_cosine = fp.store_double(cosine);
        const X87_Float80 cos_angle = fp.load(stored_cosine);
        const X87_Float80 angle = fp.load(ivp_acosd(stored_cosine));
        const X87_Float80 inverse_sine = fp.divide(one,
            fp.square_root(fp.subtract(one, fp.multiply(cos_angle, cos_angle))));
        const X87_Float80 first = fp.multiply(inverse_sine, fp.sine(fp.multiply(fp.subtract(one, step), angle)));
        const X87_Float80 second = fp.multiply(fp.multiply(fp.sine(fp.multiply(angle, step)), sign),
                                               fp.load(fp.store_double(inverse_sine)));
        for (int component = 0; component < 4; ++component)
            ivp_x87_store(fp, result[component],
                          fp.add(fp.multiply(first, a[component]), fp.multiply(second, b[component])));
    } else {
        X87_Float80 interpolated[4];
        for (int component = 0; component < 4; ++component) {
            const X87_Float80 difference = fp.subtract(fp.multiply(sign, b[component]), a[component]);
            interpolated[component] = fp.add(fp.multiply(difference, step), a[component]);
            ivp_x87_store(fp, result[component], interpolated[component]);
        }
        const X87_Float80 half = fp.load(0.5);
        const X87_Float80 w = interpolated[3];
        const X87_Float80 square_half = fp.multiply(fp.add(ivp_x87_point_square(fp, result),
                                                           fp.multiply(w, w)), half);
        X87_Float80 factor = fp.subtract(fp.load(1.5), square_half);
        for (int i = 0; i < 2; ++i)
            factor = fp.add(factor, fp.subtract(half, fp.multiply(fp.multiply(factor, factor), square_half)));
        for (int component = 0; component < 4; ++component)
            ivp_x87_store(fp, result[component], fp.multiply(factor, fp.load(result[component])));
    }
    *out = {result[0], result[1], result[2], result[3]};
}
#endif

void ivp_x87_quat_invert_mult(ivp_real_t out[4], const ivp_real_t left[4],
                              const ivp_real_t right[4])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 ax = fp.load(-left[0]), ay = fp.load(-left[1]);
    const X87_Float80 az = fp.load(-left[2]), aw = fp.load(left[3]);
    const X87_Float80 bx = fp.load(right[0]), by = fp.load(right[1]);
    const X87_Float80 bz = fp.load(right[2]), bw = fp.load(right[3]);
    const X87_Float80 x = fp.subtract(fp.add(fp.add(
        fp.multiply(ay, bz), fp.multiply(aw, bx)), fp.multiply(ax, bw)), fp.multiply(az, by));
    const X87_Float80 y = fp.subtract(fp.add(fp.add(
        fp.multiply(az, bx), fp.multiply(ay, bw)), fp.multiply(aw, by)), fp.multiply(ax, bz));
    const X87_Float80 z = fp.subtract(fp.add(fp.add(
        fp.multiply(aw, bz), fp.multiply(az, bw)), fp.multiply(ax, by)), fp.multiply(ay, bx));
    const X87_Float80 w = fp.subtract(fp.subtract(fp.subtract(
        fp.multiply(aw, bw), fp.multiply(ax, bx)), fp.multiply(ay, by)), fp.multiply(az, bz));
    ivp_x87_store(fp, out[0], x);
    ivp_x87_store(fp, out[1], y);
    ivp_x87_store(fp, out[2], z);
    ivp_x87_store(fp, out[3], w);
}

void ivp_x87_quat_mult(ivp_real_t out[4], const ivp_real_t left[4],
                       const ivp_real_t right[4])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 ax = fp.load(left[0]), ay = fp.load(left[1]);
    const X87_Float80 az = fp.load(left[2]), aw = fp.load(left[3]);
    const X87_Float80 bx = fp.load(right[0]), by = fp.load(right[1]);
    const X87_Float80 bz = fp.load(right[2]), bw = fp.load(right[3]);

    const X87_Float80 x = fp.subtract(fp.add(fp.add(fp.multiply(ax, bw),
        fp.multiply(bz, ay)), fp.multiply(aw, bx)), fp.multiply(az, by));
    const X87_Float80 y = fp.subtract(fp.add(fp.add(fp.multiply(aw, by),
        fp.multiply(bw, ay)), fp.multiply(az, bx)), fp.multiply(bz, ax));
    const X87_Float80 z = fp.subtract(fp.add(fp.add(fp.multiply(aw, bz),
        fp.multiply(ax, by)), fp.multiply(az, bw)), fp.multiply(bx, ay));
    const X87_Float80 w = fp.subtract(fp.subtract(fp.subtract(fp.multiply(aw, bw),
        fp.multiply(ax, bx)), fp.multiply(ay, by)), fp.multiply(az, bz));

    ivp_x87_store(fp, out[0], x);
    ivp_x87_store(fp, out[1], y);
    ivp_x87_store(fp, out[2], z);
    ivp_x87_store(fp, out[3], w);
}

template<bool StoreZ = true, class Vector>
static void ivp_x87_cross_extended(X87_Arithmetic &fp, const Vector *left,
                                   const Vector *right, X87_Float80 cross[3])
{
    for (int axis = 0; axis < 3; ++axis)
    {
        const int y = (axis + 1) % 3, z = (axis + 2) % 3;
        cross[axis] = fp.subtract(fp.multiply(fp.load(left->k[y]), fp.load(right->k[z])),
                                 fp.multiply(fp.load(right->k[y]), fp.load(left->k[z])));
    }
    if (StoreZ) cross[2] = fp.load(fp.store_float(cross[2]));
}

void ivp_x87_cross3f(const ivp_vec3f_t *left, const ivp_vec3f_t *right, ivp_vec3f_t *cross)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 components[3];
    ivp_x87_cross_extended(fp, left, right, components);
    for (int axis = 0; axis < 3; ++axis)
        cross->k[axis] = fp.store_float(components[axis]);
}

void ivp_x87_accumulate_world_impulse(const ivp_transform_r_t *transform, const ivp_vec3r_t *point,
                                      const ivp_vec3f_t *impulse, float inverse_mass,
                                      const ivp_vec3f_t *inverse_inertia,
                                      ivp_vec3f_t *linear, ivp_vec3f_t *angular)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    ivp_vec3f_t offset;
    for (int axis = 0; axis < 3; ++axis)
        offset.k[axis] = fp.store_float(fp.subtract(fp.load(point->k[axis]), fp.load(transform->pos.k[axis])));

    X87_Float80 cross[3];
    ivp_x87_cross_extended<false>(fp, &offset, impulse, cross);
    for (int axis = 0; axis < 3; ++axis)
        cross[axis] = fp.load(fp.store_float(cross[axis]));

    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 x = fp.multiply(fp.load(transform->rot.rows[0].k[axis]), cross[0]);
        const X87_Float80 y = fp.multiply(fp.load(transform->rot.rows[1].k[axis]), cross[1]);
        const X87_Float80 z = fp.multiply(fp.load(transform->rot.rows[2].k[axis]), cross[2]);
        X87_Float80 projected = fp.add(fp.add(z, y), x);
        // The last torque component stays wide through the inertia product and accumulation.
        projected = axis == 2 ? fp.load(fp.store_double(projected)) : fp.load(fp.store_float(projected));
        X87_Float80 rotation = fp.multiply(projected, fp.load(inverse_inertia->k[axis]));
        if (axis != 2)
            rotation = fp.load(fp.store_float(rotation));
        angular->k[axis] = fp.store_float(fp.add(fp.load(angular->k[axis]), rotation));

        X87_Float80 speed = fp.add(fp.load(linear->k[axis]), fp.multiply(fp.load(impulse->k[axis]), fp.load(inverse_mass)));
        if (axis != 0)
            speed = fp.load(fp.store_double(speed));
        linear->k[axis] = fp.store_float(speed);
    }
}

double ivp_x87_spring_impulse(const ivp_vec3f_t *direction, const ivp_vec3f_t *relative_speed,
                              float length, float rest_length, float stiffness, float axial_damping,
                              float lateral_damping, double step, double maximum_force, ivp_vec3f_t *impulse)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 stretch = fp.subtract(fp.load(length), fp.load(rest_length));
    const double elastic = fp.store_double(fp.multiply(stretch, fp.load(stiffness)));
    const X87_Float80 speed_x = fp.multiply(fp.load(direction->k[0]), fp.load(relative_speed->k[0]));
    const X87_Float80 speed_y = fp.multiply(fp.load(direction->k[1]), fp.load(relative_speed->k[1]));
    const X87_Float80 speed_z = fp.multiply(fp.load(direction->k[2]), fp.load(relative_speed->k[2]));
    const X87_Float80 axial_speed = fp.add(fp.add(speed_z, speed_y), speed_x);
    X87_Float80 force = fp.subtract(fp.load(elastic), fp.multiply(axial_speed, fp.load(axial_damping)));
    if (maximum_force > 0.0)
    {
        const X87_Float80 limit = fp.load(maximum_force);
        const X87_Float80 lower_limit = fp.negate(limit);
        if (fp.compare(force, limit) == X87_Arithmetic::GREATER)
            force = limit;
        else if (fp.compare(force, lower_limit) == X87_Arithmetic::LESS)
            force = lower_limit;
    }
    const X87_Float80 axial = fp.multiply(force, fp.load(step));
    const X87_Float80 lateral = fp.negate(fp.multiply(fp.load(lateral_damping), fp.load(step)));
    for (int axis = 0; axis < 3; ++axis)
    {
        X87_Float80 component = fp.multiply(fp.load(direction->k[axis]), axial);
        // The Z component stays wide until the lateral damping term is added.
        if (axis != 2)
            component = fp.load(fp.store_float(component));
        component = fp.add(fp.multiply(fp.load(relative_speed->k[axis]), lateral), component);
        if (axis != 0)
            component = fp.load(fp.store_double(component));
        impulse->k[axis] = fp.store_float(component);
    }
    return fp.store_double(force);
}

double ivp_x87_intersect_ray_edge(const double origin[3], const double direction[3], const double start[3],
                                  const double end[3], int axis, double point[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const int first = (axis + 1) % 3;
    const int second = (axis + 2) % 3;
    const X87_Float80 edge_first = fp.subtract(fp.load(start[first]), fp.load(end[first]));
    const X87_Float80 edge_second = fp.subtract(fp.load(start[second]), fp.load(end[second]));
    const X87_Float80 offset_first = fp.subtract(fp.load(start[first]), fp.load(origin[first]));
    const X87_Float80 offset_second = fp.subtract(fp.load(start[second]), fp.load(origin[second]));
    const X87_Float80 numerator = fp.subtract(fp.multiply(offset_first, edge_second),
                                               fp.multiply(offset_second, edge_first));
    const X87_Float80 denominator = fp.subtract(fp.multiply(fp.load(direction[first]), edge_second),
                                                 fp.multiply(fp.load(direction[second]), edge_first));
    // Write the coordinates before narrowing the ray parameter.
    const X87_Float80 distance = fp.divide(numerator, denominator);
    for (int component = 0; component < 3; ++component)
    {
        point[component] = fp.store_double(fp.add(fp.load(origin[component]),
            fp.multiply(distance, fp.load(direction[component]))));
    }
    return fp.store_double(distance);
}

float ivp_x87_reduced_inverse_mass(double first, double second, double epsilon)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 a = fp.load(first);
    const X87_Float80 b = fp.load(second);
    const X87_Float80 sum = fp.add(a, b);
    if (fp.compare(sum, fp.load(epsilon)) != X87_Arithmetic::GREATER)
        return 0.0f;
    const X87_Float80 mass = fp.divide(fp.multiply(a, b), sum);
    IVP_ASSERT(fp.compare(mass, fp.load(static_cast<double>(1e30f))) == X87_Arithmetic::LESS);
    if (fp.compare(mass, fp.load(epsilon)) != X87_Arithmetic::GREATER)
        return 0.0f;

    // The cached inverse mass is written directly to binary32.
    return fp.store_float(fp.divide(fp.load(1.0), mass));
}

static X87_Float80 ivp_x87_worst_case_effective_mass_extended(X87_Arithmetic &fp,
    const ivp_vec3f_t *point, const ivp_vec3f_t *inverse_inertia, float inverse_mass)
{
    const X87_Float80 x = fp.load(point->k[0]);
    const X87_Float80 y = fp.load(point->k[1]);
    const X87_Float80 z = fp.load(point->k[2]);
    const X87_Float80 square_x = fp.load(fp.store_float(fp.multiply(x, x)));
    const X87_Float80 square_y = fp.load(fp.store_float(fp.multiply(y, y)));
    const X87_Float80 square_z = fp.multiply(z, z);
    // The Z square stays wide until the other two components have been summed.
    const X87_Float80 squares[3] = {
        fp.load(fp.store_float(fp.add(square_y, square_z))),
        fp.load(fp.store_float(fp.add(square_x, square_z))),
        fp.add(square_x, square_y)
    };
    float maximum = fp.store_float(fp.multiply(squares[0], fp.load(inverse_inertia->k[0])));
    for (int axis = 1; axis < 3; ++axis) {
        const float component = fp.store_float(fp.multiply(squares[axis], fp.load(inverse_inertia->k[axis])));
        if (component > maximum)
            maximum = component;
    }
    return fp.divide(fp.load(1.0), fp.add(fp.load(maximum), fp.load(inverse_mass)));
}

double ivp_x87_worst_case_effective_mass(const ivp_vec3f_t *point,
                                         const ivp_vec3f_t *inverse_inertia, float inverse_mass)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.scalar_result(ivp_x87_worst_case_effective_mass_extended(fp, point, inverse_inertia, inverse_mass));
}

float ivp_x87_worst_case_inverse_mass(const ivp_vec3f_t *point, const ivp_vec3f_t *inverse_inertia,
                                      float inverse_mass, double epsilon)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 mass = ivp_x87_worst_case_effective_mass_extended(fp, point, inverse_inertia, inverse_mass);
    IVP_ASSERT(fp.compare(mass, fp.load(static_cast<double>(1e30f))) == X87_Arithmetic::LESS);
    if (fp.compare(mass, fp.load(epsilon)) != X87_Arithmetic::GREATER)
        return 0.0f;

    // Keep the effective mass in extended precision until the inverse is stored.
    return fp.store_float(fp.divide(fp.load(1.0), mass));
}

static X87_Float80 ivp_x87_round_double(X87_Arithmetic &fp, const X87_Float80 &value)
{
    return fp.load(fp.store_double(value));
}

template<class Vector>
static X87_Float80 ivp_x87_point_dot(X87_Arithmetic &fp, const Vector *left, const Vector *right)
{
    return fp.add(fp.add(fp.multiply(fp.load(left->k[2]), fp.load(right->k[2])),
                         fp.multiply(fp.load(left->k[1]), fp.load(right->k[1]))),
                  fp.multiply(fp.load(left->k[0]), fp.load(right->k[0])));
}

void ivp_x87_add_scaled_row(double *target, const double *source, double factor, int count)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 scale = fp.load(factor);
    for (int i = 0; i < count; ++i)
        target[i] = fp.store_double(fp.add(fp.multiply(fp.load(source[i]), scale), fp.load(target[i])));
}

double ivp_x87_dot_product_forward(const double *left, const double *right, int count)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 sum = fp.load(0.0);
    for (int i = 0; i < count; ++i)
        sum = fp.add(sum, fp.multiply(fp.load(left[i]), fp.load(right[i])));
    return fp.store_double(sum);
}

static X87_Float80 ivp_x87_accumulate_dot_product_backward(X87_Arithmetic &fp,
                                                           const double *left, const double *right, int count)
{
    X87_Float80 sum = fp.load(0.0);
    for (int i = count - 1; i >= 0; --i)
        sum = fp.add(sum, fp.multiply(fp.load(left[i]), fp.load(right[i])));
    return sum;
}

double ivp_x87_dot_product_backward(const double *left, const double *right, int count)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.store_double(ivp_x87_accumulate_dot_product_backward(fp, left, right, count));
}

double ivp_x87_back_substitute_lu_row(double value, const double *upper, const double *solution, int count)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 sum = ivp_x87_accumulate_dot_product_backward(fp, upper, solution, count);
    return fp.store_double(fp.subtract(fp.load(value), sum));
}

static void ivp_x87_scale_row(X87_Arithmetic &fp, double *row, int count, const X87_Float80 &scale)
{
    for (int i = 0; i < count; ++i)
        row[i] = fp.store_double(fp.multiply(scale, fp.load(row[i])));
}

double ivp_x87_indexed_dot_product_with_bias(const double *left, const double *right,
                                             const int *indices, int count, double bias)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 sum = fp.load(0.0);
    for (int i = 0; i < count; ++i)
    {
        const int index = indices[i];
        sum = fp.add(sum, fp.multiply(fp.load(left[index]), fp.load(right[index])));
    }
    return fp.store_double(fp.add(sum, fp.load(bias)));
}

double ivp_x87_dot_product_residual(const double *left, const double *right, int count, double desired)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 sum = ivp_x87_accumulate_dot_product_backward(fp, left, right, count);
    return fp.store_double(fp.subtract(sum, fp.load(desired)));
}

double ivp_x87_divide_double(double numerator, double denominator)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.store_double(fp.divide(fp.load(numerator), fp.load(denominator)));
}

double ivp_x87_multiply_double(double left, double right)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return fp.store_double(fp.multiply(fp.load(left), fp.load(right)));
}

bool ivp_x87_back_substitute_gauss_row(const double *upper, double *solution,
                                       int pivot, int count, double epsilon)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 residual = fp.load(solution[pivot]);
    for (int i = count - 1; i > pivot; --i)
        residual = fp.subtract(residual, fp.multiply(fp.load(upper[i]), fp.load(solution[i])));
    const X87_Float80 diagonal = fp.load(upper[pivot]);
    const X87_Float80 eps = fp.load(epsilon);
    if (fp.compare(fp.absolute(diagonal), eps) == X87_Arithmetic::LESS)
    {
        if (fp.compare(fp.absolute(residual), fp.multiply(eps, fp.load(1000.0))) != X87_Arithmetic::LESS)
            return false;
        solution[pivot] = 0.0;
        return true;
    }
    solution[pivot] = fp.store_double(fp.divide(residual, diagonal));
    return true;
}

void ivp_x87_normalize_lu_lower_row(double *lower, int count, int pivot)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 reciprocal = fp.divide(fp.load(1.0), fp.load(lower[pivot]));
    ivp_x87_scale_row(fp, lower, count, reciprocal);
}

void ivp_x87_normalize_lu_pivot(double *lower, double *upper, int count, int pivot)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 reciprocal = fp.divide(fp.load(1.0), fp.load(upper[pivot]));
    ivp_x87_scale_row(fp, lower, count, reciprocal);
    ivp_x87_scale_row(fp, upper + pivot + 1, count - pivot - 1, reciprocal);
}

void ivp_x87_contact_normal_jacobian(const ivp_mat3r_t *rotation, const ivp_vec3f_t *normal,
    const ivp_vec3f_t *point, bool second_body, ivp_vec3f_t *jacobian)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 local_normal[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 x = fp.multiply(fp.load(rotation->rows[0].k[axis]), fp.load(normal->k[0]));
        const X87_Float80 y = fp.multiply(fp.load(rotation->rows[1].k[axis]), fp.load(normal->k[1]));
        const X87_Float80 z = fp.multiply(fp.load(rotation->rows[2].k[axis]), fp.load(normal->k[2]));
        if (axis == 0)
            local_normal[axis] = second_body ? fp.add(fp.add(x, y), z) : fp.add(fp.add(z, y), x);
        else
            local_normal[axis] = fp.load(fp.store_float(fp.add(fp.add(y, x), z)));
    }

    const X87_Float80 x = fp.load(point->k[0]);
    const X87_Float80 y = fp.load(point->k[1]);
    const X87_Float80 z = fp.load(point->k[2]);
    // X of the rotated normal stays extended through the Y moment. The two
    // products of the Z moment are spilled before the subtraction.
    jacobian->k[0] = fp.store_float(fp.subtract(fp.multiply(local_normal[2], y), fp.multiply(local_normal[1], z)));
    jacobian->k[1] = fp.store_float(fp.subtract(fp.multiply(local_normal[0], z), fp.multiply(local_normal[2], x)));
    jacobian->k[2] = fp.store_float(fp.subtract(ivp_x87_round_double(fp, fp.multiply(local_normal[1], x)),
                                              ivp_x87_round_double(fp, fp.multiply(local_normal[0], y))));
}

void ivp_x87_inverse_transform_contact_point(const ivp_transform_r_t *transform,
                                             const ivp_vec3r_t *world, ivp_vec3f_t *local)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 offset[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 value = fp.subtract(fp.load(world->k[axis]), fp.load(transform->pos.k[axis]));
        offset[axis] = ivp_x87_round_double(fp, value);
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 y = fp.multiply(offset[1], fp.load(transform->rot.rows[1].k[axis]));
        const X87_Float80 z = fp.multiply(offset[2], fp.load(transform->rot.rows[2].k[axis]));
        const X87_Float80 x = fp.multiply(offset[0], fp.load(transform->rot.rows[0].k[axis]));
        local->k[axis] = fp.store_float(fp.add(fp.add(y, z), x));
    }
}

void ivp_x87_surface_speed(const ivp_mat3r_t *rotation, const ivp_vec3f_t *point,
                           const ivp_vec3f_t *linear_speed, const ivp_vec3f_t *angular_speed,
                           ivp_vec3f_t *velocity)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 cross[3];
    ivp_x87_cross_extended(fp, angular_speed, point, cross);

    X87_Float80 world[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const ivp_real_t *row = rotation->rows[axis].k;
        world[axis] = fp.add(fp.add(fp.multiply(cross[2], fp.load(row[2])),
                                    fp.multiply(cross[1], fp.load(row[1]))),
                             fp.multiply(cross[0], fp.load(row[0])));
        if (axis != 2)
            world[axis] = fp.load(fp.store_float(world[axis]));
        world[axis] = fp.add(world[axis], fp.load(linear_speed->k[axis]));
        if (axis == 2)
            world[axis] = fp.load(fp.store_double(world[axis]));
    }
    for (int axis = 0; axis < 3; ++axis)
        velocity->k[axis] = fp.store_float(world[axis]);
}

void ivp_x87_test_push(const ivp_vec3f_t *point, const ivp_vec3f_t *local_impulse,
                       const ivp_vec3f_t *world_impulse, float inverse_mass,
                       const ivp_vec3f_t *inverse_inertia, ivp_vec3f_t *linear_speed,
                       ivp_vec3f_t *angular_speed)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 cross[3];
    ivp_x87_cross_extended(fp, point, local_impulse, cross);
    for (int axis = 0; axis < 3; ++axis)
    {
        X87_Float80 angular = fp.multiply(cross[axis], fp.load(inverse_inertia->k[axis]));
        if (axis == 2)
            angular = fp.load(fp.store_double(angular));
        angular_speed->k[axis] = fp.store_float(angular);
    }
    const X87_Float80 inverse = fp.load(inverse_mass);
    X87_Float80 linear[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        linear[axis] = fp.multiply(fp.load(world_impulse->k[axis]), inverse);
        if (axis != 0)
            linear[axis] = fp.load(fp.store_double(linear[axis]));
    }
    for (int axis = 0; axis < 3; ++axis)
        linear_speed->k[axis] = fp.store_float(linear[axis]);
}

float ivp_x87_point_face_distance(const ivp_vec3r_t *normal, const ivp_vec3r_t *point,
                                  const ivp_vec3f_t *face_point)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.load(normal->k[0]);
    const X87_Float80 y = fp.load(normal->k[1]);
    const X87_Float80 z = fp.load(normal->k[2]);
    const X87_Float80 point_projection = fp.add(
        fp.add(fp.multiply(z, fp.load(point->k[2])), fp.multiply(x, fp.load(point->k[0]))),
        fp.multiply(y, fp.load(point->k[1])));
    const X87_Float80 face_projection = fp.add(
        fp.add(fp.multiply(y, fp.load(face_point->k[1])), fp.multiply(z, fp.load(face_point->k[2]))),
        fp.multiply(x, fp.load(face_point->k[0])));
    return fp.store_float(fp.subtract(point_projection, face_projection));
}

static X87_Float80 ivp_x87_square(X87_Arithmetic &fp, const X87_Float80 &value)
{
    return fp.multiply(value, value);
}

template<bool Transpose>
static void ivp_x87_rotate3d_impl(X87_Arithmetic &fp, const double rotation[3][3],
                                  const X87_Float80 input[3], double output[3])
{
    const X87_Float80 &vx = input[0];
    const X87_Float80 &vy = input[1];
    const X87_Float80 &vz = input[2];
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 x = fp.multiply(fp.load(Transpose ? rotation[0][axis] : rotation[axis][0]), vx);
        const X87_Float80 y = fp.multiply(fp.load(Transpose ? rotation[1][axis] : rotation[axis][1]), vy);
        const X87_Float80 z = fp.multiply(fp.load(Transpose ? rotation[2][axis] : rotation[axis][2]), vz);
        output[axis] = fp.store_double(axis == 0 ? fp.add(fp.add(z, y), x) : fp.add(fp.add(x, z), y));
    }
}

void ivp_x87_rotate3d(const double rotation[3][3], const double local[3], double world[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 input[3] = {fp.load(local[0]), fp.load(local[1]), fp.load(local[2])};
    ivp_x87_rotate3d_impl<false>(fp, rotation, input, world);
}

void ivp_x87_inverse_rotate3d(const double rotation[3][3], const double world[3], double local[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 input[3] = {fp.load(world[0]), fp.load(world[1]), fp.load(world[2])};
    ivp_x87_rotate3d_impl<true>(fp, rotation, input, local);
}

void ivp_x87_inverse_transform_point3d(const double rotation[3][3], const double origin[3],
                                       const double world[3], double local[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 offset[3] = {
        fp.subtract(fp.load(world[0]), fp.load(origin[0])),
        fp.subtract(fp.load(world[1]), fp.load(origin[1])),
        fp.subtract(fp.load(world[2]), fp.load(origin[2]))
    };
    ivp_x87_rotate3d_impl<true>(fp, rotation, offset, local);
}

template<bool Transpose>
static void ivp_x87_rotation_product(const double left[3][3], const double right[3][3],
                                     double output[3][3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    double result[3][3];
    for (int row = 0; row < 3; ++row)
    {
        const X87_Float80 vx = fp.load(left[row][0]);
        const X87_Float80 vy = fp.load(left[row][1]);
        const X87_Float80 vz = fp.load(left[row][2]);
        for (int col = 0; col < 3; ++col)
        {
            const X87_Float80 x = fp.multiply(vx, fp.load(Transpose ? right[col][0] : right[0][col]));
            const X87_Float80 y = fp.multiply(vy, fp.load(Transpose ? right[col][1] : right[1][col]));
            const X87_Float80 z = fp.multiply(vz, fp.load(Transpose ? right[col][2] : right[2][col]));
            const X87_Float80 sum = col == 0 ? fp.add(fp.add(y, x), z) : fp.add(fp.add(z, y), x);
            result[row][col] = fp.store_double(sum);
        }
    }
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            output[row][col] = result[row][col];
}

void ivp_x87_multiply_rotation3d(const double left[3][3], const double right[3][3], double output[3][3])
{
    ivp_x87_rotation_product<false>(left, right, output);
}

void ivp_x87_multiply_transposed_rotation3d(const double left[3][3], const double right[3][3],
                                            double output[3][3])
{
    ivp_x87_rotation_product<true>(left, right, output);
}

bool ivp_x87_invert_transform3d(double matrix[3][3], double origin[3], double epsilon)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 cofactors[3][3];
    for (int row = 0; row < 3; ++row)
    {
        const int next = (row + 1) % 3, last = (row + 2) % 3;
        for (int col = 0; col < 3; ++col)
        {
            const int y = (col + 1) % 3, z = (col + 2) % 3;
            X87_Float80 cofactor = fp.subtract(
                fp.multiply(fp.load(matrix[next][y]), fp.load(matrix[last][z])),
                fp.multiply(fp.load(matrix[last][y]), fp.load(matrix[next][z])));
            // The last row's X cofactor stays extended until the inverse is written.
            if (row != 2 || col != 0)
                cofactor = ivp_x87_round_double(fp, cofactor);
            cofactors[row][col] = cofactor;
        }
    }
    const X87_Float80 determinant = fp.add(fp.add(
        fp.multiply(cofactors[0][2], fp.load(matrix[0][2])),
        fp.multiply(cofactors[0][1], fp.load(matrix[0][1]))),
        fp.multiply(cofactors[0][0], fp.load(matrix[0][0])));
    const X87_Arithmetic::Relation relation = fp.compare(fp.absolute(determinant), fp.load(epsilon));
    if (relation == X87_Arithmetic::LESS || relation == X87_Arithmetic::UNORDERED)
        return false;

    const X87_Float80 inverse = fp.divide(fp.load(1.0), determinant);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            matrix[col][row] = fp.store_double(fp.multiply(cofactors[row][col], inverse));

    double translation[3];
    ivp_x87_rotate3d(matrix, origin, translation);
    for (int axis = 0; axis < 3; ++axis)
        origin[axis] = -translation[axis];
    return true;
}

template<bool StoreX, class Scalar>
static X87_Float80 ivp_x87_position_component(X87_Arithmetic &fp, const Scalar row[3],
                                              Scalar origin, const X87_Float80 point[3], int axis)
{
    X87_Float80 sum = fp.multiply(fp.load(row[0]), point[0]);
    if (axis != 2)
        sum = ivp_x87_round_double(fp, sum);
    sum = fp.add(fp.multiply(fp.load(row[1]), point[1]), sum);
    if (axis != 2)
        sum = ivp_x87_round_double(fp, sum);
    sum = fp.add(fp.multiply(fp.load(row[2]), point[2]), sum);
    if (StoreX && axis == 0)
        sum = ivp_x87_round_double(fp, sum);
    return fp.add(sum, fp.load(origin));
}

void ivp_x87_cross3r(const ivp_vec3r_t *left, const ivp_vec3r_t *right, ivp_vec3r_t *cross)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 values[3];
    ivp_x87_cross_extended<false>(fp, left, right, values);
    for (int axis = 0; axis < 3; ++axis)
        ivp_x87_store(fp, cross->k[axis], values[axis]);
}

ivp_real_t ivp_x87_edge_contact_fraction(const ivp_vec3r_t *normal, const ivp_vec3r_t *first,
                                         const ivp_vec3r_t *last, const ivp_vec3r_t *point)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 a = ivp_x87_point_dot(fp, normal, first);
    const X87_Float80 b = ivp_x87_point_dot(fp, normal, last);
    const X87_Float80 position = ivp_x87_point_dot(fp, normal, point);
    const X87_Float80 inv_span = fp.divide(fp.load(1.0), fp.subtract(a, b));
    ivp_real_t fraction;
    ivp_x87_store(fp, fraction, fp.multiply(fp.subtract(a, position), inv_span));
    return fraction;
}

void ivp_x87_advance_position(const ivp_vec3r_t *position, const ivp_vec3r_t *velocity,
                              double elapsed, ivp_vec3r_t *out)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 dt = fp.load(elapsed);
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 displacement = fp.multiply(fp.load(velocity->k[axis]), dt);
        ivp_x87_store(fp, out->k[axis], fp.add(fp.load(position->k[axis]), displacement));
    }
}

void ivp_x87_interpolate_point(const ivp_vec3r_t *first, const ivp_vec3r_t *last, ivp_real_t fraction,
                               ivp_vec3r_t *out)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 t = fp.load(fraction);
    const X87_Float80 s = fp.subtract(fp.load(1.0), t);
    for (int axis = 0; axis < 3; ++axis)
    {
        X87_Float80 a = fp.multiply(s, fp.load(first->k[axis]));
        X87_Float80 b = fp.multiply(t, fp.load(last->k[axis]));
        if (axis == 2)
        {
            a = ivp_x87_round_double(fp, a);
            b = ivp_x87_round_double(fp, b);
        }
        ivp_x87_store(fp, out->k[axis], fp.add(a, b));
    }
}

void ivp_x87_transform_point3d(const double rotation[3][3], const double origin[3],
                               const double point[3], double out[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 local[3] = {fp.load(point[0]), fp.load(point[1]), fp.load(point[2])};
    for (int axis = 0; axis < 3; ++axis)
        out[axis] = fp.store_double(ivp_x87_position_component<true>(fp, rotation[axis], origin[axis], local, axis));
}

#ifdef IVP_REAL_DOUBLE
static void ivp_x87_rotate_direction(X87_Arithmetic &fp, const ivp_mat3r_t *rotation,
                                     const ivp_vec3r_t *direction, ivp_vec3r_t *world)
{
    const X87_Float80 local[3] = {fp.load(direction->k[0]), fp.load(direction->k[1]),
                                 fp.load(direction->k[2])};
    for (int axis = 0; axis < 3; ++axis)
    {
        const double *row = rotation->rows[axis].k;
        const X87_Float80 x = fp.multiply(local[0], fp.load(row[0]));
        const X87_Float80 y = fp.multiply(local[1], fp.load(row[1]));
        const X87_Float80 z = fp.multiply(local[2], fp.load(row[2]));
        world->k[axis] = fp.store_double(axis == 0 ? fp.add(fp.add(z, y), x) :
                                                    fp.add(fp.add(x, z), y));
    }
}

double ivp_x87_edge_face_projection(const ivp_mat3r_t *edge_rotation, const ivp_vec3r_t *edge,
                                    const ivp_mat3r_t *face_rotation, const ivp_vec3r_t *normal)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    ivp_vec3r_t world_edge, world_normal;
    ivp_x87_rotate_direction(fp, edge_rotation, edge, &world_edge);
    ivp_x87_rotate_direction(fp, face_rotation, normal, &world_normal);
    return fp.store_double(ivp_x87_point_dot(fp, &world_edge, &world_normal));
}

static void ivp_x87_transform_edge(X87_Arithmetic &fp, const ivp_transform_r_t *transform,
                                   const ivp_vec3r_t edge[2], ivp_vec3r_t *point, ivp_vec3r_t *direction)
{
    const X87_Float80 local[3] = {fp.load(edge[0].k[0]), fp.load(edge[0].k[1]), fp.load(edge[0].k[2])};
    for (int axis = 0; axis < 3; ++axis)
        point->k[axis] = fp.store_double(ivp_x87_position_component<true>(
            fp, transform->rot.rows[axis].k, transform->pos.k[axis], local, axis));
    ivp_x87_rotate_direction(fp, &transform->rot, &edge[1], direction);
}

double ivp_x87_edge_edge_separation(const ivp_transform_r_t *first_transform, const ivp_vec3r_t first_edge[2],
                                    const ivp_transform_r_t *second_transform, const ivp_vec3r_t second_edge[2],
                                    double orientation)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    ivp_vec3r_t first_point, first_direction, second_point, second_direction;
    ivp_x87_transform_edge(fp, first_transform, first_edge, &first_point, &first_direction);
    ivp_x87_transform_edge(fp, second_transform, second_edge, &second_point, &second_direction);
    X87_Float80 cross[3];
    ivp_x87_cross_extended<false>(fp, &first_direction, &second_direction, cross);
    ivp_vec3r_t normal;
    for (int axis = 0; axis < 3; ++axis)
        normal.k[axis] = fp.store_double(cross[axis]);

    const X87_Float80 first_projection = ivp_x87_point_dot(fp, &normal, &first_point);
    const X87_Float80 second_projection = ivp_x87_point_dot(fp, &normal, &second_point);
    const X87_Float80 gap = fp.subtract(first_projection, second_projection);
    const float square = fp.store_float(ivp_x87_point_square(fp, normal.k));
    const X87_Float80 inverse_length = ivp_x87_isqrt(fp, fp.load(square), 2);
    return fp.store_double(fp.multiply(fp.multiply(gap, inverse_length), fp.load(orientation)));
}

double ivp_x87_point_plane_separation(const ivp_transform_r_t *point_transform,
                                      const ivp_vec3r_t *point, const ivp_transform_r_t *plane_transform,
                                      const ivp_vec3r_t *plane_point, const ivp_vec3r_t *plane_normal)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 local_point[3] = {fp.load(point->k[0]), fp.load(point->k[1]), fp.load(point->k[2])};
    const X87_Float80 local_plane[3] = {fp.load(plane_point->k[0]), fp.load(plane_point->k[1]),
                                      fp.load(plane_point->k[2])};
    X87_Float80 projection[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 world_point = ivp_x87_round_double(fp, ivp_x87_position_component<true>(
            fp, point_transform->rot.rows[axis].k, point_transform->pos.k[axis], local_point, axis));
        const double *row = plane_transform->rot.rows[axis].k;
        X87_Float80 world_plane = ivp_x87_position_component<false>(
            fp, row, plane_transform->pos.k[axis], local_plane, axis);
        if (axis != 2)
            world_plane = ivp_x87_round_double(fp, world_plane);
        X87_Float80 difference = fp.subtract(world_point, world_plane);
        // The Z projection uses the unrounded plane coordinate and displacement.
        if (axis != 2)
            difference = ivp_x87_round_double(fp, difference);
        const X87_Float80 x = fp.multiply(fp.load(row[0]), fp.load(plane_normal->k[0]));
        const X87_Float80 y = fp.multiply(fp.load(row[1]), fp.load(plane_normal->k[1]));
        const X87_Float80 z = fp.multiply(fp.load(row[2]), fp.load(plane_normal->k[2]));
        const X87_Float80 normal = ivp_x87_round_double(
            fp, axis == 2 ? fp.add(fp.add(y, x), z) : fp.add(fp.add(x, z), y));
        projection[axis] = fp.multiply(difference, normal);
    }
    return fp.store_double(fp.add(fp.add(projection[2], projection[1]), projection[0]));
}
#endif

void ivp_x87_transform_mesh_vertex(const ivp_transform_r_t *transform, const ivp_vec3f_t *vertex,
                                   ivp_vec3r_t *out)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 local[3] = {fp.load(vertex->k[0]), fp.load(vertex->k[1]), fp.load(vertex->k[2])};
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 value = ivp_x87_position_component<true>(
            fp, transform->rot.rows[axis].k, transform->pos.k[axis], local, axis);
        ivp_x87_store(fp, out->k[axis], value);
    }
}

/* The world position is stored before subtracting the target origin. Keep the
 * translated direction wide through the inverse rotation. */
void ivp_x87_transform_collision_point(const ivp_vec3f_t *point,
                                       const ivp_transform_r_t *source, const ivp_transform_r_t *target,
                                       ivp_vec3r_t *out)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 local[3] = {fp.load(point->k[0]), fp.load(point->k[1]), fp.load(point->k[2])};
    X87_Float80 translated[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 world = ivp_x87_position_component<false>(
            fp, source->rot.rows[axis].k, source->pos.k[axis], local, axis);
        translated[axis] = fp.subtract(ivp_x87_round_double(fp, world), fp.load(target->pos.k[axis]));
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        const X87_Float80 x = fp.multiply(fp.load(target->rot.rows[0].k[axis]), translated[0]);
        const X87_Float80 y = fp.multiply(fp.load(target->rot.rows[1].k[axis]), translated[1]);
        const X87_Float80 z = fp.multiply(fp.load(target->rot.rows[2].k[axis]), translated[2]);
        out->k[axis] = static_cast<ivp_real_t>(fp.store_double(fp.add(fp.add(z, y), x)));
    }
}

float ivp_x87_quat_rotation_axis(const ivp_vec3r_t *rotation, const ivp_mat3r_t *to_world,
                                 float inverse_step, ivp_vec3f_t *world_axis)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.load(rotation->k[0]), y = fp.load(rotation->k[1]), z = fp.load(rotation->k[2]);
    const X87_Float80 square = fp.add(fp.add(ivp_x87_square(fp, x), ivp_x87_square(fp, y)),
                                     ivp_x87_square(fp, z));
    if (fp.compare(square, fp.load(IVP_REF_DOUBLE_EPS)) != X87_Arithmetic::GREATER)
    {
        world_axis->k[0] = 1.0f;
        world_axis->k[1] = 0.0f;
        world_axis->k[2] = 0.0f;
        return 0.0f;
    }

    const X87_Float80 inverse = ivp_x87_isqrt(fp, square, 3);
    const X87_Float80 angle = fp.multiply(inverse, ivp_x87_round_double(fp, square));
    const X87_Float80 angle_squared = ivp_x87_square(fp, angle);
    const X87_Float80 angle_cubed = fp.multiply(angle_squared, angle);
    const X87_Float80 polynomial = fp.add(fp.add(
        fp.multiply(fp.multiply(angle_cubed, angle_squared), fp.load(0.40414f)),
        fp.multiply(angle_cubed, fp.load(1.0f / 6.0f))), angle);
    const X87_Float80 omega = fp.multiply(polynomial, fp.load(inverse_step));

    for (int axis = 0; axis < 3; ++axis)
    {
        const ivp_vec3r_t *row = &to_world->rows[axis];
        const X87_Float80 px = fp.multiply(fp.load(row->k[0]), x);
        const X87_Float80 py = fp.multiply(fp.load(row->k[1]), y);
        const X87_Float80 pz = fp.multiply(fp.load(row->k[2]), z);
        const X87_Float80 component = axis == 0 ? fp.add(fp.add(pz, py), px)
            : ivp_x87_round_double(fp, fp.add(fp.add(px, pz), py));
        world_axis->k[axis] = fp.store_float(fp.multiply(component, inverse));
    }

    return fp.store_float(fp.add(omega, omega));
}

static bool ivp_x87_quat_normalize(X87_Arithmetic &fp, double rotation[4])
{
    const X87_Float80 x = ivp_x87_square(fp, fp.load(rotation[0]));
    const X87_Float80 y = ivp_x87_square(fp, fp.load(rotation[1]));
    const X87_Float80 z = ivp_x87_square(fp, fp.load(rotation[2]));
    const X87_Float80 w = ivp_x87_square(fp, fp.load(rotation[3]));
    const X87_Float80 square = fp.add(fp.add(fp.add(x, y), z), w);
    if (fp.compare(square, fp.load(IVP_REF_DOUBLE_EPS)) != X87_Arithmetic::GREATER)
        return false;

    const X87_Float80 inverse = fp.divide(fp.load(1.0), fp.square_root(square));
    for (int i = 0; i < 4; ++i)
        rotation[i] = fp.store_double(fp.multiply(fp.load(rotation[i]), inverse));
    return true;
}

bool ivp_x87_quat_normalize(double rotation[4])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    return ivp_x87_quat_normalize(fp, rotation);
}

void ivp_x87_quat_to_mat3(const double rotation[4], ivp_mat3r_t *matrix)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x = fp.load(rotation[0]), y = fp.load(rotation[1]);
    const X87_Float80 z = fp.load(rotation[2]), w = fp.load(rotation[3]);
    const X87_Float80 one = fp.load(1.0);
    const X87_Float80 x2 = ivp_x87_round_double(fp, fp.add(x, x));
    const X87_Float80 y2 = ivp_x87_round_double(fp, fp.add(y, y));
    const X87_Float80 z2 = fp.add(z, z);
    const X87_Float80 xx = ivp_x87_round_double(fp, fp.multiply(x2, x));
    const X87_Float80 xy = ivp_x87_round_double(fp, fp.multiply(y2, x));
    const X87_Float80 xz = ivp_x87_round_double(fp, fp.multiply(z2, x));
    const X87_Float80 yy = ivp_x87_round_double(fp, fp.multiply(y2, y));
    const X87_Float80 yz = ivp_x87_round_double(fp, fp.multiply(z2, y));
    const X87_Float80 zz = ivp_x87_round_double(fp, fp.multiply(z2, z));
    const X87_Float80 wx = ivp_x87_round_double(fp, fp.multiply(x2, w));
    const X87_Float80 wy = ivp_x87_round_double(fp, fp.multiply(y2, w));
    const X87_Float80 wz = fp.multiply(z2, w);
    matrix->rows[0].k[0] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(one, fp.add(zz, yy))));
    matrix->rows[0].k[1] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(xy, wz)));
    matrix->rows[0].k[2] = static_cast<ivp_real_t>(fp.store_double(fp.add(wy, xz)));
    matrix->rows[1].k[0] = static_cast<ivp_real_t>(fp.store_double(fp.add(wz, xy)));
    matrix->rows[1].k[1] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(one, fp.add(zz, xx))));
    matrix->rows[1].k[2] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(yz, wx)));
    matrix->rows[2].k[0] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(xz, wy)));
    matrix->rows[2].k[1] = static_cast<ivp_real_t>(fp.store_double(fp.add(wx, yz)));
    matrix->rows[2].k[2] = static_cast<ivp_real_t>(fp.store_double(fp.subtract(one, fp.add(yy, xx))));
}

template<class MatrixElement>
static void ivp_x87_quat_from_rotation(X87_Arithmetic &fp, const MatrixElement &get, double rotation[4])
{
    const X87_Float80 zero = fp.load(0.0), one = fp.load(1.0), half = fp.load(0.5);
    const auto element = [&](int row, int column) { return fp.load(get(row, column)); };
    const X87_Float80 trace = fp.add(fp.add(element(2, 2), element(0, 0)), element(1, 1));
    if (fp.compare(trace, zero) == X87_Arithmetic::GREATER)
    {
        const X87_Float80 root = fp.square_root(fp.add(trace, one));
        const X87_Float80 scale = fp.divide(half, root);
        rotation[3] = fp.store_double(fp.multiply(root, half));
        rotation[0] = fp.store_double(fp.multiply(fp.subtract(element(2, 1), element(1, 2)), scale));
        rotation[1] = fp.store_double(fp.multiply(fp.subtract(element(0, 2), element(2, 0)), scale));
        rotation[2] = fp.store_double(fp.multiply(fp.subtract(element(1, 0), element(0, 1)), scale));
    }
    else
    {
        const int next[3] = {1, 2, 0};
        int i = get(1, 1) > get(0, 0) ? 1 : 0;
        if (get(2, 2) > get(i, i))
            i = 2;
        const int j = next[i], k = next[j];
        const X87_Float80 root = fp.square_root(fp.add(
            fp.subtract(element(i, i), fp.add(element(k, k), element(j, j))), one));
        const X87_Arithmetic::Relation relation = fp.compare(root, zero);
        const X87_Float80 scale = relation == X87_Arithmetic::EQUAL ||
            relation == X87_Arithmetic::UNORDERED ? root : fp.divide(half, root);
        rotation[i] = fp.store_double(fp.multiply(root, half));
        rotation[3] = fp.store_double(fp.multiply(fp.subtract(element(k, j), element(j, k)), scale));
        rotation[j] = fp.store_double(fp.multiply(fp.add(element(j, i), element(i, j)), scale));
        rotation[k] = fp.store_double(fp.multiply(fp.add(element(k, i), element(i, k)), scale));
    }
    ivp_x87_quat_normalize(fp, rotation);
}

void ivp_x87_quat_from_mat3(double rotation[4], const ivp_mat3r_t *matrix)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    ivp_x87_quat_from_rotation(fp, [&](int row, int col) { return matrix->rows[row].k[col]; },
                               rotation);
}

void ivp_x87_constraint_rotation_error(const double relative_rotation[3][3], double error[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    double rotation[4];
    ivp_x87_quat_from_rotation(fp, [&](int row, int col) { return relative_rotation[row][col]; },
                               rotation);
    const X87_Float80 cubic = fp.load(0.24f), quintic = fp.load(0.58f);
    // The X polynomial writes its linear term and intermediate powers to binary32.
    const X87_Float80 x = fp.load(fp.store_float(fp.load(rotation[0])));
    const X87_Float80 square_x = fp.multiply(fp.load(rotation[0]), x);
    const X87_Float80 cube_x = fp.multiply(square_x, x);
    const X87_Float80 fifth_x = fp.multiply(fp.multiply(cube_x, fp.load(fp.store_float(square_x))),
                                            quintic);
    error[0] = fp.store_double(fp.add(fp.add(fifth_x,
        fp.multiply(fp.load(fp.store_float(cube_x)), cubic)), fp.add(x, x)));
    for (int axis = 1; axis < 3; ++axis)
    {
        const X87_Float80 value = fp.load(rotation[axis]);
        const X87_Float80 square = fp.multiply(value, value);
        const X87_Float80 cube = fp.multiply(square, value);
        const X87_Float80 correction = fp.add(fp.multiply(fp.multiply(cube, square), quintic),
                                              fp.multiply(cube, cubic));
        error[axis] = fp.store_double(fp.add(fp.add(value, value), correction));
    }

    if (rotation[3] < 0.0)
        for (int axis = 0; axis < 3; ++axis)
            error[axis] = -error[axis];
}

void ivp_x87_quat_set_fast_multiple_with_clip(ivp_real_t out[4], const float angles[3],
                                              double factor)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 one = fp.load(1.0);
    const X87_Float80 half_factor = fp.multiply(fp.load(factor), fp.load(0.5));
    X87_Float80 x = ivp_x87_round_double(fp, fp.sine(fp.multiply(fp.load(angles[0]), half_factor)));
    X87_Float80 y = ivp_x87_round_double(fp, fp.sine(fp.multiply(fp.load(angles[1]), half_factor)));
    X87_Float80 z = fp.sine(fp.multiply(half_factor, fp.load(angles[2])));
    X87_Float80 square = fp.add(fp.add(ivp_x87_square(fp, x), ivp_x87_square(fp, y)), ivp_x87_square(fp, z));
    if (fp.compare(square, one) == X87_Arithmetic::GREATER)
    {
        const X87_Float80 scale = fp.divide(fp.load(1.0 - IVP_REF_RES_EPS), fp.square_root(square));
        x = ivp_x87_round_double(fp, fp.multiply(scale, x));
        y = ivp_x87_round_double(fp, fp.multiply(scale, y));
        z = fp.multiply(scale, ivp_x87_round_double(fp, z));
        square = fp.add(fp.add(ivp_x87_square(fp, z), ivp_x87_square(fp, y)), ivp_x87_square(fp, x));
    }
    out[0] = static_cast<ivp_real_t>(fp.store_double(x));
    out[1] = static_cast<ivp_real_t>(fp.store_double(y));
    out[2] = static_cast<ivp_real_t>(fp.store_double(z));
    out[3] = static_cast<ivp_real_t>(fp.store_double(fp.square_root(fp.subtract(one, square))));
}

void ivp_x87_surface_radius_and_deviation(const float mass_center[3], const float center[3],
                                          float radius, int deviation, float step,
                                          float *out_radius, float *out_deviation)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 offset[3];
    for (int axis = 0; axis < 3; ++axis)
        offset[axis] = fp.subtract(fp.load(mass_center[axis]), fp.load(center[axis]));

    const X87_Float80 squared_shift = fp.add(fp.add(ivp_x87_square(fp, offset[0]),
                                                     ivp_x87_square(fp, offset[1])),
                                              ivp_x87_square(fp, offset[2]));
    const X87_Float80 shift = fp.square_root(squared_shift);
    const X87_Float80 bound = fp.load(radius);
    const X87_Float80 scaled_deviation = fp.multiply(
        fp.multiply(fp.load(static_cast<float>(deviation)), fp.load(step)), bound);
    *out_radius = fp.store_float(fp.add(bound, shift));
    *out_deviation = fp.store_float(fp.add(scaled_deviation, shift));
}

/* Preserve the triangle normal's extended Z component for the signed volume
 * and squared area. X and Y are rounded to float before either product. */
void ivp_x87_inertia_mass_triangle(const float p0[3], const float p1[3], const float p2[3],
                                   double *sum_det, double *sum_squared_area, float sum_mass[3])
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    X87_Float80 a[3], b[3];
    for (int i = 0; i < 3; ++i)
    {
        a[i] = ivp_x87_round_double(fp, fp.subtract(fp.load(p2[i]), fp.load(p0[i])));
        b[i] = ivp_x87_round_double(fp, fp.subtract(fp.load(p1[i]), fp.load(p0[i])));
    }
    const X87_Float80 x = fp.load(fp.store_float(fp.subtract(fp.multiply(b[1], a[2]), fp.multiply(b[2], a[1]))));
    const X87_Float80 y = fp.load(fp.store_float(fp.subtract(fp.multiply(b[2], a[0]), fp.multiply(b[0], a[2]))));
    const X87_Float80 z = fp.subtract(fp.multiply(b[0], a[1]), fp.multiply(b[1], a[0]));
    const X87_Float80 det = fp.add(fp.add(fp.multiply(x, fp.load(p0[0])),
        fp.multiply(z, fp.load(p0[2]))), fp.multiply(y, fp.load(p0[1])));
    const X87_Float80 squared_area = fp.add(fp.add(ivp_x87_square(fp, z), ivp_x87_square(fp, y)),
                                          ivp_x87_square(fp, x));
    *sum_squared_area = fp.store_double(fp.add(squared_area, fp.load(*sum_squared_area)));
    *sum_det = fp.store_double(fp.add(det, fp.load(*sum_det)));

    const X87_Float80 mass_factor = fp.multiply(det, fp.load(0.25));
    const float *vertices[3] = {p0, p1, p2};
    double previous_y = 0.0;
    for (int vertex = 0; vertex < 3; ++vertex)
    {
        X87_Float80 mass[3];
        for (int axis = 0; axis < 3; ++axis)
        {
            const X87_Float80 previous = axis == 1 && vertex == 1 ?
                fp.load(previous_y) : fp.load(sum_mass[axis]);
            mass[axis] = fp.add(fp.multiply(fp.load(vertices[vertex][axis]), mass_factor), previous);
            if (axis != 0)
                mass[axis] = ivp_x87_round_double(fp, mass[axis]);
        }
        previous_y = fp.store_double(mass[1]);
        for (int axis = 0; axis < 3; ++axis)
            sum_mass[axis] = fp.store_float(mass[axis]);
    }
}

/* Binary64 endpoint powers are kept separate from the extended cubic integral
 * to reproduce cancellation along edges close to the integration axis. */
void ivp_x87_inertia_edge_integral(float p0_x, float p0_y, float p0_z,
    float p1_x, double vector_x, double vector_y, double vector_z,
    bool triangle_area, double fzdy, double fydz, double *area)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 x0 = fp.load(p0_x), x1 = fp.load(p1_x);
    const X87_Float80 slope_y = ivp_x87_round_double(fp, fp.divide(fp.load(vector_y), fp.load(vector_x)));
    const X87_Float80 offset_y = fp.subtract(fp.load(p0_y), fp.multiply(x0, slope_y));
    const X87_Float80 stored_y = ivp_x87_round_double(fp, offset_y);
    X87_Float80 a, b, c;
    if (triangle_area)
    {
        a = fp.multiply(fp.multiply(offset_y, offset_y), fp.load(fzdy));
        b = fp.multiply(fp.multiply(stored_y, slope_y), fp.load(fzdy));
        b = fp.add(b, b);
        c = fp.multiply(fp.multiply(slope_y, slope_y), fp.load(fzdy));
    }
    else
    {
        const X87_Float80 slope_z = ivp_x87_round_double(fp, fp.divide(fp.load(vector_z), fp.load(vector_x)));
        const X87_Float80 offset_z = fp.subtract(fp.load(p0_z), fp.multiply(x0, slope_z));
        const X87_Float80 stored_z = ivp_x87_round_double(fp, offset_z);
        a = fp.multiply(fp.add(offset_y, fp.multiply(offset_z, fp.load(fydz))), offset_z);
        b = fp.multiply(fp.multiply(stored_z, slope_z), fp.load(fydz));
        b = fp.add(fp.add(fp.add(b, b), fp.multiply(slope_z, stored_y)), fp.multiply(stored_z, slope_y));
        c = fp.multiply(fp.add(fp.multiply(slope_z, fp.load(fydz)), slope_y), slope_z);
    }
    const X87_Float80 square1 = fp.multiply(x1, x1);
    const X87_Float80 square0 = ivp_x87_round_double(fp, fp.multiply(x0, x0));
    const X87_Float80 integral2 = ivp_x87_round_double(fp, fp.multiply(fp.subtract(square1, square0), fp.load(0.5)));
    const X87_Float80 cube1 = ivp_x87_round_double(fp, fp.multiply(square1, x1));
    const X87_Float80 cube0 = ivp_x87_round_double(fp, fp.multiply(square0, x0));
    const X87_Float80 integral3 = fp.multiply(fp.subtract(cube1, cube0), fp.load(1.0 / 3.0));
    const X87_Float80 fourth1 = ivp_x87_round_double(fp, fp.multiply(cube1, x1));
    const X87_Float80 fourth0 = ivp_x87_round_double(fp, fp.multiply(cube0, x0));
    const X87_Float80 integral4 = ivp_x87_round_double(fp, fp.multiply(fp.subtract(fourth1, fourth0), fp.load(0.25)));
    const X87_Float80 integral5 = fp.multiply(fp.subtract(fp.multiply(fourth1, x1), fp.multiply(fourth0, x0)), fp.load(0.2f));
    area[0] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(fp.subtract(x1, x0), a),
        fp.multiply(integral3, c)), fp.multiply(integral2, b)), fp.load(area[0])));
    area[1] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(integral4, c),
        fp.multiply(integral3, b)), fp.multiply(integral2, a)), fp.load(area[1])));
    area[2] = fp.store_double(fp.add(fp.add(fp.add(fp.multiply(integral5, c),
        fp.multiply(integral4, b)), fp.multiply(integral3, a)), fp.load(area[2])));
}

static X87_Float80 ivp_x87_quat_sine(X87_Arithmetic &fp, const X87_Float80 &angle)
{
    const X87_Float80 cube = fp.multiply(fp.multiply(angle, angle), angle);
    return fp.subtract(angle, fp.multiply(cube, fp.load(1.0f / 6.0f)));
}

void ivp_x87_quat_set_very_fast_multiple(ivp_real_t out[4], const float angles[3],
                                        double factor)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC> fp;
    const X87_Float80 half_factor = fp.multiply(fp.load(factor), fp.load(0.5));
    const double x = fp.store_double(ivp_x87_quat_sine(fp, fp.multiply(fp.load(angles[0]), half_factor)));
    const double y = fp.store_double(ivp_x87_quat_sine(fp, fp.multiply(fp.load(angles[1]), half_factor)));
    const X87_Float80 z = ivp_x87_quat_sine(fp, fp.multiply(half_factor, fp.load(angles[2])));
    const X87_Float80 stored_x = fp.load(x), stored_y = fp.load(y);
    const X87_Float80 square = fp.add(fp.add(fp.multiply(z, z), fp.multiply(stored_y, stored_y)),
                                     fp.multiply(stored_x, stored_x));
    ivp_x87_store(fp, out[0], stored_x);
    ivp_x87_store(fp, out[1], stored_y);
    ivp_x87_store(fp, out[2], z);
    ivp_x87_store(fp, out[3], fp.square_root(fp.subtract(fp.load(1.0), square)));
}

#endif

#if IVP_X87_ARITHMETIC_TRANSCENDENTAL
double ivp_sind(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.sine(fp.load(value)));
}

double ivp_cosd(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.cosine(fp.load(value)));
}

double ivp_acosd(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.arccosine(fp.load(value)));
}

double ivp_asind(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.arcsine(fp.load(value)));
}

double ivp_atand(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.arctangent(fp.load(value), fp.load(1.0)));
}

double ivp_expd(double value)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.exponential(fp.load(value)));
}

double ivp_atan2d(double y, double x)
{
    ivp_x87_arithmetic<IVP_X87_ARITHMETIC_TRANSCENDENTAL> fp;
    return fp.scalar_result(fp.arctangent(fp.load(y), fp.load(x)));
}
#endif
