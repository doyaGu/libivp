#include "math/ivp_x87.h"

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
