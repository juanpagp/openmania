using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The floating-point policy of the simulation: the handful of operations
    /// where plain C# arithmetic would silently diverge from the reference
    /// semantics (truncating float-to-int conversions, the unsigned-to-float
    /// two-step, the libm-backed transcendentals, exact fmod).
    /// </summary>
    public static class Fp
    {
        /// <summary>
        /// The game's <c>_ftol</c>/FISTP semantics: truncation toward zero
        /// with <see cref="int.MinValue"/> (0x80000000, the integer
        /// indefinite) for NaN and out-of-range values. Rust's
        /// <c>as i32</c> saturates instead — never use a bare cast on
        /// simulation values.
        /// </summary>
        public static int Ftol(double x)
        {
            if (x > -2147483649.0d && x < 2147483648.0d)
            {
                return (int)x;
            }
            return int.MinValue;
        }

        /// <summary>
        /// The game's unsigned-int-to-float conversion: a signed load plus a
        /// conditional +2^32, each step rounded to f32. The two-step form
        /// rounds differently from a direct conversion on some inputs, so it
        /// is kept verbatim. 2^32 is built from bits, not a decimal literal.
        /// </summary>
        public static float U32ToX87Float(uint v)
        {
            // 0x1p32 == 0x4F800000
            float twoPow32 = BitConverter.Int32BitsToSingle(unchecked((int)0x4F800000));
            int signed = unchecked((int)v);
            float r = signed;
            if (signed < 0)
            {
                r = (float)(double)(r + twoPow32);
            }
            return r;
        }

        /// <summary>The CRT's double <c>sin</c>, rounded to f32 (libm).</summary>
        public static float X87Sin(float x)
        {
            return (float)Libm.Sin(x);
        }

        /// <summary>The CRT's double <c>cos</c>, rounded to f32 (libm).</summary>
        public static float X87Cos(float x)
        {
            return (float)Libm.Cos(x);
        }

        /// <summary>The CRT's double <c>exp</c>, rounded to f32 (libm).</summary>
        public static float X87Exp(float x)
        {
            return (float)Libm.Exp(x);
        }

        /// <summary>The CRT's double <c>atan2</c>, rounded to f32 (libm).</summary>
        public static float X87Atan2(float y, float x)
        {
            return (float)Libm.Atan2(y, x);
        }

        /// <summary>
        /// The CRT's double <c>fmod</c>, rounded to f32. C#'s <c>%</c> on
        /// doubles is the same exact fmod for finite operands (including the
        /// invalid-operation canonical NaN), but it canonicalizes NaN inputs
        /// instead of propagating them — the NaN-input selection is spelled
        /// out to match the reference bit-for-bit.
        /// </summary>
        public static float X87Fmod(float value, float period)
        {
            double v = value;
            double p = period;
            if (double.IsNaN(v))
            {
                return (float)Libm.Quiet(v);
            }
            if (double.IsNaN(p))
            {
                return (float)Libm.Quiet(p);
            }
            return (float)(v % p);
        }

        /// <summary>
        /// The reference square root: <c>(float)Math.Sqrt((double)x)</c>.
        /// Double sqrt of a binary32 is provably correctly rounded to f32 in
        /// one step; MathF.Sqrt and Unity's Mathf must never be used.
        /// </summary>
        public static float Sqrt(float x)
        {
            return (float)Math.Sqrt(x);
        }

        /// <summary>Rust <c>as u32</c> on f32: saturating, NaN to 0.</summary>
        public static uint RustAsU32(float x)
        {
            if (float.IsNaN(x)) return 0;
            if (x >= 4294967295.0f) return uint.MaxValue;
            if (x <= 0.0f) return 0;
            return (uint)x;
        }

        /// <summary>Rust <c>as i32</c> on f32: saturating, NaN to 0.</summary>
        public static int RustAsI32(float x)
        {
            if (float.IsNaN(x)) return 0;
            if (x >= 2147483647.0f) return int.MaxValue;
            if (x <= -2147483648.0f) return int.MinValue;
            return (int)x;
        }

        /// <summary>
        /// Rust <c>f32::min</c>: returns the non-NaN operand when one input
        /// is NaN (C#'s <see cref="Math.Min"/> propagates NaN instead).
        /// </summary>
        public static float RustMin(float a, float b)
        {
            if (float.IsNaN(a)) return b;
            if (float.IsNaN(b)) return a;
            return a < b ? a : b;
        }

        /// <summary>Rust <c>f32::max</c>: returns the non-NaN operand.</summary>
        public static float RustMax(float a, float b)
        {
            if (float.IsNaN(a)) return b;
            if (float.IsNaN(b)) return a;
            return a > b ? a : b;
        }
    }
}
