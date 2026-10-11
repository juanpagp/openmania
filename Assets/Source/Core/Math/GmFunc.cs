using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The scalar helpers of the game-math layer, with the disassembly's
    /// exact operand groupings and epsilon constants (built from bits, never
    /// decimal literals).
    /// </summary>
    public static class GmFunc
    {
        /// <summary>x87_dot3_left(a0,b0,a1,b1,a2,b2) = ((a0*b0 + a1*b1) + a2*b2).</summary>
        public static float Dot3Left(float a0, float b0, float a1, float b1, float a2, float b2)
        {
            return (float)(double)(((float)(double)(((float)(double)(a0 * b0)) + ((float)(double)(a1 * b1)))) + ((float)(double)(a2 * b2)));
        }

        /// <summary>True when the value is not NaN.</summary>
        public static bool GmfuncIsANumber(float value)
        {
            return !float.IsNaN(value);
        }

        /// <summary>Sign from the float sign bit directly (signed zero/NaN included).</summary>
        public static float GmfuncSign(float value)
        {
            uint bits = (uint)BitConverter.SingleToInt32Bits(value);
            if ((bits & 0x8000_0000u) != 0)
            {
                return -1.0f;
            }
            return 1.0f;
        }

        /// <summary>__CIacos: acos via atan2 of the sqrt(1-v^2) guard.</summary>
        public static float GmfuncAcos(float value)
        {
            float product = (float)(double)(((float)(double)(1.0f + value)) * ((float)(double)(1.0f - value)));
            float root = Fp.Sqrt(product);
            return Fp.X87Atan2(root, value);
        }

        /// <summary>asin with the clamp before the __CIasin dispatcher.</summary>
        public static float GmfuncAsinSafe(float value)
        {
            if (value < -0.999999f)
            {
                return -1.5707964f;
            }
            if (value > 0.999999f)
            {
                return 1.5707964f;
            }
            float product = (float)(double)(((float)(double)(1.0f + value)) * ((float)(double)(1.0f - value)));
            float root = Fp.Sqrt(product);
            return Fp.X87Atan2(value, root);
        }

        /// <summary>The three-argument modulo with the f64 fmod core.</summary>
        public static float GmfuncMod(float value, float lower, float upper)
        {
            if (lower < value && value < upper)
            {
                return value;
            }
            float period = (float)(double)(upper - lower);
            float relative = (float)(double)(value - lower);
            float result = Fp.X87Fmod(relative, period);
            if (result < 0.0f)
            {
                result = (float)(double)(result + period);
            }
            return (float)(double)(result + lower);
        }

        /// <summary>
        /// Normalizes a vector in place only when its squared length exceeds
        /// the magnitude epsilon (built from bits: 0x1.b7cdfcp-34f).
        /// </summary>
        public static void NormalizeVec3IfNonzero(ref GmVec3 value)
        {
            // 9.999999439624929e-11
            float lengthSq = (float)(double)(((float)(double)(((float)(double)(value.y * value.y)) + ((float)(double)(value.x * value.x)))) + ((float)(double)(value.z * value.z)));
            if (lengthSq > EpsilonMagnitude)
            {
                float length = Fp.Sqrt(lengthSq);
                float inverse = (float)(double)(1.0f / length);
                value.x = (float)(double)(inverse * value.x);
                value.y = (float)(double)(value.y * inverse);
                value.z = (float)(double)(value.z * inverse);
            }
        }

        /// <summary>9.999999439624929e-11f (bits 0x2EDBE6FE) — the normalize/length magnitude epsilon.</summary>
        public static readonly float EpsilonMagnitude =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EDB_E6FE));

        /// <summary>
        /// Signed angle around the global Y axis between two vectors. The
        /// guard scale (0.9999900000002526 as f64) and the threshold
        /// (9.999999747378752e-06f) are load-bearing.
        /// </summary>
        public static float Gmvec3GetAngle(ref GmVec3 a, ref GmVec3 b)
        {
            float dot = (float)(double)(((float)(double)(((float)(double)(a.x * b.x)) + ((float)(double)(a.y * b.y)))) + ((float)(double)(a.z * b.z)));
            // 0.9999900000002526 f64: bits 0x3FEFFFD0_00000000? No — decimal
            // literal below is exact for the f64 value used.
            double guardScale = 0.9999900000002526;
            float guardedDot = (float)((double)dot * guardScale);
            float angle = GmfuncAcos(guardedDot);
            // 9.999999747378752e-06f
            float threshold = BitConverter.Int32BitsToSingle(unchecked((int)0x3727_C5AC));
            if (angle > threshold)
            {
                GmVec3 an = a;
                GmVec3 bn = b;
                NormalizeVec3IfNonzero(ref an);
                NormalizeVec3IfNonzero(ref bn);

                float crossZ = (float)(double)(((float)(double)(bn.y * an.x)) - ((float)(double)(an.y * bn.x)));
                float crossY = (float)(double)(((float)(double)(an.z * bn.x)) - ((float)(double)(an.x * bn.z)));
                float crossX = (float)(double)(((float)(double)(bn.z * an.y)) - ((float)(double)(an.z * bn.y)));
                float signValue = (float)(double)(((float)(double)(((float)(double)(crossZ * 0.0f)) + crossY)) + ((float)(double)(crossX * 0.0f)));
                if (signValue < 0.0f)
                {
                    angle = -angle;
                }
            }
            return angle;
        }
    }
}
