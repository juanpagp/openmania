using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// A 3-component float vector (12 bytes in the reference layout). A value
    /// type: assignment copies, exactly like the reference's Copy struct.
    /// Field names x, y, z mirror the reference.
    /// </summary>
    [Serializable]
    public struct GmVec3
    {
        public float x;
        public float y;
        public float z;

        public GmVec3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static readonly GmVec3 Zero = default(GmVec3);

        /// <summary>out = A * v + A.t (A is a GmIso4). Operand order kept.</summary>
        public void SetMultIso4(ref GmVec3 v, ref GmIso4 a)
        {
            x = (float)(double)(GmFunc.Dot3Left(a.m1, v.y, v.x, a.m0, a.m2, v.z) + a.t0);
            y = (float)(double)(GmFunc.Dot3Left(a.m3, v.x, a.m4, v.y, a.m5, v.z) + a.t1);
            z = (float)(double)(GmFunc.Dot3Left(a.m6, v.x, a.m7, v.y, a.m8, v.z) + a.t2);
        }

        /// <summary>out = M * v (M is a GmMat3, no translation).</summary>
        public void SetMultMat3(ref GmVec3 v, ref GmMat3 m)
        {
            x = GmFunc.Dot3Left(m.m1, v.y, m.m0, v.x, m.m2, v.z);
            y = GmFunc.Dot3Left(m.m3, v.x, m.m4, v.y, m.m5, v.z);
            z = GmFunc.Dot3Left(m.m6, v.x, m.m7, v.y, m.m8, v.z);
        }

        /// <summary>self = A * self + A.t. The copy-first dance is preserved.</summary>
        public void MultIso4(ref GmIso4 a)
        {
            float lx = x, ly = y, lz = z;
            x = (float)(double)(((float)(double)(((float)(double)(a.m2 * lz)) + ((float)(double)(((float)(double)(a.m0 * lx)) + ((float)(double)(a.m1 * ly)))))) + a.t0);
            y = (float)(double)(GmFunc.Dot3Left(a.m3, lx, a.m4, ly, a.m5, lz) + a.t1);
            z = (float)(double)(((float)(double)(((float)(double)(lz * a.m8)) + ((float)(double)(((float)(double)(ly * a.m7)) + ((float)(double)(lx * a.m6)))))) + a.t2);
        }

        /// <summary>self = M^T * self.</summary>
        public void MultTranspose(ref GmMat3 m)
        {
            float lx = x, ly = y, lz = z;
            x = (float)(double)(((float)(double)(m.m6 * lz)) + ((float)(double)(((float)(double)(m.m0 * lx)) + ((float)(double)(m.m3 * ly)))));
            y = GmFunc.Dot3Left(m.m1, lx, m.m4, ly, m.m7, lz);
            z = (float)(double)(((float)(double)(lz * m.m8)) + ((float)(double)(((float)(double)(ly * m.m5)) + ((float)(double)(lx * m.m2)))));
        }

        /// <summary>self = M * self (in-place form).</summary>
        public void MultMat3(ref GmMat3 m)
        {
            float lx = x, ly = y, lz = z;
            float outY = GmFunc.Dot3Left(m.m3, lx, m.m4, ly, m.m5, lz);
            float outZ = GmFunc.Dot3Left(m.m6, lx, m.m7, ly, m.m8, lz);
            x = GmFunc.Dot3Left(m.m1, ly, m.m0, lx, m.m2, lz);
            y = outY;
            z = outZ;
        }

        /// <summary>Raw little-endian bytes (x, y, z).</summary>
        public void ToBytes(byte[] b, int o)
        {
            BitsUtil.PutF32(b, o, x);
            BitsUtil.PutF32(b, o + 4, y);
            BitsUtil.PutF32(b, o + 8, z);
        }

        public static GmVec3 FromBytes(byte[] b, int o)
        {
            return new GmVec3(
                BitsUtil.GetF32(b, o),
                BitsUtil.GetF32(b, o + 4),
                BitsUtil.GetF32(b, o + 8));
        }

        /// <summary>
        /// Bit-exact equality (NaN != NaN, -0 != +0): used by golden
        /// comparisons only; gameplay equality is the C# == operator, which
        /// matches the reference's derived PartialEq.
        /// </summary>
        public bool BitsEqual(ref GmVec3 other)
        {
            return BitConverter.SingleToInt32Bits(x) == BitConverter.SingleToInt32Bits(other.x)
                && BitConverter.SingleToInt32Bits(y) == BitConverter.SingleToInt32Bits(other.y)
                && BitConverter.SingleToInt32Bits(z) == BitConverter.SingleToInt32Bits(other.z);
        }
    }

    /// <summary>
    /// A 3x3 row-major matrix (36 bytes). The reference stores m: [f32; 9];
    /// arrays are not allowed inside C# value types (they alias on copy), so
    /// the elements are expanded to named fields with an indexer.
    /// </summary>
    [Serializable]
    public struct GmMat3
    {
        // Row-major: rows {m0..m2}, {m3..m5}, {m6..m8}.
        public float m0;
        public float m1;
        public float m2;
        public float m3;
        public float m4;
        public float m5;
        public float m6;
        public float m7;
        public float m8;

        public float this[int i]
        {
            get
            {
                switch (i)
                {
                    case 0: return m0;
                    case 1: return m1;
                    case 2: return m2;
                    case 3: return m3;
                    case 4: return m4;
                    case 5: return m5;
                    case 6: return m6;
                    case 7: return m7;
                    default: return m8;
                }
            }
            set
            {
                switch (i)
                {
                    case 0: m0 = value; break;
                    case 1: m1 = value; break;
                    case 2: m2 = value; break;
                    case 3: m3 = value; break;
                    case 4: m4 = value; break;
                    case 5: m5 = value; break;
                    case 6: m6 = value; break;
                    case 7: m7 = value; break;
                    default: m8 = value; break;
                }
            }
        }

        /// <summary>The identity matrix (the reference Default is NOT zero).</summary>
        public static readonly GmMat3 Identity = new GmMat3
        {
            m0 = 1.0f,
            m4 = 1.0f,
            m8 = 1.0f,
        };

        /// <summary>Builds a rotation matrix from a quaternion.</summary>
        /// <remarks>
        /// The four arguments are the quaternion in the byte order
        /// (w, x, y, z), scalar first — call sites pass
        /// (quat.x, quat.y, quat.z, quat.w), the raw field/byte order.
        /// </remarks>
        public void SetFromQuat(float a, float b, float c, float d)
        {
            float twoY = (float)((double)b * 2.0);
            float twoZ = (float)((double)c * 2.0);
            float twoW = (float)((double)d * 2.0);
            float xy2 = (float)(double)(a * twoY);
            float xz2 = (float)(double)(a * twoZ);
            float xw2 = (float)(double)(a * twoW);
            float yy2 = (float)(double)(twoY * b);
            float yz2 = (float)(double)(b * twoZ);
            float yw2 = (float)(double)(b * twoW);
            float zz2 = (float)(double)(twoZ * c);
            float zw2 = (float)(double)(c * twoW);
            float ww2 = (float)(double)(twoW * d);

            m0 = (float)(double)(((float)(double)(1.0f - zz2)) - ww2);
            m3 = (float)(double)(yz2 + xw2);
            m6 = (float)(double)(yw2 - xz2);
            m1 = (float)(double)(yz2 - xw2);
            m4 = (float)(double)(((float)(double)(1.0f - yy2)) - ww2);
            m7 = (float)(double)(zw2 + xy2);
            m2 = (float)(double)(yw2 + xz2);
            m5 = (float)(double)(zw2 - xy2);
            m8 = (float)(double)(((float)(double)(1.0f - yy2)) - zz2);
        }

        /// <summary>self = src^T.</summary>
        public void SetTranspose(ref GmMat3 src)
        {
            m0 = src.m0;
            m4 = src.m4;
            m8 = src.m8;
            m1 = src.m3;
            m3 = src.m1;
            m2 = src.m6;
            m6 = src.m2;
            m5 = src.m7;
            m7 = src.m5;
        }

        /// <summary>self = self * B.</summary>
        public void Mult(ref GmMat3 b)
        {
            float a0 = m0, a1 = m1, a2 = m2, a3 = m3, a4 = m4, a5 = m5, a6 = m6, a7 = m7, a8 = m8;
            m0 = (float)(double)(((float)(double)(b.m2 * a6)) + ((float)(double)(((float)(double)(b.m1 * a3)) + ((float)(double)(b.m0 * a0)))));
            m1 = GmFunc.Dot3Left(b.m1, a4, b.m0, a1, b.m2, a7);
            m2 = GmFunc.Dot3Left(b.m0, a2, b.m1, a5, b.m2, a8);
            m3 = GmFunc.Dot3Left(b.m4, a3, a0, b.m3, b.m5, a6);
            m4 = GmFunc.Dot3Left(b.m4, a4, b.m3, a1, b.m5, a7);
            m5 = GmFunc.Dot3Left(b.m4, a5, b.m3, a2, b.m5, a8);
            m6 = (float)(double)(((float)(double)(a6 * b.m8)) + ((float)(double)(((float)(double)(a0 * b.m6)) + ((float)(double)(a3 * b.m7)))));
            m7 = GmFunc.Dot3Left(a1, b.m6, a4, b.m7, b.m8, a7);
            m8 = GmFunc.Dot3Left(b.m6, a2, b.m7, a5, b.m8, a8);
        }

        /// <summary>self = A, then Mult(B).</summary>
        public void SetMult(ref GmMat3 a, ref GmMat3 b)
        {
            this = a;
            Mult(ref b);
        }

        /// <summary>self = M^T * self, exact operand schedule.</summary>
        public void MultTranspose(ref GmMat3 m)
        {
            // Copy self first (the reference copies the whole array up front).
            float s0 = m0, s1 = m1, s2 = m2, s3 = m3, s4 = m4, s5 = m5, s6 = m6, s7 = m7, s8 = m8;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    this[i * 3 + j] = GmFunc.Dot3Left(
                        m[i], At(s0, s1, s2, s3, s4, s5, s6, s7, s8, j),
                        m[3 + i], At(s0, s1, s2, s3, s4, s5, s6, s7, s8, 3 + j),
                        m[6 + i], At(s0, s1, s2, s3, s4, s5, s6, s7, s8, 6 + j));
                }
            }
        }

        private static float At(float s0, float s1, float s2, float s3, float s4, float s5, float s6, float s7, float s8, int i)
        {
            switch (i)
            {
                case 0: return s0;
                case 1: return s1;
                case 2: return s2;
                case 3: return s3;
                case 4: return s4;
                case 5: return s5;
                case 6: return s6;
                case 7: return s7;
                default: return s8;
            }
        }
    }

    /// <summary>
    /// Quaternion bytes (x, y, z, w) in field order. The reference stores the
    /// rotation as (w, x, y, z) scalar-first in the field NAMED x — see
    /// <see cref="GmQuat.SetFromMat3"/>; the bytes never lie.
    /// </summary>
    [Serializable]
    public struct GmQuat
    {
        public float x;
        public float y;
        public float z;
        public float w;

        /// <summary>
        /// Quaternion from a rotation matrix, scalar-first as the dyna state
        /// stores it (Shepperd's method with the axis successor table).
        /// </summary>
        public void SetFromMat3(ref GmMat3 mat)
        {
            // NEXT: DAT_00d1a86c
            // q[0..4] array over the byte layout (x,y,z,w fields in order);
            // the game writes the scalar into q[0].
            float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
            float trace = (float)(double)(((float)(double)(mat.m0 + mat.m4)) + mat.m8);
            if (0.0f < trace)
            {
                // The reference rounds (trace + 1.0) to f32 FIRST, then takes
                // the f32 sqrt — not a double sqrt rounded once.
                float s = Fp.Sqrt((float)((double)trace + 1.0));
                float f = (float)(0.5 / (double)s);
                q0 = (float)((double)s * 0.5);
                q1 = (float)(double)(((float)(double)(mat.m7 - mat.m5)) * f);
                q2 = (float)(double)(((float)(double)(mat.m2 - mat.m6)) * f);
                q3 = (float)(double)(((float)(double)(mat.m3 - mat.m1)) * f);
            }
            else
            {
                int i = mat.m0 < mat.m4 ? 1 : 0;
                if (mat[i * 4] < mat.m8)
                {
                    i = 2;
                }
                int j = (i == 0) ? 1 : (i == 1 ? 2 : 0);
                int k = (j == 0) ? 1 : (j == 1 ? 2 : 0);
                float s = Fp.Sqrt(
                    (float)((double)((float)(double)(mat[i * 4] - ((float)(double)(mat[k * 4] + mat[j * 4])))) + 1.0));
                float f = (float)(0.5 / (double)s);
                // q[i + 1] = s * 0.5
                switch (i + 1)
                {
                    case 1: q1 = (float)((double)s * 0.5); break;
                    case 2: q2 = (float)((double)s * 0.5); break;
                    case 3: q3 = (float)((double)s * 0.5); break;
                }
                q0 = (float)(double)(((float)(double)(mat[3 * k + j] - mat[3 * j + k])) * f);
                // q[j + 1]
                switch (j + 1)
                {
                    case 1: q1 = (float)(double)(((float)(double)(mat[3 * i + j] + mat[3 * j + i])) * f); break;
                    case 2: q2 = (float)(double)(((float)(double)(mat[3 * i + j] + mat[3 * j + i])) * f); break;
                    case 3: q3 = (float)(double)(((float)(double)(mat[3 * i + j] + mat[3 * j + i])) * f); break;
                }
                // q[k + 1]
                switch (k + 1)
                {
                    case 1: q1 = (float)(double)(((float)(double)(mat[3 * i + k] + mat[3 * k + i])) * f); break;
                    case 2: q2 = (float)(double)(((float)(double)(mat[3 * i + k] + mat[3 * k + i])) * f); break;
                    case 3: q3 = (float)(double)(((float)(double)(mat[3 * i + k] + mat[3 * k + i])) * f); break;
                }
            }
            x = q0;
            y = q1;
            z = q2;
            w = q3;
        }

        /// <summary>Normalizes in place. Operand orders preserved.</summary>
        public void Normalize()
        {
            float sumsq = (float)(double)(((float)(double)(((float)(double)(((float)(double)(y * y)) + ((float)(double)(x * x)))) + ((float)(double)(z * z)))) + ((float)(double)(w * w)));
            float s = Fp.Sqrt(sumsq);
            float inv = (float)(double)(1.0f / s);
            x = (float)(double)(x * inv);
            y = (float)(double)(y * inv);
            z = (float)(double)(z * inv);
            w = (float)(double)(inv * w);
        }
    }

    /// <summary>
    /// A row-major 3x3 rotation plus translation (48 bytes: m[0..8] then
    /// t[0..2]). Expanded fields; t0/t1/t2 are the translation.
    /// </summary>
    [Serializable]
    public struct GmIso4
    {
        public float m0;
        public float m1;
        public float m2;
        public float m3;
        public float m4;
        public float m5;
        public float m6;
        public float m7;
        public float m8;
        public float t0;
        public float t1;
        public float t2;

        public float this[int i]
        {
            get
            {
                switch (i)
                {
                    case 0: return m0;
                    case 1: return m1;
                    case 2: return m2;
                    case 3: return m3;
                    case 4: return m4;
                    case 5: return m5;
                    case 6: return m6;
                    case 7: return m7;
                    case 8: return m8;
                    case 9: return t0;
                    case 10: return t1;
                    default: return t2;
                }
            }
            set
            {
                switch (i)
                {
                    case 0: m0 = value; break;
                    case 1: m1 = value; break;
                    case 2: m2 = value; break;
                    case 3: m3 = value; break;
                    case 4: m4 = value; break;
                    case 5: m5 = value; break;
                    case 6: m6 = value; break;
                    case 7: m7 = value; break;
                    case 8: m8 = value; break;
                    case 9: t0 = value; break;
                    case 10: t1 = value; break;
                    default: t2 = value; break;
                }
            }
        }

        /// <summary>The identity isometry (the reference Default is NOT zero).</summary>
        public static readonly GmIso4 Identity = new GmIso4
        {
            m0 = 1.0f,
            m4 = 1.0f,
            m8 = 1.0f,
        };

        /// <summary>The rotation block as a bare matrix (copy).</summary>
        public GmMat3 Rotation()
        {
            GmMat3 r = default(GmMat3);
            r.m0 = m0; r.m1 = m1; r.m2 = m2;
            r.m3 = m3; r.m4 = m4; r.m5 = m5;
            r.m6 = m6; r.m7 = m7; r.m8 = m8;
            return r;
        }

        /// <summary>this = this * parent (rotation then translation).</summary>
        public void Mult(ref GmIso4 parent)
        {
            GmVec3 translation = new GmVec3(t0, t1, t2);
            GmMat3 rot = default(GmMat3);
            rot.m0 = m0; rot.m1 = m1; rot.m2 = m2;
            rot.m3 = m3; rot.m4 = m4; rot.m5 = m5;
            rot.m6 = m6; rot.m7 = m7; rot.m8 = m8;
            GmMat3 parentRot = default(GmMat3);
            parentRot.m0 = parent.m0; parentRot.m1 = parent.m1; parentRot.m2 = parent.m2;
            parentRot.m3 = parent.m3; parentRot.m4 = parent.m4; parentRot.m5 = parent.m5;
            parentRot.m6 = parent.m6; parentRot.m7 = parent.m7; parentRot.m8 = parent.m8;
            rot.Mult(ref parentRot);
            m0 = rot.m0; m1 = rot.m1; m2 = rot.m2;
            m3 = rot.m3; m4 = rot.m4; m5 = rot.m5;
            m6 = rot.m6; m7 = rot.m7; m8 = rot.m8;
            translation.MultIso4(ref parent);
            t0 = translation.x;
            t1 = translation.y;
            t2 = translation.z;
        }

        /// <summary>self = source^-1 (kept verbatim from the reference).</summary>
        public void SetInverse(ref GmIso4 source)
        {
            GmMat3 rot = default(GmMat3);
            rot.m0 = m0; rot.m1 = m1; rot.m2 = m2;
            rot.m3 = m3; rot.m4 = m4; rot.m5 = m5;
            rot.m6 = m6; rot.m7 = m7; rot.m8 = m8;
            GmMat3 sourceRot = default(GmMat3);
            sourceRot.m0 = source.m0; sourceRot.m1 = source.m1; sourceRot.m2 = source.m2;
            sourceRot.m3 = source.m3; sourceRot.m4 = source.m4; sourceRot.m5 = source.m5;
            sourceRot.m6 = source.m6; sourceRot.m7 = source.m7; sourceRot.m8 = source.m8;
            rot.SetTranspose(ref sourceRot);
            m0 = rot.m0; m1 = rot.m1; m2 = rot.m2;
            m3 = rot.m3; m4 = rot.m4; m5 = rot.m5;
            m6 = rot.m6; m7 = rot.m7; m8 = rot.m8;
            GmVec3 translation = new GmVec3(-source.t0, -source.t1, -source.t2);
            GmMat3 selfRot = default(GmMat3);
            selfRot.m0 = m0; selfRot.m1 = m1; selfRot.m2 = m2;
            selfRot.m3 = m3; selfRot.m4 = m4; selfRot.m5 = m5;
            selfRot.m6 = m6; selfRot.m7 = m7; selfRot.m8 = m8;
            translation.MultMat3(ref selfRot);
            t0 = translation.x;
            t1 = translation.y;
            t2 = translation.z;
        }

        public void ToBytes(byte[] b, int o)
        {
            BitsUtil.PutF32(b, o, m0);
            BitsUtil.PutF32(b, o + 4, m1);
            BitsUtil.PutF32(b, o + 8, m2);
            BitsUtil.PutF32(b, o + 12, m3);
            BitsUtil.PutF32(b, o + 16, m4);
            BitsUtil.PutF32(b, o + 20, m5);
            BitsUtil.PutF32(b, o + 24, m6);
            BitsUtil.PutF32(b, o + 28, m7);
            BitsUtil.PutF32(b, o + 32, m8);
            BitsUtil.PutF32(b, o + 36, t0);
            BitsUtil.PutF32(b, o + 40, t1);
            BitsUtil.PutF32(b, o + 44, t2);
        }

        public static GmIso4 FromBytes(byte[] b, int o)
        {
            GmIso4 r = default(GmIso4);
            r.m0 = BitsUtil.GetF32(b, o);
            r.m1 = BitsUtil.GetF32(b, o + 4);
            r.m2 = BitsUtil.GetF32(b, o + 8);
            r.m3 = BitsUtil.GetF32(b, o + 12);
            r.m4 = BitsUtil.GetF32(b, o + 16);
            r.m5 = BitsUtil.GetF32(b, o + 20);
            r.m6 = BitsUtil.GetF32(b, o + 24);
            r.m7 = BitsUtil.GetF32(b, o + 28);
            r.m8 = BitsUtil.GetF32(b, o + 32);
            r.t0 = BitsUtil.GetF32(b, o + 36);
            r.t1 = BitsUtil.GetF32(b, o + 40);
            r.t2 = BitsUtil.GetF32(b, o + 44);
            return r;
        }
    }
}
