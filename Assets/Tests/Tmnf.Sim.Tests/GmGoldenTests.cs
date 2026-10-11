using System;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>Golden suites for the gm math module.</summary>
    [TestFixture]
    public class GmGoldenTests
    {
        [Test]
        public void Dot3Left()
        {
            GoldenRunner.RunSuite("gm/dot3_left.tgn", input =>
            {
                var r = new ByteReader(input);
                float a0 = r.F32(), b0 = r.F32(), a1 = r.F32(), b1 = r.F32(), a2 = r.F32(), b2 = r.F32();
                return BitConverter.GetBytes(GmFunc.Dot3Left(a0, b0, a1, b1, a2, b2));
            });
        }

        [Test]
        public void Vec3SetMultIso4()
        {
            GoldenRunner.RunSuite("gm/vec3_set_mult_iso4.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 v = ReadVec3(r);
                GmIso4 a = ReadIso4(r);
                GmVec3 o = GmVec3.Zero;
                o.SetMultIso4(ref v, ref a);
                return WriteVec3(o);
            });
        }

        [Test]
        public void Vec3SetMultMat3()
        {
            GoldenRunner.RunSuite("gm/vec3_set_mult_mat3.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 v = ReadVec3(r);
                GmMat3 m = ReadMat3(r);
                GmVec3 o = GmVec3.Zero;
                o.SetMultMat3(ref v, ref m);
                return WriteVec3(o);
            });
        }

        [Test]
        public void Vec3MultIso4()
        {
            GoldenRunner.RunSuite("gm/vec3_mult_iso4.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 v = ReadVec3(r);
                GmIso4 a = ReadIso4(r);
                v.MultIso4(ref a);
                return WriteVec3(v);
            });
        }

        [Test]
        public void Vec3MultTranspose()
        {
            GoldenRunner.RunSuite("gm/vec3_mult_transpose.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 v = ReadVec3(r);
                GmMat3 m = ReadMat3(r);
                v.MultTranspose(ref m);
                return WriteVec3(v);
            });
        }

        [Test]
        public void Vec3MultMat3()
        {
            GoldenRunner.RunSuite("gm/vec3_mult_mat3.tgn", input =>
            {
                var r = new ByteReader(input);
                GmVec3 v = ReadVec3(r);
                GmMat3 m = ReadMat3(r);
                v.MultMat3(ref m);
                return WriteVec3(v);
            });
        }

        [Test]
        public void Mat3SetFromQuat()
        {
            GoldenRunner.RunSuite("gm/mat3_set_from_quat.tgn", input =>
            {
                var r = new ByteReader(input);
                float a = r.F32(), b = r.F32(), c = r.F32(), d = r.F32();
                GmMat3 m = default(GmMat3);
                m.SetFromQuat(a, b, c, d);
                return WriteMat3(m);
            });
        }

        [Test]
        public void Mat3SetTranspose()
        {
            GoldenRunner.RunSuite("gm/mat3_set_transpose.tgn", input =>
            {
                var r = new ByteReader(input);
                GmMat3 src = ReadMat3(r);
                GmMat3 m = default(GmMat3);
                m.SetTranspose(ref src);
                return WriteMat3(m);
            });
        }

        [Test]
        public void Mat3Mult()
        {
            GoldenRunner.RunSuite("gm/mat3_mult.tgn", input =>
            {
                var r = new ByteReader(input);
                GmMat3 a = ReadMat3(r);
                GmMat3 b = ReadMat3(r);
                a.Mult(ref b);
                return WriteMat3(a);
            });
        }

        [Test]
        public void Mat3SetMult()
        {
            GoldenRunner.RunSuite("gm/mat3_set_mult.tgn", input =>
            {
                var r = new ByteReader(input);
                GmMat3 a = ReadMat3(r);
                GmMat3 b = ReadMat3(r);
                GmMat3 m = default(GmMat3);
                m.SetMult(ref a, ref b);
                return WriteMat3(m);
            });
        }

        [Test]
        public void Mat3MultTranspose()
        {
            GoldenRunner.RunSuite("gm/mat3_mult_transpose.tgn", input =>
            {
                var r = new ByteReader(input);
                GmMat3 m = ReadMat3(r);
                GmMat3 t = ReadMat3(r);
                m.MultTranspose(ref t);
                return WriteMat3(m);
            });
        }

        [Test]
        public void Iso4Mult()
        {
            GoldenRunner.RunSuite("gm/iso4_mult.tgn", input =>
            {
                var r = new ByteReader(input);
                GmIso4 a = ReadIso4(r);
                GmIso4 b = ReadIso4(r);
                a.Mult(ref b);
                return WriteIso4(a);
            });
        }

        [Test]
        public void Iso4SetInverse()
        {
            GoldenRunner.RunSuite("gm/iso4_set_inverse.tgn", input =>
            {
                var r = new ByteReader(input);
                GmIso4 s = ReadIso4(r);
                GmIso4 a = default(GmIso4);
                a.SetInverse(ref s);
                return WriteIso4(a);
            });
        }

        [Test]
        public void Iso4Rotation()
        {
            GoldenRunner.RunSuite("gm/iso4_rotation.tgn", input =>
            {
                var r = new ByteReader(input);
                GmIso4 a = ReadIso4(r);
                GmMat3 rot = a.Rotation();
                return WriteMat3(rot);
            });
        }

        [Test]
        public void QuatSetFromMat3()
        {
            GoldenRunner.RunSuite("gm/quat_set_from_mat3.tgn", input =>
            {
                var r = new ByteReader(input);
                GmMat3 m = ReadMat3(r);
                GmQuat q = default(GmQuat);
                q.SetFromMat3(ref m);
                return WriteQuat(q);
            });
        }

        [Test]
        public void QuatNormalize()
        {
            GoldenRunner.RunSuite("gm/quat_normalize.tgn", input =>
            {
                var r = new ByteReader(input);
                GmQuat q = ReadQuat(r);
                q.Normalize();
                return WriteQuat(q);
            });
        }

        [Test]
        public void BoxTestInter()
        {
            GoldenRunner.RunSuite("gm/box_test_inter.tgn", input =>
            {
                var r = new ByteReader(input);
                GmBoxAligned a = ReadBox(r);
                GmBoxAligned b = ReadBox(r);
                return new[] { (byte)(a.TestInter(ref b) ? 1 : 0) };
            });
        }

        [Test]
        public void BoxSetMult()
        {
            GoldenRunner.RunSuite("gm/box_set_mult.tgn", input =>
            {
                var r = new ByteReader(input);
                GmBoxAligned a = ReadBox(r);
                GmIso4 iso = ReadIso4(r);
                GmBoxAligned o = default(GmBoxAligned);
                o.SetMult(ref a, ref iso);
                return WriteBox(o);
            });
        }

        [Test]
        public void Funcs()
        {
            GoldenRunner.RunSuite("gm/funcs.tgn", input =>
            {
                var r = new ByteReader(input);
                byte fn = r.U8();
                var o = new System.Collections.Generic.List<byte>();
                switch (fn)
                {
                    case 0:
                    {
                        float v = r.F32();
                        ByteWriter.U8(o, (byte)(GmFunc.GmfuncIsANumber(v) ? 1 : 0));
                        break;
                    }
                    case 1:
                    {
                        float v = r.F32();
                        ByteWriter.F32(o, GmFunc.GmfuncSign(v));
                        break;
                    }
                    case 2:
                    {
                        float v = r.F32();
                        ByteWriter.F32(o, GmFunc.GmfuncAcos(v));
                        break;
                    }
                    case 3:
                    {
                        float v = r.F32();
                        ByteWriter.F32(o, GmFunc.GmfuncAsinSafe(v));
                        break;
                    }
                    case 4:
                    {
                        float v = r.F32(), lo = r.F32(), hi = r.F32();
                        ByteWriter.F32(o, GmFunc.GmfuncMod(v, lo, hi));
                        break;
                    }
                    case 5:
                    {
                        GmVec3 v = ReadVec3(r);
                        GmFunc.NormalizeVec3IfNonzero(ref v);
                        WriteVec3To(o, v);
                        break;
                    }
                    default:
                    {
                        GmVec3 a = ReadVec3(r);
                        GmVec3 b = ReadVec3(r);
                        ByteWriter.F32(o, GmFunc.Gmvec3GetAngle(ref a, ref b));
                        break;
                    }
                }
                return o.ToArray();
            });
        }

        [Test]
        public void Constants()
        {
            // The full magic-constant bit table (f32 words then f64 words),
            // mirroring the generator's emission order.
            GoldenRunner.RunSuite("gm/constants.tgn", input =>
            {
                var r = new ByteReader(input);
                r.U8(); // selector
                var o = new System.Collections.Generic.List<byte>();
                foreach (uint bits in F32ConstantBits())
                {
                    ByteWriter.U32(o, bits);
                }
                foreach (ulong bits in F64ConstantBits())
                {
                    ByteWriter.U64(o, bits);
                }
                return o.ToArray();
            });
        }

        private static uint[] F32ConstantBits()
        {
            return new uint[]
            {
                0x2EB7_CDFCu, // TINY_SQUARED (1e-10 magnitude epsilon)
                0x374F_8B58u, // EDGE_CLOSEST_SQUARED
                0x3727_C5B0u, // CURVE_BLEND_EPSILON
                0x3F5D_B3D7u, // SPHERE_MERGE_THRESHOLD cos(pi/6)
                0x9E99_999Au, // water impulse position 0.3f
                0x2EDB_E6FEu, // NORMALIZE_EPSILON (9.999999439624929e-11)
                0x3727_C5ACu, // WHEEL_SPEED_EPSILON
                0x3F7E_B852u, // WHEEL_SPEED_DAMPING
                0x44C9_0FDBu, // WHEEL_ANGLE_PERIOD
                0x3F12_4925u, // ROULETTE_LOW
            };
        }

        private static ulong[] F64ConstantBits()
        {
            return new ulong[]
            {
                0x3EE4_F8B5_88E3_68F1uL, // NEARLY_EQUAL_SCALE / boundary eps
                0x3FEB_6DB6_E000_0000uL, // ROULETTE_HIGH
                0x3FE9_21FB_6000_0000uL, // pi/2 as f64
                0x4009_21FB_6000_0000uL, // pi as f64
                0x400C_CCCC_C000_0000uL, // 3.6f as f64
            };
        }

        // ------------------------------------------------------------------
        // serialization helpers
        // ------------------------------------------------------------------

        internal static GmVec3 ReadVec3(ByteReader r)
        {
            return new GmVec3(r.F32(), r.F32(), r.F32());
        }

        internal static GmMat3 ReadMat3(ByteReader r)
        {
            GmMat3 m = default(GmMat3);
            for (int i = 0; i < 9; i++)
            {
                m[i] = r.F32();
            }
            return m;
        }

        internal static GmQuat ReadQuat(ByteReader r)
        {
            GmQuat q = default(GmQuat);
            q.x = r.F32();
            q.y = r.F32();
            q.z = r.F32();
            q.w = r.F32();
            return q;
        }

        internal static GmIso4 ReadIso4(ByteReader r)
        {
            GmIso4 i = default(GmIso4);
            for (int k = 0; k < 9; k++)
            {
                i[k] = r.F32();
            }
            i.t0 = r.F32();
            i.t1 = r.F32();
            i.t2 = r.F32();
            return i;
        }

        internal static GmBoxAligned ReadBox(ByteReader r)
        {
            GmBoxAligned b = default(GmBoxAligned);
            b.center = ReadVec3(r);
            b.halfExtent = ReadVec3(r);
            return b;
        }

        internal static byte[] WriteVec3(GmVec3 v)
        {
            var o = new System.Collections.Generic.List<byte>(12);
            WriteVec3To(o, v);
            return o.ToArray();
        }

        internal static void WriteVec3To(System.Collections.Generic.List<byte> o, GmVec3 v)
        {
            ByteWriter.F32(o, v.x);
            ByteWriter.F32(o, v.y);
            ByteWriter.F32(o, v.z);
        }

        internal static byte[] WriteMat3(GmMat3 m)
        {
            var o = new System.Collections.Generic.List<byte>(36);
            for (int i = 0; i < 9; i++)
            {
                ByteWriter.F32(o, m[i]);
            }
            return o.ToArray();
        }

        internal static byte[] WriteQuat(GmQuat q)
        {
            return new[] { q.x, q.y, q.z, q.w } is float[] f ? ToBytes(f) : null;
        }

        private static byte[] ToBytes(float[] f)
        {
            var o = new System.Collections.Generic.List<byte>(16);
            foreach (float v in f)
            {
                ByteWriter.F32(o, v);
            }
            return o.ToArray();
        }

        internal static byte[] WriteIso4(GmIso4 i)
        {
            var o = new System.Collections.Generic.List<byte>(48);
            for (int k = 0; k < 9; k++)
            {
                ByteWriter.F32(o, i[k]);
            }
            ByteWriter.F32(o, i.t0);
            ByteWriter.F32(o, i.t1);
            ByteWriter.F32(o, i.t2);
            return o.ToArray();
        }

        internal static byte[] WriteBox(GmBoxAligned b)
        {
            var o = new System.Collections.Generic.List<byte>(24);
            WriteVec3To(o, b.center);
            WriteVec3To(o, b.halfExtent);
            return o.ToArray();
        }
    }
}
