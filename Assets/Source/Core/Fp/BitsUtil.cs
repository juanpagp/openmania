using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// Little-endian bit readers/writers shared by the byte-contract types
    /// (state dumps, image sections, golden data).
    /// </summary>
    public static class BitsUtil
    {
        public static float GetF32(byte[] b, int o)
        {
            return BitConverter.ToSingle(b, o);
        }

        public static void PutF32(byte[] b, int o, float v)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(v), 0, b, o, 4);
        }

        public static uint GetU32(byte[] b, int o)
        {
            return (uint)(b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24));
        }

        public static void PutU32(byte[] b, int o, uint v)
        {
            b[o] = (byte)v;
            b[o + 1] = (byte)(v >> 8);
            b[o + 2] = (byte)(v >> 16);
            b[o + 3] = (byte)(v >> 24);
        }

        public static int GetI32(byte[] b, int o)
        {
            return unchecked((int)GetU32(b, o));
        }

        public static void PutI32(byte[] b, int o, int v)
        {
            PutU32(b, o, unchecked((uint)v));
        }

        public static ulong GetU64(byte[] b, int o)
        {
            return GetU32(b, o) | ((ulong)GetU32(b, o + 4) << 32);
        }

        public static void PutU64(byte[] b, int o, ulong v)
        {
            PutU32(b, o, (uint)v);
            PutU32(b, o + 4, (uint)(v >> 32));
        }

        public static ushort GetU16(byte[] b, int o)
        {
            return (ushort)(b[o] | (b[o + 1] << 8));
        }

        public static void PutU16(byte[] b, int o, ushort v)
        {
            b[o] = (byte)v;
            b[o + 1] = (byte)(v >> 8);
        }

        /// <summary>Big-endian u32 (SHA-256 digests).</summary>
        public static uint GetBe32(byte[] b, int o)
        {
            return ((uint)b[o] << 24) | ((uint)b[o + 1] << 16) | ((uint)b[o + 2] << 8) | b[o + 3];
        }
    }

    /// <summary>Little-endian streaming writer for golden dumps.</summary>
    public static class ByteWriter
    {
        public static void F32(System.Collections.Generic.List<byte> b, float v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void F64(System.Collections.Generic.List<byte> b, double v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void U32(System.Collections.Generic.List<byte> b, uint v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void I32(System.Collections.Generic.List<byte> b, int v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void U64(System.Collections.Generic.List<byte> b, ulong v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void U16(System.Collections.Generic.List<byte> b, ushort v)
        {
            b.AddRange(BitConverter.GetBytes(v));
        }

        public static void U8(System.Collections.Generic.List<byte> b, byte v)
        {
            b.Add(v);
        }

        public static void Bytes(System.Collections.Generic.List<byte> b, byte[] v)
        {
            b.AddRange(v);
        }
    }

    /// <summary>Little-endian streaming reader over golden row inputs.</summary>
    public sealed class ByteReader
    {
        private readonly byte[] data;
        public int Pos;

        public ByteReader(byte[] data)
        {
            this.data = data;
        }

        public float F32() => BitConverter.ToSingle(data, Take(4));
        public double F64() => BitConverter.ToDouble(data, Take(8));
        public uint U32() => BitsUtil.GetU32(data, Take(4));
        public int I32() => BitsUtil.GetI32(data, Take(4));
        public ulong U64() => BitsUtil.GetU64(data, Take(8));
        public ushort U16() => BitsUtil.GetU16(data, Take(2));
        public byte U8() => data[Pos++];
        public byte[] Bytes(int n)
        {
            byte[] r = new byte[n];
            Buffer.BlockCopy(data, Pos, r, 0, n);
            Pos += n;
            return r;
        }

        private int Take(int n)
        {
            int p = Pos;
            Pos += n;
            return p;
        }
    }

    /// <summary>FNV-1a 64-bit over bytes — the trace state hash.</summary>
    public static class Fnva
    {
        public static ulong Hash(byte[] bytes)
        {
            ulong h = 0xcbf2_9ce4_8422_2325UL;
            foreach (byte b in bytes)
            {
                h ^= b;
                h *= 0x0000_0100_0000_01b3UL;
            }
            return h;
        }
    }
}
