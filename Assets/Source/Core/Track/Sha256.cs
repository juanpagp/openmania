using System;

namespace Tmnf.Sim
{
    /// <summary>SHA-256, transliterated from the loader's embedded implementation.</summary>
    public sealed class Sha256
    {
        private static readonly uint[] K =
        {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
            0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
            0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
            0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
            0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
            0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
            0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
            0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
            0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
        };

        private uint[] state = new uint[8];
        private ulong byteCount;
        private byte[] block = new byte[64];
        private uint blockLen;

        private static uint Rotr32(uint value, uint bits)
        {
            // The C's masked shift is total; never called with bits == 0 here.
            return (value >> (int)bits) | (value << (int)(32 - bits));
        }

        private static uint LoadBe32(byte[] p, int o)
        {
            return ((uint)p[o] << 24) | ((uint)p[o + 1] << 16) | ((uint)p[o + 2] << 8) | p[o + 3];
        }

        private static void StoreBe32(byte[] p, int o, uint value)
        {
            p[o] = (byte)(value >> 24);
            p[o + 1] = (byte)(value >> 16);
            p[o + 2] = (byte)(value >> 8);
            p[o + 3] = (byte)value;
        }

        public Sha256()
        {
            state[0] = 0x6a09e667; state[1] = 0xbb67ae85; state[2] = 0x3c6ef372; state[3] = 0xa54ff53a;
            state[4] = 0x510e527f; state[5] = 0x9b05688c; state[6] = 0x1f83d9ab; state[7] = 0x5be0cd19;
        }

        private void Transform(byte[] block64)
        {
            uint[] w = new uint[64];
            for (int i = 0; i < 16; i++)
            {
                w[i] = LoadBe32(block64, i * 4);
            }
            for (int i = 16; i < 64; i++)
            {
                uint s0 = Rotr32(w[i - 15], 7) ^ Rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
                uint s1 = Rotr32(w[i - 2], 17) ^ Rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
                w[i] = unchecked(w[i - 16] + s0 + w[i - 7] + s1);
            }

            uint a = state[0], b = state[1], c = state[2], d = state[3];
            uint e = state[4], f = state[5], g = state[6], h = state[7];
            for (int i = 0; i < 64; i++)
            {
                uint s1 = Rotr32(e, 6) ^ Rotr32(e, 11) ^ Rotr32(e, 25);
                uint choose = (e & f) ^ (~e & g);
                uint t1 = unchecked(h + s1 + choose + K[i] + w[i]);
                uint s0 = Rotr32(a, 2) ^ Rotr32(a, 13) ^ Rotr32(a, 22);
                uint majority = (a & b) ^ (a & c) ^ (b & c);
                uint t2 = unchecked(s0 + majority);
                h = g;
                g = f;
                f = e;
                e = unchecked(d + t1);
                d = c;
                c = b;
                b = a;
                a = unchecked(t1 + t2);
            }
            state[0] = unchecked(state[0] + a);
            state[1] = unchecked(state[1] + b);
            state[2] = unchecked(state[2] + c);
            state[3] = unchecked(state[3] + d);
            state[4] = unchecked(state[4] + e);
            state[5] = unchecked(state[5] + f);
            state[6] = unchecked(state[6] + g);
            state[7] = unchecked(state[7] + h);
        }

        public void Update(byte[] data)
        {
            byteCount += (ulong)data.Length;
            int offset = 0;
            int remaining = data.Length;
            while (remaining > 0)
            {
                int space = 64 - (int)blockLen;
                int take = remaining < space ? remaining : space;
                Buffer.BlockCopy(data, offset, block, (int)blockLen, take);
                blockLen += (uint)take;
                offset += take;
                remaining -= take;
                if (blockLen == 64)
                {
                    Transform(block);
                    blockLen = 0;
                }
            }
        }

        public byte[] FinalDigest()
        {
            ulong bitCount = unchecked(byteCount * 8);
            block[blockLen] = 0x80;
            blockLen += 1;
            if (blockLen > 56)
            {
                for (int i = (int)blockLen; i < 64; i++) block[i] = 0;
                Transform(block);
                blockLen = 0;
            }
            for (int i = (int)blockLen; i < 56; i++) block[i] = 0;
            for (uint i = 0; i < 8; i++)
            {
                block[63 - (int)i] = (byte)(bitCount >> (int)(i * 8));
            }
            Transform(block);
            byte[] digest = new byte[32];
            for (int i = 0; i < 8; i++)
            {
                StoreBe32(digest, i * 4, state[i]);
            }
            return digest;
        }

        public static byte[] Digest(byte[] data)
        {
            Sha256 ctx = new Sha256();
            ctx.Update(data);
            return ctx.FinalDigest();
        }
    }

    /// <summary>Little-endian field reader over snapshot bytes.</summary>
    public sealed class Reader
    {
        private readonly byte[] data;
        public int Pos;

        public Reader(byte[] data)
        {
            this.data = data;
        }

        public static Reader At(byte[] data, int pos)
        {
            return new Reader(data) { Pos = pos };
        }

        public byte U8() => data[Pos++];
        public ushort U16() { ushort v = BitsUtil.GetU16(data, Pos); Pos += 2; return v; }
        public uint U32() { uint v = BitsUtil.GetU32(data, Pos); Pos += 4; return v; }
        public ulong U64() { ulong v = BitsUtil.GetU64(data, Pos); Pos += 8; return v; }
        public float F32() { float v = BitsUtil.GetF32(data, Pos); Pos += 4; return v; }

        public GmVec3 GmVec3()
        {
            return new GmVec3(F32(), F32(), F32());
        }

        public GmIso4 GmIso4()
        {
            GmIso4 r = default(GmIso4);
            for (int i = 0; i < 9; i++) r[i] = F32();
            r.t0 = F32();
            r.t1 = F32();
            r.t2 = F32();
            return r;
        }

        public byte[] Bytes(int n)
        {
            byte[] v = new byte[n];
            Buffer.BlockCopy(data, Pos, v, 0, n);
            Pos += n;
            return v;
        }
    }
}
