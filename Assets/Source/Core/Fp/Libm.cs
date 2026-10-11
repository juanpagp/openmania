using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// Pure-software port of the `libm` crate v0.2.11 double-precision
    /// routines the simulation uses (sin, cos, exp, atan2 and their internal
    /// kernels: fdlibm-derived algorithms from FreeBSD's msun). Only double
    /// + - * /, integer bit manipulation and tables are used, so every
    /// platform produces identical bits — no system libm anywhere.
    /// </summary>
    internal static class Libm
    {
        // ------------------------------------------------------------------
        // bit helpers
        // ------------------------------------------------------------------

        private static ulong Bits(double x) => (ulong)BitConverter.DoubleToInt64Bits(x);

        private static double FromBits(ulong bits) => BitConverter.Int64BitsToDouble((long)bits);

        private static uint HighWord(double x) => (uint)(BitConverter.DoubleToInt64Bits(x) >> 32) & 0x7fffffff;

        private static uint SignBit(double x) => (uint)(BitConverter.DoubleToInt64Bits(x) >> 63);

        /// <summary>Rust `as i32` on f64: saturating, NaN -> 0.</summary>
        public static int RustAsI32(double x)
        {
            if (double.IsNaN(x)) return 0;
            if (x >= 2147483647.0) return int.MaxValue;
            if (x <= -2147483648.0) return int.MinValue;
            return (int)x;
        }

        /// <summary>Rust `as i64` on f64: saturating, NaN -> 0.</summary>
        public static long RustAsI64(double x)
        {
            if (double.IsNaN(x)) return 0;
            if (x >= 9223372036854775807.0) return long.MaxValue;
            if (x <= -9223372036854775808.0) return long.MinValue;
            return (long)x;
        }

        internal static double Fabs(double x) => FromBits(Bits(x) & 0x7fff_ffff_ffff_ffff);

        /// <summary>Sets the quiet bit (f64 mantissa bit 51), preserving sign and payload.</summary>
        internal static double Quiet(double x) =>
            FromBits(Bits(x) | (1UL << 51));

        // ------------------------------------------------------------------
        // scalbn (ldexp)
        // ------------------------------------------------------------------

        internal static double Scalbn(double x, int n)
        {
            double x1p1023 = FromBits(0x7fe0000000000000UL); // 2^1023
            double x1p53 = FromBits(0x4340000000000000UL);   // 2^53
            double x1pM1022 = FromBits(0x0010000000000000UL); // 2^-1022

            double y = x;

            if (n > 1023)
            {
                y *= x1p1023;
                n -= 1023;
                if (n > 1023)
                {
                    y *= x1p1023;
                    n -= 1023;
                    if (n > 1023)
                    {
                        n = 1023;
                    }
                }
            }
            else if (n < -1022)
            {
                // make sure final n < -53 to avoid double rounding in the
                // subnormal range
                y *= x1pM1022 * x1p53;
                n += 1022 - 53;
                if (n < -1022)
                {
                    y *= x1pM1022 * x1p53;
                    n += 1022 - 53;
                    if (n < -1022)
                    {
                        n = -1022;
                    }
                }
            }
            return y * FromBits(((ulong)(unchecked((int)0x3ff + n)) & 0xffff_ffff_ffff_ffffUL) << 52);
        }

        // ------------------------------------------------------------------
        // floor
        // ------------------------------------------------------------------

        // 2^52 == 1/f64::EPSILON (Rust). NOT 1/.NET double.Epsilon —
        // .NET's double.Epsilon is the denormal minimum (4.9e-324).
        private const double FloorToint = 4503599627370496.0;

        internal static double Floor(double x)
        {
            ulong ui = Bits(x);
            uint e = (uint)((ui >> 52) & 0x7ff);

            if ((e >= 0x3ff + 52) || (x == 0.0))
            {
                return x;
            }
            // y = int(x) - x, where int(x) is an integer neighbor of x
            double y = (ui >> 63) != 0
                ? x - FloorToint + FloorToint - x
                : x + FloorToint - FloorToint - x;
            // special case because of non-nearest rounding modes
            if (e < 0x3ff)
            {
                return (ui >> 63) != 0 ? -1.0 : 0.0;
            }
            return y > 0.0 ? x + y - 1.0 : x + y;
        }

        // ------------------------------------------------------------------
        // k_sin / k_cos (kernels on [-pi/4, pi/4])
        // ------------------------------------------------------------------

        private const double S1 = -1.66666666666666324348e-01; /* 0xBFC55555, 0x55555549 */
        private const double S2 = 8.33333333332248946124e-03;  /* 0x3F811111, 0x1110F8A6 */
        private const double S3 = -1.98412698298579493134e-04; /* 0xBF2A01A0, 0x19C161D5 */
        private const double S4 = 2.75573137070700676789e-06;  /* 0x3EC71DE3, 0x57B1FE7D */
        private const double S5 = -2.50507602534068634195e-08; /* 0xBE5AE5E6, 0x8A2B9CEB */
        private const double S6 = 1.58969099521155010221e-10;  /* 0x3DE5D93A, 0x5ACFD57C */

        private static double KSin(double x, double y, int iy)
        {
            double z = x * x;
            double w = z * z;
            double r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
            double v = z * x;
            if (iy == 0)
            {
                return x + v * (S1 + z * r);
            }
            return x - ((z * (0.5 * y - v * r) - y) - v * S1);
        }

        private const double C1 = 4.16666666666666019037e-02;  /* 0x3FA55555, 0x5555554C */
        private const double C2 = -1.38888888888741095749e-03; /* 0xBF56C16C, 0x16C15177 */
        private const double C3 = 2.48015872894767294178e-05;  /* 0x3EFA01A0, 0x19CB1590 */
        private const double C4 = -2.75573143513906633035e-07; /* 0xBE927E4F, 0x809C52AD */
        private const double C5 = 2.08757232129817482790e-09;  /* 0x3E21EE9E, 0xBDB4B1C4 */
        private const double C6 = -1.13596475577881948265e-11; /* 0xBDA8FAE9, 0xBE8838D4 */

        private static double KCos(double x, double y)
        {
            double z = x * x;
            double w = z * z;
            double r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
            double hz = 0.5 * z;
            double w2 = 1.0 - hz;
            return w2 + (((1.0 - w2) - hz) + (z * r - x * y));
        }

        // ------------------------------------------------------------------
        // rem_pio2 (argument reduction)
        // ------------------------------------------------------------------

        private const double Rpio2Eps = 2.2204460492503131e-16;
        private const double Rpio2ToInt = 1.5 / Rpio2Eps;
        private const double InvPio2 = 6.36619772367581382433e-01;  /* 0x3FE45F30, 0x6DC9C883 */
        private const double Pio2_1 = 1.57079632673412561417e+00;   /* 0x3FF921FB, 0x54400000 */
        private const double Pio2_1T = 6.07710050650619224932e-11;  /* 0x3DD0B461, 0x1A626331 */
        private const double Pio2_2 = 6.07710050630396597660e-11;   /* 0x3DD0B461, 0x1A600000 */
        private const double Pio2_2T = 2.02226624879595063154e-21;  /* 0x3BA3198A, 0x2E037073 */
        private const double Pio2_3 = 2.02226624871116645580e-21;   /* 0x3BA3198A, 0x2E000000 */
        private const double Pio2_3T = 8.47842766036889956997e-32;  /* 0x397B839A, 0x252049C1 */

        private static (int, double, double) RemPio2Medium(double x, uint ix)
        {
            // rint(x/(pi/2)), assume round-to-nearest
            double tmp = x * InvPio2 + Rpio2ToInt;
            double fn = tmp - Rpio2ToInt;
            int n = RustAsI32(fn);
            double r = x - fn * Pio2_1;
            double w = fn * Pio2_1T; // 1st round, good to 85 bits
            double y0 = r - w;
            uint ey = (uint)(Bits(y0) >> 52) & 0x7ff;
            int ex = (int)(ix >> 20);
            if (ex - (int)ey > 16)
            {
                // 2nd round, good to 118 bits
                double t = r;
                w = fn * Pio2_2;
                r = t - w;
                w = fn * Pio2_2T - ((t - r) - w);
                y0 = r - w;
                ey = (uint)(Bits(y0) >> 52) & 0x7ff;
                if (ex - (int)ey > 49)
                {
                    // 3rd round, good to 151 bits, covers all cases
                    t = r;
                    w = fn * Pio2_3;
                    r = t - w;
                    w = fn * Pio2_3T - ((t - r) - w);
                    y0 = r - w;
                }
            }
            double y1 = (r - y0) - w;
            return (n, y0, y1);
        }

        /// <summary>Returns (n, y0, y1) with x rem pi/2 = y0 + y1.</summary>
        private static (int, double, double) RemPio2(double x)
        {
            double x1p24 = FromBits(0x4170000000000000UL);

            int sign = (int)SignBit(x);
            uint ix = HighWord(x);

            if (ix <= 0x400f6a7a)
            {
                // |x| ~<= 5pi/4
                if ((ix & 0xfffff) == 0x921fb)
                {
                    return RemPio2Medium(x, ix); // cancellation -- use medium case
                }
                if (ix <= 0x4002d97c)
                {
                    // |x| ~<= 3pi/4
                    if (sign == 0)
                    {
                        double za = x - Pio2_1; // one round good to 85 bits
                        double y0 = za - Pio2_1T;
                        double y1 = (za - y0) - Pio2_1T;
                        return (1, y0, y1);
                    }
                    else
                    {
                        double zb = x + Pio2_1;
                        double y0 = zb + Pio2_1T;
                        double y1 = (zb - y0) + Pio2_1T;
                        return (-1, y0, y1);
                    }
                }
                if (sign == 0)
                {
                    double zc = x - 2.0 * Pio2_1;
                    double y0 = zc - 2.0 * Pio2_1T;
                    double y1 = (zc - y0) - 2.0 * Pio2_1T;
                    return (2, y0, y1);
                }
                else
                {
                    double zd = x + 2.0 * Pio2_1;
                    double y0 = zd + 2.0 * Pio2_1T;
                    double y1 = (zd - y0) + 2.0 * Pio2_1T;
                    return (-2, y0, y1);
                }
            }
            if (ix <= 0x401c463b)
            {
                // |x| ~<= 9pi/4
                if (ix <= 0x4015fdbc)
                {
                    // |x| ~<= 7pi/4
                    if (ix == 0x4012d97c)
                    {
                        return RemPio2Medium(x, ix); // |x| ~= 3pi/2
                    }
                    if (sign == 0)
                    {
                        double ze = x - 3.0 * Pio2_1;
                        double y0 = ze - 3.0 * Pio2_1T;
                        double y1 = (ze - y0) - 3.0 * Pio2_1T;
                        return (3, y0, y1);
                    }
                    else
                    {
                        double zf = x + 3.0 * Pio2_1;
                        double y0 = zf + 3.0 * Pio2_1T;
                        double y1 = (zf - y0) + 3.0 * Pio2_1T;
                        return (-3, y0, y1);
                    }
                }
                else
                {
                    if (ix == 0x401921fb)
                    {
                        return RemPio2Medium(x, ix); // |x| ~= 4pi/2
                    }
                    if (sign == 0)
                    {
                        double zg = x - 4.0 * Pio2_1;
                        double y0 = zg - 4.0 * Pio2_1T;
                        double y1 = (zg - y0) - 4.0 * Pio2_1T;
                        return (4, y0, y1);
                    }
                    else
                    {
                        double zh = x + 4.0 * Pio2_1;
                        double y0 = zh + 4.0 * Pio2_1T;
                        double y1 = (zh - y0) + 4.0 * Pio2_1T;
                        return (-4, y0, y1);
                    }
                }
            }
            if (ix < 0x413921fb)
            {
                // |x| ~< 2^20*(pi/2), medium size
                return RemPio2Medium(x, ix);
            }
            // all other (large) arguments
            if (ix >= 0x7ff00000)
            {
                // x is inf or NaN
                double y0 = x - x;
                double y1 = y0;
                return (0, y0, y1);
            }
            // set z = scalbn(|x|, -ilogb(x)+23)
            ulong ui = Bits(x);
            ui &= (~1UL) >> 12;
            ui |= (0x3ffUL + 23) << 52;
            double z = FromBits(ui);
            double[] tx = new double[3];
            for (int i = 0; i < 2; i++)
            {
                tx[i] = RustAsI32(z);
                z = (z - tx[i]) * x1p24;
            }
            tx[2] = z;
            // skip zero terms, first term is non-zero; the reference passes
            // &tx[..=last] — the SLICE LENGTH feeds nx/jx and changes the
            // reduction, so the array must be trimmed to exactly that length.
            int last = 2;
            while (last != 0 && tx[last] == 0.0)
            {
                last -= 1;
            }
            double[] txTrimmed = new double[last + 1];
            Array.Copy(tx, txTrimmed, last + 1);
            double[] ty = new double[3];
            int n = RemPio2Large(txTrimmed, ty, (int)((int)ix >> 20) - (0x3ff + 23), 1);
            if (sign != 0)
            {
                return (-n, -ty[0], -ty[1]);
            }
            return (n, ty[0], ty[1]);
        }

        // ------------------------------------------------------------------
        // rem_pio2_large (large-argument reduction over the 2/pi table)
        // ------------------------------------------------------------------

        private static readonly int[] InitJk = { 3, 4, 4, 6 };

        /// <summary>
        /// 396 hex digits (476 decimal) of 2/pi as 24-bit chunks. The 690-entry
        /// table (64-bit pointer width variant).
        /// </summary>
        private static readonly int[] Ipio2 = new int[]
        {
            0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62, 0x95993C, 0x439041, 0xFE5163,
            0xABDEBB, 0xC561B7, 0x246E3A, 0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
            0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41, 0x3991D6, 0x398353, 0x39F49C,
            0x845F8B, 0xBDF928, 0x3B1FF8, 0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
            0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5, 0xF17B3D, 0x0739F7, 0x8A5292,
            0xEA6BFB, 0x5FB11F, 0x8D5D08, 0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
            0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880, 0x4D7327, 0x310606, 0x1556CA,
            0x73A8C9, 0x60E27B, 0xC08C6B, 0x47C419, 0xC367CD, 0xDCE809, 0x2A8359, 0xC4768B, 0x961CA6,
            0xDDAF44, 0xD15719, 0x053EA5, 0xFF0705, 0x3F7E33, 0xE832C2, 0xDE4F98, 0x327DBB, 0xC33D26,
            0xEF6B1E, 0x5EF89F, 0x3A1F35, 0xCAF27F, 0x1D87F1, 0x21907C, 0x7C246A, 0xFA6ED5, 0x772D30,
            0x433B15, 0xC614B5, 0x9D19C3, 0xC2C4AD, 0x414D2C, 0x5D000C, 0x467D86, 0x2D71E3, 0x9AC69B,
            0x006233, 0x7CD2B4, 0x97A7B4, 0xD55537, 0xF63ED7, 0x1810A3, 0xFC764D, 0x2A9D64, 0xABD770,
            0xF87C63, 0x57B07A, 0xE71517, 0x5649C0, 0xD9D63B, 0x3884A7, 0xCB2324, 0x778AD6, 0x23545A,
            0xB91F00, 0x1B0AF1, 0xDFCE19, 0xFF319F, 0x6A1E66, 0x615799, 0x47FBAC, 0xD87F7E, 0xB76522,
            0x89E832, 0x60BFE6, 0xCDC4EF, 0x09366C, 0xD43F5D, 0xD7DE16, 0xDE3B58, 0x929BDE, 0x2822D2,
            0xE88628, 0x4D58E2, 0x32CAC6, 0x16E308, 0xCB7DE0, 0x50C017, 0xA71DF3, 0x5BE018, 0x34132E,
            0x621283, 0x014883, 0x5B8EF5, 0x7FB0AD, 0xF2E91E, 0x434A48, 0xD36710, 0xD8DDAA, 0x425FAE,
            0xCE616A, 0xA4280A, 0xB499D3, 0xF2A606, 0x7F775C, 0x83C2A3, 0x883C61, 0x78738A, 0x5A8CAF,
            0xBDD76F, 0x63A62D, 0xCBBFF4, 0xEF818D, 0x67C126, 0x45CA55, 0x36D9CA, 0xD2A828, 0x8D61C2,
            0x77C912, 0x142604, 0x9B4612, 0xC459C4, 0x44C5C8, 0x91B24D, 0xF31700, 0xAD43D4, 0xE54929,
            0x10D5FD, 0xFCBE00, 0xCC941E, 0xEECE70, 0xF53E13, 0x80F1EC, 0xC3E7B3, 0x28F8C7, 0x940593,
            0x3E71C1, 0xB3092E, 0xF3450B, 0x9C1288, 0x7B20AB, 0x9FB52E, 0xC29247, 0x2F327B, 0x6D550C,
            0x90A772, 0x1FE76B, 0x96CB31, 0x4A1679, 0xE27941, 0x89DFF4, 0x9794E8, 0x84E6E2, 0x973199,
            0x6BED88, 0x365F5F, 0x0EFDBB, 0xB49A48, 0x6CA467, 0x427271, 0x325D8D, 0xB8159F, 0x09E5BC,
            0x25318D, 0x3974F7, 0x1C0530, 0x010C0D, 0x68084B, 0x58EE2C, 0x90AA47, 0x02E774, 0x24D6BD,
            0xA67DF7, 0x72486E, 0xEF169F, 0xA6948E, 0xF691B4, 0x5153D1, 0xF20ACF, 0x339820, 0x7E4BF5,
            0x6863B2, 0x5F3EDD, 0x035D40, 0x7F8985, 0x295255, 0xC06437, 0x10D86D, 0x324832, 0x754C5B,
            0xD4714E, 0x6E5445, 0xC1090B, 0x69F52A, 0xD56614, 0x9D0727, 0x50045D, 0xDB3BB4, 0xC576EA,
            0x17F987, 0x7D6B49, 0xBA271D, 0x296996, 0xACCCC6, 0x5414AD, 0x6AE290, 0x89D988, 0x50722C,
            0xBEA404, 0x940777, 0x7030F3, 0x27FC00, 0xA871EA, 0x49C266, 0x3DE064, 0x83DD97, 0x973FA3,
            0xFD9443, 0x8C860D, 0xDE4131, 0x9D3992, 0x8C70DD, 0xE7B717, 0x3BDF08, 0x2B3715, 0xA0805C,
            0x93805A, 0x921110, 0xD8E80F, 0xAF806C, 0x4BFFDB, 0x0F9038, 0x761859, 0x15A562, 0xBBCB61,
            0xB989C7, 0xBD4010, 0x04F2D2, 0x277549, 0xF6B6EB, 0xBB22DB, 0xAA140A, 0x2F2689, 0x768364,
            0x333B09, 0x1A940E, 0xAA3A51, 0xC2A31D, 0xAEEDAF, 0x12265C, 0x4DC26D, 0x9C7A2D, 0x9756C0,
            0x833F03, 0xF6F009, 0x8C402B, 0x99316D, 0x07B439, 0x15200C, 0x5BC3D8, 0xC492F5, 0x4BADC6,
            0xA5CA4E, 0xCD37A7, 0x36A9E6, 0x9492AB, 0x6842DD, 0xDE6319, 0xEF8C76, 0x528B68, 0x37DBFC,
            0xABA1AE, 0x3115DF, 0xA1AE00, 0xDAFB0C, 0x664D64, 0xB705ED, 0x306529, 0xBF5657, 0x3AFF47,
            0xB9F96A, 0xF3BE75, 0xDF9328, 0x3080AB, 0xF68C66, 0x15CB04, 0x0622FA, 0x1DE4D9, 0xA4B33D,
            0x8F1B57, 0x09CD36, 0xE9424E, 0xA4BE13, 0xB52333, 0x1AAAF0, 0xA8654F, 0xA5C1D2, 0x0F3F0B,
            0xCD785B, 0x76F923, 0x048B7B, 0x721789, 0x53A6C6, 0xE26E6F, 0x00EBEF, 0x584A9B, 0xB7DAC4,
            0xBA66AA, 0xCFCF76, 0x1D02D1, 0x2DF1B1, 0xC1998C, 0x77ADC3, 0xDA4886, 0xA05DF7, 0xF480C6,
            0x2FF0AC, 0x9AECDD, 0xBC5C3F, 0x6DDED0, 0x1FC790, 0xB6DB2A, 0x3A25A3, 0x9AAF00, 0x9353AD,
            0x0457B6, 0xB42D29, 0x7E804B, 0xA707DA, 0x0EAA76, 0xA1597B, 0x2A1216, 0x2DB7DC, 0xFDE5FA,
            0xFEDB89, 0xFDBE89, 0x6C76E4, 0xFCA906, 0x70803E, 0x156E85, 0xFF87FD, 0x073E28, 0x336761,
            0x86182A, 0xEABD4D, 0xAFE7B3, 0x6E6D8F, 0x396795, 0x5BBF31, 0x48D784, 0x16DF30, 0x432DC7,
            0x356125, 0xCE70C9, 0xB8CB30, 0xFD6CBF, 0xA200A4, 0xE46C05, 0xA0DD5A, 0x476F21, 0xD21262,
            0x845CB9, 0x496170, 0xE0566B, 0x015299, 0x375550, 0xB7D51E, 0xC4F133, 0x5F6E13, 0xE4305D,
            0xA92E85, 0xC3B21D, 0x3632A1, 0xA4B708, 0xD4B1EA, 0x21F716, 0xE4698F, 0x77FF27, 0x80030C,
            0x2D408D, 0xA0CD4F, 0x99A520, 0xD3A2B3, 0x0A5D2F, 0x42F9B4, 0xCBDA11, 0xD0BE7D, 0xC1DB9B,
            0xBD17AB, 0x81A2CA, 0x5C6A08, 0x17552E, 0x550027, 0xF0147F, 0x8607E1, 0x640B14, 0x8D4196,
            0xDEBE87, 0x2AFDDA, 0xB6256B, 0x34897B, 0xFEF305, 0x9EBFB9, 0x4F6A68, 0xA82A4A, 0x5AC44F,
            0xBCF82D, 0x985AD7, 0x95C7F4, 0x8D4D0D, 0xA63A20, 0x5F57A4, 0xB13F14, 0x953880, 0x0120CC,
            0x86DD71, 0xB6DEC9, 0xF560BF, 0x11654D, 0x6B0701, 0xACB08C, 0xD0C0B2, 0x485551, 0x0EFB1E,
            0xC37295, 0x3B06A3, 0x3540C0, 0x7BDC06, 0xCC45E0, 0xFA294E, 0xC8CAD6, 0x41F3E8, 0xDE647C,
            0xD8649B, 0x31BED9, 0xC397A4, 0xD45877, 0xC5E369, 0x13DAF0, 0x3C3ABA, 0x461846, 0x5F7555,
            0xF5BDD2, 0xC6926E, 0x5D2EAC, 0xED440E, 0x423E1C, 0x87C461, 0xE9FD29, 0xF3D6E7, 0xCA7C22,
            0x35916F, 0xC5E008, 0x8DD7FF, 0xE26A6E, 0xC6FDB0, 0xC10893, 0x745D7C, 0xB2AD6B, 0x9D6ECD,
            0x7B723E, 0x6A11C6, 0xA9CFF7, 0xDF7329, 0xBAC9B5, 0x5100B7, 0x0DB2E2, 0x24BA74, 0x607DE5,
            0x8AD874, 0x2C150D, 0x0C1881, 0x94667E, 0x162901, 0x767A9F, 0xBEFDFD, 0xEF4556, 0x367ED9,
            0x13D9EC, 0xB9BA8B, 0xFC97C4, 0x27A831, 0xC36EF1, 0x36C594, 0x56A8D8, 0xB5A8B4, 0x0ECCCF,
            0x2D8912, 0x34576F, 0x89562C, 0xE3CE99, 0xB920D6, 0xAA5E6B, 0x9C2A3E, 0xCC5F11, 0x4A0BFD,
            0xFBF4E1, 0x6D3B8E, 0x2C86E2, 0x84D4E9, 0xA9B4FC, 0xD1EEEF, 0xC9352E, 0x61392F, 0x442138,
            0xC8D91B, 0x0AFC81, 0x6A4AFB, 0xD81C2F, 0x84B453, 0x8C994E, 0xCC2254, 0xDC552A, 0xD6C6C0,
            0x96190B, 0xB8701A, 0x649569, 0x605A26, 0xEE523F, 0x0F117F, 0x11B5F4, 0xF5CBFC, 0x2DBC34,
            0xEEBC34, 0xCC5DE8, 0x605EDD, 0x9B8E67, 0xEF3392, 0xB817C9, 0x9B5861, 0xBC57E1, 0xC68351,
            0x103ED8, 0x4871DD, 0xDD1C2D, 0xA118AF, 0x462C21, 0xD7F359, 0x987AD9, 0xC0549E, 0xFA864F,
            0xFC0656, 0xAE79E5, 0x362289, 0x22AD38, 0xDC9367, 0xAAE855, 0x382682, 0x9BE7CA, 0xA40D51,
            0xB13399, 0x0ED7A9, 0x480569, 0xF0B265, 0xA7887F, 0x974C88, 0x36D1F9, 0xB39221, 0x4A827B,
            0x21CF98, 0xDC9F40, 0x5547DC, 0x3A74E1, 0x42EB67, 0xDF9DFE, 0x5FD45E, 0xA4677B, 0x7AACBA,
            0xA2F655, 0x23882B, 0x55BA41, 0x086E59, 0x862A21, 0x834739, 0xE6E389, 0xD49EE5, 0x40FB49,
            0xE956FF, 0xCA0F1C, 0x8A59C5, 0x2BFA94, 0xC5C1D3, 0xCFC50F, 0xAE5ADB, 0x86C547, 0x624385,
            0x3B8621, 0x94792C, 0x876110, 0x7B4C2A, 0x1A2C80, 0x12BF43, 0x902688, 0x893C78, 0xE4C4A8,
            0x7BDBE5, 0xC23AC4, 0xEAF426, 0x8A67F7, 0xBF920D, 0x2BA365, 0xB1933D, 0x0B7CBD, 0xDC51A4,
            0x63DD27, 0xDDE169, 0x19949A, 0x9529A8, 0x28CE68, 0xB4ED09, 0x209F44, 0xCA984E, 0x638270,
            0x237C7E, 0x32B90F, 0x8EF5A7, 0xE75614, 0x08F121, 0x2A9DB5, 0x4D7E6F, 0x5119A5, 0xABF9B5,
            0xD6DF82, 0x61DD96, 0x023616, 0x9F3AC4, 0xA1A283, 0x6DED72, 0x7A8D39, 0xA9B882, 0x5C326B,
            0x5B2746, 0xED3400, 0x7700D2, 0x55F4FC, 0x4D5901, 0x8071E0,
        };

        private static readonly double[] Pio2Table =
        {
            1.57079625129699707031e+00, /* 0x3FF921FB, 0x40000000 */
            7.54978941586159635335e-08, /* 0x3E74442D, 0x00000000 */
            5.39030252995776476554e-15, /* 0x3CF84698, 0x80000000 */
            3.28200341580791294123e-22, /* 0x3B78CC51, 0x60000000 */
            1.27065575308067607349e-29, /* 0x39F01B83, 0x80000000 */
            1.22933308981111328932e-36, /* 0x387A2520, 0x40000000 */
            2.73370053816464559624e-44, /* 0x36E38222, 0x80000000 */
            2.16741683877804819444e-51, /* 0x3569F31D, 0x00000000 */
        };

        /// <summary>
        /// Returns the last three digits of N with y = x - N*pi/2 so |y| less
        /// than pi/2. x is given as 24-bit chunks; e0 is the exponent of x[0].
        /// prec = 1 (double).
        /// </summary>
        private static int RemPio2Large(double[] x, double[] y, int e0, int prec)
        {
            double x1p24 = FromBits(0x4170000000000000UL);  // 2^24
            double x1pM24 = FromBits(0x3e70000000000000UL); // 2^-24

            int nx = x.Length;

            double fw;
            int n = 0;
            int ih = 0;
            double z = 0.0;
            double[] f = new double[20];
            double[] fq = new double[20];
            double[] q = new double[20];
            int[] iq = new int[20];

            // initialize jk
            int jk = InitJk[prec];
            int jp = jk;

            // determine jx, jv, q0; note that 3>q0
            int jx = nx - 1;
            int jv = (e0 - 3) / 24;
            if (jv < 0)
            {
                jv = 0;
            }
            int q0 = e0 - 24 * (jv + 1);

            // set up f[0] to f[jx+jk] where f[jx+jk] = ipio2[jv+jk]
            int j = jv - jx;
            int m = jx + jk;
            for (int i1 = 0; i1 <= m; i1++)
            {
                f[i1] = j < 0 ? 0.0 : Ipio2[j];
                j += 1;
            }

            // compute q[0], q[1], ... q[jk]
            for (int i1 = 0; i1 <= jk; i1++)
            {
                fw = 0.0;
                for (int j1 = 0; j1 <= jx; j1++)
                {
                    fw += x[j1] * f[jx + i1 - j1];
                }
                q[i1] = fw;
            }

            int jz = jk;

            bool recompute = true;
            while (recompute)
            {
                recompute = false;

                // distill q[] into iq[] reversingly
                int i = 0;
                z = q[jz];
                for (int j1 = jz; j1 >= 1; j1--)
                {
                    fw = RustAsI32(x1pM24 * z);
                    iq[i] = RustAsI32(z - x1p24 * fw);
                    z = q[j1 - 1] + fw;
                    i += 1;
                }

                // compute n
                z = Scalbn(z, q0);           // actual value of z
                z -= 8.0 * Floor(z * 0.125);  // trim off integer >= 8
                n = RustAsI32(z);
                z -= n;
                ih = 0;
                if (q0 > 0)
                {
                    // need iq[jz-1] to determine n
                    i = iq[jz - 1] >> (24 - q0);
                    n += i;
                    iq[jz - 1] -= i << (24 - q0);
                    ih = iq[jz - 1] >> (23 - q0);
                }
                else if (q0 == 0)
                {
                    ih = iq[jz - 1] >> 23;
                }
                else if (z >= 0.5)
                {
                    ih = 2;
                }

                if (ih > 0)
                {
                    // q > 0.5
                    n += 1;
                    int carry = 0;
                    for (int i1 = 0; i1 < jz; i1++)
                    {
                        // compute 1-q
                        int j1 = iq[i1];
                        if (carry == 0)
                        {
                            if (j1 != 0)
                            {
                                carry = 1;
                                iq[i1] = 0x1000000 - j1;
                            }
                        }
                        else
                        {
                            iq[i1] = 0xffffff - j1;
                        }
                    }
                    if (q0 > 0)
                    {
                        // rare case: chance is 1 in 12
                        switch (q0)
                        {
                            case 1:
                                iq[jz - 1] &= 0x7fffff;
                                break;
                            case 2:
                                iq[jz - 1] &= 0x3fffff;
                                break;
                        }
                    }
                    if (ih == 2)
                    {
                        z = 1.0 - z;
                        if (carry != 0)
                        {
                            z -= Scalbn(1.0, q0);
                        }
                    }
                }

                // check if recomputation is needed
                if (z == 0.0)
                {
                    int j1 = 0;
                    for (int i1 = jk; i1 <= jz - 1; i1++)
                    {
                        j1 |= iq[i1];
                    }
                    if (j1 == 0)
                    {
                        // need recomputation
                        int k = 1;
                        while (iq[jk - k] == 0)
                        {
                            k += 1; // k = no. of terms needed
                        }

                        for (int i1 = jz + 1; i1 <= jz + k; i1++)
                        {
                            // add q[jz+1] to q[jz+k]
                            f[jx + i1] = Ipio2[jv + i1];
                            fw = 0.0;
                            for (int j2 = 0; j2 <= jx; j2++)
                            {
                                fw += x[j2] * f[jx + i1 - j2];
                            }
                            q[i1] = fw;
                        }
                        jz += k;
                        recompute = true;
                    }
                }
            }

            // chop off zero terms
            if (z == 0.0)
            {
                jz -= 1;
                q0 -= 24;
                while (iq[jz] == 0)
                {
                    jz -= 1;
                    q0 -= 24;
                }
            }
            else
            {
                // break z into 24-bit if necessary
                z = Scalbn(z, -q0);
                if (z >= x1p24)
                {
                    fw = RustAsI32(x1pM24 * z);
                    iq[jz] = RustAsI32(z - x1p24 * fw);
                    jz += 1;
                    q0 += 24;
                    iq[jz] = RustAsI32(fw);
                }
                else
                {
                    iq[jz] = RustAsI32(z);
                }
            }

            // convert integer "bit" chunk to floating-point value
            fw = Scalbn(1.0, q0);
            for (int i1 = jz; i1 >= 0; i1--)
            {
                q[i1] = fw * iq[i1];
                fw *= x1pM24;
            }

            // compute PIo2[0,...,jp]*q[jz,...,0]
            for (int i1 = jz; i1 >= 0; i1--)
            {
                fw = 0.0;
                int k = 0;
                while (k <= jp && k <= jz - i1)
                {
                    fw += Pio2Table[k] * q[i1 + k];
                    k += 1;
                }
                fq[jz - i1] = fw;
            }

            // compress fq[] into y[]
            switch (prec)
            {
                case 0:
                    fw = 0.0;
                    for (int i1 = jz; i1 >= 0; i1--)
                    {
                        fw += fq[i1];
                    }
                    y[0] = ih == 0 ? fw : -fw;
                    break;
                case 1:
                case 2:
                    fw = 0.0;
                    for (int i1 = jz; i1 >= 0; i1--)
                    {
                        fw += fq[i1];
                    }
                    y[0] = ih == 0 ? fw : -fw;
                    fw = fq[0] - fw;
                    for (int i1 = 1; i1 <= jz; i1++)
                    {
                        fw += fq[i1];
                    }
                    y[1] = ih == 0 ? fw : -fw;
                    break;
                default:
                    // painful (quad) path unused here
                    fw = 0.0;
                    for (int i1 = jz; i1 >= 0; i1--)
                    {
                        fw += fq[i1];
                    }
                    y[0] = ih == 0 ? fw : -fw;
                    fw = fq[0] - fw;
                    for (int i1 = 1; i1 <= jz; i1++)
                    {
                        fw += fq[i1];
                    }
                    y[1] = ih == 0 ? fw : -fw;
                    break;
            }
            return n & 7;
        }

        // ------------------------------------------------------------------
        // public API: sin / cos / exp / atan / atan2
        // ------------------------------------------------------------------

        /// <summary>The sine of <paramref name="x"/> (radians), f64.</summary>
        public static double Sin(double x)
        {
            double x1p120 = FromBits(0x4770000000000000UL); // 2^120

            // high word of x
            uint ix = HighWord(x);

            // |x| ~< pi/4
            if (ix <= 0x3fe921fb)
            {
                if (ix < 0x3e500000)
                {
                    // |x| < 2^-26: raise inexact/underflow flags like the C
                    return x; // (flag effects are not observable here)
                }
                return KSin(x, 0.0, 0);
            }

            // sin(Inf or NaN) is NaN
            if (ix >= 0x7ff00000)
            {
                return x - x;
            }

            // argument reduction needed
            var (n, y0, y1) = RemPio2(x);
            switch (n & 3)
            {
                case 0: return KSin(y0, y1, 1);
                case 1: return KCos(y0, y1);
                case 2: return -KSin(y0, y1, 1);
                default: return -KCos(y0, y1);
            }
        }

        /// <summary>The cosine of <paramref name="x"/> (radians), f64.</summary>
        public static double Cos(double x)
        {
            uint ix = HighWord(x);

            // |x| ~< pi/4
            if (ix <= 0x3fe921fb)
            {
                if (ix < 0x3e46a09e)
                {
                    // if x < 2^-27 * sqrt(2): raise inexact if x != 0
                    if (RustAsI32(x) == 0)
                    {
                        return 1.0;
                    }
                }
                return KCos(x, 0.0);
            }

            // cos(Inf or NaN) is NaN
            if (ix >= 0x7ff00000)
            {
                return x - x;
            }

            var (n, y0, y1) = RemPio2(x);
            switch (n & 3)
            {
                case 0: return KCos(y0, y1);
                case 1: return -KSin(y0, y1, 1);
                case 2: return -KCos(y0, y1);
                default: return KSin(y0, y1, 1);
            }
        }

        private const double ExpHalf0 = 0.5;
        private const double ExpHalf1 = -0.5;
        private const double Ln2Hi = 6.93147180369123816490e-01; /* 0x3fe62e42, 0xfee00000 */
        private const double Ln2Lo = 1.90821492927058770002e-10;  /* 0x3dea39ef, 0x35793c76 */
        private const double InvLn2 = 1.44269504088896338700e+00; /* 0x3ff71547, 0x652b82fe */
        private const double P1 = 1.66666666666666019037e-01;     /* 0x3FC55555, 0x5555553E */
        private const double P2 = -2.77777777770155933842e-03;    /* 0xBF66C16C, 0x16BEBD93 */
        private const double P3 = 6.61375632143793436117e-05;     /* 0x3F11566A, 0xAF25DE2C */
        private const double P4 = -1.65339022054652515390e-06;    /* 0xBEBBBD41, 0xC5D26BF1 */
        private const double P5 = 4.13813679705723846039e-08;     /* 0x3E663769, 0x72BEA4D0 */

        /// <summary>The exponential of <paramref name="x"/> (base e), f64.</summary>
        public static double Exp(double x)
        {
            double x1p1023 = FromBits(0x7fe0000000000000UL);  // 2^1023
            double x1pM149 = FromBits(0x36a0000000000000UL);  // 2^-149

            double hi;
            double lo;
            double c;
            double xx;
            double y;
            int k;
            int sign;

            uint hx = (uint)(BitConverter.DoubleToInt64Bits(x) >> 32);
            sign = (int)(hx >> 31);
            hx &= 0x7fffffff; // high word of |x|

            // special cases
            if (hx >= 0x4086232b)
            {
                // if |x| >= 708.39...
                if (double.IsNaN(x))
                {
                    return x;
                }
                if (x > 709.782712893383973096)
                {
                    // overflow if x != inf
                    x *= x1p1023;
                    return x;
                }
                if (x < -708.39641853226410622)
                {
                    // underflow if x != -inf
                    if (x < -745.13321910194110842)
                    {
                        return 0.0;
                    }
                }
            }

            // argument reduction
            if (hx > 0x3fd62e42)
            {
                // if |x| > 0.5 ln2
                if (hx >= 0x3ff0a2b2)
                {
                    // if |x| >= 1.5 ln2
                    k = RustAsI32(InvLn2 * x + (sign != 0 ? ExpHalf1 : ExpHalf0));
                }
                else
                {
                    k = 1 - sign - sign;
                }
                hi = x - k * Ln2Hi; // k*ln2hi is exact here
                lo = k * Ln2Lo;
                x = hi - lo;
            }
            else if (hx > 0x3e300000)
            {
                // if |x| > 2^-28
                k = 0;
                hi = x;
                lo = 0.0;
            }
            else
            {
                // inexact if x != 0
                return 1.0 + x;
            }

            // x is now in primary range
            xx = x * x;
            c = x - xx * (P1 + xx * (P2 + xx * (P3 + xx * (P4 + xx * P5))));
            y = 1.0 + (x * c / (2.0 - c) - lo + hi);
            if (k == 0)
            {
                return y;
            }
            return Scalbn(y, k);
        }

        private static readonly double[] AtanHi =
        {
            4.63647609000806093515e-01, /* atan(0.5)hi 0x3FDDAC67, 0x0561BB4F */
            7.85398163397448278999e-01, /* atan(1.0)hi 0x3FE921FB, 0x54442D18 */
            9.82793723247329054082e-01, /* atan(1.5)hi 0x3FEF730B, 0xD281F69B */
            1.57079632679489655800e+00, /* atan(inf)hi 0x3FF921FB, 0x54442D18 */
        };

        private static readonly double[] AtanLo =
        {
            2.26987774529616870924e-17, /* atan(0.5)lo 0x3C7A2B7F, 0x222F65E2 */
            3.06161699786838301793e-17, /* atan(1.0)lo 0x3C81A626, 0x33145C07 */
            1.39033110312309984516e-17, /* atan(1.5)lo 0x3C700788, 0x7AF0CBBD */
            6.12323399573676603587e-17, /* atan(inf)lo 0x3C91A626, 0x33145C07 */
        };

        private static readonly double[] At =
        {
            3.33333333333329318027e-01,  /* 0x3FD55555, 0x5555550D */
            -1.99999999998764832476e-01, /* 0xBFC99999, 0x9998EBC4 */
            1.42857142725034663711e-01,  /* 0x3FC24924, 0x920083FF */
            -1.11111104054623557880e-01, /* 0xBFBC71C6, 0xFE231671 */
            9.09088713343650656196e-02,  /* 0x3FB745CD, 0xC54C206E */
            -7.69187620504482999495e-02, /* 0xBFB3B0F2, 0xAF749A6D */
            6.66107313738753120669e-02,  /* 0x3FB10D66, 0xA0D03D51 */
            -5.83357013379057348645e-02, /* 0xBFADDE2D, 0x52DEFD9A */
            4.97687799461593236017e-02,  /* 0x3FA97B4B, 0x24760DEB */
            -3.65315727442169155270e-02, /* 0xBFA2B444, 0x2C6A6C2F */
            1.62858201153657823623e-02,  /* 0x3F90AD3A, 0xE322DA11 */
        };

        /// <summary>The arctangent of <paramref name="x"/>, f64.</summary>
        public static double Atan(double x)
        {
            uint ix = (uint)(BitConverter.DoubleToInt64Bits(x) >> 32);
            uint sign = ix >> 31;
            ix &= 0x7fff_ffff;
            if (ix >= 0x4410_0000)
            {
                if (double.IsNaN(x))
                {
                    return x;
                }

                double zq = AtanHi[3] + FromBits(0x0380_0000UL); // 0x1p-120f
                return sign != 0 ? -zq : zq;
            }

            int id;
            if (ix < 0x3fdc_0000)
            {
                // |x| < 0.4375
                if (ix < 0x3e40_0000)
                {
                    // |x| < 2^-27
                    return x;
                }
                id = -1;
            }
            else
            {
                x = Fabs(x);
                if (ix < 0x3ff30000)
                {
                    // |x| < 1.1875
                    if (ix < 0x3fe60000)
                    {
                        // 7/16 <= |x| < 11/16
                        x = (2.0 * x - 1.0) / (2.0 + x);
                        id = 0;
                    }
                    else
                    {
                        // 11/16 <= |x| < 19/16
                        x = (x - 1.0) / (x + 1.0);
                        id = 1;
                    }
                }
                else if (ix < 0x40038000)
                {
                    // |x| < 2.4375
                    x = (x - 1.5) / (1.0 + 1.5 * x);
                    id = 2;
                }
                else
                {
                    // 2.4375 <= |x| < 2^66
                    x = -1.0 / x;
                    id = 3;
                }
            }

            double z = x * x;
            double w = z * z;
            // break sum from i=0 to 10 At[i]z**(i+1) into odd and even poly
            double s1 = z * (At[0] + w * (At[2] + w * (At[4] + w * (At[6] + w * (At[8] + w * At[10])))));
            double s2 = w * (At[1] + w * (At[3] + w * (At[5] + w * (At[7] + w * At[9]))));

            if (id < 0)
            {
                return x - x * (s1 + s2);
            }

            double z2 = AtanHi[id] - (x * (s1 + s2) - AtanLo[id] - x);
            return sign != 0 ? -z2 : z2;
        }

        private const double Atan2Pi = 3.1415926535897931160e+00;   /* 0x400921FB, 0x54442D18 */
        private const double Atan2PiLo = 1.2246467991473531772e-16; /* 0x3CA1A626, 0x33145C07 */

        /// <summary>The arctangent of y/x, f64, quadrant-correct.</summary>
        public static double Atan2(double y, double x)
        {
            if (double.IsNaN(x) || double.IsNaN(y))
            {
                // NaN selection of the reference build: y's NaN wins (the
                // compiled x + y evaluates with y as the destination
                // operand). Spelled out so the runtime's own NaN selection
                // can never leak through.
                if (double.IsNaN(y))
                {
                    return Quiet(y);
                }
                return Quiet(x);
            }
            uint ix = (uint)(BitConverter.DoubleToInt64Bits(x) >> 32);
            uint lx = (uint)BitConverter.DoubleToInt64Bits(x);
            uint iy = (uint)(BitConverter.DoubleToInt64Bits(y) >> 32);
            uint ly = (uint)BitConverter.DoubleToInt64Bits(y);
            if (((ix - 0x3ff00000) | lx) == 0)
            {
                // x = 1.0
                return Atan(y);
            }
            uint m = ((iy >> 31) & 1) | ((ix >> 30) & 2); // 2*sign(x)+sign(y)
            ix &= 0x7fffffff;
            iy &= 0x7fffffff;

            // when y = 0
            if ((iy | ly) == 0)
            {
                switch (m)
                {
                    case 0: return y;      // atan(+-0, +anything) = +-0
                    case 1: return y;      // atan(+-0, +anything) = +-0
                    case 2: return Atan2Pi;    // atan(+0, -anything) = PI
                    default: return -Atan2Pi;  // atan(-0, -anything) = -PI
                }
            }
            // when x = 0
            if ((ix | lx) == 0)
            {
                return (m & 1) != 0 ? -Atan2Pi / 2.0 : Atan2Pi / 2.0;
            }
            // when x is INF
            if (ix == 0x7ff00000)
            {
                if (iy == 0x7ff00000)
                {
                    switch (m)
                    {
                        case 0: return Atan2Pi / 4.0;        // atan(+INF, +INF)
                        case 1: return -Atan2Pi / 4.0;      // atan(-INF, +INF)
                        case 2: return 3.0 * Atan2Pi / 4.0; // atan(+INF, -INF)
                        default: return -3.0 * Atan2Pi / 4.0; // atan(-INF, -INF)
                    }
                }
                else
                {
                    switch (m)
                    {
                        case 0: return 0.0;     // atan(+..., +INF)
                        case 1: return -0.0;    // atan(-..., +INF)
                        case 2: return Atan2Pi; // atan(+..., -INF)
                        default: return -Atan2Pi; // atan(-..., -INF)
                    }
                }
            }
            // |y/x| > 0x1p64
            long ixSigned = unchecked((int)ix);
            long iySigned = unchecked((int)iy);
            if (unchecked(ixSigned + (64L << 20)) < iySigned || iy == 0x7ff00000)
            {
                return (m & 1) != 0 ? -Atan2Pi / 2.0 : Atan2Pi / 2.0;
            }

            // z = atan(|y/x|) without spurious underflow
            double z;
            if ((m & 2) != 0 && unchecked(iySigned + (64L << 20)) < ixSigned)
            {
                // |y/x| < 0x1p-64, x<0
                z = 0.0;
            }
            else
            {
                z = Atan(Fabs(y / x));
            }
            switch (m)
            {
                case 0: return z;                    // atan(+, +)
                case 1: return -z;                   // atan(-, +)
                case 2: return Atan2Pi - (z - Atan2PiLo); // atan(+, -)
                default: return (z - Atan2PiLo) - Atan2Pi; // atan(-, -)
            }
        }
    }
}
