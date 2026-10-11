using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>
    /// A floating-point primitive battery for cross-runtime diagnosis.
    /// Every kernel is a computation pattern the simulation pipeline
    /// actually uses, evaluated over fixed inputs; the results are raw f32
    /// bit patterns. On a conforming runtime (desktop .NET, Mono 6.8,
    /// Mono 6.12, and Mono's interpreter are all verified identical), the
    /// bits equal the generated reference table — a runtime that diverges
    /// shows exactly WHICH primitive differs by comparing kernel by kernel.
    ///
    /// There are two batteries:
    /// <list type="bullet">
    /// <item><see cref="RunKernels"/> — the RAW battery. Diverges on
    /// runtimes that evaluate f32 expression trees with f64 intermediates
    /// (Unity's embedded Mono does: it never enabled upstream Mono's
    /// float32 JIT optimization, so multi-op f32 trees are computed in
    /// double and rounded once at boundaries). On such runtimes the
    /// 'association' rows flip by exactly one ulp — that divergence is
    /// ENVIRONMENT CHARACTERIZATION, not a bug in this port.</item>
    /// <item><see cref="RunKernelsGuarded"/> — the same kernels with every
    /// f32 arithmetic op explicitly materialized through
    /// (float)(double)(...) guards, exactly like the simulation core is
    /// transformed. Every intermediate is rounded once per operation on
    /// EVERY runtime, so the guarded battery must match the same reference
    /// table everywhere — this is the verdict.</item>
    /// </list>
    ///
    /// The diagnostics scene reports both (see
    /// Assets/Source/Unity/TmnfSimDiagnostics.cs and
    /// Assets/Docs/FLOAT_PROMOTION.md for the full story).
    /// </summary>
    public static class FpDiagnostics
    {
        /// <summary>One kernel's name and its result bit patterns.</summary>
        public struct KernelResult
        {
            public string name;
            public uint[] bits;
        }

        // Pipeline-representative inputs: wheel radii, speeds, epsilons,
        // tuning magnitudes, engine RPM, track extents.
        private static readonly float[] Xs =
        {
            0.0f, 1.0f, -1.0f, 0.1f, 87.9932556f, 32.4543343f, 5662.453125f,
            9.999999439624929e-11f, 1e-40f, 3.4028235e38f, 0.5f, 1e22f,
            0.3333333432674408f, 123456.789f, -0.0f, 6.02e23f,
        };

        private static uint Bits(float v)
        {
            return unchecked((uint)BitConverter.SingleToInt32Bits(v));
        }

        /// <summary>Runs every kernel; results are deterministic.</summary>
        public static List<KernelResult> RunKernels()
        {
            var results = new List<KernelResult>();

            // (float)Math.Sqrt((double)x) — the port's Sqrt contract.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits((float)Math.Sqrt((double)x)));
                }
                results.Add(new KernelResult { name = "sqrt-dbl-narrow", bits = bits.ToArray() });
            }
            // Fp.Sqrt — the core helper every norm goes through.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits(Fp.Sqrt(x)));
                }
                results.Add(new KernelResult { name = "fp-sqrt", bits = bits.ToArray() });
            }
            // Abs / negation / the -0.0 normalizing add.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits(Math.Abs(x)));
                    bits.Add(Bits(-x));
                    bits.Add(Bits(x + 0.0f));
                }
                results.Add(new KernelResult { name = "abs-neg-add0", bits = bits.ToArray() });
            }
            // f32 -> f64 arithmetic -> f32 single narrowing.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits((float)((double)x * 3.0 / 7.0)));
                }
                results.Add(new KernelResult { name = "dbl-arithmetic-narrow", bits = bits.ToArray() });
            }
            // Written-operand-order associations (never reassociated).
            {
                float a = 0.3333333432674408f, b = 87.9932556f, c = 32.4543343f;
                uint[] bits =
                {
                    Bits(a * b + c), Bits(a * (b + c)),
                    Bits((a * b) * c), Bits(a * (b * c)),
                    Bits((a + b) + c), Bits(a + (b + c)),
                };
                results.Add(new KernelResult { name = "association", bits = bits });
            }
            // Remainder (the fmod lane).
            {
                uint[] bits =
                {
                    Bits(87.9932556f % 0.3333333432674408f),
                    Bits(-87.9932556f % 0.3333333432674408f),
                    Bits(5662.453125f % 100.0f),
                    Bits(0.1f % 0.033333335f),
                };
                results.Add(new KernelResult { name = "rem", bits = bits });
            }
            // Norm-style sqrt of a sum of squares, three associations.
            {
                float a = 0.3333333432674408f, b = 87.9932556f, c = 32.4543343f;
                uint[] bits =
                {
                    Bits((float)Math.Sqrt((double)(a * a + b * b + c * c))),
                    Bits((float)Math.Sqrt((double)((a * a + b * b) + c * c))),
                    Bits(Fp.Sqrt(a * a + b * b + c * c)),
                };
                results.Add(new KernelResult { name = "vec-length", bits = bits });
            }
            // The f64 single-rounding block (GetSpeed-style).
            {
                double d = 0.0;
                foreach (float x in Xs)
                {
                    d += (double)x * 0.017453292519943295;
                }
                ulong dbits = unchecked((ulong)BitConverter.DoubleToInt64Bits(d));
                results.Add(new KernelResult
                {
                    name = "dbl-block",
                    bits = new uint[] { (uint)dbits, (uint)(dbits >> 32), Bits((float)d) },
                });
            }
            // ftol (truncation + range guard).
            {
                uint[] bits =
                {
                    unchecked((uint)Fp.Ftol(3.9e9f)),
                    unchecked((uint)Fp.Ftol(-3.9e9f)),
                    unchecked((uint)Fp.Ftol(123456.789f)),
                    unchecked((uint)Fp.Ftol(-123456.789f)),
                };
                results.Add(new KernelResult { name = "ftol", bits = bits });
            }
            // Long fma-free update chains (drift amplifiers).
            {
                float f = 1.0f;
                for (int i = 0; i < 1000; i++)
                {
                    f = (f * 1.0000001f) + 0.0000001f;
                }
                float g = 123456.789f;
                for (int i = 0; i < 1000; i++)
                {
                    g = g - (g / 1000000.0f) + 0.001f;
                }
                results.Add(new KernelResult
                {
                    name = "chains",
                    bits = new[] { Bits(f), Bits(g) },
                });
            }
            return results;
        }

        /// <summary>
        /// The same battery as <see cref="RunKernels"/>, but every f32
        /// arithmetic operation is explicitly materialized through
        /// (float)(double)(...) guards — each intermediate value is rounded
        /// to f32 exactly once, per operation, on every runtime (including
        /// ones that promote f32 expression trees to f64 intermediates,
        /// like Unity's embedded Mono). On conforming runtimes the guards
        /// are provable identity: a single f32 +,-,*,/,% computed in f64 and
        /// rounded once is the correctly-rounded f32 result (the f64
        /// significand is 53 bits, at least 2*24+2), so the expected bits
        /// are the SAME <see cref="FpDiagnosticsReference"/> table.
        /// </summary>
        public static List<KernelResult> RunKernelsGuarded()
        {
            var results = new List<KernelResult>();

            // (float)Math.Sqrt((double)x) — pure double path + one
            // narrowing; no f32 arithmetic to guard (identity).
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits((float)Math.Sqrt((double)x)));
                }
                results.Add(new KernelResult { name = "sqrt-dbl-narrow", bits = bits.ToArray() });
            }
            // Fp.Sqrt — a method boundary (rounding sink); its internals
            // are guarded in the core itself.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits(Fp.Sqrt(x)));
                }
                results.Add(new KernelResult { name = "fp-sqrt", bits = bits.ToArray() });
            }
            // Abs / negation / the -0.0 normalizing add (the add guarded).
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits(Math.Abs(x)));
                    bits.Add(Bits(-x));
                    bits.Add(Bits((float)(double)(x + 0.0f)));
                }
                results.Add(new KernelResult { name = "abs-neg-add0", bits = bits.ToArray() });
            }
            // f32 -> f64 arithmetic -> f32 — double-typed ops, no f32 trees.
            {
                var bits = new List<uint>();
                foreach (float x in Xs)
                {
                    bits.Add(Bits((float)((double)x * 3.0 / 7.0)));
                }
                results.Add(new KernelResult { name = "dbl-arithmetic-narrow", bits = bits.ToArray() });
            }
            // Written-operand-order associations, guarded per-op — this is
            // the battery that pinpoints f32 tree promotion when the raw
            // rows flip by one ulp.
            {
                float a = 0.3333333432674408f, b = 87.9932556f, c = 32.4543343f;
                uint[] bits =
                {
                    Bits((float)(double)((float)(double)(a * b) + c)),
                    Bits((float)(double)(a * (float)(double)(b + c))),
                    Bits((float)(double)((float)(double)(a * b) * c)),
                    Bits((float)(double)(a * (float)(double)(b * c))),
                    Bits((float)(double)((float)(double)(a + b) + c)),
                    Bits((float)(double)(a + (float)(double)(b + c))),
                };
                results.Add(new KernelResult { name = "association", bits = bits });
            }
            // Remainder — fmod of widened f32 operands is exact, and the
            // guard restores the per-op f32 rounding anyway.
            {
                uint[] bits =
                {
                    Bits((float)(double)(87.9932556f % 0.3333333432674408f)),
                    Bits((float)(double)(-87.9932556f % 0.3333333432674408f)),
                    Bits((float)(double)(5662.453125f % 100.0f)),
                    Bits((float)(double)(0.1f % 0.033333335f)),
                };
                results.Add(new KernelResult { name = "rem", bits = bits });
            }
            // Norm-style sqrt of a sum of squares — the f32 tree guarded
            // per-op before the widening (kept in a local: every binop is
            // wrapped, so the local holds the per-op-rounded value on
            // every runtime).
            {
                float a = 0.3333333432674408f, b = 87.9932556f, c = 32.4543343f;
                float sumSq = (float)(double)((float)(double)((float)(double)(a * a) + (float)(double)(b * b)) + (float)(double)(c * c));
                uint[] bits =
                {
                    Bits((float)Math.Sqrt((double)(sumSq))),
                    Bits((float)Math.Sqrt((double)(sumSq))),
                    Bits(Fp.Sqrt(sumSq)),
                };
                results.Add(new KernelResult { name = "vec-length", bits = bits });
            }
            // The f64 single-rounding block — double accumulation only.
            {
                double d = 0.0;
                foreach (float x in Xs)
                {
                    d += (double)x * 0.017453292519943295;
                }
                ulong dbits = unchecked((ulong)BitConverter.DoubleToInt64Bits(d));
                results.Add(new KernelResult
                {
                    name = "dbl-block",
                    bits = new uint[] { (uint)dbits, (uint)(dbits >> 32), Bits((float)d) },
                });
            }
            // ftol (truncation + range guard) — call boundaries.
            {
                uint[] bits =
                {
                    unchecked((uint)Fp.Ftol(3.9e9f)),
                    unchecked((uint)Fp.Ftol(-3.9e9f)),
                    unchecked((uint)Fp.Ftol(123456.789f)),
                    unchecked((uint)Fp.Ftol(-123456.789f)),
                };
                results.Add(new KernelResult { name = "ftol", bits = bits });
            }
            // Long update chains, guarded per-op.
            {
                float f = 1.0f;
                for (int i = 0; i < 1000; i++)
                {
                    f = (float)(double)((float)(double)(f * 1.0000001f) + 0.0000001f);
                }
                float g = 123456.789f;
                for (int i = 0; i < 1000; i++)
                {
                    g = (float)(double)((float)(double)(g - (float)(double)(g / 1000000.0f)) + 0.001f);
                }
                results.Add(new KernelResult
                {
                    name = "chains",
                    bits = new[] { Bits(f), Bits(g) },
                });
            }
            return results;
        }

        /// <summary>The FNV-1a 64 hash over every kernel's bits.</summary>
        public static ulong Fingerprint(List<KernelResult> results)
        {
            var bytes = new List<byte>(4096);
            foreach (KernelResult r in results)
            {
                foreach (uint b in r.bits)
                {
                    bytes.Add((byte)b);
                    bytes.Add((byte)(b >> 8));
                    bytes.Add((byte)(b >> 16));
                    bytes.Add((byte)(b >> 24));
                }
            }
            return Fnva.Hash(bytes.ToArray());
        }
    }
}
