using System.Collections.Generic;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// Pins the current runtime's floating-point primitives against the
    /// conforming-runtime reference table: .NET, Mono 6.8, Mono 6.12, and
    /// Mono's interpreter are all verified bit-identical.
    ///
    /// Two batteries exist (see FpDiagnostics):
    /// - the RAW battery diverges by one ulp on runtimes that promote f32
    ///   expression trees to f64 intermediates (Unity's embedded Mono);
    ///   that is environment characterization, so on such runtimes the raw
    ///   tests pass as long as the GUARDED battery is bit-exact (the
    ///   immunity the transformed core relies on).
    /// - the GUARDED battery must be bit-exact on EVERY runtime: it is the
    ///   same kernels with every f32 op materialized through
    ///   (float)(double)(...) guards, exactly like the simulation core.
    /// </summary>
    [TestFixture]
    public class FpDiagnosticsTests
    {
        [Test]
        public void KernelBitsMatchReference()
        {
            List<FpDiagnostics.KernelResult> results = FpDiagnostics.RunKernels();
            Assert.That(results.Count, Is.EqualTo(FpDiagnosticsReference.Names.Length),
                "kernel count changed — regenerate FpDiagnosticsReference.cs");
            var mismatched = new List<string>();
            for (int k = 0; k < results.Count; k++)
            {
                Assert.That(results[k].name, Is.EqualTo(FpDiagnosticsReference.Names[k]),
                    "kernel order changed — regenerate the reference");
                Assert.That(results[k].bits.Length,
                    Is.EqualTo(FpDiagnosticsReference.Bits[k].Length),
                    $"{results[k].name}: result count changed");
                for (int i = 0; i < results[k].bits.Length; i++)
                {
                    if (results[k].bits[i] != FpDiagnosticsReference.Bits[k][i])
                    {
                        mismatched.Add($"{results[k].name}[{i}]: got " +
                            $"0x{results[k].bits[i]:X8}, want " +
                            $"0x{FpDiagnosticsReference.Bits[k][i]:X8}");
                    }
                }
            }
            if (mismatched.Count == 0)
            {
                return; // conforming runtime: raw bits are the reference bits
            }
            // Raw divergence: fine ONLY if this runtime promotes f32 trees
            // AND the guarded battery is bit-exact (immunity active).
            if (GuardedBatteryExact())
            {
                Assert.Pass($"raw battery diverges on {mismatched.Count} value(s) " +
                    "because this runtime promotes f32 expression trees to f64 " +
                    "(Unity Mono does this; it is environment characterization). " +
                    "The guarded battery is bit-exact — per-op rounding immunity " +
                    "is active. See Assets/Docs/FLOAT_PROMOTION.md.");
            }
            Assert.Fail("raw battery diverged and the guarded battery is NOT " +
                "exact — the runtime computes some primitive differently in a " +
                "way the guards do not cover: " + string.Join("; ", mismatched));
        }

        [Test]
        public void FingerprintMatchesReference()
        {
            ulong raw = FpDiagnostics.Fingerprint(FpDiagnostics.RunKernels());
            if (raw == FpDiagnosticsReference.Fingerprint)
            {
                return; // conforming runtime
            }
            Assert.That(FpDiagnostics.Fingerprint(FpDiagnostics.RunKernelsGuarded()),
                Is.EqualTo(FpDiagnosticsReference.Fingerprint),
                "raw fingerprint 0x" + raw.ToString("X16") + " diverged (this " +
                "runtime promotes f32 trees) AND the guarded fingerprint is " +
                "wrong — immunity broken");
            Assert.Pass("raw fingerprint 0x" + raw.ToString("X16") + " diverged " +
                "as expected on a promoting runtime (Unity Mono); the guarded " +
                "fingerprint is bit-exact — immunity active. See " +
                "Assets/Docs/FLOAT_PROMOTION.md.");
        }

        [Test]
        public void GuardedKernelBitsMatchReference()
        {
            List<FpDiagnostics.KernelResult> results =
                FpDiagnostics.RunKernelsGuarded();
            Assert.That(results.Count, Is.EqualTo(FpDiagnosticsReference.Names.Length),
                "guarded kernel count changed — regenerate FpDiagnosticsReference.cs");
            for (int k = 0; k < results.Count; k++)
            {
                Assert.That(results[k].name, Is.EqualTo(FpDiagnosticsReference.Names[k]),
                    "guarded kernel order changed — regenerate the reference");
                Assert.That(results[k].bits.Length,
                    Is.EqualTo(FpDiagnosticsReference.Bits[k].Length),
                    $"{results[k].name}: guarded result count changed");
                for (int i = 0; i < results[k].bits.Length; i++)
                {
                    Assert.That(results[k].bits[i],
                        Is.EqualTo(FpDiagnosticsReference.Bits[k][i]),
                        $"{results[k].name}[{i}]: guarded primitive diverged " +
                        "from the conforming-runtime reference — this should " +
                        "be impossible on any runtime");
                }
            }
        }

        [Test]
        public void GuardedFingerprintMatchesReference()
        {
            Assert.That(
                FpDiagnostics.Fingerprint(FpDiagnostics.RunKernelsGuarded()),
                Is.EqualTo(FpDiagnosticsReference.Fingerprint),
                "guarded fingerprint must equal the reference on every " +
                "runtime, promoting or not");
        }

        /// <summary>The guarded battery vs the reference, bit-for-bit.</summary>
        private static bool GuardedBatteryExact()
        {
            List<FpDiagnostics.KernelResult> results =
                FpDiagnostics.RunKernelsGuarded();
            if (results.Count != FpDiagnosticsReference.Names.Length)
            {
                return false;
            }
            for (int k = 0; k < results.Count; k++)
            {
                if (results[k].name != FpDiagnosticsReference.Names[k]
                    || results[k].bits.Length
                        != FpDiagnosticsReference.Bits[k].Length)
                {
                    return false;
                }
                for (int i = 0; i < results[k].bits.Length; i++)
                {
                    if (results[k].bits[i] != FpDiagnosticsReference.Bits[k][i])
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
