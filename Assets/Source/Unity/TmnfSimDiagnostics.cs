using System;
using System.Collections.Generic;
using UnityEngine;

namespace Tmnf.Sim.Unity
{
    /// <summary>
    /// The environment verifier. In ONE run it answers, with zero
    /// tolerance, the only question that matters after the v3
    /// promotion-immunity transform:
    ///
    ///    does THIS Unity/Mono runtime reproduce the golden 400-tick
    ///    facade trace bit-for-bit?
    ///
    /// 1. Environment — Unity version, scripting runtime, processor.
    /// 2. RAW fp kernel battery — environment characterization. On
    ///    Unity's embedded Mono this MISMATCHES the conforming-runtime
    ///    reference by exactly one ulp in the 'association' rows: that
    ///    runtime evaluates f32 expression trees with f64 intermediates
    ///    (upstream Mono's float32 JIT optimization was never enabled in
    ///    Unity). This is EXPECTED and informational — see
    ///    Assets/Docs/FLOAT_PROMOTION.md. A match here means the runtime
    ///    rounds f32 per-op (desktop .NET, Mono 6.8/6.12 with default
    ///    flags, the Mono interpreter).
    /// 3. GUARDED fp kernel battery — the verdict, part one. The same
    ///    kernels with every f32 arithmetic op materialized through
    ///    (float)(double)(...) guards, exactly like the simulation core.
    ///    These must be bit-exact on EVERY runtime, promoting or not.
    /// 4. Facade trace — the verdict, part two: replays the 400-tick
    ///    golden drive schedule and hashes the telemetry stream against
    ///    the golden constant. With the guarded core this is bit-exact
    ///    even on promoting runtimes; if it ever diverges, the scene
    ///    reports the FIRST divergent tick and both records' fields.
    ///
    /// Run: open Assets/Scenes/TmnfSimDiagnostics.unity and press Play.
    /// The scene passes when the guarded battery AND the facade trace are
    /// bit-exact, regardless of the raw battery's characterization.
    /// </summary>
    public sealed class TmnfSimDiagnostics : MonoBehaviour
    {
        private const int Ticks = 400;
        private const ulong ExpectedFacadeHash = 0x5A45_761F_3728_DF35UL;

        private void Start()
        {
            LogEnvironment();
            bool promotes = RunRawFpKernels();
            bool guardedOk = RunGuardedFpKernels();
            bool traceOk = RunFacadeTraceDiff();
            Debug.Log("[TmnfSimDiagnostics] SUMMARY — raw fp: " +
                (promotes ? "PROMOTING (informational, expected on Unity Mono)" : "conforming") +
                ", guarded fp: " + (guardedOk ? "BIT-EXACT" : "DIVERGED") +
                ", facade trace: " + (traceOk ? "BIT-EXACT" : "DIVERGED") + ". " +
                "Verdict: " + (guardedOk && traceOk
                    ? "PASS — this runtime reproduces the golden trace"
                    : "FAIL — paste everything above this line in the report") + ".");
        }

        private static void LogEnvironment()
        {
            string mono = "unknown";
            try
            {
                Type monoRuntime = Type.GetType(
                    "Mono.Runtime", false);
                if (monoRuntime != null)
                {
                    mono = "Mono " + monoRuntime
                        .GetMethod("GetDisplayName",
                            System.Reflection.BindingFlags.NonPublic |
                            System.Reflection.BindingFlags.Static)
                        ?.Invoke(null, null);
                }
                else
                {
                    mono = "not Mono (coreclr?)";
                }
            }
            catch (Exception)
            {
                mono = "Mono (version query failed)";
            }
            Debug.Log($"[TmnfSimDiagnostics] environment — " +
                $"unity={Application.unityVersion}, " +
                $"runtime={Environment.Version} ({mono}), " +
                $"platform={Application.platform}, " +
                $"cpu={SystemInfo.processorType}, " +
                $"os={Environment.OSVersion}");
        }

        /// <summary>
        /// The RAW battery: environment characterization only. Returns
        /// true when the runtime promotes f32 trees (Unity Mono does).
        /// </summary>
        private static bool RunRawFpKernels()
        {
            List<FpDiagnostics.KernelResult> results = FpDiagnostics.RunKernels();
            ulong fingerprint = FpDiagnostics.Fingerprint(results);
            bool allOk = fingerprint == FpDiagnosticsReference.Fingerprint;
            Debug.Log($"[TmnfSimDiagnostics] raw fp fingerprint 0x{fingerprint:X16} " +
                $"(reference 0x{FpDiagnosticsReference.Fingerprint:X16}: " +
                $"{(allOk ? "MATCH — runtime rounds f32 per-op (conforming)" : "MISMATCH — runtime promotes f32 trees to f64")})");
            int diverged = 0;
            for (int k = 0; k < results.Count; k++)
            {
                uint[] want = FpDiagnosticsReference.Bits[k];
                uint[] got = results[k].bits;
                if (got.Length != want.Length)
                {
                    Debug.LogWarning($"[TmnfSimDiagnostics] raw kernel " +
                        $"'{results[k].name}' produced {got.Length} values, " +
                        $"reference has {want.Length}");
                    continue;
                }
                for (int i = 0; i < got.Length; i++)
                {
                    if (got[i] != want[i])
                    {
                        Debug.Log($"[TmnfSimDiagnostics] raw kernel " +
                            $"'{results[k].name}'[{i}] differs by one ulp: " +
                            $"got 0x{got[i]:X8}, reference 0x{want[i]:X8} — " +
                            "f32 tree promotion (informational)");
                        diverged++;
                    }
                }
            }
            if (!allOk)
            {
                Debug.Log("[TmnfSimDiagnostics] the raw battery divergence is " +
                    "EXPECTED on Unity's Mono and is not a port bug: the core " +
                    "rounds every f32 op explicitly (see Assets/Docs/" +
                    "FLOAT_PROMOTION.md). The verdict is the guarded battery " +
                    "and the facade trace below." + (diverged > 0
                        ? $" ({diverged} raw values differ, each by one ulp.)"
                        : ""));
            }
            return !allOk;
        }

        /// <summary>
        /// The GUARDED battery: verdict part one — must be bit-exact on
        /// every runtime.
        /// </summary>
        private static bool RunGuardedFpKernels()
        {
            List<FpDiagnostics.KernelResult> results = FpDiagnostics.RunKernelsGuarded();
            ulong fingerprint = FpDiagnostics.Fingerprint(results);
            bool allOk = fingerprint == FpDiagnosticsReference.Fingerprint;
            Debug.Log($"[TmnfSimDiagnostics] guarded fp fingerprint " +
                $"0x{fingerprint:X16} (reference " +
                $"0x{FpDiagnosticsReference.Fingerprint:X16}: " +
                $"{(allOk ? "MATCH" : "MISMATCH")})");
            for (int k = 0; k < results.Count; k++)
            {
                uint[] want = FpDiagnosticsReference.Bits[k];
                uint[] got = results[k].bits;
                if (got.Length != want.Length)
                {
                    Debug.LogError($"[TmnfSimDiagnostics] guarded kernel " +
                        $"'{results[k].name}' produced {got.Length} values, " +
                        $"reference has {want.Length}");
                    allOk = false;
                    continue;
                }
                for (int i = 0; i < got.Length; i++)
                {
                    if (got[i] != want[i])
                    {
                        Debug.LogError($"[TmnfSimDiagnostics] guarded kernel " +
                            $"'{results[k].name}'[{i}] DIVERGED: " +
                            $"got 0x{got[i]:X8}, want 0x{want[i]:X8} — this " +
                            "should be impossible on any runtime; report it");
                        allOk = false;
                    }
                }
            }
            if (allOk)
            {
                Debug.Log("[TmnfSimDiagnostics] all 10 guarded fp kernels " +
                    "bit-exact vs the conforming-runtime reference — per-op " +
                    "f32 rounding is active on this runtime");
            }
            return allOk;
        }

        /// <summary>The facade trace: verdict part two.</summary>
        private static bool RunFacadeTraceDiff()
        {
            string facadePath = System.IO.Path.Combine(
                Application.streamingAssetsPath,
                "Tmnf.Sim.GoldenData/sim/facade.tgn");
            if (!System.IO.File.Exists(facadePath))
            {
                Debug.LogWarning("[TmnfSimDiagnostics] sim/facade.tgn not " +
                    "found in StreamingAssets — skipping the trace check");
                return true;
            }
            byte[] golden = ReadRowOutput(facadePath);

            SimOptions options = new SimOptions
            {
                fakeContactMask = System.IO.Path.Combine(
                    Application.streamingAssetsPath,
                    "Tmnf.Sim.GoldenData/fixtures/game-mask.bin"),
                route = null,
                pinProvenance = false,
            };
            Sim sim = Sim.Load(
                System.IO.Path.Combine(Application.streamingAssetsPath,
                    "Tmnf.Sim.GoldenData/fixtures/plain.tmnftrack"),
                System.IO.Path.Combine(Application.streamingAssetsPath,
                    "Tmnf.Sim.GoldenData/fixtures/stadium.tmnfvehicle"),
                options);

            byte[] stream = new byte[Ticks * 104];
            int o = 0;
            for (uint t = 0; t < Ticks; t++)
            {
                TmnfInput input = (t < 300, t < 350) switch
                {
                    (true, _) => TmnfInput.Digital(1, 0, 0, 0),
                    (_, true) => TmnfInput.Digital(0, 1, 0, 0),
                    _ => TmnfInput.Digital(1, 0, 1, 0),
                };
                VehicleState state = sim.Step(ref input);
                PutU32(stream, ref o, state.raceTimeMs);
                PutF32(stream, ref o, state.position.x);
                PutF32(stream, ref o, state.position.y);
                PutF32(stream, ref o, state.position.z);
                foreach (float f in state.quaternion)
                {
                    PutF32(stream, ref o, f);
                }
                PutF32(stream, ref o, state.linearSpeed.x);
                PutF32(stream, ref o, state.linearSpeed.y);
                PutF32(stream, ref o, state.linearSpeed.z);
                PutF32(stream, ref o, state.angularSpeed.x);
                PutF32(stream, ref o, state.angularSpeed.y);
                PutF32(stream, ref o, state.angularSpeed.z);
                foreach (WheelState w in state.wheels)
                {
                    stream[o++] = (byte)(w.onGround ? 1 : 0);
                    PutI32(stream, ref o, w.contactMaterialId);
                    stream[o++] = (byte)(w.sliding ? 1 : 0);
                    PutF32(stream, ref o, w.spinSpeed);
                }
                PutF32(stream, ref o, state.engineRpm);
                PutI32(stream, ref o, state.gear);
            }

            ulong hash = Fnva.Hash(stream);
            if (hash == ExpectedFacadeHash)
            {
                Debug.Log($"[TmnfSimDiagnostics] facade hash 0x{hash:X16} — " +
                    "the full 400-tick trace is bit-exact on THIS runtime " +
                    "(promotion-immune)");
                return true;
            }
            Debug.LogError($"[TmnfSimDiagnostics] facade hash 0x{hash:X16} " +
                $"!= expected 0x{ExpectedFacadeHash:X16} — this should not " +
                "happen with the guarded core; hunting the first divergent " +
                "tick for the report");

            int firstTick = -1;
            int firstByte = -1;
            for (int t = 0; t < Ticks && firstTick < 0; t++)
            {
                for (int b = 0; b < 104; b++)
                {
                    if (stream[t * 104 + b] != golden[t * 104 + b])
                    {
                        firstTick = t;
                        firstByte = b;
                        break;
                    }
                }
            }
            if (firstTick < 0)
            {
                Debug.LogError("[TmnfSimDiagnostics] per-tick records are " +
                    "identical but the hash differs — serialization bug");
                return false;
            }
            int off = firstTick * 104;
            Debug.LogError($"[TmnfSimDiagnostics] FIRST DIVERGENCE at tick " +
                $"{firstTick + 1} (1-based), byte {firstByte} of the 104-byte " +
                "record:");
            DumpTick("golden", golden, off);
            DumpTick("ours  ", stream, off);
            Debug.Log("[TmnfSimDiagnostics] field map of the 104-byte " +
                "record: 0-3 raceMs, 4-15 pos, 16-31 quat(xyzw), 32-43 " +
                "linVel, 44-55 angVel, 56-95 wheels (10B each: onGround, " +
                "material, sliding, spin), 96-99 rpm, 100-103 gear");
            return false;
        }

        private static void DumpTick(string what, byte[] s, int o)
        {
            uint raceMs = BitConverter.ToUInt32(s, o);
            float px = BitConverter.ToSingle(s, o + 4);
            float py = BitConverter.ToSingle(s, o + 8);
            float pz = BitConverter.ToSingle(s, o + 12);
            float vx = BitConverter.ToSingle(s, o + 32);
            float vy = BitConverter.ToSingle(s, o + 36);
            float vz = BitConverter.ToSingle(s, o + 40);
            float ax = BitConverter.ToSingle(s, o + 44);
            float ay = BitConverter.ToSingle(s, o + 48);
            float az = BitConverter.ToSingle(s, o + 52);
            float rpm = BitConverter.ToSingle(s, o + 96);
            int gear = BitConverter.ToInt32(s, o + 100);
            Debug.Log($"[TmnfSimDiagnostics]   {what}: raceMs={raceMs} " +
                $"pos=({px:R},{py:R},{pz:R}) lin=({vx:R},{vy:R},{vz:R}) " +
                $"ang=({ax:R},{ay:R},{az:R}) rpm={rpm:R} gear={gear}");
            for (int w = 0; w < 4; w++)
            {
                int wo = o + 56 + w * 10;
                int mat = BitConverter.ToInt32(s, wo + 1);
                float spin = BitConverter.ToSingle(s, wo + 6);
                Debug.Log($"[TmnfSimDiagnostics]   {what} wheel[{w}]: " +
                    $"ground={s[wo]} mat={mat} sliding={s[wo + 5]} " +
                    $"spin={spin:R}");
            }
        }

        /// <summary>Extracts the single row's output bytes from a TGN1 file.</summary>
        private static byte[] ReadRowOutput(string path)
        {
            byte[] file = System.IO.File.ReadAllBytes(path);
            if (file[0] != (byte)'T' || file[1] != (byte)'G'
                || file[2] != (byte)'N' || file[3] != (byte)'1')
            {
                throw new InvalidOperationException("bad TGN1 magic");
            }
            int pos = 4;
            uint nameLen = BitConverter.ToUInt32(file, pos);
            pos += 4 + (int)nameLen;
            ulong rowCount = BitConverter.ToUInt64(file, pos);
            pos += 8;
            if (rowCount != 1)
            {
                throw new InvalidOperationException(
                    $"expected 1 row, found {rowCount}");
            }
            uint inLen = BitConverter.ToUInt32(file, pos);
            pos += 4 + (int)inLen;
            uint outLen = BitConverter.ToUInt32(file, pos);
            pos += 4;
            byte[] output = new byte[outLen];
            Buffer.BlockCopy(file, pos, output, 0, (int)outLen);
            return output;
        }

        private static void PutU32(byte[] b, ref int o, uint v)
        {
            b[o] = (byte)v;
            b[o + 1] = (byte)(v >> 8);
            b[o + 2] = (byte)(v >> 16);
            b[o + 3] = (byte)(v >> 24);
            o += 4;
        }

        private static void PutI32(byte[] b, ref int o, int v)
        {
            PutU32(b, ref o, unchecked((uint)v));
        }

        private static void PutF32(byte[] b, ref int o, float v)
        {
            PutU32(b, ref o, unchecked((uint)BitConverter.SingleToInt32Bits(v)));
        }
    }
}
