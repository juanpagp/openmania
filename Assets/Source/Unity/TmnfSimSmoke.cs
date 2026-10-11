using System;
using UnityEngine;

namespace Tmnf.Sim.Unity
{
    /// <summary>
    /// The smoke scene's verifier: replays the golden drive schedule
    /// (400 ticks, accelerate → brake → accelerate+steer-left) through the
    /// Sim facade INDEPENDENTLY of frame timing, serializes the telemetry
    /// stream in the facade-suite byte layout, and compares its FNV-1a hash
    /// against the golden constant baked in from the checked-in data.
    ///
    /// A PASS here proves the whole stack end-to-end inside Unity — loaders,
    /// SHA-256 track hashing, decode, world construction, the full force
    /// pipeline, the race layer, and the facade observation — with zero
    /// tolerance: one flipped mantissa bit anywhere fails the scene.
    ///
    /// Run: open Assets/Scenes/TmnfSimSmoke.unity and press Play. The
    /// Console shows the tick-by-tick progress line and the final verdict.
    /// </summary>
    public sealed class TmnfSimSmoke : MonoBehaviour
    {
        /// <summary>
        /// FNV-1a 64 over the 400-tick facade telemetry stream, baked from
        /// Assets/StreamingAssets/Tmnf.Sim.GoldenData/sim/facade.tgn
        /// (regenerate with scripts/facade_hash.py when the golden data is
        /// regenerated).
        /// </summary>
        private const ulong ExpectedFacadeHash = 0x5A45_761F_3728_DF35;

        private const int Ticks = 400;

        private void Start()
        {
            string streaming = Application.streamingAssetsPath;
            Debug.Log($"[TmnfSimSmoke] fixtures from {streaming}");

            SimOptions options = new SimOptions
            {
                fakeContactMask = System.IO.Path.Combine(
                    streaming, "Tmnf.Sim.GoldenData/fixtures/game-mask.bin"),
                route = null,
                pinProvenance = false,
            };
            Sim sim = Sim.Load(
                System.IO.Path.Combine(
                    streaming, "Tmnf.Sim.GoldenData/fixtures/plain.tmnftrack"),
                System.IO.Path.Combine(
                    streaming, "Tmnf.Sim.GoldenData/fixtures/stadium.tmnfvehicle"),
                options);

            // The facade-suite telemetry stream: per tick
            // u32 raceTimeMs, vec3 pos, quat(x,y,z,w), vec3 linVel,
            // vec3 angVel, 4 wheels (u8 onGround, i32 material, u8 sliding,
            // f32 spin), f32 rpm, i32 gear — 104 bytes.
            byte[] stream = new byte[Ticks * 104];
            int o = 0;
            for (uint t = 0; t < Ticks; t++)
            {
                // The drive schedule: t<300 accelerate; 300..350 brake;
                // t>=350 accelerate + steer LEFT.
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

                if (t == Ticks - 1)
                {
                    Debug.Log($"[TmnfSimSmoke] final tick {t + 1}: " +
                        $"pos=({state.position.x:G9}, {state.position.y:G9}, {state.position.z:G9}) " +
                        $"speed={state.linearSpeed.z:G9} m/s gear={state.gear} rpm={state.engineRpm:G9}");
                }
            }

            ulong hash = Fnva.Hash(stream);
            if (hash == ExpectedFacadeHash)
            {
                Debug.Log($"[TmnfSimSmoke] SMOKE PASS — facade hash 0x{hash:X16} " +
                          $"matches the golden data ({Ticks} ticks, bit-exact). " +
                          "The per-op f32 guards make this exact on every " +
                          "runtime, including Unity's promoting Mono (see " +
                          "Assets/Docs/FLOAT_PROMOTION.md).");
            }
            else
            {
                Debug.LogError($"[TmnfSimSmoke] SMOKE FAIL — facade hash " +
                               $"0x{hash:X16} != expected 0x{ExpectedFacadeHash:X16}. " +
                               "This should not happen on any runtime — the core " +
                               "is promotion-immune. Run the TmnfSimDiagnostics " +
                               "scene and paste its full console output in the " +
                               "report.");
            }
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
