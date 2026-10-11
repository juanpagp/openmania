using System;
using System.Collections.Generic;
using UnityEngine;

namespace Tmnf.Sim.Unity
{
    /// <summary>
    /// The whole pipeline with ZERO asset files. Everything the simulation
    /// consumes — the TMNFTRK1 track, the TMNFM6G1 Stadium vehicle, the
    /// TMNFROU1 route and the 128x128 fake-contact mask — is built in
    /// memory by TmnfFixtures from pure data, fed through the same
    /// loaders and validators the file lane uses, and driven through the
    /// Sim facade with the race layer active.
    ///
    /// The example runs two self-verifying passes:
    ///   1. The golden 400-tick facade drive schedule — the FNV-1a hash of
    ///      the telemetry stream must equal the golden constant, proving
    ///      the pure-data images are bit-identical to the golden fixtures
    ///      (and the whole stack is exact on this runtime).
    ///   2. A full lap: straight, left around the crash wall, right to
    ///      straighten, then full throttle to the finish at z=200 — the
    ///      race layer records the finish time.
    ///
    /// Run: open Assets/Scenes/TmnfSimPureData.unity and press Play.
    /// Use this as the starting point for your own scenes: build (or load)
    /// the images, construct the Sim, step it from FixedUpdate at 100 Hz
    /// (or use TmnfSim for the frame-time-decoupled driver), and read
    /// VehicleState every tick.
    /// </summary>
    public sealed class TmnfSimPureData : MonoBehaviour
    {
        private const int FacadeTicks = 400;
        private const ulong ExpectedFacadeHash = 0x5A45_761F_3728_DF35UL;

        private void Start()
        {
            Debug.Log("[TmnfSimPureData] building all fixtures from pure " +
                "data (no files)…");
            byte[] trackBytes = Fixtures.TmnfFixtures.PlainTrack();
            byte[] vehicleBytes = Fixtures.TmnfFixtures.StadiumVehicle();
            byte[] routeBytes = Fixtures.TmnfFixtures.Route();
            byte[] maskBytes = Fixtures.TmnfFixtures.GameMask();
            Debug.Log($"[TmnfSimPureData] built — track {trackBytes.Length} B, " +
                $"vehicle {vehicleBytes.Length} B, route {routeBytes.Length} B, " +
                $"mask {maskBytes.Length} B");

            RunGoldenFacadeCheck(trackBytes, vehicleBytes, maskBytes);
            RunFullLap(trackBytes, vehicleBytes, routeBytes, maskBytes);
            Debug.Log("[TmnfSimPureData] done — the whole pipeline ran from " +
                "pure data. See TmnfSim.cs for the frame-decoupled driver " +
                "and TmnfCoordinates.cs for the handedness conversion.");
        }

        /// <summary>
        /// Pass 1: the golden facade schedule from the in-memory images.
        /// </summary>
        private static void RunGoldenFacadeCheck(
            byte[] trackBytes, byte[] vehicleBytes, byte[] maskBytes)
        {
            TrackLoader.TmnfTrack track =
                TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            Sim sim = new Sim(track, vehicleBytes, maskBytes, new SimOptions
            {
                fakeContactMask = null,
                route = null,
                pinProvenance = false,
            });

            var stream = new List<byte>(FacadeTicks * 104);
            for (uint t = 0; t < FacadeTicks; t++)
            {
                TmnfInput input = (t < 300, t < 350) switch
                {
                    (true, _) => TmnfInput.Digital(1, 0, 0, 0),
                    (_, true) => TmnfInput.Digital(0, 1, 0, 0),
                    _ => TmnfInput.Digital(1, 0, 1, 0),
                };
                VehicleState state = sim.Step(ref input);
                ByteWriter.U32(stream, state.raceTimeMs);
                ByteWriter.F32(stream, state.position.x);
                ByteWriter.F32(stream, state.position.y);
                ByteWriter.F32(stream, state.position.z);
                foreach (float f in state.quaternion)
                {
                    ByteWriter.F32(stream, f);
                }
                ByteWriter.F32(stream, state.linearSpeed.x);
                ByteWriter.F32(stream, state.linearSpeed.y);
                ByteWriter.F32(stream, state.linearSpeed.z);
                ByteWriter.F32(stream, state.angularSpeed.x);
                ByteWriter.F32(stream, state.angularSpeed.y);
                ByteWriter.F32(stream, state.angularSpeed.z);
                foreach (WheelState w in state.wheels)
                {
                    ByteWriter.U8(stream, (byte)(w.onGround ? 1 : 0));
                    ByteWriter.I32(stream, w.contactMaterialId);
                    ByteWriter.U8(stream, (byte)(w.sliding ? 1 : 0));
                    ByteWriter.F32(stream, w.spinSpeed);
                }
                ByteWriter.F32(stream, state.engineRpm);
                ByteWriter.I32(stream, state.gear);
                if (t % 100 == 99)
                {
                    Debug.Log($"[TmnfSimPureData] facade tick {t + 1}: " +
                        $"pos=({state.position.x:R}, {state.position.y:R}, " +
                        $"{state.position.z:R}) speed={state.linearSpeed.z:R} " +
                        $"m/s gear={state.gear} rpm={state.engineRpm:R}");
                }
            }
            ulong hash = Fnva.Hash(stream.ToArray());
            if (hash == ExpectedFacadeHash)
            {
                Debug.Log($"[TmnfSimPureData] facade check PASS — hash " +
                    $"0x{hash:X16} matches the golden trace " +
                    $"({FacadeTicks} ticks, bit-exact, zero files)." +
                    " This runtime reproduces the golden trace — the " +
                    "per-op f32 guards make the pipeline immune to f32->f64 " +
                    "tree promotion (see Assets/Docs/FLOAT_PROMOTION.md).");
            }
            else
            {
                Debug.LogError($"[TmnfSimPureData] facade check FAIL — hash " +
                    $"0x{hash:X16} != expected 0x{ExpectedFacadeHash:X16}. " +
                    "This should not happen on any runtime — the core is " +
                    "promotion-immune. Please run the TmnfSimDiagnostics " +
                    "scene and paste its full console output in the report.");
            }
        }

        /// <summary>
        /// Pass 2: a full lap with the race layer bound from pure data —
        /// straight, left around the crash wall, right to straighten,
        /// throttle to the finish.
        /// </summary>
        private static void RunFullLap(
            byte[] trackBytes, byte[] vehicleBytes, byte[] routeBytes,
            byte[] maskBytes)
        {
            TrackLoader.TmnfTrack track =
                TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            Sim sim = new Sim(track, vehicleBytes, maskBytes, new SimOptions
            {
                fakeContactMask = null,
                routeBytes = routeBytes,
                pinProvenance = false,
            });

            int finishAt = -1;
            for (uint t = 0; t < 3000 && finishAt < 0; t++)
            {
                TmnfInput input = t switch
                {
                    var tt when tt < 350 => TmnfInput.Digital(1, 0, 0, 0),
                    var tt when tt < 450 => TmnfInput.Digital(1, 0, 1, 0),
                    var tt when tt < 600 => TmnfInput.Digital(1, 0, 0, 1),
                    _ => TmnfInput.Digital(1, 0, 0, 0),
                };
                try
                {
                    sim.Step(ref input);
                }
                catch (InvalidOperationException)
                {
                    // The ported upstream quirk: the facade's State()
                    // observes canRespawn, which panics on finished races —
                    // the panic itself marks the finishing tick.
                    finishAt = (int)t;
                    break;
                }
                Race.TmnfRaceState? race = sim.Race();
                if (race.HasValue && race.Value.finished != 0)
                {
                    finishAt = (int)t;
                }
            }
            if (finishAt >= 0)
            {
                Race.TmnfRaceState race = sim.Race().Value;
                Debug.Log($"[TmnfSimPureData] lap finished at tick " +
                    $"{finishAt} — finish time {race.finishTimeMs} ms, " +
                    $"checkpoints {race.visitedCount}, laps " +
                    $"{race.completedLaps}. (The upstream facade panics when " +
                    "observing a finished race — read sim.Race() instead of " +
                    "stepping further; see Assets/Docs/REPORT.md.)");
            }
            else
            {
                Debug.LogError("[TmnfSimPureData] the lap never finished — " +
                    "the schedule or the pure-data route diverged");
            }
        }
    }
}
