using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Tmnf.Sim;
using Tmnf.Sim.Fixtures;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// The pure-data fixtures: everything the simulation consumes, built
    /// entirely in memory. Each builder must reproduce the checked-in
    /// golden fixture byte-for-byte, and the 400-tick facade drive
    /// schedule replayed from the in-memory images must hash to the same
    /// golden constant the smoke scene checks — proving the whole
    /// pipeline runs identically with zero files on disk.
    /// </summary>
    [TestFixture]
    public class FixturesTests
    {
        private static void AssertSameBytes(byte[] built, byte[] expected, string what)
        {
            Assert.That(built.Length, Is.EqualTo(expected.Length),
                $"{what}: built {built.Length} bytes, expected {expected.Length}");
            for (int i = 0; i < expected.Length; i++)
            {
                if (built[i] != expected[i])
                {
                    Assert.Fail($"{what}: first divergence at byte {i}: " +
                        $"expected 0x{expected[i]:X2}, built 0x{built[i]:X2}");
                }
            }
        }

        [Test]
        public void PlainTrackMatchesGolden()
        {
            AssertSameBytes(TmnfFixtures.PlainTrack(),
                File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/plain.tmnftrack")),
                "plain.tmnftrack");
        }

        [Test]
        public void PoolTrackMatchesGolden()
        {
            AssertSameBytes(TmnfFixtures.PoolTrack(),
                File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/pool.tmnftrack")),
                "pool.tmnftrack");
        }

        [Test]
        public void RouteMatchesGolden()
        {
            AssertSameBytes(TmnfFixtures.Route(),
                File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/loop.tmnfroute")),
                "loop.tmnfroute");
        }

        [Test]
        public void StadiumVehicleMatchesGolden()
        {
            AssertSameBytes(TmnfFixtures.StadiumVehicle(),
                File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle")),
                "stadium.tmnfvehicle");
        }

        [Test]
        public void GameMaskMatchesGolden()
        {
            AssertSameBytes(TmnfFixtures.GameMask(),
                File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/game-mask.bin")),
                "game-mask.bin");
        }

        /// <summary>
        /// The smoke scene's exact protocol, run from pure data: the hash of
        /// the 400-tick facade telemetry stream must equal the golden
        /// constant baked into TmnfSimSmoke (0x5A45761F3728DF35, the FNV-1a
        /// of sim/facade.tgn's row output).
        /// </summary>
        [Test]
        public void FacadeHashFromPureData()
        {
            const int ticks = 400;
            const ulong expectedHash = 0x5A45_761F_3728_DF35UL;

            TrackLoader.TmnfTrack track =
                TrackLoader.TmnfTrackLoadBytes(TmnfFixtures.PlainTrack(), null);
            Sim sim = new Sim(track, TmnfFixtures.StadiumVehicle(),
                TmnfFixtures.GameMask(), new SimOptions
                {
                    fakeContactMask = null,
                    route = null,
                    pinProvenance = false,
                });

            var stream = new List<byte>(ticks * 104);
            for (uint t = 0; t < ticks; t++)
            {
                TmnfInput input = (t < 300, t < 350) switch
                {
                    (true, _) => TmnfInput.Digital(1, 0, 0, 0),
                    (_, true) => TmnfInput.Digital(0, 1, 0, 0),
                    _ => TmnfInput.Digital(1, 0, 1, 0),
                };
                VehicleState state = sim.Step(ref input);
                ByteWriter.U32(stream, state.raceTimeMs);
                GmGoldenTests.WriteVec3To(stream, state.position);
                foreach (float f in state.quaternion)
                {
                    ByteWriter.F32(stream, f);
                }
                GmGoldenTests.WriteVec3To(stream, state.linearSpeed);
                GmGoldenTests.WriteVec3To(stream, state.angularSpeed);
                foreach (WheelState w in state.wheels)
                {
                    ByteWriter.U8(stream, (byte)(w.onGround ? 1 : 0));
                    ByteWriter.I32(stream, w.contactMaterialId);
                    ByteWriter.U8(stream, (byte)(w.sliding ? 1 : 0));
                    ByteWriter.F32(stream, w.spinSpeed);
                }
                ByteWriter.F32(stream, state.engineRpm);
                ByteWriter.I32(stream, state.gear);
            }

            ulong hash = Fnva.Hash(stream.ToArray());
            Assert.That(hash, Is.EqualTo(expectedHash),
                "pure-data facade hash diverged from the golden trace");
        }

        /// <summary>
        /// The full race pipeline from pure data: route bound from bytes, one
        /// lap driven around the crash wall (straight, left, right, straight
        /// — the wall-avoiding schedule found by the fixture-debug harness),
        /// finish recorded at tick 793 (7930 ms).
        /// </summary>
        [Test]
        public void RaceFinishesFromPureData()
        {
            TrackLoader.TmnfTrack track =
                TrackLoader.TmnfTrackLoadBytes(TmnfFixtures.PlainTrack(), null);
            Sim sim = new Sim(track, TmnfFixtures.StadiumVehicle(),
                TmnfFixtures.GameMask(), new SimOptions
                {
                    fakeContactMask = null,
                    routeBytes = TmnfFixtures.Route(),
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
                    // so the panic itself marks the finishing tick.
                    finishAt = (int)t;
                    break;
                }
                Race.TmnfRaceState? race = sim.Race();
                if (race.HasValue && race.Value.finished != 0)
                {
                    finishAt = (int)t;
                }
            }
            Assert.That(finishAt, Is.EqualTo(793),
                "the pure-data race should finish at tick 793");
            Race.TmnfRaceState final = sim.Race().Value;
            Assert.That(final.finished, Is.EqualTo(1u));
            Assert.That(final.finishTimeMs, Is.EqualTo(7940u));
        }
    }
}
