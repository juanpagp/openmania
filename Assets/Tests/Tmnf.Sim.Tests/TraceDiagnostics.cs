using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// Divergence hunter for the whole-sim traces: replays a trace and walks
    /// the expected/actual record streams in lockstep, naming the first
    /// diverging tick, telemetry field and (with a dump-all expected file)
    /// the exact state field. Driven by env vars, never part of the gate:
    ///   TMNF_TRACE       suite short name (drive, burnout, pool, ...)
    ///   TMNF_DUMPALL_DIR directory of a TMNF_GOLDEN_DUMP_ALL=1 generation
    /// </summary>
    [TestFixture]
    public class TraceDiagnostics
    {
        private const int DumpSize = 180 + 0x878 + 4 * 0x2fc + 8 + 4 + 136;
        private const int TelemetrySize = 105;

        [Test]
        public void Diagnose()
        {
            string suite = Environment.GetEnvironmentVariable("TMNF_TRACE") ?? "drive";
            string dumpAllDir = Environment.GetEnvironmentVariable("TMNF_DUMPALL_DIR");
            string trackFile = suite == "pool" ? "pool.tmnftrack" : "plain.tmnftrack";

            string expectedPath = dumpAllDir != null
                ? Path.Combine(dumpAllDir, "sim", $"trace_{suite}.tgn")
                : GoldenRunner.GoldenPath($"sim/trace_{suite}.tgn");
            Assert.That(File.Exists(expectedPath), Is.True, $"missing {expectedPath}");

            List<GoldenRunner.Row> rows = GoldenRunner.ReadRows($"sim/trace_{suite}.tgn");
            byte[] input = rows[0].Input;
            // The expected output: the dump-all generation when provided
            // (same inputs — the traces are deterministic).
            byte[] expected = ReadRowOutput(expectedPath, 0);

            // Replay with dumps on EVERY tick so the state at the divergence
            // tick can be diffed when the expected side has dumps too.
            byte[] actual = RunTraceDumpAll(input, trackFile);

            Walk(suite, expected, actual, dumpAllDir != null);
        }

        /// <summary>
        /// Mirrors tools/tmnf-golden/src/bin/diag_drive.rs: replays a
        /// schedule (TMNF_DIAG_SCHED: drive|burnout|respawn) for
        /// TMNF_DIAG_TICKS ticks with the TMNF_DBG instrumentation active.
        /// Compare its stderr with the Rust bin's line by line.
        /// </summary>
        [Test]
        public void DiagDrive()
        {
            int ticks = int.Parse(
                Environment.GetEnvironmentVariable("TMNF_DIAG_TICKS") ?? "420");
            string sched = Environment.GetEnvironmentVariable("TMNF_DIAG_SCHED") ?? "drive";
            Environment.SetEnvironmentVariable("TMNF_DBG", "1");
            bool withRoute = sched != "burnout";

            string tmp = Path.Combine(Path.GetTempPath(), "tmnf-sim-tests");
            Directory.CreateDirectory(tmp);
            string trackPath = Path.Combine(tmp, "plain.tmnftrack");
            if (!File.Exists(trackPath))
            {
                File.WriteAllBytes(trackPath,
                    File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/plain.tmnftrack")));
            }
            string routePath = Path.Combine(tmp, "loop.tmnfroute");
            if (!File.Exists(routePath))
            {
                File.WriteAllBytes(routePath,
                    File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/loop.tmnfroute")));
            }
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob = File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
            byte[] mask = File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/game-mask.bin"));
            World world = new World(track, vehicleBlob, new WorldOptions
            {
                fakeContactMask = mask,
                pinProvenance = false,
            });
            RouteLoader.TmnfRoute route = withRoute ? RouteLoader.TmnfRouteLoad(routePath, null) : null;
            world.route = route;
            Race.TmnfRaceState race = default(Race.TmnfRaceState);
            if (route != null)
            {
                GmIso4 iso = world.Live().RotPosIso();
                Race.TmnfRaceReset(route, ref race, ref iso);
            }
            for (uint t = 0; t < (uint)ticks; t++)
            {
                uint now = unchecked(t * 10);
                TMNFRaceInputs input = default(TMNFRaceInputs);
                switch (sched)
                {
                    case "burnout":
                    {
                        int a = t < 200 ? 1 : t < 400 ? 1 : t < 500 ? 0 : 1;
                        int b = t < 200 ? 1 : t < 400 ? 0 : t < 500 ? 1 : 0;
                        input.accelerateTime = a != 0 ? now : 0;
                        input.accelerate = a;
                        input.brakeTime = b != 0 ? now : 0;
                        input.brake = b;
                        break;
                    }
                    case "respawn":
                    {
                        input.respawn = (t == 400 || t == 401) ? 1u : 0u;
                        int b = t >= 500 && t < 560 ? 1 : 0;
                        input.accelerateTime = now;
                        input.accelerate = 1;
                        input.brakeTime = b != 0 ? now : 0;
                        input.brake = b;
                        break;
                    }
                    default:
                    {
                        // drive_packet (the generator's schedule)
                        int a = t < 300 ? 1 : t < 350 ? 0 : t < 500 ? 1 : t < 600 ? 1 : t < 700 ? 0 : 1;
                        int b = t >= 300 && t < 350 ? 1 : 0;
                        int l = t >= 500 && t < 600 ? 0 : t >= 350 && t < 500 ? 1 : 0;
                        int r = t >= 500 && t < 600 ? 1 : 0;
                        input.steerLeftTime = l != 0 ? now : 0;
                        input.steerLeft = l;
                        input.steerRightTime = r != 0 ? now : 0;
                        input.steerRight = r;
                        input.accelerateTime = a != 0 ? now : 0;
                        input.accelerate = a;
                        input.brakeTime = b != 0 ? now : 0;
                        input.brake = b;
                        break;
                    }
                }
                if (input.respawn != 0)
                {
                    GmIso4? spawn = Race.TmnfRaceRespawnLocation(ref race);
                    if (spawn.HasValue)
                    {
                        GmIso4 sp = spawn.Value;
                        world.Respawn(ref sp);
                    }
                }
                world.ApplyInputs(ref input);
                world.AdvanceTimer(10);
                world.PhysicsStep2(10);
                if (route != null && race.finished == 0)
                {
                    GmIso4 iso = world.Live().RotPosIso();
                    ulong contacts = world.triggerContacts;
                    Race.TmnfRaceStep(route, ref race, contacts, ref iso);
                }
            }
        }

        /// <summary>Reads row `index`'s output bytes from a .tgn file.</summary>
        private static byte[] ReadRowOutput(string path, int index)
        {
            byte[] file = File.ReadAllBytes(path);
            int pos = 4;
            uint nameLen = BitsUtil.GetU32(file, pos); pos += 4 + (int)nameLen;
            ulong rowCount = BitsUtil.GetU64(file, pos); pos += 8;
            Assert.That((ulong)index, Is.LessThan(rowCount));
            for (ulong i = 0; i <= (ulong)index; i++)
            {
                uint inLen = BitsUtil.GetU32(file, pos); pos += 4 + (int)inLen;
                uint outLen = BitsUtil.GetU32(file, pos); pos += 4;
                if (i == (ulong)index)
                {
                    byte[] o = new byte[outLen];
                    Buffer.BlockCopy(file, pos, o, 0, (int)outLen);
                    return o;
                }
                pos += (int)outLen;
            }
            throw new InvalidOperationException("row not found");
        }

        // ------------------------------------------------------------------
        // record walking
        // ------------------------------------------------------------------

        private struct Rec
        {
            public ulong Hash;
            public byte[] Telemetry;
            public bool HasDump;
            public byte[] Dump;
            public bool End;
        }

        private static Rec ReadRec(byte[] b, ref int pos)
        {
            if (pos + 8 + TelemetrySize + 1 > b.Length)
            {
                return new Rec { End = true };
            }
            Rec r = new Rec();
            r.Hash = BitsUtil.GetU64(b, pos); pos += 8;
            r.Telemetry = new byte[TelemetrySize];
            Buffer.BlockCopy(b, pos, r.Telemetry, 0, TelemetrySize); pos += TelemetrySize;
            r.HasDump = b[pos++] != 0;
            if (r.HasDump)
            {
                r.Dump = new byte[DumpSize];
                Buffer.BlockCopy(b, pos, r.Dump, 0, DumpSize); pos += DumpSize;
            }
            return r;
        }

        private void Walk(string suite, byte[] expected, byte[] actual, bool bothDump)
        {
            int ep = 0, ap = 0;
            int recIndex = 0;
            while (true)
            {
                Rec e = ReadRec(expected, ref ep);
                Rec a = ReadRec(actual, ref ap);
                if (e.End || a.End)
                {
                    break;
                }
                if (e.Hash != a.Hash)
                {
                    ReportDivergence(suite, recIndex, e, a, bothDump);
                    return;
                }
                // hash equal but telemetry could still differ? hash covers the
                // dump inputs, telemetry is derived — compare anyway when
                // hashes match (cheap safety net).
                for (int i = 0; i < TelemetrySize; i++)
                {
                    if (e.Telemetry[i] != a.Telemetry[i])
                    {
                        ReportDivergence(suite, recIndex, e, a, bothDump);
                        return;
                    }
                }
                recIndex++;
            }
            // tail: panic markers
            bool ePan = ep < expected.Length && expected[ep] != 0;
            bool aPan = ap < actual.Length && actual[ap] != 0;
            if (ePan && aPan)
            {
                uint eTick = BitsUtil.GetU32(expected, ep + 1);
                uint aTick = BitsUtil.GetU32(actual, ap + 1);
                Assert.Fail(
                    $"both panic but at different ticks: expected {eTick}, actual {aTick} " +
                    $"(records compared: {recIndex})");
            }
            else if (ePan != aPan)
            {
                Assert.Fail($"panic mismatch: expected={(ePan ? BitsUtil.GetU32(expected, ep + 1) : -1)} " +
                            $"actual={(aPan ? BitsUtil.GetU32(actual, ap + 1) : -1)} (records: {recIndex})");
            }
            Assert.That(recIndex, Is.GreaterThan(0), "no records compared");
            Assert.Pass($"trace {suite}: {recIndex} records all match");
        }

        private void ReportDivergence(string suite, int recIndex, Rec e, Rec a, bool bothDump)
        {
            // record 0 = post-construction; record k = after tick k-1.
            int tick = recIndex - 1;
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"trace {suite}: FIRST DIVERGENCE at record {recIndex} (tick {tick})");
            sb.AppendLine($"  hash expected 0x{e.Hash:X16}, actual 0x{a.Hash:X16}");
            sb.Append(TelemetryDiff(e.Telemetry, a.Telemetry));
            if (bothDump && e.HasDump && a.HasDump)
            {
                sb.Append(DumpDiff(e.Dump, a.Dump));
            }
            else
            {
                sb.AppendLine("  (no dump-all expected available for the field diff — set TMNF_DUMPALL_DIR)");
            }
            Assert.Fail(sb.ToString());
        }

        // ------------------------------------------------------------------
        // field maps
        // ------------------------------------------------------------------

        private static readonly (int, string)[] TelemetryMap =
        {
            (0, "elapsedMs"), (4, "pos.x"), (8, "pos.y"), (12, "pos.z"),
            (16, "quat.y"), (20, "quat.z"), (24, "quat.w"), (28, "quat.x"),
            (32, "linVel.x"), (36, "linVel.y"), (40, "linVel.z"),
            (44, "angVel.x"), (48, "angVel.y"), (52, "angVel.z"),
            (56, "w0.onGround"), (57, "w0.material"), (61, "w0.sliding"), (62, "w0.spin"),
            (66, "w1.onGround"), (67, "w1.material"), (71, "w1.sliding"), (72, "w1.spin"),
            (76, "w2.onGround"), (77, "w2.material"), (81, "w2.sliding"), (82, "w2.spin"),
            (86, "w3.onGround"), (87, "w3.material"), (91, "w3.sliding"), (92, "w3.spin"),
            (96, "engineRpm"), (100, "gear"), (104, "respawnAvailable"),
        };

        private static string TelemetryDiff(byte[] e, byte[] a)
        {
            var sb = new System.Text.StringBuilder();
            foreach ((int off, string name) in TelemetryMap)
            {
                int w = off & ~3;
                if (w + 4 <= e.Length && w + 4 <= a.Length)
                {
                    uint ev = BitsUtil.GetU32(e, w);
                    uint av = BitsUtil.GetU32(a, w);
                    if (ev != av)
                    {
                        sb.AppendLine($"  TELEMETRY {name}: expected 0x{ev:X8} ({FmtF32(ev)}), " +
                                      $"actual 0x{av:X8} ({FmtF32(av)})");
                    }
                }
            }
            return sb.ToString();
        }

        private static string FmtF32(uint bits)
        {
            float f = BitConverter.Int32BitsToSingle(unchecked((int)bits));
            return float.IsFinite(f) ? f.ToString("G9") : f.ToString();
        }

        private static readonly (int, string)[] DynaMap =
        {
            (0x00, "quat.x"), (0x04, "quat.y"), (0x08, "quat.z"), (0x0c, "quat.w"),
            (0x10, "rot.m0"), (0x14, "rot.m1"), (0x18, "rot.m2"),
            (0x1c, "rot.m3"), (0x20, "rot.m4"), (0x24, "rot.m5"),
            (0x28, "rot.m6"), (0x2c, "rot.m7"), (0x30, "rot.m8"),
            (0x34, "pos.x"), (0x38, "pos.y"), (0x3c, "pos.z"),
            (0x40, "linVel.x"), (0x44, "linVel.y"), (0x48, "linVel.z"),
            (0x4c, "linVelAdded.x"), (0x50, "linVelAdded.y"), (0x54, "linVelAdded.z"),
            (0x58, "angVel.x"), (0x5c, "angVel.y"), (0x60, "angVel.z"),
            (0x64, "force.x"), (0x68, "force.y"), (0x6c, "force.z"),
            (0x70, "torque.x"), (0x74, "torque.y"), (0x78, "torque.z"),
            (0x7c, "invInertiaWorld.m0"), (0x80, "invInertiaWorld.m1"), (0x84, "invInertiaWorld.m2"),
            (0x88, "invInertiaWorld.m3"), (0x8c, "invInertiaWorld.m4"), (0x90, "invInertiaWorld.m5"),
            (0x94, "invInertiaWorld.m6"), (0x98, "invInertiaWorld.m7"), (0x9c, "invInertiaWorld.m8"),
            (0xa0, "tail0"), (0xa4, "tail1"), (0xa8, "tail2"), (0xac, "tail3"), (0xb0, "tail4"),
        };

        private static readonly (int, string)[] CarMap =
        {
            (0x050, "inputGas"), (0x054, "inputBrake"), (0x058, "inputSteer"),
            (0x1dc, "model6.pivotPosition.x"), (0x1e0, "model6.pivotPosition.y"), (0x1e4, "model6.pivotPosition.z"),
            (0x1e8, "compute.simulationGate(over pivotAxis.x)"), (0x1ec, "pivotAxis.y"), (0x1f0, "pivotAxis.z"),
            (0x1fc, "compute.eventLevelC"),
            (0x200, "compute.eventSourceC/Ab"),
            (0x214, "compute.springC.stiffness"), (0x218, "springC.damping"), (0x21c, "springC.value"),
            (0x220, "springC.target"), (0x224, "springC.velocity"),
            (0x228, "compute.springA.stiffness"), (0x22c, "springA.damping"), (0x230, "springA.value"),
            (0x234, "springA.target"), (0x238, "springA.velocity"),
            (0x23c, "compute.contactRise"), (0x240, "compute.contactDecay"),
            (0x2e0, "compute.localSpeedLimit"), (0x2e4, "engineMode"), (0x2e8, "wheelCount"),
            (0x59c, "engine.maxRpm"), (0x5b0, "engine.brakingFactor(computedBrakeForce)"),
            (0x5b4, "engine.rpm"), (0x5b8, "engine.targetRpm"), (0x5bc, "engine.clutch"),
            (0x5c0, "engine.shiftTimer"), (0x5c4, "model6.reverseMode/engine.reverse"),
            (0x5c8, "engine.gear"), (0x5cc, "model6.reverseSpeedThreshold"),
            (0x5d0, "turboEpochTick"), (0x5d4, "airControlImmediate"),
            (0x5d8, "compute.state5d8(model6.contactBlockCount)"), (0x5dc, "sideContact(model6.sideContact)"),
            (0x5e0, "lastSideContactTick"), (0x5e4, "airControlLocked"), (0x5e8, "steeringValue"),
            (0x5f0, "turboProgress"), (0x5f4, "turboFactor"), (0x5f8, "turboStartTick"),
            (0x5fc, "turboEndTick"), (0x600, "turboType"), (0x604, "rouletteToken"),
            (0x608, "rouletteValue"), (0x60c, "flag60c"), (0x610, "compute.airImpulseCooldownTick"),
            (0x614, "airControlTick"), (0x618, "airControlSpeed.x"), (0x61c, "airControlSpeed.y"),
            (0x620, "airControlSpeed.z"), (0x624, "compute.effectAccumulator"), (0x628, "turboActive"),
            (0x62c, "model6.lastSlidingTick"), (0x630, "model6.slidingStartTick"),
            (0x634, "model6.slidingElapsedTicks"),
            (0x650, "compute.lastForceTick"), (0x654, "compute.eventLevelA"), (0x658, "compute.eventLevelB"),
            (0x660, "compute.peakEventLevelB"), (0x664, "compute.peakEventLevelA"),
            (0x668, "compute.peakEventLevelC"), (0x670, "compute.eventMetricA"),
            (0x674, "compute.eventMetricB"), (0x678, "compute.eventMetricC"),
            (0x69c, "driveMode"), (0x6a0, "forceWheelSpeed"),
            (0x6a4, "model6.modelIso.m0"), (0x6a8, "modelIso.m1"), (0x6ac, "modelIso.m2"),
            (0x6b0, "modelIso.m3"), (0x6b4, "modelIso.m4"), (0x6b8, "modelIso.m5"),
            (0x6bc, "modelIso.m6"), (0x6c0, "modelIso.m7"), (0x6c4, "modelIso.m8"),
            (0x6c8, "modelIso.t0"), (0x6cc, "modelIso.t1"), (0x6d0, "modelIso.t2"),
            (0x6d4, "compute.normalizedForce.x(rolloverAxis.x)"), (0x6d8, "normalizedForce.y"),
            (0x6dc, "normalizedForce.z"),
            (0x6e0, "model6.orbitCenter.x"), (0x6e4, "orbitCenter.y"), (0x6e8, "orbitCenter.z"),
            (0x6ec, "model6.orbitInitialRadius"), (0x6f0, "model6.orbitRadius"),
            (0x6f4, "model6.burnoutStartTick"), (0x6f8, "model6.burnoutTransitionTick"),
            (0x6fc, "model6.orbitAxis.x"), (0x700, "orbitAxis.y"), (0x704, "orbitAxis.z"),
            (0x708, "model6.orbitSign"),
            (0x70c, "currentLocalSpeed.x"), (0x710, "currentLocalSpeed.y"), (0x714, "currentLocalSpeed.z"),
            (0x73c, "blockWheelSpeed"), (0x744, "engineLimitFlag"), (0x748, "gearDownshiftFlag"),
            (0x74c, "compute.airEffectMode"),
            (0x818, "totalForceAdded.x"), (0x81c, "totalForceAdded.y"), (0x820, "totalForceAdded.z"),
            (0x824, "totalImpulseAdded.x"), (0x828, "totalImpulseAdded.y"), (0x82c, "totalImpulseAdded.z"),
            (0x840, "model6.axleWidth"),
        };

        private static readonly (int, string)[] WheelMapBase =
        {
            (0x000, "active"), (0x004, "steerable"), (0x008, "radius"),
            (0x0b4 + 0x00, "rt.damperAbsorb"), (0x0b4 + 0x04, "rt.field04"), (0x0b4 + 0x08, "rt.field08"),
            (0x0b4 + 0x54, "rt.field54.x"), (0x0b4 + 0x58, "rt.field54.y"), (0x0b4 + 0x5c, "rt.field54.z"),
            (0x0b4 + 0x6c, "rt.field6c"), (0x0b4 + 0x70, "rt.hasGroundContact"),
            (0x0b4 + 0x74, "rt.contactMaterialId"), (0x0b4 + 0x78, "rt.isSliding"),
            (0x0b4 + 0x7c, "rt.relativeRotzAxis.x"), (0x0b4 + 0x80, "rt.relativeRotzAxis.y"),
            (0x0b4 + 0x84, "rt.relativeRotzAxis.z"), (0x0b4 + 0x88, "rt.contactBodyWord"),
            (0x0b4 + 0x8c, "rt.groundContactCount"),
            (0x0b4 + 0x90, "rt.field90.x"), (0x0b4 + 0x94, "rt.field90.y"), (0x0b4 + 0x98, "rt.field90.z"),
            (0x0b4 + 0x9c, "rt.rotationPhase"), (0x0b4 + 0xa0, "rt.blendValue"), (0x0b4 + 0xa4, "rt.blendTarget"),
            (0x15c, "field15c"),
            (0x160, "contactRelativeLocalDistance.x"), (0x164, ".y"), (0x168, ".z"),
        };

        private static readonly (int, string)[] RaceMap =
        {
            (0, "visitedCheckpoints(u64)"), (8, "triggerContacts(u64)"), (16, "visitedCount"),
            (20, "completedLaps"), (24, "passedRaceCheckpoints"), (28, "elapsedTicks"),
            (32, "finishTimeMs"), (36, "projectionSegment"), (40, "centerlineSegment"),
            (44, "offTrackTicks"), (48, "stuckTicks"), (52, "arcLength"), (56, "lateralOffset"),
            (60, "halfWidth"), (64, "corridorLateral"), (68, "corridorVertical"),
            (72, "unwrappedProgress"), (76, "bestProgress"), (80, "previousProgress"),
            (84, "respawnLocation.m0"), (88, ".m1"), (92, ".m2"), (96, ".m3"), (100, ".m4"),
            (104, ".m5"), (108, ".m6"), (112, ".m7"), (116, ".m8"), (120, ".t0"), (124, ".t1"),
            (128, ".t2"), (132, "finished"), (133, "outsideCorridor"), (134, "respawnAvailable"),
            (135, "reserved"),
        };

        private static string DumpDiff(byte[] e, byte[] a)
        {
            var sb = new System.Text.StringBuilder();
            int carBase = 180;
            int wheelsBase = 180 + 0x878;
            int triggerBase = 180 + 0x878 + 4 * 0x2fc;
            int tickBase = triggerBase + 8;
            int raceBase = tickBase + 4;
            void Region(byte[] x, byte[] y, int off, int len, string tag, (int, string)[] map)
            {
                for (int i = 0; i < len; i += 4)
                {
                    if (off + i + 4 > x.Length || off + i + 4 > y.Length)
                    {
                        return;
                    }
                    uint ev = BitsUtil.GetU32(x, off + i);
                    uint av = BitsUtil.GetU32(y, off + i);
                    if (ev != av)
                    {
                        string name = "?";
                        foreach ((int mo, string mn) in map)
                        {
                            if (mo == i)
                            {
                                name = mn;
                                break;
                            }
                        }
                        sb.AppendLine($"  DUMP {tag}+0x{i:X3} {name}: expected 0x{ev:X8} ({FmtF32(ev)}), " +
                                      $"actual 0x{av:X8} ({FmtF32(av)})");
                    }
                }
            }
            Region(e, a, 0, 180, "dyna", DynaMap);
            Region(e, a, carBase, 0x878, "car", CarMap);
            for (int w = 0; w < 4; w++)
            {
                var wheelMap = new List<(int, string)>();
                foreach ((int off, string name) in WheelMapBase)
                {
                    wheelMap.Add((off, $"w{w}.{name}"));
                }
                Region(e, a, wheelsBase + w * 0x2fc, 0x2fc, $"wheel{w}", wheelMap.ToArray());
            }
            if (BitsUtil.GetU64(e, triggerBase) != BitsUtil.GetU64(a, triggerBase))
            {
                sb.AppendLine($"  DUMP triggerContacts: expected 0x{BitsUtil.GetU64(e, triggerBase):X16}, " +
                              $"actual 0x{BitsUtil.GetU64(a, triggerBase):X16}");
            }
            if (BitsUtil.GetU32(e, tickBase) != BitsUtil.GetU32(a, tickBase))
            {
                sb.AppendLine($"  DUMP tickTime: expected {BitsUtil.GetU32(e, tickBase)}, " +
                              $"actual {BitsUtil.GetU32(a, tickBase)}");
            }
            Region(e, a, raceBase, 136, "race", RaceMap);
            return sb.ToString();
        }

        // ------------------------------------------------------------------
        // replay (dumps on every tick)
        // ------------------------------------------------------------------

        private static byte[] RunTraceDumpAll(byte[] input, string trackFile)
        {
            ByteReader r = new ByteReader(input);
            uint scheduleId = r.U32();
            uint tickCount = r.U32();
            byte trackId = r.U8();
            bool hasRoute = r.U8() != 0;
            var packets = new TMNFRaceInputs[tickCount];
            for (int t = 0; t < (int)tickCount; t++)
            {
                packets[t] = ReadPacket(r);
            }

            string tmp = Path.Combine(Path.GetTempPath(), "tmnf-sim-tests");
            Directory.CreateDirectory(tmp);
            string trackPath = Path.Combine(tmp, trackFile);
            if (!File.Exists(trackPath))
            {
                File.WriteAllBytes(trackPath,
                    File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/" + trackFile)));
            }
            string routePath = Path.Combine(tmp, "loop.tmnfroute");
            if (!File.Exists(routePath))
            {
                File.WriteAllBytes(routePath,
                    File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/loop.tmnfroute")));
            }
            RouteLoader.TmnfRoute route = hasRoute ? RouteLoader.TmnfRouteLoad(routePath, null) : null;
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob = File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
            byte[] mask = File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/game-mask.bin"));
            World world = new World(track, vehicleBlob, new WorldOptions
            {
                fakeContactMask = mask,
                pinProvenance = false,
            });
            world.route = route;
            Race.TmnfRaceState race = default(Race.TmnfRaceState);
            if (route != null)
            {
                GmIso4 iso = world.Live().RotPosIso();
                Race.TmnfRaceReset(route, ref race, ref iso);
            }

            var output = new List<byte>();
            void Record()
            {
                var hb = new List<byte>();
                hb.AddRange(world.Live().ToBytes());
                var (carB, wheelsB) = world.WritePlayerGameState();
                hb.AddRange(carB);
                hb.AddRange(wheelsB);
                ByteWriter.U64(hb, world.triggerContacts);
                ByteWriter.U32(hb, world.tickTime);
                SimTraceTests.WriteRaceStatePublic(hb, ref race);
                ByteWriter.U64(output, Fnva.Hash(hb.ToArray()));
                ref CHmsStateDyna s = ref world.Live();
                ByteWriter.U32(output, unchecked(race.elapsedTicks * 10));
                GmGoldenTests.WriteVec3To(output, s.pos);
                float[] q = { s.quat.y, s.quat.z, s.quat.w, s.quat.x };
                foreach (float f in q)
                {
                    ByteWriter.F32(output, f);
                }
                GmGoldenTests.WriteVec3To(output, s.linVel);
                GmGoldenTests.WriteVec3To(output, s.angVel);
                for (int wk = 0; wk < 4; wk++)
                {
                    CSceneVehicleCarWheelRealTimeState rt = world.vehicle.wheels[wk].realTime;
                    ByteWriter.U8(output, (byte)(rt.hasGroundContact != 0 ? 1 : 0));
                    ByteWriter.I32(output, rt.contactMaterialId);
                    ByteWriter.U8(output, (byte)(rt.isSliding != 0 ? 1 : 0));
                    ByteWriter.F32(output, rt.field6c);
                }
                ByteWriter.F32(output, world.vehicle.engine.rpm);
                ByteWriter.I32(output, world.vehicle.engine.gear);
                ByteWriter.U8(output, (byte)(Race.TmnfRaceRespawnLocation(ref race).HasValue ? 1 : 0));
                ByteWriter.U8(output, 1);
                output.AddRange(hb.ToArray());
            }

            Record();
            bool panicked = false;
            uint panickedAt = 0;
            for (uint t = 0; t < tickCount; t++)
            {
                TMNFRaceInputs pkt = packets[t];
                try
                {
                    if (pkt.respawn != 0)
                    {
                        GmIso4? spawn = Race.TmnfRaceRespawnLocation(ref race);
                        if (spawn.HasValue)
                        {
                            GmIso4 sp = spawn.Value;
                            world.Respawn(ref sp);
                        }
                    }
                    world.ApplyInputs(ref pkt);
                    world.AdvanceTimer(10);
                    world.PhysicsStep2(10);
                    if (route != null && race.finished == 0)
                    {
                        GmIso4 iso = world.Live().RotPosIso();
                        ulong contacts = world.triggerContacts;
                        Race.TmnfRaceStep(route, ref race, contacts, ref iso);
                    }
                    Record();
                }
                catch (Exception ex) when (
                    ex is IndexOutOfRangeException
                    || ex is InvalidOperationException
                    || ex is OverflowException)
                {
                    panicked = true;
                    panickedAt = t;
                    break;
                }
            }
            ByteWriter.U8(output, (byte)(panicked ? 1 : 0));
            if (panicked)
            {
                ByteWriter.U32(output, panickedAt);
            }
            return output.ToArray();
        }

        private static TMNFRaceInputs ReadPacket(ByteReader r)
        {
            TMNFRaceInputs p = default(TMNFRaceInputs);
            p.steerLeftTime = r.U32();
            p.reserved04 = r.U32();
            p.steerLeft = r.I32();
            p.steerRightTime = r.U32();
            p.reserved10 = r.U32();
            p.steerRight = r.I32();
            p.steerAnalogTime = r.U32();
            p.reserved1c = r.U32();
            p.steerAnalog = r.F32();
            p.accelerateTime = r.U32();
            p.reserved28 = r.U32();
            p.accelerate = r.I32();
            p.brakeTime = r.U32();
            p.reserved34 = r.U32();
            p.brake = r.I32();
            p.gasAnalogTime = r.U32();
            p.respawn = r.U32();
            p.gasAnalog = r.F32();
            return p;
        }
    }
}
