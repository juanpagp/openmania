using System;
using System.Collections.Generic;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// The whole-simulation golden traces: full-pipeline bit-exactness per
    /// tick. Each trace row carries the input schedule; the runner replays
    /// it through the World + race harness and compares per-tick FNV-1a
    /// hashes, telemetry, and (for early ticks) full state dumps.
    /// </summary>
    [TestFixture]
    public class SimTraceTests
    {
        private static void WriteRaceState(System.Collections.Generic.List<byte> o, ref Race.TmnfRaceState s)
        {
            WriteRaceStatePublic(o, ref s);
        }

        /// <summary>Shared with the divergence-hunting diagnostics.</summary>
        internal static void WriteRaceStatePublic(System.Collections.Generic.List<byte> o, ref Race.TmnfRaceState s)
        {
            ByteWriter.U64(o, s.visitedCheckpoints);
            ByteWriter.U64(o, s.triggerContacts);
            ByteWriter.U32(o, s.visitedCount);
            ByteWriter.U32(o, s.completedLaps);
            ByteWriter.U32(o, s.passedRaceCheckpoints);
            ByteWriter.U32(o, s.elapsedTicks);
            ByteWriter.U32(o, s.finishTimeMs);
            ByteWriter.U32(o, s.projectionSegment);
            ByteWriter.U32(o, s.centerlineSegment);
            ByteWriter.U32(o, s.offTrackTicks);
            ByteWriter.U32(o, s.stuckTicks);
            ByteWriter.F32(o, s.arcLength);
            ByteWriter.F32(o, s.lateralOffset);
            ByteWriter.F32(o, s.halfWidth);
            ByteWriter.F32(o, s.corridorLateral);
            ByteWriter.F32(o, s.corridorVertical);
            ByteWriter.F32(o, s.unwrappedProgress);
            ByteWriter.F32(o, s.bestProgress);
            ByteWriter.F32(o, s.previousProgress);
            byte[] iso = new byte[48];
            s.respawnLocation.ToBytes(iso, 0);
            o.AddRange(iso);
            ByteWriter.U8(o, s.finished);
            ByteWriter.U8(o, s.outsideCorridor);
            ByteWriter.U8(o, s.respawnAvailable);
            ByteWriter.U8(o, s.reserved);
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

        /// <summary>Runs one trace file and produces the raw comparison bytes.</summary>
        private static byte[] RunTrace(string relativePath, string trackFile, string routeFile)
        {
            List<GoldenRunner.Row> rows = GoldenRunner.ReadRows(relativePath);
            Assert.That(rows.Count, Is.EqualTo(1), "trace files carry a single row");
            ByteReader r = new ByteReader(rows[0].Input);
            uint scheduleId = r.U32();
            uint tickCount = r.U32();
            byte trackId = r.U8();
            bool withRoute = r.U8() != 0;
            TMNFRaceInputs[] packets = new TMNFRaceInputs[tickCount];
            for (int t = 0; t < (int)tickCount; t++)
            {
                packets[t] = ReadPacket(r);
            }

            string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-sim-tests");
            System.IO.Directory.CreateDirectory(tmp);
            string trackPath = System.IO.Path.Combine(tmp, trackFile);
            System.IO.File.WriteAllBytes(trackPath,
                System.IO.File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/" + trackFile)));
            RouteLoader.TmnfRoute route = withRoute
                ? RouteLoader.TmnfRouteLoad(
                    WriteRouteFixture(tmp), null)
                : null;

            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob = System.IO.File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
            byte[] mask = System.IO.File.ReadAllBytes(
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

            uint dumpTicks = tickCount > 2000 ? 16u : 32u;
            var output = new List<byte>();

            void Record(bool withDump)
            {
                var hb = new List<byte>();
                hb.AddRange(world.Live().ToBytes());
                var (carB, wheelsB) = world.WritePlayerGameState();
                hb.AddRange(carB);
                hb.AddRange(wheelsB);
                ByteWriter.U64(hb, world.triggerContacts);
                ByteWriter.U32(hb, world.tickTime);
                WriteRaceState(hb, ref race);
                ByteWriter.U64(output, Fnva.Hash(hb.ToArray()));
                // telemetry
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
                if (withDump)
                {
                    ByteWriter.U8(output, 1);
                    output.AddRange(hb.ToArray());
                }
                else
                {
                    ByteWriter.U8(output, 0);
                }
            }

            // t = 0: post-construction state (includes the settle tick).
            Record(true);
            bool panicked = false;
            uint panickedAt = 0;
            for (uint t = 0; t < tickCount; t++)
            {
                TMNFRaceInputs input = packets[t];
                try
                {
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
                    Record(t < dumpTicks);
                }
                catch (Exception ex) when (
                    ex is IndexOutOfRangeException
                    || ex is InvalidOperationException
                    || ex is OverflowException)
                {
                    // The oracle panicked mid-tick (the upstream water bug on
                    // the pool track); the trace must end at exactly this tick.
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

        private static string WriteRouteFixture(string tmp)
        {
            string routePath = System.IO.Path.Combine(tmp, "loop.tmnfroute");
            if (!System.IO.File.Exists(routePath))
            {
                System.IO.File.WriteAllBytes(routePath,
                    System.IO.File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/loop.tmnfroute")));
            }
            return routePath;
        }

        [Test]
        public void DiagnoseTick0()
        {
            // Compare the tick-0 state dump field by field.
            List<GoldenRunner.Row> rows = GoldenRunner.ReadRows("sim/trace_drive.tgn");
            ByteReader r = new ByteReader(rows[0].Input);
            r.U32(); r.U32(); r.U8(); r.U8();
            // (packets skipped — only the tick-0 record matters)
            byte[] expected = rows[0].Output;
            // Rebuild the tick-0 record with the harness.
            string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-sim-tests");
            string trackPath = System.IO.Path.Combine(tmp, "plain.tmnftrack");
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob = System.IO.File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
            byte[] mask = System.IO.File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/game-mask.bin"));
            World world = new World(track, vehicleBlob, new WorldOptions
            {
                fakeContactMask = mask,
                pinProvenance = false,
            });
            RouteLoader.TmnfRoute route = RouteLoader.TmnfRouteLoad(
                System.IO.Path.Combine(tmp, "loop.tmnfroute"), null);
            world.route = route;
            Race.TmnfRaceState race = Race.TmnfRaceState.Create();
            GmIso4 iso0 = world.Live().RotPosIso();
            Race.TmnfRaceReset(route, ref race, ref iso0);

            var hb = new List<byte>();
            hb.AddRange(world.Live().ToBytes());
            var (car, wheels) = world.WritePlayerGameState();
            hb.AddRange(car);
            hb.AddRange(wheels);
            ByteWriter.U64(hb, world.triggerContacts);
            ByteWriter.U32(hb, world.tickTime);
            byte[] mine = hb.ToArray();
            // expected layout: [u64 hash][110B telemetry][u8 hasDump][dump]
            // skip 8 (hash) + 110 (telemetry) + 1 (hasDump)
            int dumpOff = 8 + 105 + 1;
            byte[] expDump = new byte[expected.Length - dumpOff];
            Array.Copy(expected, dumpOff, expDump, 0, expDump.Length);
            int n = Math.Min(mine.Length, expDump.Length);
            int firstDiff = -1;
            for (int i = 0; i < n; i++)
            {
                if (mine[i] != expDump[i]) { firstDiff = i; break; }
            }
            Console.WriteLine($"mine len={mine.Length} expDump len={expDump.Length} firstDiff={firstDiff}");
            if (firstDiff >= 0)
            {
                int w32 = firstDiff & ~3;
                Console.WriteLine($"first diff byte {firstDiff}: mine=0x{BitConverter.ToUInt32(mine, w32):X8} exp=0x{BitConverter.ToUInt32(expDump, w32):X8}");
                // dyna block = 180 bytes: map offset
                if (firstDiff < 180)
                {
                    Console.WriteLine($"-> DYNA offset 0x{firstDiff:X} (field at 0x{firstDiff & ~3:X})");
                }
                else if (firstDiff < 180 + 0x878)
                {
                    Console.WriteLine($"-> CAR offset 0x{firstDiff - 180:X}");
                }
                else
                {
                    Console.WriteLine($"-> WHEELS offset 0x{firstDiff - 180 - 0x878:X}");
                }
                for (int i = Math.Max(0, firstDiff - 16); i < Math.Min(n, firstDiff + 16); i += 4)
                {
                    Console.WriteLine($"  @{i}: mine=0x{BitConverter.ToUInt32(mine, i):X8} exp=0x{BitConverter.ToUInt32(expDump, i):X8} {(mine[i] == expDump[i] ? "" : " <<<")}");
                }
            }
            Assert.That(firstDiff, Is.EqualTo(-1), "tick-0 dump differs");
        }

        [Test]
        public void TraceDrive()
        {
            GoldenRunner.RunSuite("sim/trace_drive.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void TraceRandomDigital()
        {
            GoldenRunner.RunSuite("sim/trace_random_digital.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void TraceRandomAnalog()
        {
            GoldenRunner.RunSuite("sim/trace_random_analog.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void TraceBurnout()
        {
            GoldenRunner.RunSuite("sim/trace_burnout.tgn", input => RunTraceBytes(input, "plain.tmnftrack", false));
        }

        [Test]
        public void TraceCrash()
        {
            GoldenRunner.RunSuite("sim/trace_crash.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void TraceRespawn()
        {
            GoldenRunner.RunSuite("sim/trace_respawn.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void TracePool()
        {
            // The pool track ends in the upstream water-friction panic at a
            // deterministic tick — the trace carries the marker.
            GoldenRunner.RunSuite("sim/trace_pool.tgn", input => RunTraceBytes(input, "pool.tmnftrack", false));
        }

        [Test]
        public void TraceLongRun()
        {
            // 100k ticks (the long-run stress trace).
            GoldenRunner.RunSuite("sim/trace_longrun.tgn", input => RunTraceBytes(input, "plain.tmnftrack", true));
        }

        [Test]
        public void SimFacade()
        {
            GoldenRunner.RunSuite("sim/facade.tgn", rowInput =>
            {
                var r = new ByteReader(rowInput);
                r.U8(); // selector
                string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-sim-tests");
                System.IO.Directory.CreateDirectory(tmp);
                string trackPath = System.IO.Path.Combine(tmp, "plain.tmnftrack");
                if (!System.IO.File.Exists(trackPath))
                {
                    System.IO.File.WriteAllBytes(trackPath,
                        System.IO.File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/plain.tmnftrack")));
                }
                TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
                byte[] vehicleBlob = System.IO.File.ReadAllBytes(
                    GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
                byte[] mask = System.IO.File.ReadAllBytes(
                    GoldenRunner.GoldenPath("fixtures/game-mask.bin"));
                Sim sim = new Sim(track, vehicleBlob, mask, new SimOptions
                {
                    fakeContactMask = null,
                    route = null,
                    pinProvenance = false,
                });
                var o = new List<byte>();
                for (uint t = 0; t < 400; t++)
                {
                    // The drive schedule (0..400 of it): t>=350 steers LEFT
                    // (drive_packet: 350..500 -> accelerate + steer_left).
                    (int a, int b, int l, int rr) = (int)t switch
                    {
                        var tt when tt < 300 => (1, 0, 0, 0),
                        var tt when tt < 350 => (0, 1, 0, 0),
                        _ => (1, 0, 1, 0),
                    };
                    TmnfInput input = TmnfInput.Digital(a, b, l, rr);
                    VehicleState state = sim.Step(ref input);
                    ByteWriter.U32(o, state.raceTimeMs);
                    GmGoldenTests.WriteVec3To(o, state.position);
                    foreach (float f in state.quaternion)
                    {
                        ByteWriter.F32(o, f);
                    }
                    GmGoldenTests.WriteVec3To(o, state.linearSpeed);
                    GmGoldenTests.WriteVec3To(o, state.angularSpeed);
                    foreach (WheelState wk in state.wheels)
                    {
                        ByteWriter.U8(o, (byte)(wk.onGround ? 1 : 0));
                        ByteWriter.I32(o, wk.contactMaterialId);
                        ByteWriter.U8(o, (byte)(wk.sliding ? 1 : 0));
                        ByteWriter.F32(o, wk.spinSpeed);
                    }
                    ByteWriter.F32(o, state.engineRpm);
                    ByteWriter.I32(o, state.gear);
                }
                return o.ToArray();
            });
        }

        private static byte[] RunTraceBytes(byte[] input, string trackFile, bool withRoute)
        {
            // The trace runner inlined (avoids re-reading the row).
            return RunTraceDirect(input, trackFile, withRoute);
        }

        private static byte[] RunTraceDirect(byte[] input, string trackFile, bool withRoute)
        {
            // same as RunTrace but from raw input bytes
            ByteReader r = new ByteReader(input);
            uint scheduleId = r.U32();
            uint tickCount = r.U32();
            byte trackId = r.U8();
            bool route = r.U8() != 0;
            TMNFRaceInputs[] packets = new TMNFRaceInputs[tickCount];
            for (int t = 0; t < (int)tickCount; t++)
            {
                packets[t] = ReadPacket(r);
            }

            string tmp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "tmnf-sim-tests");
            System.IO.Directory.CreateDirectory(tmp);
            string trackPath = System.IO.Path.Combine(tmp, trackFile);
            if (!System.IO.File.Exists(trackPath))
            {
                System.IO.File.WriteAllBytes(trackPath,
                    System.IO.File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/" + trackFile)));
            }
            RouteLoader.TmnfRoute tmnfRoute = route ? RouteLoader.TmnfRouteLoad(WriteRouteFixture(tmp), null) : null;
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoad(trackPath, null);
            byte[] vehicleBlob = System.IO.File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"));
            byte[] mask = System.IO.File.ReadAllBytes(
                GoldenRunner.GoldenPath("fixtures/game-mask.bin"));
            World world = new World(track, vehicleBlob, new WorldOptions
            {
                fakeContactMask = mask,
                pinProvenance = false,
            });
            world.route = tmnfRoute;
            // The generator starts from TmnfRaceState::default() — ALL
            // ZEROS (respawn_location/previous_car_transform are zero
            // isos, not identity) — and resets only when a route exists.
            Race.TmnfRaceState race = default(Race.TmnfRaceState);
            if (tmnfRoute != null)
            {
                GmIso4 iso = world.Live().RotPosIso();
                Race.TmnfRaceReset(tmnfRoute, ref race, ref iso);
            }

            uint dumpTicks = tickCount > 2000 ? 16u : 32u;
            var output = new List<byte>();

            void Record(bool withDump)
            {
                var hb = new List<byte>();
                hb.AddRange(world.Live().ToBytes());
                var (carB, wheelsB) = world.WritePlayerGameState();
                hb.AddRange(carB);
                hb.AddRange(wheelsB);
                ByteWriter.U64(hb, world.triggerContacts);
                ByteWriter.U32(hb, world.tickTime);
                WriteRaceState(hb, ref race);
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
                if (withDump)
                {
                    ByteWriter.U8(output, 1);
                    output.AddRange(hb.ToArray());
                }
                else
                {
                    ByteWriter.U8(output, 0);
                }
            }

            Record(true);
            bool panicked = false;
            uint panickedAt = 0;
            for (uint t = 0; t < tickCount; t++)
            {
                TMNFRaceInputs input2 = packets[t];
                try
                {
                    if (input2.respawn != 0)
                    {
                        GmIso4? spawn = Race.TmnfRaceRespawnLocation(ref race);
                        if (spawn.HasValue)
                        {
                            GmIso4 sp = spawn.Value;
                            world.Respawn(ref sp);
                        }
                    }
                    world.ApplyInputs(ref input2);
                    world.AdvanceTimer(10);
                    world.PhysicsStep2(10);
                    if (tmnfRoute != null && race.finished == 0)
                    {
                        GmIso4 iso = world.Live().RotPosIso();
                        ulong contacts = world.triggerContacts;
                        Race.TmnfRaceStep(tmnfRoute, ref race, contacts, ref iso);
                    }
                    Record(t < dumpTicks);
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
    }
}
