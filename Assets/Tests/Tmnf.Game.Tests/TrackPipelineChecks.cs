using System;
using System.Collections.Generic;
using Tmnf.Game.Pure;
using Tmnf.Sim;
using Tmnf.Sim.Fixtures;

namespace Tmnf.Game.Tests
{
    // 'Sim' inside any Tmnf.* namespace resolves to the sibling namespace
    // Tmnf.Sim before the imported class; alias the facade explicitly.
    using Sim = global::Tmnf.Sim.Sim;

    /// <summary>
    /// The verification suite for the gameplay bridge around the physics
    /// core — written against plain asserts (no test framework) so the very
    /// same file runs under the Unity Test Runner (via the NUnit wrappers in
    /// GamePipelineTests.cs) AND inside the standalone dotnet harness that
    /// verifies the integration without a Unity editor.
    ///
    /// What is covered end-to-end, through the REAL loaders and the REAL
    /// simulation (no mocks):
    ///   1. the core still reproduces the golden facade trace (environment);
    ///   2. meshes built by the track image bridge are properly collidable
    ///      (drive, ground contact, non-uniform scale, one-sided winding);
    ///   3. the route image bridge loads and the race layer behaves:
    ///      checkpoints in any order, finish rejected until all checkpoints
    ///      are collected, zero-checkpoint tracks, multiple finishes;
    ///   4. restarting from the same cached images leaves no stale state;
    ///   5. the entry BVH culling finds meshes laid out in a large grid;
    ///   6. timing formatting and the personal-best table.
    /// </summary>
    public static class TrackPipelineChecks
    {
        public sealed class CheckFailedException : Exception
        {
            public CheckFailedException(string message) : base(message) { }
        }

        private static void Check(bool condition, string message)
        {
            if (!condition)
            {
                throw new CheckFailedException(message);
            }
        }

        private static void CheckEqual<T>(T expected, T actual, string what)
        {
            Check(EqualityComparer<T>.Default.Equals(expected, actual),
                $"{what}: expected {expected}, got {actual}");
        }

        // ------------------------------------------------------------------
        // fixture helpers (mirror what the Unity-side extractor produces)
        // ------------------------------------------------------------------

        /// <summary>A floor quad in TMNF world space at height y.</summary>
        private static GameMeshSpec FloorQuad(
            float halfX, float halfZ, float y, bool windingUp)
        {
            // Wound so the right-hand-rule normal points +Y (windingUp) or -Y.
            GameMeshSpec mesh = new GameMeshSpec();
            mesh.name = "floor";
            mesh.vertices = new float[]
            {
                -halfX, y, -halfZ,
                -halfX, y, halfZ,
                halfX, y, halfZ,
                halfX, y, -halfZ,
            };
            if (windingUp)
            {
                // (v0, v1, v2) then (v0, v2, v3): counter-clockwise seen from +Y.
                mesh.triangles = new uint[] { 0, 1, 2, 0, 2, 3 };
                mesh.faceNormals = new float[] { 0, 1, 0, 0, 1, 0 };
            }
            else
            {
                mesh.triangles = new uint[] { 0, 2, 1, 0, 3, 2 };
                mesh.faceNormals = new float[] { 0, -1, 0, 0, -1, 0 };
            }
            mesh.materialId = 0;
            return mesh;
        }

        private static GameTrackSpec FloorTrack(
            float halfX, float halfZ, float y, bool windingUp)
        {
            GameTrackSpec spec = new GameTrackSpec();
            spec.meshes.Add(FloorQuad(halfX, halfZ, y, windingUp));
            return spec;
        }

        private static GameTriggerSpec Trigger(
            float cx, float cy, float cz, float hx, float hy, float hz)
        {
            return GameTriggerSpec.Box(cx, cy, cz, hx, hy, hz);
        }

        private static RouteLoader.TmnfRoute LoadRoute(GameRouteSpec routeSpec)
        {
            byte[] bytes = TmnfRouteImage.Build(routeSpec);
            return RouteLoader.TmnfRouteLoadBytes(bytes, null);
        }

        private static Sim BuildSim(
            GameTrackSpec trackSpec, GameRouteSpec routeSpec,
            byte[] vehicle, byte[] mask, out byte[] trackBytes, out byte[] routeBytes)
        {
            trackBytes = TmnfTrackImage.Build(trackSpec);
            routeBytes = TmnfRouteImage.Build(routeSpec);
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            return new Sim(track, vehicle, mask, new SimOptions
            {
                fakeContactMask = null,
                routeBytes = routeBytes,
                pinProvenance = false,
            });
        }

        private static GmIso4 StartAt(float x, float y, float z)
        {
            GmIso4 iso = GmIso4.Identity;
            iso.t0 = x;
            iso.t1 = y;
            iso.t2 = z;
            return iso;
        }

        private static TmnfInput Throttle => TmnfInput.Digital(1, 0, 0, 0);

        private static VehicleState lastObserved;

        /// <summary>
        /// Steps; the facade's finished-race quirk (State() refusing a
        /// finished race) surfaces as a return of the last observed state
        /// — exactly what the game's TmnfSim driver does with its
        /// OnRaceFinished event.
        /// </summary>
        private static VehicleState StepOrFinish(Sim sim, TmnfInput input)
        {
            try
            {
                VehicleState state = sim.Step(input);
                lastObserved = state;
                return state;
            }
            catch (InvalidOperationException)
            {
                if (IsFinished(sim))
                {
                    return lastObserved;
                }
                throw;
            }
        }

        private static bool IsFinished(Sim sim)
        {
            Race.TmnfRaceState? race = sim.Race();
            return race.HasValue && race.Value.finished != 0;
        }

        // ==================================================================
        // 1. environment: the compiled core reproduces the golden trace
        // ==================================================================

        public static void GoldenFacadeHash()
        {
            byte[] trackBytes = TmnfFixtures.PlainTrack();
            byte[] vehicleBytes = TmnfFixtures.StadiumVehicle();
            byte[] maskBytes = TmnfFixtures.GameMask();
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            Sim sim = new Sim(track, vehicleBytes, maskBytes, new SimOptions
            {
                fakeContactMask = null,
                route = null,
                pinProvenance = false,
            });

            List<byte> stream = new List<byte>(400 * 104);
            for (uint t = 0; t < 400; t++)
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
            }
            ulong hash = Fnva.Hash(stream.ToArray());
            CheckEqual(0x5A45_761F_3728_DF35UL, hash,
                "golden facade hash (the compiled core must reproduce the golden trace)");
        }

        // ==================================================================
        // 2. the mesh bridge produces collidable geometry
        // ==================================================================

        public static void BridgeFloorDrives()
        {
            GameTrackSpec trackSpec = FloorTrack(400f, 400f, 0f, true);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.startIso[9] = 0f;
            routeSpec.startIso[10] = 1f;
            routeSpec.startIso[11] = 0f;
            routeSpec.finishes.Add(Trigger(0, 5, 800, 20, 20, 20));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);
            GmIso4 start = StartAt(0, 1, 0);
            sim.World.Respawn(ref start);

            VehicleState state = default;
            for (int t = 0; t < 300; t++)
            {
                state = StepOrFinish(sim, Throttle);
            }
            Check(state.position.z > 40f,
                $"the car must drive forward on the bridged floor (z={state.position.z}; " +
                "the golden fixture reaches 53 m over the same schedule)");
            Check(state.position.y > -1f && state.position.y < 5f,
                $"the car must stay on the floor (y={state.position.y})");
            Check(state.wheels[0].onGround || state.wheels[1].onGround
                || state.wheels[2].onGround || state.wheels[3].onGround,
                "at least one wheel must report ground contact on the bridged floor");
            CheckEqual(3000u, state.raceTimeMs, "race clock must advance at 100 Hz");
        }

        public static void BridgeOneSidedWinding()
        {
            // A floor wound the other way (normals down) is solid from BELOW:
            // a car dropped from above falls through it.
            GameTrackSpec trackSpec = FloorTrack(400f, 400f, 0f, false);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.finishes.Add(Trigger(0, 5, 800, 20, 20, 20));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);
            GmIso4 start = StartAt(0, 1, 0);
            sim.World.Respawn(ref start);

            float lowest = float.PositiveInfinity;
            for (int t = 0; t < 300; t++)
            {
                VehicleState state = StepOrFinish(sim, TmnfInput.Idle());
                lowest = Math.Min(lowest, state.position.y);
            }
            Check(lowest < -20f,
                $"a car above a downward-facing floor must fall through " +
                $"(one-sided collision), lowest y={lowest}");
        }

        public static void BridgeScaledMeshBake()
        {
            // The extractor bakes world transforms into vertices; a floor
            // scaled 3x in X and moved up must behave exactly like its
            // baked geometry (car rests at the baked height).
            GameMeshSpec mesh = FloorQuad(100f, 200f, 0f, true);
            for (int i = 0; i < mesh.vertices.Length; i += 3)
            {
                mesh.vertices[i + 0] *= 3.0f; // X scale baked in
                mesh.vertices[i + 1] += 12.0f; // Y offset baked in
            }
            GameTrackSpec trackSpec = new GameTrackSpec();
            trackSpec.meshes.Add(mesh);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.startIso[10] = 13f; // 1 m above the baked floor
            routeSpec.finishes.Add(Trigger(0, 17, 800, 20, 20, 20));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);
            GmIso4 start = StartAt(0, 13f, 0);
            sim.World.Respawn(ref start);

            VehicleState state = default;
            for (int t = 0; t < 200; t++)
            {
                state = StepOrFinish(sim, TmnfInput.Idle());
            }
            Check(state.position.y > 11.5f && state.position.y < 14.5f,
                $"the car must rest on the baked floor near y=12 (y={state.position.y})");
            Check(state.wheels[0].onGround || state.wheels[1].onGround
                || state.wheels[2].onGround || state.wheels[3].onGround,
                "wheels must contact the baked, scaled floor");
        }

        public static void BridgeEntryBvhGrid()
        {
            // 400 tiles on a 20x20 grid (40 m pitch): the car placed above
            // a far tile must find it through the entry BVH + static grid.
            GameTrackSpec trackSpec = new GameTrackSpec();
            for (int ix = 0; ix < 20; ix++)
            {
                for (int iz = 0; iz < 20; iz++)
                {
                    GameMeshSpec tile = FloorQuad(19f, 19f, 0f, true);
                    for (int i = 0; i < tile.vertices.Length; i += 3)
                    {
                        tile.vertices[i + 0] += (ix - 10) * 40f;
                        tile.vertices[i + 2] += (iz - 10) * 40f;
                    }
                    trackSpec.meshes.Add(tile);
                }
            }
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.finishes.Add(Trigger(0, 5, 800, 20, 20, 20));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);

            // Above the tile at (ix=17, iz=12): world x = 7*40 = 280, z = 2*40 = 80.
            GmIso4 start = StartAt(280f, 1f, 80f);
            sim.World.Respawn(ref start);

            VehicleState state = default;
            for (int t = 0; t < 120; t++)
            {
                state = StepOrFinish(sim, TmnfInput.Idle());
            }
            Check(state.position.y > -1f && state.position.y < 5f,
                $"the car must rest on the far tile through the BVH (y={state.position.y})");
            Check(state.wheels[0].onGround || state.wheels[1].onGround
                || state.wheels[2].onGround || state.wheels[3].onGround,
                "wheels must report contact on the far tile");
        }

        // ==================================================================
        // 3. the route bridge + race layer behavior
        // ==================================================================

        /// <summary>
        /// Spawns the car a whisker before a trigger's entry face (front
        /// wheels on the plane): the trigger volumes are closed boxes whose
        /// faces only detect the car from OUTSIDE, within an ellipsoid
        /// radius — exactly like driving through a checkpoint gate.
        /// </summary>
        private static GmIso4 SpawnBeforeTrigger(Tmnf.Game.Pure.GameTriggerSpec trigger)
        {
            return StartAt(trigger.cx, 1.0f, trigger.cz - trigger.hz - 1.78f);
        }

        public static void RouteCheckpointsAnyOrder()
        {
            GameTrackSpec trackSpec = FloorTrack(600f, 600f, 0f, true);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.checkpoints.Add(Trigger(0, 5, 30, 5, 5, 5));
            routeSpec.checkpoints.Add(Trigger(0, 5, 60, 5, 5, 5));
            routeSpec.checkpoints.Add(Trigger(0, 5, 90, 5, 5, 5));
            routeSpec.finishes.Add(Trigger(0, 5, 120, 5, 5, 5));

            RouteLoader.TmnfRoute route = LoadRoute(routeSpec);
            CheckEqual(3u, route.metadata.checkpointCount, "checkpoint count");
            CheckEqual(1u, route.metadata.finishCount, "finish count");

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);

            // Collect the checkpoints in the order 2 -> 0 -> 1.
            int[] order = { 2, 0, 1 };
            foreach (int index in order)
            {
                GmIso4 spot = SpawnBeforeTrigger(routeSpec.checkpoints[index]);
                sim.World.Respawn(ref spot);
                for (int t = 0; t < 5; t++)
                {
                    StepOrFinish(sim, Throttle);
                }
                Race.TmnfRaceState race = sim.Race().Value;
                Check(race.visitedCount >= (uint)Array.IndexOf(order, index) + 1,
                    $"checkpoint {index} must be collectible in any order " +
                    $"(visited={race.visitedCount})");
            }
            Race.TmnfRaceState collected = sim.Race().Value;
            CheckEqual(3u, collected.visitedCount, "all three checkpoints collected");

            // Finish only after everything was collected.
            GmIso4 finish = SpawnBeforeTrigger(routeSpec.finishes[0]);
            sim.World.Respawn(ref finish);
            bool finished = false;
            for (int t = 0; t < 15 && !finished; t++)
            {
                StepOrFinish(sim, Throttle);
                finished = IsFinished(sim);
            }
            Check(finished, "the run must finish after all checkpoints were collected");
            Check(sim.Race().Value.finishTimeMs > 0, "the finish time must be recorded");
        }

        public static void RouteFinishNeedsAllCheckpoints()
        {
            GameTrackSpec trackSpec = FloorTrack(600f, 600f, 0f, true);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.checkpoints.Add(Trigger(0, 5, 30, 5, 5, 5));
            routeSpec.checkpoints.Add(Trigger(0, 5, 60, 5, 5, 5));
            routeSpec.checkpoints.Add(Trigger(0, 5, 90, 5, 5, 5));
            routeSpec.finishes.Add(Trigger(0, 5, 120, 5, 5, 5));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);

            // Collect only checkpoint 1, then cross the finish: rejected.
            GmIso4 spot = SpawnBeforeTrigger(routeSpec.checkpoints[1]);
            sim.World.Respawn(ref spot);
            for (int t = 0; t < 5; t++)
            {
                StepOrFinish(sim, Throttle);
            }
            CheckEqual(1u, sim.Race().Value.visitedCount, "one checkpoint collected");

            GmIso4 finish = SpawnBeforeTrigger(routeSpec.finishes[0]);
            sim.World.Respawn(ref finish);
            for (int t = 0; t < 15; t++)
            {
                StepOrFinish(sim, Throttle);
            }
            Check(!IsFinished(sim),
                "the finish must NOT count while checkpoints are missing");
            CheckEqual(1u, sim.Race().Value.visitedCount,
                "checkpoint progress must survive a rejected finish");

            // Collect the remaining two, then finish.
            foreach (int index in new[] { 0, 2 })
            {
                GmIso4 at = SpawnBeforeTrigger(routeSpec.checkpoints[index]);
                sim.World.Respawn(ref at);
                for (int t = 0; t < 5; t++)
                {
                    StepOrFinish(sim, Throttle);
                }
            }
            CheckEqual(3u, sim.Race().Value.visitedCount, "all checkpoints collected late");

            sim.World.Respawn(ref finish);
            bool finished = false;
            for (int t = 0; t < 15 && !finished; t++)
            {
                StepOrFinish(sim, Throttle);
                finished = IsFinished(sim);
            }
            Check(finished, "the finish must count once every checkpoint is collected");
        }

        public static void RouteNoCheckpointsMultipleFinishes()
        {
            GameTrackSpec trackSpec = FloorTrack(600f, 600f, 0f, true);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.finishes.Add(Trigger(0, 5, 120, 5, 5, 5));
            routeSpec.finishes.Add(Trigger(60, 5, 180, 5, 5, 5));

            RouteLoader.TmnfRoute route = LoadRoute(routeSpec);
            CheckEqual(0u, route.metadata.checkpointCount, "no checkpoints");
            CheckEqual(2u, route.metadata.finishCount, "two finishes");

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);

            // The second finish object works on a checkpoint-less track.
            GmIso4 second = SpawnBeforeTrigger(routeSpec.finishes[1]);
            sim.World.Respawn(ref second);
            bool finished = false;
            for (int t = 0; t < 15 && !finished; t++)
            {
                StepOrFinish(sim, Throttle);
                finished = IsFinished(sim);
            }
            Check(finished, "a checkpoint-less track must finish through any finish object");
        }

        public static void RouteValidationRejects()
        {
            // No finish.
            GameRouteSpec empty = new GameRouteSpec();
            try
            {
                TmnfRouteImage.Build(empty);
                Check(false, "a route without finishes must be rejected");
            }
            catch (InvalidOperationException e)
            {
                Check(e.Message.Contains("finish"),
                    "the rejection must name the missing finish");
            }

            // Too many checkpoints.
            GameRouteSpec many = new GameRouteSpec();
            many.finishes.Add(Trigger(0, 5, 10, 1, 1, 1));
            for (int i = 0; i < 64; i++)
            {
                many.checkpoints.Add(Trigger(0, 5, 10 + 5 * i, 1, 1, 1));
            }
            try
            {
                TmnfRouteImage.Build(many);
                Check(false, "64 checkpoints + 1 finish must be rejected");
            }
            catch (InvalidOperationException)
            {
                // expected
            }

            // Degenerate trigger.
            GameRouteSpec flat = new GameRouteSpec();
            flat.finishes.Add(Trigger(0, 5, 10, 0, 1, 1));
            try
            {
                TmnfRouteImage.Build(flat);
                Check(false, "a flat trigger must be rejected");
            }
            catch (InvalidOperationException)
            {
                // expected
            }

            // Empty track.
            try
            {
                TmnfTrackImage.Build(new GameTrackSpec());
                Check(false, "an empty track must be rejected");
            }
            catch (InvalidOperationException e)
            {
                Check(e.Message.Contains("meshes"),
                    "the rejection must name the missing geometry");
            }
        }

        // ==================================================================
        // 4. restart leaves no stale state
        // ==================================================================

        public static void RestartFromSameImagesIsClean()
        {
            GameTrackSpec trackSpec = FloorTrack(600f, 600f, 0f, true);
            GameRouteSpec routeSpec = new GameRouteSpec();
            routeSpec.checkpoints.Add(Trigger(0, 5, 30, 5, 5, 5));
            routeSpec.finishes.Add(Trigger(0, 5, 600, 20, 20, 20));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask,
                out byte[] trackBytes, out byte[] routeBytes);

            GmIso4 start = StartAt(0, 1, 0);
            sim.World.Respawn(ref start);
            for (int t = 0; t < 250; t++)
            {
                StepOrFinish(sim, Throttle);
            }
            VehicleState driven = sim.State();
            Check(driven.position.z > 25f, "the car must have driven before the restart");
            Race.TmnfRaceState raced = sim.Race().Value;
            Check(raced.elapsedTicks > 200, "the race clock must have advanced");

            // Full restart: fresh track/route objects from the same bytes
            // (exactly what the game's reset does).
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            sim = new Sim(track, vehicle, mask, new SimOptions
            {
                fakeContactMask = null,
                routeBytes = routeBytes,
                pinProvenance = false,
            });
            sim.World.Respawn(ref start);

            VehicleState fresh = sim.State();
            Check(Math.Abs(fresh.position.x) < 0.01f
                && Math.Abs(fresh.position.z) < 0.01f,
                $"the restarted car must be back at the start ({fresh.position.x}, {fresh.position.z})");
            Check(fresh.linearSpeed.x == 0f && fresh.linearSpeed.y == 0f
                && fresh.linearSpeed.z == 0f,
                "the restarted car must have no residual velocity");
            Check(fresh.angularSpeed.x == 0f && fresh.angularSpeed.y == 0f
                && fresh.angularSpeed.z == 0f,
                "the restarted car must have no residual angular velocity");
            Race.TmnfRaceState freshRace = sim.Race().Value;
            CheckEqual(0u, freshRace.elapsedTicks, "the restarted race clock must be zero");
            CheckEqual(0u, freshRace.visitedCheckpoints, "no checkpoints may survive");
            CheckEqual(0u, freshRace.visitedCount, "no checkpoint count may survive");
            Check(freshRace.finished == 0, "the restarted race must not be finished");

            // It must still be drivable afterwards (300+ ticks crosses the
            // checkpoint face at z=25 — the golden schedule reaches 53 m).
            VehicleState state = default;
            for (int t = 0; t < 320; t++)
            {
                state = StepOrFinish(sim, Throttle);
            }
            Check(state.position.z > 30f, "the restarted car must drive again");
            CheckEqual(1u, sim.Race().Value.visitedCount,
                "the checkpoint at z=30 must be collected on the restarted run");
        }

        // ==================================================================
        // 4b. the shipped sample track, played end to end
        // ==================================================================

        /// <summary>
        /// The geometry of Assets/Scenes/Tracks/SampleTrack01.unity exactly
        /// as the extractor would read it (Unity primitives with the scene's
        /// transforms, mirrored into TMNF space): a 400 m floor, walls at
        /// x = +-9, an 8-degree ramp at z = -40, checkpoints at z = +60 and
        /// -20, finish at -150, spawn at (0, 1, 170) facing -Z.
        /// </summary>
        private static GameTrackSpec SampleTrack01Spec()
        {
            GameTrackSpec spec = new GameTrackSpec();
            spec.meshes.Add(FloorQuad(200f, 200f, 0f, true));

            // Walls: cubes half (0.5, 1, 180) at (±9, 1, 0).
            foreach (float side in new[] { -1f, 1f })
            {
                spec.meshes.Add(BoxMesh(
                    new float[] { side * 9f, 1f, 0f },
                    new float[] { 0.5f, 1f, 180f },
                    0f));
            }

            // Ramp: cube half (8, 0.25, 9) at Unity (0, 0.9, 40), rotated
            // -8 degrees about X — the same vertices TransformPoint +
            // mirror would produce.
            spec.meshes.Add(BoxMesh(
                new float[] { 0f, 0.9f, -40f },
                new float[] { 8f, 0.25f, 9f },
                8f));

            return spec;
        }

        /// <summary>
        /// A box mesh (12 triangles) with center/half extents/X-rotation,
        /// all in TMNF space (the scene's Unity transforms are mirrored
        /// before they get here, exactly like the extractor does).
        /// </summary>
        private static GameMeshSpec BoxMesh(
            float[] center, float[] half, float rotationDegreesX)
        {
            double rad = Math.PI / 180.0 * rotationDegreesX;
            double c = Math.Cos(rad), s = Math.Sin(rad);

            float[] v = new float[8 * 3];
            for (int corner = 0; corner < 8; corner++)
            {
                double x = ((corner & 1) == 0 ? -1 : 1) * half[0];
                double y = ((corner & 2) == 0 ? -1 : 1) * half[1];
                double z = ((corner & 4) == 0 ? -1 : 1) * half[2];
                double y1 = y * c - z * s;
                double z1 = y * s + z * c;
                v[corner * 3 + 0] = (float)(x + center[0]);
                v[corner * 3 + 1] = (float)(y1 + center[1]);
                v[corner * 3 + 2] = (float)(z1 + center[2]);
            }

            // Corners: bit0 = +x, bit1 = +y, bit2 = +z. Windings verified
            // against the right-hand rule so the normals point outward.
            uint[] mesh_tris = new uint[]
            {
                2, 0, 4, 2, 4, 6,     // -X
                1, 3, 7, 1, 7, 5,     // +X
                0, 1, 5, 0, 5, 4,     // -Y
                3, 2, 6, 3, 6, 7,     // +Y
                0, 3, 1, 0, 2, 3,     // -Z
                4, 5, 7, 4, 7, 6,     // +Z
            };
            // Base normals per face, rotated with the box.
            double[][] baseNormals =
            {
                new[] { -1.0, 0.0, 0.0 },
                new[] { 1.0, 0.0, 0.0 },
                new[] { 0.0, -1.0, 0.0 },
                new[] { 0.0, 1.0, 0.0 },
                new[] { 0.0, 0.0, -1.0 },
                new[] { 0.0, 0.0, 1.0 },
            };
            float[] normals = new float[36];
            for (int face = 0; face < 6; face++)
            {
                double nx = baseNormals[face][0];
                double ny = baseNormals[face][1];
                double nz = baseNormals[face][2];
                double ny1 = ny * c - nz * s;
                double nz1 = ny * s + nz * c;
                normals[face * 6 + 0] = (float)nx;
                normals[face * 6 + 1] = (float)ny1;
                normals[face * 6 + 2] = (float)nz1;
                normals[face * 6 + 3] = (float)nx;
                normals[face * 6 + 4] = (float)ny1;
                normals[face * 6 + 5] = (float)nz1;
            }

            GameMeshSpec mesh = new GameMeshSpec();
            mesh.name = "box";
            mesh.vertices = v;
            mesh.triangles = mesh_tris;
            mesh.faceNormals = normals;
            mesh.materialId = 16; // asphalt, like the sample's TrackGeometry
            return mesh;
        }


        public static void SampleTrack01Playable()
        {
            GameTrackSpec trackSpec = SampleTrack01Spec();
            GameRouteSpec routeSpec = new GameRouteSpec();
            // Unity (0,2,-60) -> TMNF (0,2,60); (0,2,20) -> (0,2,-20);
            // (0,2,150) -> (0,2,-150). Spawn: (0,1,-170) -> (0,1,170),
            // facing Unity +Z -> the car drives toward TMNF -Z.
            routeSpec.startIso[9] = 0f;
            routeSpec.startIso[10] = 1f;
            routeSpec.startIso[11] = 170f;
            routeSpec.startIso[0] = -1f;  // RotY180: car forward -> -Z
            routeSpec.startIso[4] = 1f;
            routeSpec.startIso[8] = -1f;
            routeSpec.checkpoints.Add(Trigger(0, 2, 60, 9, 5, 5));
            routeSpec.checkpoints.Add(Trigger(0, 2, -20, 9, 5, 5));
            routeSpec.finishes.Add(Trigger(0, 2, -150, 9, 5, 5));

            byte[] vehicle = TmnfFixtures.StadiumVehicle();
            byte[] mask = TmnfFixtures.GameMask();
            Sim sim = BuildSim(trackSpec, routeSpec, vehicle, mask, out _, out _);

            GmIso4 start = StartAt(0, 1, 170);
            // Facing -Z (the sample's spawn orientation).
            start.m0 = -1f;
            start.m8 = -1f;
            sim.World.Respawn(ref start);

            // Full throttle down the straight, over the ramp, through both
            // checkpoints, across the finish.
            bool finished = false;
            VehicleState state = default;
            for (int t = 0; t < 4000 && !finished; t++)
            {
                state = StepOrFinish(sim, Throttle);
                finished = IsFinished(sim);
                if (t % 500 == 0)
                {
                    Race.TmnfRaceState race = sim.Race().Value;
                    System.Console.WriteLine(
                        $"  [sample01] t={t} z={state.position.z:F1} y={state.position.y:F2} " +
                        $"cps={race.visitedCount}");
                }
            }
            Check(finished, "the sample track must be completable at full throttle");
            CheckEqual(2u, sim.Race().Value.visitedCount,
                "both sample checkpoints must be collected on the way");
            Check(sim.Race().Value.finishTimeMs > 0, "the finish time must be recorded");
            System.Console.WriteLine($"  [sample01] finished in {sim.Race().Value.finishTimeMs} ms");
        }

        // ==================================================================
        // 5. timing + personal bests
        // ==================================================================

        public static void TimeFormatting()
        {
            CheckEqual("00:42.381", RaceTimeFormat.Time(42381), "time format");
            CheckEqual("12:05.000", RaceTimeFormat.Time(725000), "time format (minutes)");
            CheckEqual("00:00.000", RaceTimeFormat.Time(-5), "negative times clamp to zero");
            CheckEqual("-00:01.250", RaceTimeFormat.Delta(-1250), "negative delta");
            CheckEqual("+00:01.250", RaceTimeFormat.Delta(1250), "positive delta");
            CheckEqual("+00:00.000", RaceTimeFormat.Delta(0), "zero delta");
        }

        public static void PersonalBestLogic()
        {
            PersonalBestData data = new PersonalBestData();
            Check(data.Get("track") < 0, "no best exists initially");
            Check(data.Submit("track", 50000), "a first finish must set the best");
            Check(!data.Submit("track", 60000), "a slower run must not overwrite");
            Check(!data.Submit("track", 50000), "an equal run must not overwrite");
            Check(data.Submit("track", 49000), "a faster run must overwrite");
            CheckEqual(49000, data.Get("track"), "the best time must be the minimum");

            string json = data.ToJson();
            PersonalBestData roundTrip = PersonalBestData.FromJson(json);
            CheckEqual(49000, roundTrip.Get("track"), "the JSON round-trip must keep entries");

            PersonalBestData escaped = new PersonalBestData();
            escaped.Submit("Assets/Tracks/My \"Weird\" Track.unity", 12345);
            PersonalBestData parsed = PersonalBestData.FromJson(escaped.ToJson());
            CheckEqual(12345, parsed.Get("Assets/Tracks/My \"Weird\" Track.unity"),
                "quoted keys must round-trip");

            try
            {
                PersonalBestData.FromJson("{\"a\":1,,}");
                Check(false, "malformed JSON must be rejected");
            }
            catch (FormatException)
            {
                // expected
            }
        }
    }
}
