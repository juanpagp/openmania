using System;
using System.Collections.Generic;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>
    /// The golden harness: builds deterministic test states from the fixture
    /// image + a seed, mirroring the Rust generator's build_test_* helpers
    /// field-for-field.
    /// </summary>
    public static class GoldenHarness
    {
        public static Harness Create()
        {
            return new Harness(
                GoldenRunner.GoldenPath("fixtures/stadium.tmnfvehicle"),
                System.IO.File.ReadAllBytes(GoldenRunner.GoldenPath("fixtures/game-mask.bin")));
        }
    }

    /// <summary>The fixture bundle (image + mask) with seeded builders.</summary>
    public sealed class Harness
    {
        public VehicleTuning tuning;
        public Vehicle baseVehicle;
        public CHmsDynaParams baseParams;
        public CHmsStateDyna baseState;
        public byte[] mask;
        public byte[] vehicleBlob;

        /// <summary>The fixed-seed LCG (identical to the Rust generator).</summary>
        public sealed class Lcg
        {
            private ulong state;

            public Lcg(ulong seed) => state = seed;

            public uint NextU32()
            {
                state = unchecked(state * 6364136223846793005UL + 1442695040888963407UL);
                return (uint)(state >> 33);
            }

            /// <summary>A uniform u64: high half first, then low half.</summary>
            public ulong NextU64()
            {
                return ((ulong)NextU32() << 32) | NextU32();
            }

            public float NextF32Physical()
            {
                uint bits = NextU32();
                uint mantissa = bits & 0x007f_ffff;
                int exponent = (int)(((bits >> 24) % 41 + 41) % 41) - 20;
                uint sign = (bits >> 31) & 1;
                return BitConverter.Int32BitsToSingle(
                    unchecked((int)((sign << 31) | ((uint)(exponent + 127) << 23) | mantissa)));
            }

            public float NextF32Range(float scale)
            {
                float unit = (float)((double)NextU32() / 65536.0 / 65536.0);
                return (unit * 2.0f - 1.0f) * scale;
            }
        }

        public Harness(string vehiclePath, byte[] maskBytes)
        {
            vehicleBlob = System.IO.File.ReadAllBytes(vehiclePath);
            (tuning, baseVehicle) = World.DecodeTuningFromBlob(vehicleBlob);
            // Bind the fake-contact mask like World does.
            mask = maskBytes;
            // (mask binding: ground materials with a mask flag get the image)
            // The generator binds masks to every material with a null mask;
            // the fixture image marks exactly the road/grass/rubber rows.
            // DecodeTuning leaves them null; bind where the record says so:
            BindMasksFromImage();
            // Fill the wheel/body tree wiring exactly like the world does:
            // decode leaves wheelTreeRefs empty; the world sizes it to 4
            // zeros first, then patches per collision-tree record.
            tuning.wheelTreeRefs = new List<uint> { 0, 0, 0, 0 };
            foreach (World.VehicleCollisionTreeRecord raw in World.ValidateVehicleGraph(vehicleBlob, null, false).collisionChildren)
            {
                uint kind = raw.kind & 0xff;
                if (kind == 1)
                {
                    tuning.wheelTreeRefs[(int)raw.wheelIndex] = raw.objectId;
                }
                else
                {
                    tuning.bodyTreeRefs.Add(raw.objectId);
                }
            }
            World.VehicleSnapshotHeader header = World.VehicleSnapshotHeader.FromBytes(vehicleBlob);
            byte[] pb = new byte[0x5c];
            Array.Copy(vehicleBlob, (int)header.paramsOffset, pb, 0, 0x5c);
            baseParams = CHmsDynaParams.FromBytes(pb);
            byte[] sb = new byte[180];
            Array.Copy(vehicleBlob, (int)header.stateOffset, sb, 0, 180);
            baseState = CHmsStateDyna.FromBytes(sb);
        }

        private void BindMasksFromImage()
        {
            // The generator binds the shared mask to EVERY material whose
            // mask is still null after decode — and decode leaves them ALL
            // null — so effectively every ground material carries the mask.
            // (World::new's flag-based binding is narrower; the vehicle
            // suites were generated under the broader generator binding.)
            for (int i = 0; i < tuning.groundMaterials.Count; i++)
            {
                if (tuning.groundMaterials[i].fakeContactMask == null)
                {
                    tuning.groundMaterials[i].fakeContactMask = mask;
                }
            }
        }

        /// <summary>Deterministically drifts the vehicle state from the base.</summary>
        public Vehicle BuildVehicle(ulong seed)
        {
            Lcg lcg = new Lcg(seed ^ 0x5eed_0000_0000_0001UL);
            Vehicle v = CloneVehicle(baseVehicle);
            int mode = (int)(lcg.NextU32() % 4);
            v.inputGas = Math.Max(lcg.NextF32Range(1.0f), 0.0f);
            v.inputBrake = Math.Max(lcg.NextF32Range(1.0f), 0.0f);
            v.inputSteer = lcg.NextF32Range(1.0f);
            v.steeringValue = lcg.NextF32Range(1.0f);
            v.engine.rpm = lcg.NextF32Range(8000.0f);
            v.engine.targetRpm = lcg.NextF32Range(8000.0f);
            v.engine.clutch = lcg.NextF32Range(0.4f) + 0.8f;
            v.engine.shiftTimer = lcg.NextF32Range(0.06f);
            v.engine.gear = (int)(lcg.NextU32() % 6);
            v.engine.reverse = (int)(lcg.NextU32() % 2);
            v.engineMode = (int)(lcg.NextU32() % 5);
            v.driveMode = (int)(lcg.NextU32() % 4);
            v.currentLocalSpeed = new GmVec3(
                lcg.NextF32Range(10.0f), lcg.NextF32Range(5.0f), lcg.NextF32Range(60.0f));
            v.integrationFlags = 7;
            v.turboActive = (int)(lcg.NextU32() % 2);
            v.turboType = (TMNFVehicleTurboType)(lcg.NextU32() % 3);
            v.turboFactor = lcg.NextF32Range(5.0f);
            v.turboStartTick = lcg.NextU32() % 1000;
            v.turboEndTick = v.turboStartTick + (lcg.NextU32() % 400);
            v.turboProgress = lcg.NextF32Range(1.0f);
            v.rouletteToken = lcg.NextU32() % 4;
            v.rouletteValue = lcg.NextF32Range(1.0f);
            v.rouletteModulus = 7;
            v.turboEpochTick = lcg.NextU32() % 100;
            v.tickTime = lcg.NextU32() % 100_000;
            v.airControlImmediate = (int)(lcg.NextU32() % 2);
            v.airControlLocked = (int)(lcg.NextU32() % 2);
            v.airControlTick = lcg.NextU32() % 1000;
            v.airControlSpeed = new GmVec3(
                lcg.NextF32Range(5.0f), lcg.NextF32Range(5.0f), lcg.NextF32Range(5.0f));
            v.sideContact = (int)(lcg.NextU32() % 2);
            v.lastSideContactTick = lcg.NextU32() % 1000;
            v.airborneFrictionGate = (int)(lcg.NextU32() % 2);
            v.frictionInputSelector = (int)(lcg.NextU32() % 2);
            v.flag60c = (lcg.NextU32() % 4 == 0) ? 1 : 0;
            v.forceWheelSpeed = (int)(lcg.NextU32() % 2);
            v.blockWheelSpeed = (lcg.NextU32() % 4 == 0) ? 1 : 0;
            v.forcedWheelSpeed = lcg.NextF32Range(50.0f);
            v.engine.brakingFactor = lcg.NextF32Range(5.0f);
            v.model6.reverseMode = (int)(lcg.NextU32() % 2);
            v.model6.contactBlockCount = (int)(lcg.NextU32() % 2);
            v.model6.sideContact = (int)(lcg.NextU32() % 2);
            v.model6.lastSlidingTick = lcg.NextU32() % 1000;
            v.model6.slidingStartTick = lcg.NextU32() % 1000;
            v.model6.rolloverAxis = new GmVec3(
                lcg.NextF32Range(1.0f), lcg.NextF32Range(1.0f), lcg.NextF32Range(1.0f));
            v.model6.orbitCenter = new GmVec3(
                lcg.NextF32Range(3.0f), lcg.NextF32Range(1.0f), lcg.NextF32Range(3.0f));
            v.model6.orbitRadius = lcg.NextF32Range(4.0f) + 0.5f;
            v.model6.orbitInitialRadius = lcg.NextF32Range(4.0f) + 0.5f;
            v.model6.orbitAxis = new GmVec3(
                lcg.NextF32Range(0.2f), 0.9f + lcg.NextF32Range(0.1f), lcg.NextF32Range(0.2f));
            v.model6.orbitSign = lcg.NextF32Range(1.0f);
            v.model6.burnoutStartTick = lcg.NextU32() % 1000;
            v.model6.burnoutTransitionTick = lcg.NextU32() % 1000;
            v.compute.computedBrakeForce = lcg.NextF32Range(10.0f);
            v.compute.contactRise = lcg.NextF32Range(1.0f);
            v.compute.contactDecay = lcg.NextF32Range(1.0f);
            v.compute.effectAccumulator = lcg.NextF32Range(1.0f);
            v.compute.eventMetricA = lcg.NextF32Range(50.0f);
            v.compute.eventMetricB = lcg.NextF32Range(50.0f);
            v.compute.eventMetricC = lcg.NextF32Range(50.0f);
            v.compute.eventLevelA = lcg.NextU32() % 3;
            v.compute.eventLevelB = lcg.NextU32() % 3;
            v.compute.eventLevelC = lcg.NextU32() % 3;
            v.compute.simulationGate = 1.0f;
            foreach (CSceneVehicleCarWheel w in v.wheels)
            {
                w.realTime.damperAbsorb = lcg.NextF32Range(0.4f);
                w.realTime.field04 = lcg.NextF32Range(3.0f);
                w.realTime.field08 = lcg.NextF32Range(0.1f);
                w.realTime.field6c = lcg.NextF32Range(80.0f);
                w.realTime.rotationPhase = lcg.NextF32Range(1600.0f);
                w.realTime.blendValue = lcg.NextF32Range(1.0f);
                w.realTime.blendTarget = lcg.NextF32Range(1.0f);
                w.realTime.hasGroundContact = (lcg.NextU32() % 4 != 0) ? 1 : 0;
                w.realTime.contactMaterialId = mode switch
                {
                    0 => (int)(lcg.NextU32() % 31),
                    1 => 0,
                    2 => 7,
                    _ => 6,
                };
                w.realTime.isSliding = (int)(lcg.NextU32() % 2);
                w.realTime.groundContactCount = (int)(lcg.NextU32() % 3);
                w.realTime.field54 = new GmVec3(
                    lcg.NextF32Range(2.0f), lcg.NextF32Range(0.5f), lcg.NextF32Range(2.0f));
                w.realTime.field90 = new GmVec3(
                    lcg.NextF32Range(0.4f), 0.9f + lcg.NextF32Range(0.2f), lcg.NextF32Range(0.4f));
                w.realTime.relativeRotzAxis = new GmVec3(
                    lcg.NextF32Range(0.2f), 0.9f + lcg.NextF32Range(0.2f), lcg.NextF32Range(0.2f));
                w.contactBody = (lcg.NextU32() % 2 == 0) ? 1 : (uint?)null;
            }
            return v;
        }

        /// <summary>Deterministically drifts the dyna from the base.</summary>
        public CHmsDyna BuildDyna(ulong seed)
        {
            Lcg lcg = new Lcg(seed ^ 0x5eed_0000_0000_0002UL);
            CHmsDynaParams parameters = baseParams;
            parameters.mass = lcg.NextF32Range(1500.0f) + 0.5f;
            CHmsStateDyna state = baseState;
            state.pos = new GmVec3(
                lcg.NextF32Range(200.0f), lcg.NextF32Range(30.0f), lcg.NextF32Range(200.0f));
            state.linVel = new GmVec3(
                lcg.NextF32Range(40.0f), lcg.NextF32Range(20.0f), lcg.NextF32Range(60.0f));
            state.angVel = new GmVec3(
                lcg.NextF32Range(8.0f), lcg.NextF32Range(8.0f), lcg.NextF32Range(8.0f));
            state.force = new GmVec3(
                lcg.NextF32Range(8000.0f), lcg.NextF32Range(8000.0f), lcg.NextF32Range(8000.0f));
            state.torque = new GmVec3(
                lcg.NextF32Range(800.0f), lcg.NextF32Range(800.0f), lcg.NextF32Range(800.0f));
            CHmsDyna dyna = new CHmsDyna(parameters, 1, 0, 0.0f);
            dyna.SetLive(state);
            dyna.dirtyFlag = 1;
            return dyna;
        }

        /// <summary>Full canonical vehicle dump (game-state blocks + extras).</summary>
        public static byte[] DumpVehicle(Harness h, Vehicle v)
        {
            var (car, wheels) = GameStateExport.DumpVehicleGameState(h.vehicleBlob, v);
            var o = new List<byte>();
            o.AddRange(car);
            o.AddRange(wheels);
            ByteWriter.U32(o, v.tickTime);
            ByteWriter.U32(o, v.rouletteModulus);
            foreach (CSceneVehicleCarWheel w in v.wheels)
            {
                GmGoldenTests.WriteVec3To(o, w.impulsePoint);
                ByteWriter.I32(o, w.contactBody.HasValue ? (int)w.contactBody.Value : -1);
            }
            ByteWriter.U8(o, v.turboSoundAttached ? (byte)1 : (byte)0);
            return o.ToArray();
        }

        /// <summary>Full canonical dyna dump.</summary>
        public static byte[] DumpDyna(CHmsDyna d)
        {
            var o = new List<byte>();
            o.AddRange(d.@params.ToBytes());
            o.AddRange(d.Live().ToBytes());
            o.AddRange(d.StateB().ToBytes());
            o.AddRange(d.tempState.ToBytes());
            ByteWriter.I32(o, d.clampAngular);
            ByteWriter.F32(o, d.maxAngularSpeed);
            ByteWriter.I32(o, d.dirtyFlag);
            ByteWriter.I32(o, d.mode);
            ByteWriter.U32(o, (uint)d.replacementBuf.Count);
            foreach (GmVec3 v in d.replacementBuf)
            {
                GmGoldenTests.WriteVec3To(o, v);
            }
            return o.ToArray();
        }

        /// <summary>Deep vehicle clone (the reference's derived Clone).</summary>
        private static Vehicle CloneVehicle(Vehicle src)
        {
            Vehicle v = new Vehicle();
            v.inputGas = src.inputGas;
            v.inputBrake = src.inputBrake;
            v.inputSteer = src.inputSteer;
            v.wheelCount = src.wheelCount;
            v.engine = src.engine;
            v.currentLocalSpeed = src.currentLocalSpeed;
            v.totalForceAdded = src.totalForceAdded;
            v.totalImpulseAdded = src.totalImpulseAdded;
            v.engineMode = src.engineMode;
            v.turboActive = src.turboActive;
            v.driveMode = src.driveMode;
            v.engineLimitFlag = src.engineLimitFlag;
            v.gearDownshiftFlag = src.gearDownshiftFlag;
            v.forceWheelSpeed = src.forceWheelSpeed;
            v.flag60c = src.flag60c;
            v.blockWheelSpeed = src.blockWheelSpeed;
            v.forcedWheelSpeed = src.forcedWheelSpeed;
            v.integrationFlags = src.integrationFlags;
            v.turboEpochTick = src.turboEpochTick;
            v.airControlImmediate = src.airControlImmediate;
            v.airControlLocked = src.airControlLocked;
            v.steeringValue = src.steeringValue;
            v.turboProgress = src.turboProgress;
            v.turboFactor = src.turboFactor;
            v.turboStartTick = src.turboStartTick;
            v.turboEndTick = src.turboEndTick;
            v.turboType = src.turboType;
            v.rouletteToken = src.rouletteToken;
            v.rouletteValue = src.rouletteValue;
            v.airControlTick = src.airControlTick;
            v.airControlSpeed = src.airControlSpeed;
            v.rouletteModulus = src.rouletteModulus;
            v.bodyBox = src.bodyBox;
            v.turboSoundAttached = src.turboSoundAttached;
            v.frictionInputSelector = src.frictionInputSelector;
            v.sideContact = src.sideContact;
            v.lastSideContactTick = src.lastSideContactTick;
            v.airborneFrictionGate = src.airborneFrictionGate;
            v.wheelContactAbsorbCount = src.wheelContactAbsorbCount;
            v.bodyContactCount = src.bodyContactCount;
            v.bodyContactPositionSum = src.bodyContactPositionSum;
            v.bodyContactNormalSum = src.bodyContactNormalSum;
            v.tickTime = src.tickTime;
            v.model6.pivotPosition = src.model6.pivotPosition;
            v.model6.pivotAxis = src.model6.pivotAxis;
            v.model6.reverseMode = src.model6.reverseMode;
            v.model6.reverseSpeedThreshold = src.model6.reverseSpeedThreshold;
            v.model6.contactBlockCount = src.model6.contactBlockCount;
            v.model6.sideContact = src.model6.sideContact;
            v.model6.lastSlidingTick = src.model6.lastSlidingTick;
            v.model6.slidingStartTick = src.model6.slidingStartTick;
            v.model6.slidingElapsedTicks = src.model6.slidingElapsedTicks;
            v.model6.modelIso = src.model6.modelIso;
            v.model6.rolloverAxis = src.model6.rolloverAxis;
            v.model6.orbitCenter = src.model6.orbitCenter;
            v.model6.orbitInitialRadius = src.model6.orbitInitialRadius;
            v.model6.orbitRadius = src.model6.orbitRadius;
            v.model6.burnoutStartTick = src.model6.burnoutStartTick;
            v.model6.burnoutTransitionTick = src.model6.burnoutTransitionTick;
            v.model6.orbitAxis = src.model6.orbitAxis;
            v.model6.orbitSign = src.model6.orbitSign;
            v.model6.axleWidth = src.model6.axleWidth;
            v.compute.simulationGate = src.compute.simulationGate;
            v.compute.eventLevelC = src.compute.eventLevelC;
            v.compute.eventSourceC = src.compute.eventSourceC;
            v.compute.eventSourceAb = src.compute.eventSourceAb;
            v.compute.springC = src.compute.springC;
            v.compute.springA = src.compute.springA;
            v.compute.contactRise = src.compute.contactRise;
            v.compute.contactDecay = src.compute.contactDecay;
            v.compute.historyForceLimit = src.compute.historyForceLimit;
            v.compute.historyForceScale = src.compute.historyForceScale;
            v.compute.springValueLimit = src.compute.springValueLimit;
            v.compute.localSpeedLimit = src.compute.localSpeedLimit;
            v.compute.airEffectThreshold = src.compute.airEffectThreshold;
            v.compute.brakeInputScale = src.compute.brakeInputScale;
            v.compute.groundedDragScale = src.compute.groundedDragScale;
            v.compute.computedBrakeForce = src.compute.computedBrakeForce;
            v.compute.state5d8 = src.compute.state5d8;
            v.compute.airImpulseCooldownTick = src.compute.airImpulseCooldownTick;
            v.compute.effectAccumulator = src.compute.effectAccumulator;
            v.compute.lastForceTick = src.compute.lastForceTick;
            v.compute.eventLevelA = src.compute.eventLevelA;
            v.compute.eventLevelB = src.compute.eventLevelB;
            v.compute.peakEventLevelB = src.compute.peakEventLevelB;
            v.compute.peakEventLevelA = src.compute.peakEventLevelA;
            v.compute.peakEventLevelC = src.compute.peakEventLevelC;
            v.compute.peakEventSourceAb = src.compute.peakEventSourceAb;
            v.compute.peakEventSourceC = src.compute.peakEventSourceC;
            v.compute.eventMetricA = src.compute.eventMetricA;
            v.compute.eventMetricB = src.compute.eventMetricB;
            v.compute.eventMetricC = src.compute.eventMetricC;
            v.compute.normalizedForce = src.compute.normalizedForce;
            v.compute.airEffectMode = src.compute.airEffectMode;
            foreach (CSceneVehicleCarWheel w in src.wheels)
            {
                CSceneVehicleCarWheel c = new CSceneVehicleCarWheel();
                c.active = w.active;
                c.steerable = w.steerable;
                c.radius = w.radius;
                c.surfaceSource = w.surfaceSource;
                c.surfaceLocation = w.surfaceLocation;
                c.field70 = (float[])w.field70.Clone();
                c.fielda0 = w.fielda0;
                c.fielda4 = w.fielda4;
                c.offsetFromVehicle = w.offsetFromVehicle;
                c.realTime = w.realTime;
                c.realTime.reserved60 = (byte[])w.realTime.reserved60.Clone();
                c.field15c = w.field15c;
                c.contactRelativeLocalDistance = w.contactRelativeLocalDistance;
                c.impulsePoint = w.impulsePoint;
                c.contactBody = w.contactBody;
                c.previousSync.bytes = (byte[])w.previousSync.bytes.Clone();
                c.sync.bytes = (byte[])w.sync.bytes.Clone();
                c.field234.bytes = (byte[])w.field234.bytes.Clone();
                c.asyncState.bytes = (byte[])w.asyncState.bytes.Clone();
                v.wheels.Add(c);
            }
            return v;
        }
    }
}
