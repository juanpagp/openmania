using System;
using System.Collections.Generic;
using NUnit.Framework;
using Tmnf.Sim;

namespace Tmnf.Sim.Tests
{
    /// <summary>Golden suites for the vehicle layer (all submodules).</summary>
    [TestFixture]
    public class VehicleGoldenTests
    {
        private static Harness Harness => GoldenHarness.Create();

        [Test]
        public void CurveCompile()
        {
            GoldenRunner.RunSuite("vehicle/curve_compile.tgn", input =>
            {
                var r = new ByteReader(input);
                uint count = r.U32();
                CFuncKeys keys = new CFuncKeys();
                keys.count = count;
                keys.positions = new float[count];
                for (int i = 0; i < (int)count; i++)
                {
                    keys.positions[i] = r.F32();
                }
                Curve.CfuncKeysCompile(keys);
                var o = new List<byte>();
                foreach (float l in keys.lowerBounds)
                {
                    ByteWriter.F32(o, l);
                }
                foreach (float u in keys.upperBounds)
                {
                    ByteWriter.F32(o, u);
                }
                return o.ToArray();
            });
        }

        [Test]
        public void CurveBoundingIndices()
        {
            GoldenRunner.RunSuitePanicAware("vehicle/curve_bounding_indices.tgn", (input, output) =>
            {
                var r = new ByteReader(input);
                CFuncKeysReal curve = ReadCurve(r);
                float position = r.F32();
                uint cached = r.U32();
                bool forward = r.U8() != 0;
                uint lower = cached;
                uint upper = 0xdead_beefu;
                try
                {
                    Curve.CfuncKeysGetBoundingIndices(curve.keys, position, ref lower, ref upper, forward);
                    ByteWriter.U32(output, lower);
                    ByteWriter.U32(output, upper);
                }
                catch (Exception ex) when (IsPanic(ex))
                {
                    throw;
                }
            });
        }

        [Test]
        public void CurveBlend()
        {
            GoldenRunner.RunSuitePanicAware("vehicle/curve_blend.tgn", (input, output) =>
            {
                var r = new ByteReader(input);
                CFuncKeysReal curve = ReadCurve(r);
                float position = r.F32();
                uint cached = r.U32();
                bool forward = r.U8() != 0;
                uint lower = cached;
                uint upper = 0xdead_beefu;
                float blend = -1.0f;
                try
                {
                    bool ok = Curve.CfuncKeysComputeBlendCoef(curve.keys, position, ref lower, ref upper, ref blend, forward);
                    ByteWriter.U8(output, (byte)(ok ? 1 : 0));
                    ByteWriter.U32(output, lower);
                    ByteWriter.U32(output, upper);
                    ByteWriter.F32(output, blend);
                }
                catch (Exception ex) when (IsPanic(ex))
                {
                    throw;
                }
            });
        }

        [Test]
        public void CurveGetValue()
        {
            GoldenRunner.RunSuite("vehicle/curve_get_value.tgn", input =>
            {
                var r = new ByteReader(input);
                CFuncKeysReal curve = ReadCurve(r);
                float position = r.F32();
                uint cached = r.U32();
                uint lower = cached;
                float value = -1.0f;
                Curve.CfuncKeysRealGetValueOut(curve, position, ref value, ref lower);
                uint lower2 = cached;
                float value2 = Curve.CfuncKeysRealGetValue(curve, position, ref lower2);
                var o = new List<byte>();
                ByteWriter.F32(o, value);
                ByteWriter.U32(o, lower);
                ByteWriter.F32(o, value2);
                ByteWriter.U32(o, lower2);
                return o.ToArray();
            });
        }

        [Test]
        public void CurveGetters()
        {
            GoldenRunner.RunSuite("vehicle/curve_getters.tgn", input =>
            {
                var r = new ByteReader(input);
                CSceneVehicleCarTuningCurveSet set = ReadCurveSet(r);
                byte getter = r.U8();
                float speed = r.F32();
                float value = getter switch
                {
                    0 => Curve.CsceneVehicleCarTuningGetAccelFromSpeed(set, speed),
                    1 => Curve.CsceneVehicleCarTuningGetRolloverLateralFromSpeed(set, speed),
                    2 => Curve.CsceneVehicleCarTuningGetMaxSideFrictionFromSpeed(set, speed),
                    3 => Curve.CsceneVehicleCarTuningGetLateralContactSlowDownFromSpeed(set, speed),
                    4 => Curve.CsceneVehicleCarTuningGetSteerSlowDownFromSpeed(set, speed),
                    5 => Curve.CsceneVehicleCarTuningGetRolloverLateralCoefFromAngle(set, speed),
                    6 => Curve.CsceneVehicleCarTuningGetSteerDriveTorqueFromSpeed(set, speed),
                    7 => Curve.CsceneVehicleCarTuningM4GetSteerRadiusFromSpeed(set, speed),
                    8 => Curve.CsceneVehicleCarTuningM4GetMaxFrictionForceFromSpeed(set, speed),
                    9 => Curve.CsceneVehicleCarTuningM5GetAccelFromSpeed(set, speed),
                    10 => Curve.CsceneVehicleCarTuningM5GetSlippingAccelFromSpeed(set, speed),
                    11 => Curve.CsceneVehicleCarTuningM5GetSteerSlowDownFromSpeed(set, speed),
                    12 => Curve.CsceneVehicleCarTuningM5GetLateralContactSlowDownFromSpeed(set, speed),
                    13 => Curve.CsceneVehicleCarTuningGetWaterFrictionFromSpeed(set, speed),
                    14 => Curve.CsceneVehicleCarTuningM6GetModulationFromDamperAbsorbVal(set, speed),
                    15 => Curve.CsceneVehicleCarTuningM6GetRearGearAccelFromSpeed(set, speed),
                    16 => Curve.CsceneVehicleCarTuningM6GetBurnoutRadiusFromSpeed(set, speed),
                    17 => Curve.CsceneVehicleCarTuningM6GetLateralSpeedFromBurnoutRadius(set, speed),
                    18 => Curve.CsceneVehicleCarTuningM6GetBurnoutRolloverFromSpeed(set, speed),
                    19 => Curve.CsceneVehicleCarTuningM6GetDonutRolloverFromSpeed(set, speed),
                    _ => Curve.CsceneVehicleCarTuningM6GetRolloverLateralFromSpeedRatio(set, speed),
                };
                return BitConverter.GetBytes(value);
            });
        }

        [Test]
        public void InputMapper()
        {
            GoldenRunner.RunSuite("vehicle/input_mapper.tgn", input =>
            {
                TMNFRaceInputs packet = TMNFRaceInputs.FromBytes(input);
                Vehicle vehicle = new Vehicle();
                CHmsDyna dyna = new CHmsDyna(DefaultParams(), 1, 0, 0.0f);
                VehicleTuning tuning = new VehicleTuning();
                VehicleCtx ctx = new VehicleCtx(vehicle, tuning, dyna);
                VehicleBase.UpdateVehicleStateFromInputs(ref packet, ref ctx);
                var o = new List<byte>();
                ByteWriter.F32(o, ctx.vehicle.inputGas);
                ByteWriter.F32(o, ctx.vehicle.inputBrake);
                ByteWriter.F32(o, ctx.vehicle.inputSteer);
                return o.ToArray();
            });
        }

        [Test]
        public void WheelRtIntegrate()
        {
            GoldenRunner.RunSuite("vehicle/wheel_rt_integrate.tgn", input =>
            {
                var r = new ByteReader(input);
                CSceneVehicleCarWheelRealTimeState rt = ReadRt(r);
                float dt = r.F32();
                VehicleBase.WheelRealTimeStateIntegrate(ref rt, dt);
                var o = new List<byte>();
                WriteRt(o, ref rt);
                return o.ToArray();
            });
        }

        [Test]
        public void Spring()
        {
            GoldenRunner.RunSuite("vehicle/spring.tgn", input =>
            {
                var r = new ByteReader(input);
                GmSpringFloat s = default(GmSpringFloat);
                s.stiffness = r.F32();
                s.damping = r.F32();
                s.value = r.F32();
                s.target = r.F32();
                s.velocity = r.F32();
                float dt = r.F32();
                VehicleBase.GmSpringFloatIntegrate(ref s, dt);
                return new[]
                {
                    s.stiffness, s.damping, s.value, s.target, s.velocity,
                } is float[] f ? ToBytes(f) : null;
            });
        }

        [Test]
        public void SdnaImpulse()
        {
            GoldenRunner.RunSuite("vehicle/sdna_impulse.tgn", input =>
            {
                var r = new ByteReader(input);
                float mass = r.F32();
                GmMat3 inertia = default(GmMat3);
                for (int i = 0; i < 9; i++) inertia[i] = r.F32();
                float restitution = r.F32();
                GmVec3 relativeSpeed = GmGoldenTests.ReadVec3(r);
                GmVec3 normal = GmGoldenTests.ReadVec3(r);
                GmVec3 lever = GmGoldenTests.ReadVec3(r);
                GmVec3 outv = VehicleBase.SdynaMathComputeImpulse(
                    mass, ref inertia, restitution, ref relativeSpeed, ref normal, ref lever);
                return GmGoldenTests.WriteVec3(outv);
            });
        }

        [Test]
        public void EngineIntegrate()
        {
            GoldenRunner.RunSuite("vehicle/engine_integrate.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                float throttle = r.F32();
                float dt = r.F32();
                bool oldModel = r.U8() != 0;
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleTuning tuning = h.tuning;
                if (oldModel)
                {
                    tuning.@base.engineModel = 3;
                }
                VehicleCtx ctx = new VehicleCtx(vehicle, tuning, dyna);
                VehicleBase.EngineIntegrate(ref ctx, throttle, dt);
                var o = new List<byte>();
                ByteWriter.F32(o, ctx.vehicle.engine.maxRpm);
                ByteWriter.F32(o, ctx.vehicle.engine.brakingFactor);
                ByteWriter.F32(o, ctx.vehicle.engine.rpm);
                ByteWriter.F32(o, ctx.vehicle.engine.targetRpm);
                ByteWriter.F32(o, ctx.vehicle.engine.clutch);
                ByteWriter.F32(o, ctx.vehicle.engine.shiftTimer);
                ByteWriter.I32(o, ctx.vehicle.engine.reverse);
                ByteWriter.I32(o, ctx.vehicle.engine.gear);
                ByteWriter.I32(o, ctx.vehicle.engineMode);
                ByteWriter.I32(o, ctx.vehicle.engineLimitFlag);
                ByteWriter.I32(o, ctx.vehicle.gearDownshiftFlag);
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void IntegrateVehicle()
        {
            GoldenRunner.RunSuite("vehicle/integrate_vehicle.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                float dt = r.F32();
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                VehicleAux.IntegrateVehicle(ref ctx, dt);
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void CreateFakeContacts()
        {
            GoldenRunner.RunSuite("vehicle/create_fake_contacts.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                VehicleAux.CreateFakeContacts(ref ctx, corpus => GmIso4.Identity);
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void Turbo()
        {
            GoldenRunner.RunSuite("vehicle/turbo.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                uint tick = r.U32();
                byte mode = r.U8();
                uint duration = r.U32();
                float factor = r.F32();
                byte turboType = r.U8();
                uint token = r.U32();
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                switch (mode)
                {
                    case 0:
                        VehicleAux.UpdateTurbo(ref ctx, tick);
                        break;
                    default:
                        VehicleAux.EnableTurbo(ref ctx, tick, duration, factor,
                            (TMNFVehicleTurboType)turboType, token);
                        break;
                }
                var o = new List<byte>();
                ByteWriter.F32(o, ctx.vehicle.turboProgress);
                ByteWriter.F32(o, ctx.vehicle.turboFactor);
                ByteWriter.U32(o, ctx.vehicle.turboStartTick);
                ByteWriter.U32(o, ctx.vehicle.turboEndTick);
                ByteWriter.U8(o, (byte)ctx.vehicle.turboType);
                ByteWriter.U32(o, ctx.vehicle.rouletteToken);
                ByteWriter.F32(o, ctx.vehicle.rouletteValue);
                return o.ToArray();
            });
        }

        [Test]
        public void RouletteValue01()
        {
            GoldenRunner.RunSuite("vehicle/roulette_value01.tgn", input =>
            {
                var r = new ByteReader(input);
                uint value = r.U32();
                uint divisor = r.U32();
                return BitConverter.GetBytes(VehicleAux.GetRouletteValue01(value, divisor));
            });
        }

        [Test]
        public void AirControl()
        {
            GoldenRunner.RunSuite("vehicle/air_control.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                GmVec3 angular = GmGoldenTests.ReadVec3(r);
                uint tick = r.U32();
                bool suppress = r.U8() != 0;
                bool reset = r.U8() != 0;
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                VehicleAux.ComputeAirControl(ref ctx, ref angular, tick, suppress, reset);
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void WaterForces()
        {
            // In-water rows PANIC (the upstream decode never binds the aux
            // water-friction curve); the rows carry panic markers.
            GoldenRunner.RunSuitePanicAware("vehicle/water_forces.tgn", (input, output) =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                TmnfTrackWater water = ReadWater(r);
                GmVec3 existing = GmGoldenTests.ReadVec3(r);
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                bool applied = VehicleAux.ApplyWaterForces(ref ctx, water, ref existing);
                ByteWriter.U8(output, (byte)(applied ? 1 : 0));
                output.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                output.AddRange(Harness.DumpDyna(ctx.dyna));
            });
        }

        [Test]
        public void ContactQueries()
        {
            GoldenRunner.RunSuite("vehicle/contact_queries.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                byte query = r.U8();
                GmVec3 normal = GmGoldenTests.ReadVec3(r);
                GmVec3 velocity = GmGoldenTests.ReadVec3(r);
                GmVec3 lateralAxis = GmGoldenTests.ReadVec3(r);
                float load = r.F32();
                bool grounded = r.U8() != 0;
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                var o = new List<byte>();
                switch (query)
                {
                    case 0:
                    {
                        var (a, s) = VehicleContact.GetSlopeAdherence(ref ctx, ref normal);
                        ByteWriter.F32(o, a);
                        ByteWriter.F32(o, s);
                        break;
                    }
                    case 1:
                    {
                        var (vals, has) = VehicleContact.ComputeVehicleGroundMaterialVals(ref ctx);
                        ByteWriter.F32(o, vals.acceleration);
                        ByteWriter.F32(o, vals.braking);
                        ByteWriter.F32(o, vals.steering);
                        ByteWriter.F32(o, vals.lateralGrip);
                        ByteWriter.U8(o, (byte)(has ? 1 : 0));
                        break;
                    }
                    default:
                    {
                        CSceneVehicleMaterialBlendableVals material = default(CSceneVehicleMaterialBlendableVals);
                        material.acceleration = r.F32();
                        material.braking = r.F32();
                        material.steering = r.F32();
                        material.lateralGrip = r.F32();
                        var (f, sliding) = VehicleContact.GetLateralFriction(
                            ref ctx, ref velocity, ref lateralAxis, ref material, load, grounded);
                        ByteWriter.F32(o, f);
                        ByteWriter.U8(o, (byte)(sliding ? 1 : 0));
                        break;
                    }
                }
                return o.ToArray();
            });
        }

        [Test]
        public void AbsorbContact()
        {
            GoldenRunner.RunSuite("vehicle/absorb_contact.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                CHmsPhysicalContact c = CHmsPhysicalContact.Create();
                c.treeRef = r.U32();
                c.otherSurfaceMaterial = r.U16();
                c.normal = GmGoldenTests.ReadVec3(r);
                c.position = GmGoldenTests.ReadVec3(r);
                c.relativeSpeed = GmGoldenTests.ReadVec3(r);
                c.replacement = GmGoldenTests.ReadVec3(r);
                int other = r.I32();
                c.otherBodyCorpusRef = other >= 0 ? (uint)other : (uint?)null;
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                // The generator resolves the other body with
                // GmIso4::default() — Rust's derived Default is ALL ZEROS
                // (not identity!), so the resolver must return a zero iso.
                VehicleContact.AbsorbContact(ref ctx, ref c, corpus => default(GmIso4));
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void FrictionForces()
        {
            GoldenRunner.RunSuite("vehicle/friction_forces.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                GmVec3 velocity = GmGoldenTests.ReadVec3(r);
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                VehicleContact.ApplyFrictionForces(ref ctx, ref velocity);
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        [Test]
        public void ComputeForces()
        {
            GoldenRunner.RunSuite("vehicle/compute_forces.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                float dt = r.F32();
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                Model6External ext = new Model6External
                {
                    modelIsoSource = dyna.Live().RotPosIso(),
                    bodyReferencePosition = new GmVec3(0.0f, 0.4f, 0.3f),
                };
                TmnfTrackWater water = new TmnfTrackWater
                {
                    cellX = 32.0f,
                    cellZ = 32.0f,
                    level = -1000.0f,
                    floor = -1001.0f,
                };
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                VehicleCompute.ComputeExternal external = new VehicleCompute.ComputeExternal
                {
                    ext = ext,
                    water = water,
                    postForce = PostForce,
                    contactToken = corpus => corpus,
                    // GmIso4::default() in the generator = all zeros.
                    bodyIso = corpus => default(GmIso4),
                    fakeContactsActive = false,
                    waterForcesActive = false,
                };
                VehicleCompute.CsceneVehicleCarComputeForces(ref ctx, external, dt);
                var o = new List<byte>();
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        private static void PostForce(VehicleCtx ctx)
        {
            ctx.vehicle.engine.brakingFactor = ctx.vehicle.compute.computedBrakeForce;
        }

        [Test]
        public void Model6Donut()
        {
            // The generator draws the branch decisions and the steering
            // angle from its SUITE-level LCG (seed 0x0500_0000_0000_002b),
            // not from the per-row seed: per row the stream is
            // u64 (seed), [u32 donut?], [u32 burnout? — only if not donut],
            // f32 steering. Replaying it across rows keeps the closure
            // stateful in lockstep with the generator.
            Harness.Lcg suiteLcg = new Harness.Lcg(0x0500_0000_0000_002bUL);
            GoldenRunner.RunSuite("vehicle/model6_donut.tgn", input =>
            {
                var r = new ByteReader(input);
                ulong seed = r.U64();
                float dt = r.F32();
                float steeringAngle = r.F32();
                Harness h = Harness;
                Vehicle vehicle = h.BuildVehicle(seed);
                // Replay the generator's outer-stream draws in order.
                ulong replaySeed = suiteLcg.NextU64();
                if (replaySeed != seed)
                {
                    throw new AssertionException(
                        $"model6_donut: suite LCG out of sync: replay {replaySeed} != row seed {seed}");
                }
                if (suiteLcg.NextU32() % 2 == 0)
                {
                    vehicle.driveMode = 2;
                    vehicle.model6.orbitAxis = new GmVec3(0.0f, 1.0f, 0.0f);
                    vehicle.model6.orbitSign = 1.0f;
                    vehicle.model6.orbitRadius = 3.0f;
                    vehicle.model6.orbitInitialRadius = 2.5f;
                    vehicle.model6.orbitCenter = new GmVec3(1.0f, 0.5f, 2.0f);
                }
                else if (suiteLcg.NextU32() % 2 == 0)
                {
                    vehicle.driveMode = 1; // burnout enter wave
                }
                float replaySteering = suiteLcg.NextF32Range(0.8f);
                if (BitConverter.SingleToInt32Bits(replaySteering)
                    != BitConverter.SingleToInt32Bits(steeringAngle))
                {
                    throw new AssertionException(
                        $"model6_donut: steering replay {replaySteering} != input {steeringAngle}");
                }
                CHmsDyna dyna = h.BuildDyna(seed ^ 0xabcd);
                Model6External ext = new Model6External
                {
                    modelIsoSource = GmIso4.Identity,
                    bodyReferencePosition = new GmVec3(0.0f, 0.4f, 0.3f),
                };
                TmnfTrackWater water = new TmnfTrackWater
                {
                    cellX = 32.0f,
                    cellZ = 32.0f,
                    level = -1000.0f,
                    floor = -1001.0f,
                };
                VehicleCtx ctx = new VehicleCtx(vehicle, h.tuning, dyna);
                GmVec3 localSpeed = ctx.dyna.GetLocalLinearSpeed();
                GmVec3 localAngular = ctx.dyna.GetLocalAngularSpeed();
                GmVec3 existingForce = ctx.dyna.GetLocalForce();
                int sliding = 0;
                float effectPos = 0.0f;
                CSceneVehicleMaterialBlendableVals material = default(CSceneVehicleMaterialBlendableVals);
                material.acceleration = 1.0f;
                material.braking = 1.0f;
                material.steering = 1.0f;
                material.lateralGrip = 1.0f;
                GmVec3 up = new GmVec3(0.0f, 1.0f, 0.0f);
                var slope = VehicleContact.GetSlopeAdherence(ref ctx, ref up);
                VehicleModel6.ComputeForcesModel6(
                    ref ctx, ref ext, water, dt, ref existingForce,
                    slope.Item1, slope.Item2, ref localSpeed, ref localAngular,
                    steeringAngle, true, ref material, ref sliding, ref effectPos);
                var o = new List<byte>();
                ByteWriter.I32(o, sliding);
                ByteWriter.F32(o, effectPos);
                o.AddRange(Harness.DumpVehicle(h, ctx.vehicle));
                o.AddRange(Harness.DumpDyna(ctx.dyna));
                return o.ToArray();
            });
        }

        // ------------------------------------------------------------------
        // helpers
        // ------------------------------------------------------------------

        private static bool IsPanic(Exception ex)
        {
            return ex is IndexOutOfRangeException;
        }

        private static CHmsDynaParams DefaultParams()
        {
            CHmsDynaParams p = default(CHmsDynaParams);
            p.mass = 1.0f;
            p.invInertiaBody = GmMat3.Identity;
            p.substepLen = 0.01f;
            return p;
        }

        private static CFuncKeysReal ReadCurve(ByteReader r)
        {
            CFuncKeysReal curve = new CFuncKeysReal();
            curve.keys.count = r.U32();
            curve.keys.positions = new float[curve.keys.count];
            for (int i = 0; i < (int)curve.keys.count; i++)
            {
                curve.keys.positions[i] = r.F32();
            }
            curve.values = new float[curve.keys.count];
            for (int i = 0; i < (int)curve.keys.count; i++)
            {
                curve.values[i] = r.F32();
            }
            curve.interpolation = r.I32();
            curve.keys.lowerBounds = new float[curve.keys.count];
            for (int i = 0; i < (int)curve.keys.count; i++)
            {
                curve.keys.lowerBounds[i] = r.F32();
            }
            curve.keys.upperBounds = new float[curve.keys.count];
            for (int i = 0; i < (int)curve.keys.count; i++)
            {
                curve.keys.upperBounds[i] = r.F32();
            }
            return curve;
        }

        private static CSceneVehicleCarTuningCurveSet ReadCurveSet(ByteReader r)
        {
            CSceneVehicleCarTuningCurveSet set = new CSceneVehicleCarTuningCurveSet();
            CFuncKeysReal ReadSlot()
            {
                if (r.U8() == 0)
                {
                    return null;
                }
                return ReadCurve(r);
            }
            set.accelFromSpeed = ReadSlot();
            set.rolloverLateralFromSpeed = ReadSlot();
            set.maxSideFrictionFromSpeed = ReadSlot();
            set.lateralContactSlowdownFromSpeed = ReadSlot();
            set.steerSlowdownFromSpeed = ReadSlot();
            set.rolloverLateralCoefFromAngle = ReadSlot();
            set.steerDriveTorqueFromSpeed = ReadSlot();
            set.m4SteerRadiusFromSpeed = ReadSlot();
            set.m4MaxFrictionForceFromSpeed = ReadSlot();
            set.m5SlippingAccelFromSpeed = ReadSlot();
            set.waterFrictionFromSpeed = ReadSlot();
            set.m6DamperModulation = ReadSlot();
            set.m6RearGearAccelFromSpeed = ReadSlot();
            set.m6RolloverLateralFromSpeedRatio = ReadSlot();
            set.m6BurnoutRadiusFromSpeed = ReadSlot();
            set.m6LateralSpeedFromBurnoutRadius = ReadSlot();
            set.m6DonutRolloverFromSpeed = ReadSlot();
            set.m6BurnoutRolloverFromSpeed = ReadSlot();
            set.m5SlippingAccelScale = r.F32();
            set.damperMax = r.F32();
            set.damperMin = r.F32();
            return set;
        }

        private static TmnfTrackWater ReadWater(ByteReader r)
        {
            TmnfTrackWater water = new TmnfTrackWater();
            water.cellX = r.F32();
            water.cellZ = r.F32();
            water.originX = r.F32();
            water.originZ = r.F32();
            water.width = r.U32();
            water.height = r.U32();
            water.defaultCell = r.U8();
            uint cellsLen = r.U32();
            water.cells = r.Bytes((int)cellsLen);
            water.level = r.F32();
            water.floor = r.F32();
            return water;
        }

        private static CSceneVehicleCarWheelRealTimeState ReadRt(ByteReader r)
        {
            CSceneVehicleCarWheelRealTimeState rt = default(CSceneVehicleCarWheelRealTimeState);
            rt.damperAbsorb = r.F32();
            rt.field04 = r.F32();
            rt.field08 = r.F32();
            rt.basis0 = default(GmMat3);
            for (int i = 0; i < 9; i++) rt.basis0[i] = r.F32();
            rt.basis1 = default(GmMat3);
            for (int i = 0; i < 9; i++) rt.basis1[i] = r.F32();
            rt.field54 = GmGoldenTests.ReadVec3(r);
            rt.reserved60 = r.Bytes(0x0c);
            rt.field6c = r.F32();
            rt.hasGroundContact = r.I32();
            rt.contactMaterialId = r.I32();
            rt.isSliding = r.I32();
            rt.relativeRotzAxis = GmGoldenTests.ReadVec3(r);
            rt.contactBodyWord = r.U32();
            rt.groundContactCount = r.I32();
            rt.field90 = GmGoldenTests.ReadVec3(r);
            rt.rotationPhase = r.F32();
            rt.blendValue = r.F32();
            rt.blendTarget = r.F32();
            return rt;
        }

        private static void WriteRt(List<byte> o, ref CSceneVehicleCarWheelRealTimeState rt)
        {
            ByteWriter.F32(o, rt.damperAbsorb);
            ByteWriter.F32(o, rt.field04);
            ByteWriter.F32(o, rt.field08);
            for (int i = 0; i < 9; i++) ByteWriter.F32(o, rt.basis0[i]);
            for (int i = 0; i < 9; i++) ByteWriter.F32(o, rt.basis1[i]);
            GmGoldenTests.WriteVec3To(o, rt.field54);
            o.AddRange(rt.reserved60);
            ByteWriter.F32(o, rt.field6c);
            ByteWriter.I32(o, rt.hasGroundContact);
            ByteWriter.I32(o, rt.contactMaterialId);
            ByteWriter.I32(o, rt.isSliding);
            GmGoldenTests.WriteVec3To(o, rt.relativeRotzAxis);
            ByteWriter.U32(o, rt.contactBodyWord);
            ByteWriter.I32(o, rt.groundContactCount);
            GmGoldenTests.WriteVec3To(o, rt.field90);
            ByteWriter.F32(o, rt.rotationPhase);
            ByteWriter.F32(o, rt.blendValue);
            ByteWriter.F32(o, rt.blendTarget);
        }

        private static byte[] ToBytes(float[] f)
        {
            var o = new List<byte>();
            foreach (float v in f)
            {
                ByteWriter.F32(o, v);
            }
            return o.ToArray();
        }
    }
}
