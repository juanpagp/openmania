using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>World construction, tuning decode, tick, response, respawn.</summary>
    public sealed partial class World
    {
        // ------------------------------------------------------------------
        // helpers over the blob
        // ------------------------------------------------------------------

        private static uint U32At(byte[] b, int o) => BitsUtil.GetU32(b, o);
        private static int I32At(byte[] b, int o) => BitsUtil.GetI32(b, o);
        private static float F32At(byte[] b, int o) => BitsUtil.GetF32(b, o);
        private static GmVec3 Vec3At(byte[] b, int o) => GmVec3.FromBytes(b, o);
        private static GmIso4 Iso4At(byte[] b, int o) => GmIso4.FromBytes(b, o);
        private static GmBoxAligned BoxAt(byte[] b, int o) => GmBoxAligned.FromBytes(b, o);

        private static GmIso4 VehicleTreeWorldIso(VehicleTree node, ref GmIso4 parent)
        {
            if ((node.flags & 4u) == 0)
            {
                return parent;
            }
            GmIso4 result = node.localIso;
            result.Mult(ref parent);
            return result;
        }

        // ------------------------------------------------------------------
        // curve reading
        // ------------------------------------------------------------------

        private static CFuncKeysReal ReadCurveReal(byte[] blob, VehicleCurveDescriptor descriptor)
        {
            int count = (int)descriptor.count;
            float[] positions = new float[count];
            float[] values = new float[count];
            for (int i = 0; i < count; i++)
            {
                positions[i] = F32At(blob, (int)descriptor.positionsOffset + 4 * i);
                values[i] = F32At(blob, (int)descriptor.valuesOffset + 4 * i);
            }
            CFuncKeysReal curve = new CFuncKeysReal();
            curve.keys.count = descriptor.count;
            curve.keys.positions = positions;
            curve.keys.lowerBounds = new float[0];
            curve.keys.upperBounds = new float[0];
            curve.values = values;
            curve.interpolation = descriptor.interpolation;
            Curve.CfuncKeysCompile(curve.keys);
            return curve;
        }

        private static int LookupCurveDescriptor(
            List<VehicleCurveDescriptor> descriptors, uint ownerKind, uint ownerKey, uint fieldOffset)
        {
            for (int i = 0; i < descriptors.Count; i++)
            {
                VehicleCurveDescriptor c = descriptors[i];
                if (c.ownerKind == ownerKind
                    && c.ownerKey == ownerKey
                    && c.fieldOffset == fieldOffset)
                {
                    return i;
                }
            }
            return -1;
        }

        // ------------------------------------------------------------------
        // decode_tuning
        // ------------------------------------------------------------------

        /// <summary>decode_tuning: the tuning + initial vehicle from the graph.</summary>
        internal static (VehicleTuning, Vehicle) DecodeTuning(VehicleGraph graph)
        {
            byte[] blob = graph.blob;
            uint key = graph.header.activeTuningKey;
            int rawOff = (int)graph.tuningDescriptors[(int)key].rawOffset;
            int carOff = (int)graph.header.carOffset;
            float Rawf(uint o) => F32At(blob, rawOff + (int)o);
            int Rawi(uint o) => I32At(blob, rawOff + (int)o);
            uint Rawu(uint o) => U32At(blob, rawOff + (int)o);
            float Carf(uint o) => F32At(blob, carOff + (int)o);
            int Cari(uint o) => I32At(blob, carOff + (int)o);
            uint Caru(uint o) => U32At(blob, carOff + (int)o);

            VehicleTuning tuning = new VehicleTuning();



            // --- active + primary curves ---
            var active = new List<CFuncKeysReal>(ActiveCurveCount);
            for (int i = 0; i < ActiveCurveCount; i++)
            {
                int idx = LookupCurveDescriptor(
                    graph.curveDescriptors, VehicleCurveOwnerTuning, key, ActiveCurveOffsets[i]);
                if (idx < 0)
                {
                    if (i == CurveSteeringAngle)
                    {
                        active.Add(null);
                    }
                    else
                    {
                        throw new InvalidOperationException("tmnf world: vehicle curve lookup failed");
                    }
                }
                else
                {
                    active.Add(ReadCurveReal(blob, graph.curveDescriptors[idx]));
                }
            }
            var primary = new List<CFuncKeysReal>(PrimaryCurveCount);
            for (int i = 0; i < PrimaryCurveCount; i++)
            {
                int idx = LookupCurveDescriptor(
                    graph.curveDescriptors, VehicleCurveOwnerStruct, 0, PrimaryCurveOffsets[i]);
                if (idx < 0)
                {
                    throw new InvalidOperationException("tmnf world: vehicle curve lookup failed");
                }
                primary.Add(ReadCurveReal(blob, graph.curveDescriptors[idx]));
            }

            // --- curve set ---
            {
                CSceneVehicleCarTuningCurveSet set = tuning.curves;
                set.accelFromSpeed = active[CurveAccel];
                set.rolloverLateralFromSpeed = active[CurveRolloverLateral];
                set.maxSideFrictionFromSpeed = active[CurveMaxSideFriction];
                set.lateralContactSlowdownFromSpeed = active[CurveLateralContactSlowdown];
                set.steerSlowdownFromSpeed = active[CurveSteerSlowdown];
                set.rolloverLateralCoefFromAngle = active[CurveRolloverAngle];
                set.steerDriveTorqueFromSpeed = active[CurveSteerDriveTorque];
                set.m4SteerRadiusFromSpeed = active[CurveM4SteerRadius];
                set.m4MaxFrictionForceFromSpeed = active[CurveM4MaxFriction];
                set.m5SlippingAccelFromSpeed = active[CurveM5SlippingAccel];
                set.m5SlippingAccelScale = Rawf(0x1e4);
                set.waterFrictionFromSpeed = active[CurveWaterFriction];
                set.damperMax = Rawf(0x11c);
                set.damperMin = Rawf(0x120);
                set.m6DamperModulation = active[CurveM6DamperModulation];
                set.m6RearGearAccelFromSpeed = active[CurveM6RearGearAccel];
                set.m6RolloverLateralFromSpeedRatio = active[CurveM6RolloverRatio];
                set.m6BurnoutRadiusFromSpeed = active[CurveM6BurnoutRadius];
                set.m6LateralSpeedFromBurnoutRadius = active[CurveM6BurnoutLateralSpeed];
                set.m6DonutRolloverFromSpeed = active[CurveM6DonutRollover];
                set.m6BurnoutRolloverFromSpeed = active[CurveM6BurnoutRollover];
            }

            // --- gearboxes ---
            {
                List<float>[] destinations =
                {
                    tuning.@base.gearRatios,
                    tuning.@base.gearUpshift,
                    tuning.@base.gearDownshift,
                    tuning.@base.gearAux,
                };
                for (int i = 0; i < GearboxCount; i++)
                {
                    int idx = -1;
                    for (int gi = 0; gi < graph.gearboxDescriptors.Count; gi++)
                    {
                        VehicleGearboxDescriptor gd = graph.gearboxDescriptors[gi];
                        if (gd.ownerKey == key && gd.fieldOffset == GearboxOffsets[i])
                        {
                            idx = gi;
                            break;
                        }
                    }
                    if (idx < 0)
                    {
                        throw new InvalidOperationException("tmnf world: vehicle gearbox lookup failed");
                    }
                    VehicleGearboxDescriptor found = graph.gearboxDescriptors[idx];
                    for (int k = 0; k < GearboxValueCount; k++)
                    {
                        destinations[i].Add(F32At(blob, (int)found.dataOffset + 4 * k));
                    }
                }
            }

            // --- base tuning ---
            {
                CSceneVehicleCarTuning t = tuning.@base;
                t.damperMax = Rawf(0x11c);
                t.damperMin = Rawf(0x120);
                t.gearRatioCount = 6;
                t.engineModel = Rawi(0x354);
                t.forwardSpeedLimitScale = Rawf(0x02c);
                t.engineRpmReverseAccel = Rawf(0x2ec);
                t.engineRpmAccel = Rawf(0x2f0);
                t.engineRpmDecel = Rawf(0x2f4);
                t.engineRpmTurboDecel = Rawf(0x31c);
                t.engineRpmFollowAccel = Rawf(0x320);
                t.engineRpmLowAccel = Rawf(0x324);
                t.engineRpmHighDecel = Rawf(0x328);
                t.speed32c = Rawf(0x32c);
                t.speed330 = Rawf(0x330);
                t.speed334 = Rawf(0x334);
                t.speed338 = Rawf(0x338);
                t.suspensionModel = Rawi(0x350);
                t.suspensionStiffness = Rawf(0x114);
                t.suspensionDamping = Rawf(0x118);
                t.suspensionRestLength = Rawf(0x124);
                t.suspensionScale = Rawf(0x128);
            }

            // --- aux tuning ---
            {
                CSceneVehicleCarTuningAux t = tuning.aux;
                t.steeringSpeedBase = Rawf(0x06c);
                t.steeringSpeedScale = Rawf(0x070);
                t.steeringSlewRate = Rawf(0x094);
                t.airTorqueLinear = Rawf(0x158);
                t.airTorqueQuadratic = Rawf(0x15c);
                t.suspensionFollowRate = Rawf(0x194);
                t.airControlWindowTicks = Rawu(0x364);
                t.airReversalThreshold = Rawf(0x368);
                t.waterBuoyancy = Rawf(0x204);
                t.waterEntrySpeedThreshold = Rawf(0x208);
                t.waterEntrySpeedMinimum = Rawf(0x20c);
                t.waterAngularDragLinear = Rawf(0x21c);
                t.waterAngularDragQuadratic = Rawf(0x220);
                t.airVerticalCurve = active[CurveAirVertical];
                t.steeringAngleCurve = active[CurveSteeringAngle];
                CFuncKeysReal Mk(float[] values)
                {
                    CFuncKeysReal c = new CFuncKeysReal();
                    c.keys.count = 4;
                    c.keys.positions = (float[])TmnfWaterImpulsePositions.Clone();
                    c.keys.lowerBounds = new float[0];
                    c.keys.upperBounds = new float[0];
                    c.values = values;
                    c.interpolation = 0;
                    Curve.CfuncKeysCompile(c.keys);
                    return c;
                }
                t.waterImpulseVerticalCurve = Mk((float[])TmnfWaterImpulseVerticalValues.Clone());
                t.waterImpulseHorizontalCurve = Mk((float[])TmnfWaterImpulseHorizontalValues.Clone());
                // NB: waterFrictionCurve (+0x218) is NOT decoded here — an
                // upstream quirk faithfully kept; in-water ticks panic.
            }

            // --- contact tuning ---
            {
                TMNFVehicleContactTuning t = tuning.contact;
                t.frictionForce = Rawf(0x058);
                t.extraFrictionForce = Rawf(0x05c);
                t.slopeAdherenceMin = Rawf(0x0d4);
                t.slopeAdherenceMax = Rawf(0x0d8);
                t.slopeSecondaryMin = Rawf(0x0dc);
                t.slopeSecondaryMax = Rawf(0x0e0);
                t.angularYScale = Rawf(0x0e8);
                t.angularXzScale = Rawf(0x0ec);
                t.damperMax = Rawf(0x11c);
                t.maxAngularSpeed = Rawf(0x14c);
                t.maxLinearSpeedDelta = Rawf(0x150);
                t.bodyTangentRatio = Rawf(0x170);
                t.bodyTangentRatioMaterial4 = Rawf(0x174);
                t.restitutionAirMaterial4 = Rawf(0x178);
                t.restitutionAir = Rawf(0x17c);
                t.restitutionGround = Rawf(0x184);
                t.restitutionGroundMaterial4 = Rawf(0x18c);
                t.lateralLinear = Rawf(0x1a8);
                t.lateralQuadratic = Rawf(0x1ac);
                t.lateralGroundScale = Rawf(0x1c0);
                t.lateralContactDurationTicks = Rawu(0x1e8);
                t.wheelContactModel = Rawi(0x350);
                t.frictionModel = Rawi(0x354);
            }

            // --- model 6 tuning ---
            {
                CSceneVehicleCarModel6Tuning t = tuning.model6;
                t.forwardSpeedLimitScale = Rawf(0x02c);
                t.reverseSpeedLimitScale = Rawf(0x030);
                t.brakeBase = Rawf(0x040);
                t.brakeSpeedScale = Rawf(0x044);
                t.forwardBrakeLimitSliding = Rawf(0x048);
                t.forwardBrakeLimit = Rawf(0x04c);
                t.speedLimitForce = Rawf(0x060);
                t.verticalForceScale = Rawf(0x064);
                t.wheelSteerSineLimit = Rawf(0x074);
                t.steerSlowdownScale = Rawf(0x07c);
                t.wheelTorqueScale = Rawf(0x098);
                t.slidingSteerTorqueScale = Rawf(0x09c);
                t.lateralForceScale = Rawf(0x0a4);
                t.slidingLateralLimitScale = Rawf(0x0b0);
                t.lateralOverflowBlend = Rawf(0x0b4);
                t.wheelOverflowBlend = Rawf(0x0e4);
                t.verticalForceDivisor = Rawf(0x160);
                t.tractionLossScale = Rawf(0x200);
                t.burnoutTriggerScale = Rawf(0x228);
                t.burnoutTriggerLimit = Rawf(0x22c);
                t.rolloverAxisMinLength = Rawf(0x234);
                t.rolloverTorqueXScale = Rawf(0x238);
                t.rolloverTorqueZScale = Rawf(0x23c);
                t.slidingBrakeScale = Rawf(0x240);
                t.brakingLateralLimitScale = Rawf(0x244);
                t.reverseBrakeLimitSliding = Rawf(0x248);
                t.reverseBrakeLimit = Rawf(0x24c);
                t.burnoutSpeedMax = Rawf(0x254);
                t.burnoutSpeedMin = Rawf(0x258);
                t.donutLateralForceScale = Rawf(0x264);
                t.donutYawAngleScale = Rawf(0x268);
                t.donutSteerLinear = Rawf(0x26c);
                t.donutSteerQuadratic = Rawf(0x270);
                t.donutCountersteerScale = Rawf(0x274);
                t.donutRadiusExponent = Rawf(0x278);
                t.donutRadialSpeedExponent = Rawf(0x27c);
                t.donutRadiusMin = Rawf(0x280);
                t.donutLateralSpeedLimit = Rawf(0x284);
                t.donutNormalAngleLimit = Rawf(0x28c);
                t.donutAnglePositiveLimit = Rawf(0x290);
                t.donutAngleNegativeLimit = Rawf(0x294);
                t.burnoutEnterTicks = Rawu(0x298);
                t.burnoutEnterAccelScale = Rawf(0x29c);
                t.burnoutEnterLateralScale = Rawf(0x2a0);
                t.burnoutExitTicks = Rawu(0x2a8);
                t.burnoutExitAccelScale = Rawf(0x2ac);
                t.burnoutExitExtraAccel = Rawf(0x2b8);
                t.material6LongitudinalScale = Rawf(0x33c);
                t.material6GasDenominator = Rawf(0x340);
                t.material6VerticalShape = Rawf(0x344);
                t.material6VerticalScale = Rawf(0x348);
            }

            // --- compute tuning ---
            {
                TMNFVehicleComputeTuning t = tuning.compute;
                t.eventCLevel1Max = Rawf(0x028);
                t.groundedDragTerm = Rawf(0x058);
                t.activeContactStopThreshold = Rawf(0x0a4);
                t.normalTurboFactor = Rawf(0x0f0);
                t.rouletteTurboFactor = Rawf(0x0f4);
                t.normalTurboDuration = Rawu(0x0f8);
                t.rouletteTurboDuration = Rawu(0x0fc);
                t.airImpulseScale = Rawf(0x104);
                t.specialForceFieldScale = Rawf(0x108);
                t.airborneLinearDrag = Rawf(0x154);
                t.groundedForceFieldScale = Rawf(0x160);
                t.airborneForceFieldScale = Rawf(0x164);
                t.normalizedForceDivisor = Rawf(0x228);
                t.effectCurveBias = Rawf(0x384);
                t.eventAbLevel2Min = Rawf(0x398);
                t.eventAbTrigger = Rawf(0x39c);
                t.eventCTrigger = Rawf(0x3a0);
                t.effectCurve = active[CurveEffect];
                t.contactDecayCurve = primary[0];
                t.contactRiseCurve = primary[1];
            }

            // --- ground materials + ids ---
            {
                int idsOff = (int)graph.header.groundIdsOffset;
                for (int i = 0; i < 31; i++)
                {
                    tuning.groundMaterialIndices.Add(U32At(blob, idsOff + 4 * i));
                }
                foreach (uint id in tuning.groundMaterialIndices)
                {
                    if (id >= graph.header.groundMaterialCount)
                    {
                        throw new InvalidOperationException("tmnf world: ground-id table maps a physical material outside the material manager");
                    }
                }
                foreach (VehicleMaterialRecord raw in graph.materials)
                {
                    TMNFVehicleGroundMaterial m = new TMNFVehicleGroundMaterial();
                    m.values = (float[])raw.values.Clone();
                    m.fakeContactMask = null; // bound to the world mask below
                    m.fakeContactPeriodX = raw.fakeContactPeriodX;
                    m.fakeContactPeriodZ = raw.fakeContactPeriodZ;
                    m.fakeContactImpulseScale = raw.fakeContactImpulseScale;
                    m.fakeContactImpulseLimit = raw.fakeContactImpulseLimit;
                    tuning.groundMaterials.Add(m);
                }
            }

            // --- vehicle mutable initial state ---
            Vehicle vehicle = new Vehicle();
            vehicle.inputGas = Carf(0x050);
            vehicle.inputBrake = Carf(0x054);
            vehicle.inputSteer = Carf(0x058);
            vehicle.wheelCount = 4;
            {
                vehicle.engine.maxRpm = Carf(0x59c);
                vehicle.engine.brakingFactor = Carf(0x59c + 0x14);
                vehicle.engine.rpm = Carf(0x59c + 0x18);
                vehicle.engine.targetRpm = Carf(0x59c + 0x1c);
                vehicle.engine.clutch = Carf(0x59c + 0x20);
                vehicle.engine.shiftTimer = Carf(0x59c + 0x24);
                vehicle.engine.reverse = Cari(0x59c + 0x28);
                vehicle.engine.gear = Cari(0x59c + 0x2c);
            }
            vehicle.currentLocalSpeed = Vec3At(blob, carOff + 0x70c);
            vehicle.totalForceAdded = Vec3At(blob, carOff + 0x818);
            vehicle.totalImpulseAdded = Vec3At(blob, carOff + 0x824);
            vehicle.engineMode = Cari(0x2e4);
            vehicle.turboActive = Cari(0x628);
            vehicle.driveMode = Cari(0x69c);
            vehicle.engineLimitFlag = Cari(0x744);
            vehicle.gearDownshiftFlag = Cari(0x748);
            vehicle.forceWheelSpeed = Cari(0x6a0);
            vehicle.flag60c = Cari(0x60c);
            vehicle.blockWheelSpeed = Cari(0x73c);
            vehicle.forcedWheelSpeed = Rawf(0x2bc);

            // aux state
            vehicle.integrationFlags = Caru(0x2f4);
            vehicle.turboEpochTick = Caru(0x5d0);
            vehicle.airControlImmediate = Cari(0x5d4);
            vehicle.airControlLocked = Cari(0x5e4);
            vehicle.steeringValue = Carf(0x5e8);
            vehicle.turboProgress = Carf(0x5f0);
            vehicle.turboFactor = Carf(0x5f4);
            vehicle.turboStartTick = Caru(0x5f8);
            vehicle.turboEndTick = Caru(0x5fc);
            vehicle.turboType = Cari(0x600) switch
            {
                1 => TMNFVehicleTurboType.Normal,
                2 => TMNFVehicleTurboType.Roulette,
                _ => TMNFVehicleTurboType.None,
            };
            vehicle.rouletteToken = Caru(0x604);
            vehicle.rouletteValue = Carf(0x608);
            vehicle.airControlTick = Caru(0x614);
            vehicle.airControlSpeed = Vec3At(blob, carOff + 0x618);
            vehicle.rouletteModulus = uint.MaxValue;
            vehicle.bodyBox = BoxAt(blob, carOff + 0x1dc);
            vehicle.turboSoundAttached = false;

            // contact state
            vehicle.frictionInputSelector = Cari(0x5c4);
            vehicle.sideContact = Cari(0x5dc);
            vehicle.lastSideContactTick = Caru(0x5e0);
            vehicle.airborneFrictionGate = Cari(0x5e4);
            vehicle.wheelContactAbsorbCount = Caru(0x67c);
            vehicle.bodyContactCount = Caru(0x680);
            vehicle.bodyContactPositionSum = Vec3At(blob, carOff + 0x684);
            vehicle.bodyContactNormalSum = Vec3At(blob, carOff + 0x690);
            vehicle.tickTime = graph.header.tick;

            // model 6 state
            {
                CSceneVehicleCarModel6State s = vehicle.model6;
                s.pivotPosition = Vec3At(blob, carOff + 0x1dc);
                s.pivotAxis = Vec3At(blob, carOff + 0x1e8);
                s.reverseMode = Cari(0x5c4);
                s.reverseSpeedThreshold = Carf(0x5cc);
                s.contactBlockCount = Cari(0x5d8);
                s.sideContact = Cari(0x5dc);
                s.lastSlidingTick = Caru(0x62c);
                s.slidingStartTick = Caru(0x630);
                s.slidingElapsedTicks = Caru(0x634);
                s.modelIso = Iso4At(blob, carOff + 0x6a4);
                s.rolloverAxis = Vec3At(blob, carOff + 0x6d4);
                s.orbitCenter = Vec3At(blob, carOff + 0x6e0);
                s.orbitInitialRadius = Carf(0x6ec);
                s.orbitRadius = Carf(0x6f0);
                s.burnoutStartTick = Caru(0x6f4);
                s.burnoutTransitionTick = Caru(0x6f8);
                s.orbitAxis = Vec3At(blob, carOff + 0x6fc);
                s.orbitSign = Carf(0x708);
                s.axleWidth = Carf(0x840);
            }

            // compute state
            {
                TMNFVehicleComputeState s = vehicle.compute;
                s.simulationGate = Carf(0x1e8);
                s.eventLevelC = Caru(0x1fc);
                s.eventSourceC = blob[carOff + 0x200];
                s.eventSourceAb = blob[carOff + 0x201];
                s.springC = new GmSpringFloat
                {
                    stiffness = Carf(0x214),
                    damping = Carf(0x218),
                    value = Carf(0x21c),
                    target = Carf(0x220),
                    velocity = Carf(0x224),
                };
                s.springA = new GmSpringFloat
                {
                    stiffness = Carf(0x228),
                    damping = Carf(0x22c),
                    value = Carf(0x230),
                    target = Carf(0x234),
                    velocity = Carf(0x238),
                };
                s.contactRise = Carf(0x23c);
                s.contactDecay = Carf(0x240);
                s.historyForceLimit = Carf(0x244);
                s.historyForceScale = Carf(0x24c);
                s.springValueLimit = Carf(0x250);
                s.localSpeedLimit = Carf(0x2e0);
                s.airEffectThreshold = Carf(0x05c);
                s.brakeInputScale = Carf(0x5a8);
                s.groundedDragScale = Carf(0x5ac);
                s.computedBrakeForce = Carf(0x5b0);
                s.state5d8 = Cari(0x5d8);
                s.airImpulseCooldownTick = Caru(0x610);
                s.effectAccumulator = Carf(0x624);
                s.lastForceTick = Caru(0x650);
                s.eventLevelA = Caru(0x654);
                s.eventLevelB = Caru(0x658);
                s.peakEventLevelB = Caru(0x660);
                s.peakEventLevelA = Caru(0x664);
                s.peakEventLevelC = Caru(0x668);
                s.peakEventSourceAb = blob[carOff + 0x66c];
                s.peakEventSourceC = blob[carOff + 0x66d];
                s.eventMetricA = Carf(0x670);
                s.eventMetricB = Carf(0x674);
                s.eventMetricC = Carf(0x678);
                s.normalizedForce = Vec3At(blob, carOff + 0x6d4);
                s.airEffectMode = Cari(0x74c);
            }

            // wheels
            for (int i = 0; i < 4; i++)
            {
                int w = (int)graph.header.wheelsOffset + i * WheelSize;
                byte[] wheelBytes = new byte[WheelSize];
                Array.Copy(blob, w, wheelBytes, 0, WheelSize);
                CSceneVehicleCarWheel wheel = new CSceneVehicleCarWheel();
                wheel.active = I32At(wheelBytes, 0x000);
                wheel.steerable = I32At(wheelBytes, 0x004);
                wheel.radius = F32At(wheelBytes, 0x008);
                for (int k = 0; k < 12; k++)
                {
                    wheel.field70[k] = F32At(wheelBytes, 0x070 + 4 * k);
                }
                wheel.fielda0 = I32At(wheelBytes, 0x0a0);
                wheel.fielda4 = I32At(wheelBytes, 0x0a4);
                wheel.offsetFromVehicle = Vec3At(wheelBytes, 0x0a8);
                wheel.realTime = DecodeWheelRealTime(wheelBytes, 0x0b4);
                wheel.field15c = I32At(wheelBytes, 0x15c);
                wheel.contactRelativeLocalDistance = Vec3At(wheelBytes, 0x160);
                wheel.surfaceSource = Iso4At(wheelBytes, 0x010);
                wheel.surfaceLocation = Iso4At(wheelBytes, 0x040);
                wheel.previousSync = DecodeWheelState(wheelBytes, 0x16c);
                wheel.sync = DecodeWheelState(wheelBytes, 0x1d0);
                wheel.field234 = DecodeWheelState(wheelBytes, 0x234);
                wheel.asyncState = DecodeWheelState(wheelBytes, 0x298);
                vehicle.wheels.Add(wheel);
            }

            return (tuning, vehicle);
        }

        private static CSceneVehicleCarWheelRealTimeState DecodeWheelRealTime(byte[] b, int o)
        {
            CSceneVehicleCarWheelRealTimeState rt = default(CSceneVehicleCarWheelRealTimeState);
            float F(int off) => F32At(b, off);
            rt.damperAbsorb = F(o + 0x00);
            rt.field04 = F(o + 0x04);
            rt.field08 = F(o + 0x08);
            rt.basis0 = default(GmMat3);
            for (int i = 0; i < 9; i++) rt.basis0[i] = F(o + 0x0c + 4 * i);
            rt.basis1 = default(GmMat3);
            for (int i = 0; i < 9; i++) rt.basis1[i] = F(o + 0x30 + 4 * i);
            rt.field54 = GmVec3.FromBytes(b, o + 0x54);
            rt.reserved60 = new byte[0x0c];
            Array.Copy(b, o + 0x60, rt.reserved60, 0, 0x0c);
            rt.field6c = F(o + 0x6c);
            rt.hasGroundContact = I32At(b, o + 0x70);
            rt.contactMaterialId = I32At(b, o + 0x74);
            rt.isSliding = I32At(b, o + 0x78);
            rt.relativeRotzAxis = GmVec3.FromBytes(b, o + 0x7c);
            rt.contactBodyWord = U32At(b, o + 0x88);
            rt.groundContactCount = I32At(b, o + 0x8c);
            rt.field90 = GmVec3.FromBytes(b, o + 0x90);
            rt.rotationPhase = F(o + 0x9c);
            rt.blendValue = F(o + 0xa0);
            rt.blendTarget = F(o + 0xa4);
            return rt;
        }

        private static CSceneVehicleCarWheelState DecodeWheelState(byte[] b, int o)
        {
            CSceneVehicleCarWheelState s = CSceneVehicleCarWheelState.Create();
            Array.Copy(b, o, s.bytes, 0, 0x64);
            return s;
        }

        /// <summary>Golden hook: decode from a validated blob.</summary>
        public static (VehicleTuning, Vehicle) DecodeTuningFromBlob(byte[] blob)
        {
            VehicleGraph graph = ValidateVehicleGraph(blob, null, false);
            return DecodeTuning(graph);
        }

        // ------------------------------------------------------------------
        // construction
        // ------------------------------------------------------------------

        /// <summary>World_CreateFromBlob: builds a world from the pieces.</summary>
        public World(TrackLoader.TmnfTrack track, byte[] vehicleBlob, WorldOptions options)
        {
            byte[] mask = options.fakeContactMask;
            if (mask == null)
            {
                throw new InvalidOperationException("tmnf world: fake-contact mask missing; supply one at runtime");
            }
            if (mask.Length != 128 * 128)
            {
                throw new InvalidOperationException("tmnf world: fake-contact mask must be 128x128 bytes");
            }
            VehicleGraph graph = ValidateVehicleGraph(
                vehicleBlob, track.header.trackSha256, options.pinProvenance);

            (VehicleTuning tuning, Vehicle vehicle) = DecodeTuning(graph);
            // The wheel/body tree wiring (decode_collision_world's fill).
            for (int i = 0; i < 4; i++)
            {
                tuning.wheelTreeRefs.Add(0);
            }
            foreach (VehicleCollisionTreeRecord raw in graph.collisionChildren)
            {
                uint kind = raw.kind & VehicleCollisionKindMask;
                if (kind == VehicleCollisionTreeWheel)
                {
                    tuning.wheelTreeRefs[(int)raw.wheelIndex] = raw.objectId;
                }
                else
                {
                    tuning.bodyTreeRefs.Add(raw.objectId);
                }
            }
            // Bind the fake-contact masks (the C's link_vehicle).
            for (int i = 0; i < graph.materials.Count; i++)
            {
                if (graph.materials[i].fakeContactMask != 0)
                {
                    tuning.groundMaterials[i].fakeContactMask = mask;
                }
            }

            // The dyna.
            int stateOff = (int)graph.header.stateOffset;
            byte[] liveStateBytes = new byte[180];
            Array.Copy(vehicleBlob, stateOff, liveStateBytes, 0, 180);
            CHmsStateDyna liveState = CHmsStateDyna.FromBytes(liveStateBytes);
            int paramsOff = (int)graph.header.paramsOffset;
            byte[] paramsBytes = new byte[ParamsSize];
            Array.Copy(vehicleBlob, paramsOff, paramsBytes, 0, ParamsSize);
            CHmsDynaParams dynaParams = CHmsDynaParams.FromBytes(paramsBytes);
            int dynaRaw = (int)graph.header.dynaOffset;
            byte[] tempBytes = new byte[180];
            Array.Copy(vehicleBlob, dynaRaw + 0x274, tempBytes, 0, 180);
            CHmsStateDyna tempState = CHmsStateDyna.FromBytes(tempBytes);
            int clampAngular = I32At(vehicleBlob, dynaRaw + 0x0c0);
            float maxAngularSpeed = F32At(vehicleBlob, dynaRaw + 0x0c4);
            int dirtyFlag = I32At(vehicleBlob, dynaRaw + 0x33c);
            int mode = I32At(vehicleBlob, dynaRaw + 0x340);
            dyna = new CHmsDyna(dynaParams, mode, clampAngular, maxAngularSpeed);
            dyna.dirtyFlag = dirtyFlag;
            dyna.SetLive(liveState);
            dyna.SetStateB(tempState);
            dyna.tempState = tempState;

            // The collision trees (decode_collision_world + link_collision).
            int nodeCount = (int)graph.header.collisionChildCount;
            int[] wheelNodeOfLocal = { -1, -1, -1, -1 };
            for (int i = 0; i < nodeCount; i++)
            {
                VehicleCollisionTreeRecord raw = graph.collisionChildren[i];
                uint kind = raw.kind & VehicleCollisionKindMask;
                VehicleTree node = new VehicleTree();
                node.objectRef = raw.objectId;
                node.flags = raw.flags;
                node.boxAligned = raw.boxAligned;
                node.localIso = raw.localIso;
                if (kind == VehicleCollisionTreeWheel)
                {
                    int wheelIndex = (int)raw.wheelIndex;
                    wheelNodeOfLocal[wheelIndex] = i;
                    node.wheel = raw.wheelIndex;
                    // Wheel impulse point starts at the box center.
                    vehicle.wheels[wheelIndex].impulsePoint = raw.boxAligned.center;
                }
                if (raw.surfaceId != 0)
                {
                    // The shape payload: 12 bytes of radii at offset 0.
                    GmVec3 radii = Vec3At(raw.shape, 0);
                    GmSurfGeom geom;
                    if (raw.geometryType == GmSurfTypes.GmSurfSphere)
                    {
                        GmSurfSphere sphere = default(GmSurfSphere);
                        sphere.@base = default(GmSurf);
                        sphere.@base.materialIndex = raw.geometryMaterialIndex;
                        sphere.@base.surfType = raw.geometryType;
                        sphere.@base.reserved = raw.geometryReserved;
                        sphere.radius = radii.x;
                        geom = new GmSurfGeomSphere { shape = sphere };
                    }
                    else
                    {
                        GmSurfEllipsoid ellipsoid = default(GmSurfEllipsoid);
                        ellipsoid.@base = default(GmSurf);
                        ellipsoid.@base.materialIndex = raw.geometryMaterialIndex;
                        ellipsoid.@base.surfType = raw.geometryType;
                        ellipsoid.@base.reserved = raw.geometryReserved;
                        ellipsoid.radii = radii;
                        geom = new GmSurfGeomEllipsoid { shape = ellipsoid };
                    }
                    int matOff = (int)graph.header.collisionMaterialIdsOffset
                        + (int)raw.materialIndex;
                    byte[] materialIds = new byte[raw.materialCount];
                    Array.Copy(vehicleBlob, matOff, materialIds, 0, (int)raw.materialCount);
                    CPlugSurface surface = new CPlugSurface();
                    surface.geom = geom;
                    surface.materialIds = materialIds;
                    surface.materialCount = raw.materialCount;
                    node.surface = surface;
                }
                nodes.Add(node);
            }

            // Pre-order linking: the root owns every record not claimed as a
            // child by an earlier node.
            uint ChildCountOf(int i) => graph.collisionChildren[i].kind >> (int)VehicleCollisionChildShift;
            var rootChildrenLocal = new List<int>();
            {
                uint pending = 0;
                for (int i = 0; i < nodeCount; i++)
                {
                    if (pending != 0)
                    {
                        pending -= 1;
                    }
                    else
                    {
                        rootChildrenLocal.Add(i);
                    }
                    pending += ChildCountOf(i);
                }
            }
            {
                void Assign(int parent, ref int next)
                {
                    uint count = ChildCountOf(parent);
                    var kids = new List<int>();
                    for (uint k = 0; k < count; k++)
                    {
                        if (next >= nodes.Count)
                        {
                            throw new InvalidOperationException("tmnf world: vehicle collision node is missing children");
                        }
                        int child = next;
                        next += 1;
                        Assign(child, ref next);
                        kids.Add(child);
                    }
                    nodes[parent].children = kids;
                }
                int next = 0;
                var roots = rootChildrenLocal.ToArray();
                foreach (int r in roots)
                {
                    if (r != next)
                    {
                        throw new InvalidOperationException("tmnf world: vehicle collision tree construction failed");
                    }
                    next += 1;
                    Assign(r, ref next);
                }
                if (next != nodeCount)
                {
                    throw new InvalidOperationException("tmnf world: vehicle collision tree construction failed");
                }
            }

            uint playerCorpusRef = track.staticResponseCount;
            for (int i = 0; i < nodeCount; i++)
            {
                contactBuffers.Add(new SHmsSphereBufferContact());
            }

            GmVec3 bodyReferencePosition = Vec3At(vehicleBlob, (int)graph.header.bodyReferenceOffset);

            WorldCold cold = new WorldCold();
            cold.vehicleBlob = vehicleBlob;
            cold.header = graph.header; // (VehicleSnapshotHeader)
            cold.tuning = tuning;
            CHmsResponseBody playerResponseBody = default(CHmsResponseBody);
            playerResponseBody.corpusRef = playerCorpusRef;
            playerResponseBody.classificationFlags = 0x19847004u;
            playerResponseBody.responseFlags = 0xff618001u;
            playerResponseBody.responseWeight = 1.0f;
            playerResponseBody.dyna = 0;
            playerResponseBody.hasContactSink = true;
            playerResponseBody.sink = AbsorbSink.VehicleBody;
            cold.playerResponseBody = playerResponseBody;
            cold.responseMaterial = new CHmsResponseMaterial
            {
                category = 3,
                responseMode = 1,
                sideEnabled = new[] { 1u, 1u },
            };
            cold.gravityActive = 1;
            cold.gravityValue = new GmVec3(0.0f, -10.0f, 0.0f);
            cold.bodyReferencePosition = bodyReferencePosition;

            this.cold = cold;
            this.track = track;
            this.route = null;
            this.vehicle = vehicle;
            this.rootChildren = rootChildrenLocal;
            this.rootObjectRef = graph.collisionRoot.objectId;
            this.rootFlags = graph.collisionRoot.flags;
            this.rootBox = graph.collisionRoot.boxAligned;
            this.rootLocalIso = graph.collisionRoot.localIso;
            this.wheelNodeOf = wheelNodeOfLocal;
            this.currentMaterial = 0;
            this.currentCorpus1 = 0;
            this.currentCorpus2 = 0;
            this.staticGroupBound = false;
            this.mergeBuffers = new List<int>();
            this.collisionBuffer = new CHmsCollisionBuffer();
            this.linearDragScale = 1.0f;
            this.angularDragScale = 1.0f;
            this.triggerContacts = 0;
            this.sceneFlags = 0x2000u;
            this.tickTime = graph.header.tick;
            this.playerCorpusRef = playerCorpusRef;

            // The snapshot begins at Model6 entry: restore the force field
            // scale the in-progress ComputeCorpusForces consumes.
            dyna.@params.forceFieldScale =
                cold.tuning.compute.groundedForceFieldScale;

            // finish_snapshot_tick: run one tick to settle the state.
            PhysicsStep2(10);
        }

        /// <summary>The live rigid-body state.</summary>
        public ref CHmsStateDyna Live() => ref dyna.Live();

        /// <summary>A CPlugTree view of the vehicle tree (race triggers).</summary>
        public CPlugTree[] CarTreeSnapshot()
        {
            var trees = new List<CPlugTree>(1 + nodes.Count);
            CPlugTree root = new CPlugTree();
            root.objectRef = rootObjectRef;
            root.flags = rootFlags;
            root.boxAligned = rootBox;
            root.localIso = rootLocalIso;
            root.surface = null;
            root.children = new int[rootChildren.Count];
            for (int i = 0; i < rootChildren.Count; i++)
            {
                root.children[i] = rootChildren[i] + 1;
            }
            trees.Add(root);
            foreach (VehicleTree n in nodes)
            {
                CPlugTree t = new CPlugTree();
                t.objectRef = n.objectRef;
                t.flags = n.flags;
                t.boxAligned = n.boxAligned;
                t.localIso = n.localIso;
                t.surface = n.surface?.Clone();
                t.children = new int[n.children.Count];
                for (int i = 0; i < n.children.Count; i++)
                {
                    t.children[i] = n.children[i] + 1;
                }
                trees.Add(t);
            }
            return trees.ToArray();
        }

        /// <summary>0x0055F3B0: the uniform gravity field value.</summary>
        public GmVec3? Gravity()
        {
            if (cold.gravityActive != 0)
            {
                return cold.gravityValue;
            }
            return null;
        }

        /// <summary>World_AdvanceTimer — milliseconds, not ticks.</summary>
        public void AdvanceTimer(uint tickMs)
        {
            tickTime = unchecked(tickTime + tickMs);
            vehicle.tickTime = tickTime;
        }

        /// <summary>0x004FE500 on the player vehicle.</summary>
        public void ApplyInputs(ref TMNFRaceInputs inputs)
        {
            VehicleCtx ctx = new VehicleCtx(vehicle, cold.tuning, dyna);
            VehicleBase.UpdateVehicleStateFromInputs(ref inputs, ref ctx);
        }

        /// <summary>
        /// World_WritePlayerGameState: the 0x878-byte car block and the
        /// 4x0x2fc-byte wheel blocks, with every simulated field patched in.
        /// </summary>
        public (byte[], byte[]) WritePlayerGameState()
        {
            return GameStateExport.DumpVehicleGameState(cold.vehicleBlob, vehicle);
        }

        /// <summary>set_surface_location for every wheel after suspension.</summary>
        private void SyncWheelSurfaces()
        {
            for (int w = 0; w < 4; w++)
            {
                int node = wheelNodeOf[w];
                if (node == -1)
                {
                    continue;
                }
                GmIso4 location = vehicle.wheels[w].surfaceLocation;
                nodes[node].localIso = location;
                nodes[node].boxAligned.center = new GmVec3(location.t0, location.t1, location.t2);
                vehicle.wheels[w].impulsePoint = new GmVec3(location.t0, location.t1, location.t2);
            }
        }
    }
}
