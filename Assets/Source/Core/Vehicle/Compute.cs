using System;

namespace Tmnf.Sim
{
    /// <summary>vehicle_compute.c (0x007C69E0): the top force orchestrator.</summary>
    public static class VehicleCompute
    {
        // COMPUTE_LENGTH_EPSILON 0x1.b7cdfcp-34f
        private static readonly float ComputeLengthEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EB7_CDFC));
        // COMPUTE_VALUE_EPSILON 0x1.4f8b58p-17f
        private static readonly float ComputeValueEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x374F_8B58));
        // KMH_PER_MPS: float 3.6 promoted to double
        private static readonly double KhmPerMps =
            BitConverter.Int64BitsToDouble(unchecked((long)0x400C_CCCC_C000_0000));

        /// <summary>World-supplied seams for one force computation.</summary>
        public sealed class ComputeExternal
        {
            public Model6External ext;
            public TmnfTrackWater water;
            /// <summary>Invoked after all forces are accumulated.</summary>
            public Action<VehicleCtx> postForce;
            /// <summary>corpus ref to token for turbo roulette.</summary>
            public Func<uint, uint> contactToken;
            /// <summary>The static-body iso resolver for turbo detection.</summary>
            public Func<uint, GmIso4> bodyIso;
            public bool fakeContactsActive;
            public bool waterForcesActive;
        }

        private static float VecLengthSquaredYxz(ref GmVec3 value)
        {
            return (float)(double)(((float)(double)(((float)(double)(value.y * value.y)) + ((float)(double)(value.x * value.x)))) + ((float)(double)(value.z * value.z)));
        }

        private static float Clamp01(float value)
        {
            if (float.IsNaN(value) || (double)value < 0.0)
            {
                return 0.0f;
            }
            if (1.0 < (double)value)
            {
                return 1.0f;
            }
            return value;
        }

        private static float ClampSymmetric(float value, float limit)
        {
            float lower = -limit;
            if ((double)value < (double)lower)
            {
                return lower;
            }
            if ((double)limit < (double)value)
            {
                return limit;
            }
            return value;
        }

        private static void SetVehicleLinearSpeed(ref VehicleCtx ctx, ref GmVec3 speed)
        {
            ctx.dyna.SetLocalLinearSpeed(ref speed);
        }

        private static void AddVehicleImpulse(ref VehicleCtx ctx, ref GmVec3 impulse)
        {
            ctx.dyna.AddLocalImpulse(ref impulse);
            ctx.vehicle.totalImpulseAdded.x = (float)(double)(ctx.vehicle.totalImpulseAdded.x + impulse.x);
            ctx.vehicle.totalImpulseAdded.y = (float)(double)(impulse.y + ctx.vehicle.totalImpulseAdded.y);
            ctx.vehicle.totalImpulseAdded.z = (float)(double)(impulse.z + ctx.vehicle.totalImpulseAdded.z);
        }

        /// <summary>
        /// Diagnostic hook (env-gated, non-behavioral): prints the world
        /// force/torque after a labeled pipeline stage when TMNF_DBG is set.
        /// </summary>
        private static void DbgStage(ref VehicleCtx ctx, string stage)
        {
            if (Environment.GetEnvironmentVariable("TMNF_DBG") == null)
            {
                return;
            }
            GmVec3 f = ctx.dyna.Live().force;
            GmVec3 t = ctx.dyna.Live().torque;
            Console.Error.WriteLine(
                $"STG {stage} tick={ctx.vehicle.tickTime} " +
                $"force=({BitConverter.SingleToInt32Bits(f.x):X8},{BitConverter.SingleToInt32Bits(f.y):X8},{BitConverter.SingleToInt32Bits(f.z):X8}) " +
                $"torque=({BitConverter.SingleToInt32Bits(t.x):X8},{BitConverter.SingleToInt32Bits(t.y):X8},{BitConverter.SingleToInt32Bits(t.z):X8})");
        }

        private static void ClampVehicleLinearSpeed(ref VehicleCtx ctx, ref GmVec3 speed)
        {
            float maximum = ctx.vehicle.compute.localSpeedLimit;
            float maximumSquared = (float)(double)(maximum * maximum);
            float lengthSquared = VecLengthSquaredYxz(ref speed);

            if ((double)maximumSquared < (double)lengthSquared
                && (double)ComputeLengthEpsilon < (double)maximumSquared)
            {
                float length = Fp.Sqrt(lengthSquared);
                float scale = (float)(double)(maximum / length);

                speed.x = (float)(double)(scale * speed.x);
                speed.y = (float)(double)(speed.y * scale);
                speed.z = (float)(double)(scale * speed.z);
                SetVehicleLinearSpeed(ref ctx, ref speed);
            }
        }

        private static float ComputeSteeringAngle(ref VehicleCtx ctx, ref GmVec3 linearSpeed)
        {
            CSceneVehicleCarTuningAux tuning = ctx.tuning.aux;
            float denominator = (float)(double)(((float)(double)(Math.Abs(linearSpeed.z) * tuning.steeringSpeedScale))
                + tuning.steeringSpeedBase);
            float angle;

            if ((double)ComputeValueEpsilon <= (double)denominator)
            {
                float reciprocal = (float)(double)(1.0f / denominator);
                angle = GmFunc.GmfuncAsinSafe(reciprocal);
            }
            else
            {
                angle = 0.0f;
            }
            if (DbgHook.Enabled)
            {
                Console.Error.WriteLine(
                    $"STR tick={ctx.vehicle.tickTime} " +
                    $"speedz={BitConverter.SingleToInt32Bits(linearSpeed.z):X8} " +
                    $"denom={BitConverter.SingleToInt32Bits(denominator):X8} " +
                    $"angle={BitConverter.SingleToInt32Bits(angle):X8} " +
                    $"steerval={BitConverter.SingleToInt32Bits(ctx.vehicle.steeringValue):X8}");
            }
            return (float)(double)(-(ctx.vehicle.steeringValue) * angle);
        }

        private static (bool, bool) ScanWheelContacts(ref VehicleCtx ctx)
        {
            bool anyContact = false;
            bool anyActiveContact = false;
            foreach (CSceneVehicleCarWheel wheel in ctx.vehicle.wheels)
            {
                if (wheel.realTime.hasGroundContact != 0)
                {
                    anyContact = true;
                    if (wheel.active != 0)
                    {
                        anyActiveContact = true;
                    }
                }
            }
            return (anyContact, anyActiveContact);
        }

        private static void ApplyAirEffect(
            ref VehicleCtx ctx,
            ComputeExternal external,
            ref GmVec3 existingForce,
            uint tick,
            bool grounded)
        {
            float airEffectThreshold = ctx.vehicle.compute.airEffectThreshold;
            if (!((double)ComputeValueEpsilon < (double)airEffectThreshold))
            {
                return;
            }

            switch (ctx.vehicle.compute.airEffectMode)
            {
                case 1:
                    if (grounded
                        && ctx.vehicle.compute.airImpulseCooldownTick < tick)
                    {
                        GmVec3 impulse = new GmVec3(-existingForce.x, -existingForce.y, -existingForce.z);
                        float lengthSquared = VecLengthSquaredYxz(ref impulse);

                        if ((double)ComputeLengthEpsilon < (double)lengthSquared)
                        {
                            float length = Fp.Sqrt(lengthSquared);
                            float inverseLength = (float)(double)(1.0f / length);

                            impulse.x = (float)(double)(inverseLength * impulse.x);
                            impulse.y = (float)(double)(impulse.y * inverseLength);
                            impulse.z = (float)(double)((float)(double)(impulse.y * 0.0f) + (float)(double)(impulse.z * inverseLength));
                            impulse.x = impulse.x; // (order preserved)
                        }
                        impulse.x = (float)(double)(ctx.tuning.compute.airImpulseScale * impulse.x);
                        impulse.y = (float)(double)(impulse.y * ctx.tuning.compute.airImpulseScale);
                        impulse.z = (float)(double)(ctx.tuning.compute.airImpulseScale * impulse.z);
                        AddVehicleImpulse(ref ctx, ref impulse);
                        ctx.vehicle.compute.airImpulseCooldownTick = unchecked(tick + 100);
                    }
                    break;
                case 2:
                    ctx.dyna.@params.forceFieldScale = ctx.tuning.compute.specialForceFieldScale;
                    break;
                case 3:
                    ctx.vehicle.turboType = TMNFVehicleTurboType.Normal;
                    ctx.vehicle.turboFactor = ctx.tuning.compute.normalTurboFactor;
                    break;
            }
        }

        private static uint ContactToken(ComputeExternal external, uint? body)
        {
            if (!body.HasValue)
            {
                return 0;
            }
            return external.contactToken(body.Value);
        }

        private static void UpdateSurfaceEffects(ref VehicleCtx ctx, ComputeExternal external, uint tick)
        {
            var turbo = VehicleContact.IsGroundContactId(ref ctx, 7);
            if (turbo.HasValue)
            {
                GmVec3 relativeAxis = turbo.Value.Item1;
                float factor = (float)(double)(ctx.tuning.compute.normalTurboFactor * relativeAxis.z);
                VehicleAux.EnableTurbo(ref ctx, tick,
                    ctx.tuning.compute.normalTurboDuration, factor,
                    TMNFVehicleTurboType.Normal, 0);
            }
            var turbo2 = VehicleContact.IsGroundContactId(ref ctx, 26);
            if (turbo2.HasValue)
            {
                GmVec3 relativeAxis = turbo2.Value.Item1;
                float factor = (float)(double)(ctx.tuning.compute.rouletteTurboFactor * relativeAxis.z);
                VehicleAux.EnableTurbo(ref ctx, tick,
                    ctx.tuning.compute.rouletteTurboDuration, factor,
                    TMNFVehicleTurboType.Normal, 0);
            }
            var turbo3 = VehicleContact.IsGroundContactId(ref ctx, 30);
            if (turbo3.HasValue)
            {
                GmVec3 relativeAxis = turbo3.Value.Item1;
                uint? body = turbo3.Value.Item2;
                float factor = (float)(double)(ctx.tuning.compute.normalTurboFactor * relativeAxis.z);
                uint token = ContactToken(external, body);

                VehicleAux.EnableTurbo(ref ctx, tick,
                    ctx.tuning.compute.normalTurboDuration, factor,
                    TMNFVehicleTurboType.Roulette, token);
            }
            if (VehicleContact.IsGroundContactId(ref ctx, 29).HasValue)
            {
                ctx.vehicle.flag60c = 1;
            }
            VehicleAux.UpdateTurbo(ref ctx, tick);
        }

        private static void UpdateEventLevels(ref VehicleCtx ctx, uint tick)
        {
            float metricA = ctx.vehicle.compute.eventMetricA;
            float metricB = ctx.vehicle.compute.eventMetricB;
            float metricC = ctx.vehicle.compute.eventMetricC;
            float abTrigger = ctx.tuning.compute.eventAbTrigger;
            float abLevel2Min = ctx.tuning.compute.eventAbLevel2Min;
            float cTrigger = ctx.tuning.compute.eventCTrigger;
            float cLevel1Max = ctx.tuning.compute.eventCLevel1Max;

            if ((double)abTrigger < (double)metricA)
            {
                if ((double)metricA <= (double)abLevel2Min)
                {
                    if (ctx.vehicle.compute.eventLevelA == 0)
                    {
                        ctx.vehicle.compute.eventLevelA = 1;
                    }
                }
                else if (ctx.vehicle.compute.eventLevelA < 2)
                {
                    ctx.vehicle.compute.eventLevelA = 2;
                }
            }
            if ((double)abTrigger < (double)metricB)
            {
                if ((double)metricB <= (double)abLevel2Min)
                {
                    if (ctx.vehicle.compute.eventLevelB == 0)
                    {
                        ctx.vehicle.compute.eventLevelB = 1;
                    }
                }
                else if (ctx.vehicle.compute.eventLevelB < 2)
                {
                    ctx.vehicle.compute.eventLevelB = 2;
                }
            }
            if ((double)cTrigger < (double)metricC)
            {
                if ((double)metricC <= (double)cLevel1Max)
                {
                    if (ctx.vehicle.compute.eventLevelC == 0)
                    {
                        ctx.vehicle.compute.eventLevelC = 1;
                    }
                }
                else if (ctx.vehicle.compute.eventLevelC < 2)
                {
                    ctx.vehicle.compute.eventLevelC = 2;
                }
            }

            if (ctx.vehicle.compute.peakEventLevelB < ctx.vehicle.compute.eventLevelB)
            {
                ctx.vehicle.compute.peakEventLevelB = ctx.vehicle.compute.eventLevelB;
                ctx.vehicle.compute.peakEventSourceAb = ctx.vehicle.compute.eventSourceAb;
            }
            if (ctx.vehicle.compute.peakEventLevelA < ctx.vehicle.compute.eventLevelA)
            {
                ctx.vehicle.compute.peakEventLevelA = ctx.vehicle.compute.eventLevelA;
                ctx.vehicle.compute.peakEventSourceAb = ctx.vehicle.compute.eventSourceAb;
            }
            if (ctx.vehicle.compute.peakEventLevelC < ctx.vehicle.compute.eventLevelC)
            {
                ctx.vehicle.compute.peakEventLevelC = ctx.vehicle.compute.eventLevelC;
                ctx.vehicle.compute.peakEventSourceC = ctx.vehicle.compute.eventSourceC;
            }
            ctx.vehicle.compute.lastForceTick = tick;
        }

        private static void UpdateForceHistory(
            ref VehicleCtx ctx,
            float dt,
            ref GmVec3 oldForce,
            ref GmVec3 oldImpulse)
        {
            GmSpringFloat springA = ctx.vehicle.compute.springA;
            VehicleBase.GmSpringFloatIntegrate(ref springA, dt);
            ctx.vehicle.compute.springA = springA;
            float history = (float)(double)(((float)(double)(oldForce.x * dt)) + oldImpulse.x);
            history = ClampSymmetric(history, ctx.vehicle.compute.historyForceLimit);
            float adjustment = (float)(double)(((float)(double)(history / ctx.vehicle.compute.historyForceLimit))
                * ctx.vehicle.compute.historyForceScale);
            ctx.vehicle.compute.springA.value = ClampSymmetric(
                ctx.vehicle.compute.springA.value,
                ctx.vehicle.compute.springValueLimit);
            ctx.vehicle.compute.springA.velocity = ClampSymmetric(
                (float)(double)(ctx.vehicle.compute.springA.velocity + adjustment),
                ctx.vehicle.compute.historyForceScale);

            GmSpringFloat springC = ctx.vehicle.compute.springC;
            VehicleBase.GmSpringFloatIntegrate(ref springC, dt);
            ctx.vehicle.compute.springC = springC;
            float history2 = (float)(double)(((float)(double)(-oldForce.z * dt)) - oldImpulse.z);
            history2 = ClampSymmetric(history2, ctx.vehicle.compute.historyForceLimit);
            float adjustment2 = (float)(double)(((float)(double)(history2 / ctx.vehicle.compute.historyForceLimit))
                * ctx.vehicle.compute.historyForceScale);
            ctx.vehicle.compute.springC.value = ClampSymmetric(
                ctx.vehicle.compute.springC.value,
                ctx.vehicle.compute.springValueLimit);
            ctx.vehicle.compute.springC.velocity = ClampSymmetric(
                (float)(double)(ctx.vehicle.compute.springC.velocity + adjustment2),
                ctx.vehicle.compute.historyForceScale);
        }

        private static void UpdateContactAccumulators(ref VehicleCtx ctx, float dt, ref GmVec3 linearSpeed)
        {
            float direction = VehicleContact.IsAllWheelGroundContactId(ref ctx, 6) ? 1.0f : -1.0f;
            float speedKmh =
                Math.Abs((float)((double)linearSpeed.z * KhmPerMps));
            if (ctx.tuning.compute.contactRiseCurve == null)
            {
                throw new InvalidOperationException("tmnf: contact rise curve missing");
            }
            if (ctx.tuning.compute.contactDecayCurve == null)
            {
                throw new InvalidOperationException("tmnf: contact decay curve missing");
            }

            uint lower = 0;
            float curveValue = Curve.CfuncKeysRealGetValue(
                ctx.tuning.compute.contactRiseCurve, speedKmh, ref lower);
            float delta = (float)(double)(((float)(double)(curveValue * dt)) * direction);
            ctx.vehicle.compute.contactRise =
                Clamp01((float)(double)(delta + ctx.vehicle.compute.contactRise));

            lower = 0;
            curveValue = Curve.CfuncKeysRealGetValue(
                ctx.tuning.compute.contactDecayCurve, speedKmh, ref lower);
            delta = (float)(double)(((float)(double)(curveValue * dt)) * direction);
            ctx.vehicle.compute.contactDecay =
                Clamp01((float)(double)(delta + ctx.vehicle.compute.contactDecay));
        }

        private static void ResetContactFrame(ref VehicleCtx ctx)
        {
            foreach (CSceneVehicleCarWheel wheel in ctx.vehicle.wheels)
            {
                wheel.realTime.hasGroundContact = 0;
                wheel.realTime.groundContactCount = 0;
                wheel.realTime.field54 = GmVec3.Zero;
                wheel.realTime.field90 = GmVec3.Zero;
                // memcpy of a zero u16 into the low half
                wheel.realTime.contactMaterialId =
                    (int)(((uint)wheel.realTime.contactMaterialId & 0xFFFF0000u) | 0u);
                wheel.field15c = 0;
            }
            ctx.vehicle.compute.eventMetricA = 0.0f;
            ctx.vehicle.compute.state5d8 = 0;
            ctx.vehicle.compute.eventMetricB = 0.0f;
            ctx.vehicle.airControlImmediate = 0;
            ctx.vehicle.compute.eventMetricC = 0.0f;
            ctx.vehicle.sideContact = 0;
            ctx.vehicle.wheelContactAbsorbCount = 0;
            ctx.vehicle.bodyContactCount = 0;
            ctx.vehicle.bodyContactPositionSum = GmVec3.Zero;
            ctx.vehicle.bodyContactNormalSum = GmVec3.Zero;
        }

        private static void FinishActiveFrame(
            ref VehicleCtx ctx,
            float dt,
            ref GmVec3 linearSpeed,
            ref GmVec3 oldForce,
            ref GmVec3 oldImpulse,
            float effectCurvePosition)
        {
            GmVec3 force = ctx.dyna.GetLocalForce();
            float inverseDivisor = (float)(double)(1.0f / ctx.tuning.compute.normalizedForceDivisor);
            ctx.vehicle.compute.normalizedForce = new GmVec3(
                (float)(double)(inverseDivisor * force.x),
                (float)(double)(force.y * inverseDivisor),
                (float)(double)(inverseDivisor * force.z));

            if (ctx.tuning.compute.effectCurve == null)
            {
                throw new InvalidOperationException("tmnf: effect curve missing");
            }
            uint lower = 0;
            float curveValue = Curve.CfuncKeysRealGetValue(
                ctx.tuning.compute.effectCurve, effectCurvePosition, ref lower);
            float delta = (float)(double)(dt * ((float)(double)(curveValue + ctx.tuning.compute.effectCurveBias)));
            ctx.vehicle.compute.effectAccumulator =
                Clamp01((float)(double)(ctx.vehicle.compute.effectAccumulator + delta));

            UpdateForceHistory(ref ctx, dt, ref oldForce, ref oldImpulse);
            UpdateContactAccumulators(ref ctx, dt, ref linearSpeed);
            ResetContactFrame(ref ctx);
        }

        /// <summary>0x007C69E0: the top force caller for one 10 ms tick.</summary>
        public static void CsceneVehicleCarComputeForces(
            ref VehicleCtx ctx,
            ComputeExternal external,
            float dt)
        {
            GmVec3 oldImpulse = ctx.vehicle.totalImpulseAdded;
            GmVec3 oldForce = ctx.vehicle.totalForceAdded;
            uint integrationFlags = ctx.vehicle.integrationFlags;

            ctx.vehicle.totalForceAdded = GmVec3.Zero;
            ctx.vehicle.totalImpulseAdded = GmVec3.Zero;

            if ((integrationFlags & 0x30u) != 0
                || (double)ctx.vehicle.compute.simulationGate < 0.0)
            {
                GmVec3 zero = GmVec3.Zero;
                ctx.dyna.SetLocalLinearSpeed(ref zero);
                ctx.dyna.SetLocalAngularSpeed(ref zero);
                ctx.dyna.SetLocalForce(ref zero);
                ctx.dyna.SetLocalTorque(ref zero);
                return;
            }

            if (external.fakeContactsActive)
            {
                throw new InvalidOperationException("tmnf: fake-contacts branch not ported");
            }
            VehicleAux.CreateFakeContacts(ref ctx, external.bodyIso);
            VehicleAux.IntegrateVehicle(ref ctx, dt);

            uint tick = VehicleContact.CmwTimerAdapterGetTickTime(ref ctx);
            bool grounded = VehicleContact.IsGroundContact(ref ctx);
            ctx.dyna.@params.forceFieldScale = grounded
                ? ctx.tuning.compute.groundedForceFieldScale
                : ctx.tuning.compute.airborneForceFieldScale;
            ctx.dyna.@params.dragLinear = grounded
                ? 0.0f
                : ctx.tuning.compute.airborneLinearDrag;

            if ((integrationFlags & 2u) == 0)
            {
                return;
            }

            GmVec3 linearSpeed = ctx.dyna.GetLocalLinearSpeed();
            if ((integrationFlags & 8u) != 0)
            {
                linearSpeed.x = 0.0f;
                linearSpeed.z = 0.0f;
                SetVehicleLinearSpeed(ref ctx, ref linearSpeed);
                FinishActiveFrame(ref ctx, dt, ref linearSpeed, ref oldForce, ref oldImpulse, 0.0f);
                return;
            }

            GmVec3 angularSpeed = ctx.dyna.GetLocalAngularSpeed();
            GmVec3 existingForce = ctx.dyna.GetLocalForce();
            ctx.vehicle.compute.computedBrakeForce = 0.0f;
            VehicleContact.ApplyFrictionForces(ref ctx, ref linearSpeed);
            DbgStage(ref ctx, "FRIC");
            ClampVehicleLinearSpeed(ref ctx, ref linearSpeed);
            var (groundMaterial, hasGroundMaterial) =
                VehicleContact.ComputeVehicleGroundMaterialVals(ref ctx);
            var (slopeAdherence, slopeSecondary) =
                VehicleContact.GetSlopeAdherence(ref ctx, ref existingForce);
            float steeringAngle = ComputeSteeringAngle(ref ctx, ref linearSpeed);
            ctx.vehicle.currentLocalSpeed = linearSpeed;

            if (external.waterForcesActive)
            {
                throw new InvalidOperationException("tmnf: water-forces branch not ported");
            }
            // 0x007C69E0 tests +0x354 for 3, 4, 5 in that order; the else
            // branch is Model 3 — the legacy models are not ported.
            int model6Sliding = 0;
            float effectCurvePosition = 0.0f;
            if (ctx.tuning.contact.frictionModel == 5)
            {
                VehicleModel6.ComputeForcesModel6(
                    ref ctx,
                    ref external.ext,
                    external.water,
                    dt, // model_value
                    ref existingForce,
                    slopeAdherence,       // lateral_force_factor
                    slopeSecondary,       // longitudinal_force_factor
                    ref linearSpeed,
                    ref angularSpeed,
                    steeringAngle,
                    hasGroundMaterial,    // grounded
                    ref groundMaterial,
                    ref model6Sliding,
                    ref effectCurvePosition);
                DbgStage(ref ctx, "M6");
            }
            else
            {
                throw new InvalidOperationException("tmnf: legacy car models (friction_model != 5) are not ported");
            }

            {
                var (anyContact, anyActiveContact) = ScanWheelContacts(ref ctx);
                if (anyContact)
                {
                    float braking = (float)(double)(-(ctx.vehicle.inputBrake)
                        * ctx.vehicle.compute.brakeInputScale);
                    float groundedDrag = (float)(double)(ctx.tuning.compute.groundedDragTerm
                        * ctx.vehicle.compute.groundedDragScale);

                    ctx.vehicle.compute.computedBrakeForce =
                        (float)(double)(braking - groundedDrag);
                }
                if (anyActiveContact
                    && (double)ctx.tuning.compute.activeContactStopThreshold < 0.0)
                {
                    linearSpeed.x = 0.0f;
                    SetVehicleLinearSpeed(ref ctx, ref linearSpeed);
                }
                // The reset argument is [esp+0x2c]: any wheel with contact
                // that is active.
                VehicleAux.ComputeAirControl(
                    ref ctx, ref angularSpeed, tick, grounded, anyActiveContact);
            }
            DbgStage(ref ctx, "BRK");

            ApplyAirEffect(ref ctx, external, ref existingForce, tick, grounded);
            DbgStage(ref ctx, "AEF");
            UpdateEventLevels(ref ctx, tick);
            UpdateSurfaceEffects(ref ctx, external, tick);
            DbgStage(ref ctx, "SUR");
            external.postForce?.Invoke(ctx);

            FinishActiveFrame(
                ref ctx, dt, ref linearSpeed, ref oldForce, ref oldImpulse,
                effectCurvePosition);
        }

        /// <summary>0x007C7D40: the scene callback form (provenance kept).</summary>
        public static void CcallbackSceneVehicleCarComputeForcesComputeForces(
            ref VehicleCtx ctx,
            ComputeExternal external,
            float dt)
        {
            CsceneVehicleCarComputeForces(ref ctx, external, dt);
        }
    }
}
