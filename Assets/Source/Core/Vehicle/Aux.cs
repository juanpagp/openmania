using System;

namespace Tmnf.Sim
{
    /// <summary>vehicle_aux.c: turbo/roulette, air control, water, integration, fake contacts.</summary>
    public static class VehicleAux
    {
        // AUX_EPSILON 0x1.4f8b58p-17f
        private static readonly float AuxEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3727_C5AC));
        // ROULETTE_LOW 0x1.24924ap-1f — float 4/7
        private static readonly float RouletteLow =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F12_4925));
        // ROULETTE_HIGH 0x1.b6db6ep-1 — float 6/7 promoted to double
        private static readonly double RouletteHigh =
            BitConverter.Int64BitsToDouble(unchecked((long)0x3FEB_6DB6_E000_0000));
        // 0x1.ccccccp-1 — float-widened 0.9
        private static readonly double Float09AsF64 =
            BitConverter.Int64BitsToDouble(unchecked((long)0x3FEC_CCCC_C000_0000));
        // 0x1.ccccccp+1 — float-widened 3.6
        private static readonly double Float36AsF64 =
            BitConverter.Int64BitsToDouble(unchecked((long)0x400C_CCCC_C000_0000));
        // 0x1.921fb6p+1 — float pi promoted to double
        private static readonly double FloatPiAsF64 =
            BitConverter.Int64BitsToDouble(unchecked((long)0x4009_21FB_6000_0000));

        private const uint TmnfFakeContactMaskWidth = 128;
        private const uint TmnfFakeContactMaskHeight = 128;

        /// <summary>The native 0x00586200 view for the two auxiliary curves.</summary>
        private static float AuxCurveGetValue(CFuncKeysReal curve, float position)
        {
            uint lower, upper;
            float difference;
            float blend;
            float lowerWeight;

            if (curve.keys.count == 0)
            {
                return 0.0f;
            }
            if (curve.keys.count == 1)
            {
                return curve.values[0];
            }
            if (position < curve.keys.lowerBounds[0])
            {
                return curve.values[0];
            }
            lower = curve.keys.count - 1;
            if (curve.keys.upperBounds[lower] < position)
            {
                return curve.values[lower];
            }

            lower = 0;
            upper = 1;
            uint attempts = 1;
            while (attempts <= curve.keys.count)
            {
                if (curve.keys.lowerBounds[lower] <= position
                    && position <= curve.keys.upperBounds[upper])
                {
                    break;
                }
                lower = upper;
                upper += 1;
                if (upper == curve.keys.count)
                {
                    upper = 0;
                }
                attempts += 1;
            }
            if (curve.interpolation == 1)
            {
                return curve.values[lower];
            }

            difference = (float)(double)(curve.keys.positions[upper] - curve.keys.positions[lower]);
            if (Math.Abs((double)difference) < (double)AuxEpsilon)
            {
                blend = 0.0f;
            }
            else
            {
                blend = (float)(double)(((float)(double)(position - curve.keys.positions[lower])) / difference);
            }
            lowerWeight = (float)(1.0 - (double)blend);
            return (float)(double)(((float)(double)(lowerWeight * curve.values[lower])) + ((float)(double)(blend * curve.values[upper])));
        }

        /// <summary>0x008E0D70: rotate a matrix around Y (used by IntegrateVehicle).</summary>
        private static void Mat3RotateY(ref GmMat3 matrix, float angle)
        {
            float sine = Fp.X87Sin(angle);
            float cosine = Fp.X87Cos(angle);
            float negativeSine = -sine;

            float oldValue = matrix.m0;
            matrix.m0 = (float)(double)(((float)(double)(matrix.m6 * sine)) + ((float)(double)(cosine * oldValue)));
            matrix.m6 = (float)(double)(((float)(double)(matrix.m6 * cosine)) + ((float)(double)(negativeSine * oldValue)));
            oldValue = matrix.m1;
            matrix.m1 = (float)(double)(((float)(double)(oldValue * cosine)) + ((float)(double)(matrix.m7 * sine)));
            matrix.m7 = (float)(double)(((float)(double)(matrix.m7 * cosine)) + ((float)(double)(negativeSine * oldValue)));
            oldValue = matrix.m2;
            matrix.m2 = (float)(double)(((float)(double)(oldValue * cosine)) + ((float)(double)(matrix.m8 * sine)));
            matrix.m8 = (float)(double)(((float)(double)(matrix.m8 * cosine)) + ((float)(double)(negativeSine * oldValue)));
        }

        /// <summary>0x004FF950/0x004FFAC0: map coordinate with truncating fistp.</summary>
        private static uint WaterMapCoordinate(float value, float origin, float cell)
        {
            float offset = (float)(double)(value - origin);
            float scaled = (float)(double)(offset / cell);
            // (uint32_t)(int64_t)F(x): truncation toward zero, low 32 bits.
            long wide = Libm.RustAsI64(scaled);
            return unchecked((uint)wide);
        }

        private static bool WaterMapIsInside(TmnfTrackWater map, float x, float z)
        {
            uint ix = WaterMapCoordinate(x, map.originX, map.cellX);
            uint iz = WaterMapCoordinate(z, map.originZ, map.cellZ);
            return ix < map.width && iz < map.height;
        }

        private static byte WaterMapValue(TmnfTrackWater map, float x, float z)
        {
            uint ix = WaterMapCoordinate(x, map.originX, map.cellX);
            uint iz = WaterMapCoordinate(z, map.originZ, map.cellZ);
            if (ix >= map.width || iz >= map.height)
            {
                return map.defaultCell;
            }
            return map.cells[(int)(iz * map.width + ix)];
        }

        private static float WaterLengthSquared(ref GmVec3 v)
        {
            return (float)(double)(((float)(double)(((float)(double)(v.y * v.y)) + ((float)(double)(v.x * v.x)))) + ((float)(double)(v.z * v.z)));
        }

        /// <summary>0x007F3F40: water friction (curve +0x218 from key zero).</summary>
        private static float WaterFrictionFromSpeed(CSceneVehicleCarTuningAux tuning, float speed)
        {
            float kmh = (float)((double)speed * Float36AsF64);
            uint lowerIndex = 0;
            float value = 0.0f;
            if (tuning.waterFrictionCurve == null)
            {
                // The decode never binds this curve (an upstream quirk);
                // in-water ticks panic — kept faithful.
                throw new InvalidOperationException(
                    "tmnf: water friction curve is absent (the original dereferences it unconditionally)");
            }
            Curve.CfuncKeysRealGetValueOut(tuning.waterFrictionCurve, kmh, ref value, ref lowerIndex);
            return value;
        }

        /// <summary>0x007BE690: central impulse in the car's local frame.</summary>
        private static void AddVehicleCentralImpulse(ref VehicleCtx ctx, ref GmVec3 impulse)
        {
            ctx.dyna.AddLocalImpulse(ref impulse);
            ctx.vehicle.totalImpulseAdded.x = (float)(double)(ctx.vehicle.totalImpulseAdded.x + impulse.x);
            ctx.vehicle.totalImpulseAdded.y = (float)(double)(impulse.y + ctx.vehicle.totalImpulseAdded.y);
            ctx.vehicle.totalImpulseAdded.z = (float)(double)(impulse.z + ctx.vehicle.totalImpulseAdded.z);
        }

        /// <summary>0x007C3CC8..: fmod by the period, |.|/period * extent, fistp.</summary>
        private static uint FakeContactMaskCoordinate(float value, float period, uint extent)
        {
            float wrapped = Fp.X87Fmod(value, period);
            float fraction = (float)(double)(Math.Abs(wrapped) / period);
            float scaled = (float)(double)(fraction * extent);
            uint coordinate = unchecked((uint)Fp.Ftol(scaled));

            if (coordinate >= extent)
            {
                throw new InvalidOperationException("tmnf: fake contact mask coordinate outside the image");
            }
            return coordinate;
        }

        // ------------------------------------------------------------------
        // roulette + turbo
        // ------------------------------------------------------------------

        /// <summary>0x007BC820: maps a roulette remainder to {0, .5, 1}.</summary>
        public static float GetRouletteValue01(uint value, uint divisor)
        {
            uint remainder = value % divisor;
            float numerator = Fp.U32ToX87Float(remainder);
            float denominator = Fp.U32ToX87Float(divisor);
            float ratio = (float)(double)(numerator / denominator);

            if (ratio < RouletteLow)
            {
                return 0.0f;
            }
            if ((double)ratio < RouletteHigh)
            {
                return 0.5f;
            }
            return 1.0f;
        }

        /// <summary>0x007BC890: roulette value to boost multiplier.</summary>
        public static float GetRouletteBoostFactorFromValue01(float value)
        {
            return (float)(double)(value + 1.0f);
        }

        /// <summary>0x007BC8B0: advances the active turbo interval.</summary>
        public static void UpdateTurbo(ref VehicleCtx ctx, uint tick)
        {
            if (ctx.vehicle.turboType != TMNFVehicleTurboType.None)
            {
                if (ctx.vehicle.turboEndTick < tick)
                {
                    ctx.vehicle.turboType = TMNFVehicleTurboType.None;
                }
                if (ctx.vehicle.turboType != TMNFVehicleTurboType.None)
                {
                    float elapsed =
                        Fp.U32ToX87Float(unchecked(tick - ctx.vehicle.turboStartTick));
                    float duration = Fp.U32ToX87Float(
                        unchecked(ctx.vehicle.turboEndTick - ctx.vehicle.turboStartTick));

                    ctx.vehicle.turboProgress = (float)(double)(elapsed / duration);
                    return;
                }
            }
            ctx.vehicle.turboProgress = 0.0f;
        }

        /// <summary>0x007BCF90: starts or refreshes a turbo.</summary>
        public static void EnableTurbo(
            ref VehicleCtx ctx,
            uint tick,
            uint duration,
            float factor,
            TMNFVehicleTurboType turboType,
            uint rouletteToken)
        {
            if (ctx.vehicle.turboType != turboType)
            {
                ctx.vehicle.turboStartTick = tick;
                // (The turbo sound callback is audio-only; not represented.)
                ctx.vehicle.rouletteToken = 0;
            }
            if (turboType == TMNFVehicleTurboType.Normal)
            {
                ctx.vehicle.turboFactor = factor;
            }
            else if (turboType == TMNFVehicleTurboType.Roulette
                && ctx.vehicle.rouletteToken != rouletteToken)
            {
                float value = GetRouletteValue01(
                    unchecked(tick - ctx.vehicle.turboEpochTick),
                    ctx.vehicle.rouletteModulus);

                ctx.vehicle.rouletteValue = value;
                ctx.vehicle.turboFactor = (float)(double)(GetRouletteBoostFactorFromValue01(value) * factor);
                ctx.vehicle.rouletteToken = rouletteToken;
            }
            ctx.vehicle.turboEndTick = unchecked(tick + duration);
            ctx.vehicle.turboType = turboType;
        }

        // ------------------------------------------------------------------
        // suspension integration
        // ------------------------------------------------------------------

        /// <summary>0x007BD3F0: integrates suspension and its surface transform.</summary>
        public static void WheelIntegrate(ref VehicleCtx ctx, int wheelIndex, float dt)
        {
            switch (ctx.tuning.@base.suspensionModel)
            {
                case 0:
                {
                    float absorb;
                    {
                        absorb = (float)(double)(ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb
                            - ctx.vehicle.wheels[wheelIndex].realTime.field08);
                        ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb = absorb;
                        ctx.vehicle.wheels[wheelIndex].realTime.field08 = 0.0f;
                    }
                    ctx.vehicle.wheels[wheelIndex].surfaceLocation =
                        ctx.vehicle.wheels[wheelIndex].surfaceSource;
                    float spring = (float)(double)(((float)(double)(ctx.tuning.@base.suspensionRestLength - absorb))
                        * ctx.tuning.@base.suspensionStiffness);
                    float damping = (float)(double)(ctx.tuning.@base.suspensionDamping
                        * ctx.vehicle.wheels[wheelIndex].realTime.field04);
                    float acceleration = (float)(double)(spring - damping);
                    float velocity = (float)(double)(((float)(double)(acceleration * dt))
                        + ctx.vehicle.wheels[wheelIndex].realTime.field04);
                    ctx.vehicle.wheels[wheelIndex].realTime.field04 = velocity;
                    absorb = (float)(double)(((float)(double)(velocity * dt)) + absorb);
                    ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb = absorb;
                    float verticalOffset = -absorb;
                    float horizontalZero = (float)(double)(verticalOffset * 0.0f);
                    GmIso4 surfaceLocation = ctx.vehicle.wheels[wheelIndex].surfaceLocation;
                    surfaceLocation.t0 = (float)(double)(surfaceLocation.t0 + horizontalZero);
                    surfaceLocation.t1 = (float)(double)(verticalOffset + surfaceLocation.t1);
                    surfaceLocation.t2 = (float)(double)(surfaceLocation.t2 + horizontalZero);
                    ctx.vehicle.wheels[wheelIndex].surfaceLocation = surfaceLocation;
                    break;
                }
                case 1:
                case 2:
                {
                    float oldAbsorb = ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb;
                    float effective = (float)(double)(oldAbsorb - ctx.vehicle.wheels[wheelIndex].realTime.field08);
                    float delta = (float)(double)(ctx.tuning.@base.suspensionRestLength - effective);
                    float correction = (float)(double)(((float)(double)(delta * dt)) * ctx.tuning.aux.suspensionFollowRate);

                    ctx.vehicle.wheels[wheelIndex].surfaceLocation =
                        ctx.vehicle.wheels[wheelIndex].surfaceSource;
                    float absorb = (float)(double)(correction + effective);
                    ctx.vehicle.wheels[wheelIndex].realTime.field04 =
                        (float)(double)(((float)(double)(absorb - oldAbsorb)) / dt);
                    ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb = absorb;
                    ctx.vehicle.wheels[wheelIndex].realTime.field08 = 0.0f;
                    float verticalOffset = -absorb;
                    float horizontalZero = (float)(double)(verticalOffset * 0.0f);
                    GmIso4 surfaceLocation = ctx.vehicle.wheels[wheelIndex].surfaceLocation;
                    surfaceLocation.t0 = (float)(double)(horizontalZero + surfaceLocation.t0);
                    surfaceLocation.t1 = (float)(double)(surfaceLocation.t1 + verticalOffset);
                    surfaceLocation.t2 = (float)(double)(surfaceLocation.t2 + horizontalZero);
                    ctx.vehicle.wheels[wheelIndex].surfaceLocation = surfaceLocation;
                    break;
                }
            }
        }

        // ------------------------------------------------------------------
        // air control
        // ------------------------------------------------------------------

        /// <summary>0x007BF1D0: local angular control while airborne.</summary>
        public static void ComputeAirControl(
            ref VehicleCtx ctx,
            ref GmVec3 angularSpeed,
            uint tick,
            bool suppressTorque,
            bool reset)
        {
            GmVec3 torqueDirection = new GmVec3(-angularSpeed.x, -angularSpeed.y, -angularSpeed.z);
            bool reversed = false;

            if ((ctx.tuning.@base.engineModel == 4 || ctx.tuning.@base.engineModel == 5)
                && ctx.vehicle.airControlLocked != 0)
            {
                return;
            }
            bool skipControl;
            if (reset)
            {
                ctx.vehicle.airControlTick = tick;
                ctx.vehicle.airControlSpeed = angularSpeed;
                skipControl = true;
            }
            else if (ctx.vehicle.airControlImmediate != 0)
            {
                ctx.vehicle.airControlSpeed = angularSpeed;
                skipControl = true;
            }
            else
            {
                skipControl = ctx.tuning.aux.airControlWindowTicks
                    <= unchecked(tick - ctx.vehicle.airControlTick);
            }

            if (!skipControl)
            {
                GmVec3 controlledSpeed = angularSpeed;

                if ((AuxEpsilon < ctx.vehicle.inputSteer
                    && (double)ctx.vehicle.airControlSpeed.y < 0.0)
                    || (ctx.vehicle.inputSteer < -AuxEpsilon
                    && 0.0 < (double)ctx.vehicle.airControlSpeed.y))
                {
                    if ((double)ctx.tuning.aux.airReversalThreshold
                        < Math.Abs((double)angularSpeed.y))
                    {
                        reversed = true;
                        ctx.vehicle.airControlSpeed.y = angularSpeed.y;
                    }
                }
                else
                {
                    if ((AuxEpsilon < ctx.vehicle.inputSteer
                        && 0.0 < (double)ctx.vehicle.airControlSpeed.y)
                        || (ctx.vehicle.inputSteer < -AuxEpsilon
                        && (double)ctx.vehicle.airControlSpeed.y < 0.0))
                    {
                        reversed = true;
                    }
                    ctx.vehicle.airControlSpeed.y = angularSpeed.y;
                }
                controlledSpeed.y = ctx.vehicle.airControlSpeed.y;

                if (ctx.tuning.@base.engineModel == 4 || ctx.tuning.@base.engineModel == 5)
                {
                    if (AuxEpsilon < ctx.vehicle.inputBrake
                        && 0.0 < (double)ctx.vehicle.airControlSpeed.x)
                    {
                        ctx.vehicle.airControlSpeed.x = 0.0f;
                    }
                    else
                    {
                        ctx.vehicle.airControlSpeed.x = angularSpeed.x;
                    }
                    controlledSpeed.x = ctx.vehicle.airControlSpeed.x;
                }
                if (reversed)
                {
                    torqueDirection.x = (float)(double)(torqueDirection.x * 3.0f);
                    torqueDirection.y = (float)(double)(torqueDirection.y * 3.0f);
                    torqueDirection.z = (float)(double)(torqueDirection.z * 3.0f);
                }
                if (!suppressTorque)
                {
                    if (ctx.tuning.aux.airVerticalCurve == null)
                    {
                        throw new InvalidOperationException("tmnf: air vertical curve is absent");
                    }
                    float curveValue = AuxCurveGetValue(
                        ctx.tuning.aux.airVerticalCurve,
                        (float)Math.Abs((double)angularSpeed.z));
                    torqueDirection.z = (float)(double)(torqueDirection.z * curveValue);
                }
                // SetLocalAngularSpeed: angVel = rot * controlled_speed.
                ctx.dyna.SetLocalAngularSpeed(ref controlledSpeed);
            }

            // apply_torque:
            if (!suppressTorque)
            {
                float xySquared = (float)(double)(((float)(double)(torqueDirection.y * torqueDirection.y))
                    + ((float)(double)(torqueDirection.x * torqueDirection.x)));
                float lengthSquared = (float)(double)(xySquared + ((float)(double)(torqueDirection.z * torqueDirection.z)));
                float length = Fp.Sqrt(lengthSquared);

                if (AuxEpsilon <= length)
                {
                    float inverseLength = (float)(double)(1.0f / length);
                    GmVec3 torque = GmVec3.Zero;

                    torque.x = (float)(double)(inverseLength * torqueDirection.x);
                    torque.y = (float)(double)(inverseLength * torqueDirection.y);
                    torque.z = (float)(double)(inverseLength * torqueDirection.z);
                    float quadraticTerm = (float)(double)(((float)(double)(ctx.tuning.aux.airTorqueQuadratic * length)) * length);
                    float linearTerm = (float)(double)(length * ctx.tuning.aux.airTorqueLinear);
                    float magnitude = (float)(double)(quadraticTerm + linearTerm);
                    torque.x = (float)(double)(magnitude * torque.x);
                    torque.y = (float)(double)(torque.y * magnitude);
                    torque.z = (float)(double)(magnitude * torque.z);
                    VehicleBase.AddVehicleTorque(ref ctx, ref torque);
                }
            }
        }

        // ------------------------------------------------------------------
        // water forces
        // ------------------------------------------------------------------

        /// <summary>
        /// 0x007C2910: applies the water drag/buoyancy force and torque.
        /// Returns true when applied (false includes the entry-impulse tick).
        /// NOTE: in-water rows throw (the upstream water-friction decode bug).
        /// </summary>
        public static bool ApplyWaterForces(
            ref VehicleCtx ctx,
            TmnfTrackWater water,
            ref GmVec3 existingForce)
        {
            GmMat3 rotation = ctx.dyna.Live().rot;
            GmBoxAligned box = default(GmBoxAligned);
            GmBoxAligned bodyBox = ctx.vehicle.bodyBox;
            GmIso4 liveIso = ctx.dyna.Live().RotPosIso();
            box.SetMult(ref bodyBox, ref liveIso);
            float halfY = Math.Abs(box.halfExtent.y);
            float top = (float)(double)(box.center.y + halfY);
            float bottom = (float)(double)(box.center.y - halfY);

            // map gate
            if (WaterMapIsInside(water, box.center.x, box.center.z)
                || water.defaultCell != 1
                || water.level <= bottom)
            {
                if (top <= water.floor)
                {
                    return false;
                }
                if (water.level <= bottom)
                {
                    return false;
                }
                if (WaterMapValue(water, box.center.x, box.center.z) != 1)
                {
                    return false;
                }
            }
            float depth = (float)(double)(water.level - bottom);
            if (!(0.5f < depth))
            {
                return false;
            }

            GmVec3 localSpeed = ctx.dyna.GetLocalLinearSpeed();
            GmVec3 worldSpeed = GmVec3.Zero;
            worldSpeed.SetMultMat3(ref localSpeed, ref rotation);
            float horizontalSquared =
                (float)(double)(((float)(double)(worldSpeed.x * worldSpeed.x)) + ((float)(double)(worldSpeed.z * worldSpeed.z)));
            float threshold = ctx.tuning.aux.waterEntrySpeedThreshold;

            // entry-impulse eligibility
            if (ctx.vehicle.airControlImmediate == 0
                && (double)depth < Float09AsF64
                && (double)((float)(double)(water.level - top)) < 0.0
                && worldSpeed.y < -AuxEpsilon)
            {
                float ratio;
                uint lowerIndex;
                bool applyImpulse;

                if (horizontalSquared <= (float)(double)(threshold * threshold))
                {
                    float minimum = ctx.tuning.aux.waterEntrySpeedMinimum;
                    float speedSquared = WaterLengthSquared(ref localSpeed);
                    float minimumSquared = (float)(double)(minimum * minimum);

                    applyImpulse = minimumSquared < speedSquared;
                    ratio = 0.0f;
                    // memcpy of stale float bits seeds the curve cache slot
                    lowerIndex = unchecked((uint)BitConverter.SingleToInt32Bits(minimumSquared));
                }
                else
                {
                    float horizontal = Fp.Sqrt(horizontalSquared);

                    ratio = (float)(double)((-horizontal) / worldSpeed.y);
                    applyImpulse = !((double)ratio < 0.0);
                    lowerIndex = unchecked((uint)BitConverter.SingleToInt32Bits(horizontal));
                }
                if (applyImpulse)
                {
                    float vertical = 0.0f;
                    float horizontal = 0.0f;
                    GmVec3 impulse = GmVec3.Zero;

                    if (ctx.tuning.aux.waterImpulseVerticalCurve == null)
                    {
                        throw new InvalidOperationException("tmnf: water impulse vertical curve is absent (the original dereferences it unconditionally)");
                    }
                    if (ctx.tuning.aux.waterImpulseHorizontalCurve == null)
                    {
                        throw new InvalidOperationException("tmnf: water impulse horizontal curve is absent (the original dereferences it unconditionally)");
                    }
                    Curve.CfuncKeysRealGetValueOut(
                        ctx.tuning.aux.waterImpulseVerticalCurve, ratio, ref vertical, ref lowerIndex);
                    Curve.CfuncKeysRealGetValueOut(
                        ctx.tuning.aux.waterImpulseHorizontalCurve, ratio, ref horizontal, ref lowerIndex);
                    impulse.x = (float)(double)((-horizontal) * worldSpeed.x);
                    impulse.y = (float)(double)((-vertical) * worldSpeed.y);
                    impulse.z = (float)(double)((-horizontal) * worldSpeed.z);
                    impulse.MultTranspose(ref rotation);
                    AddVehicleCentralImpulse(ref ctx, ref impulse);
                    return false;
                }
            }

            // drag, angular drag, buoyancy
            float speed = Fp.Sqrt(WaterLengthSquared(ref localSpeed));
            GmVec3 drag = GmVec3.Zero;
            float scale;
            if (AuxEpsilon < speed)
            {
                scale = -WaterFrictionFromSpeed(ctx.tuning.aux, speed);
                drag.x = (float)(double)(scale * localSpeed.x);
                drag.y = (float)(double)(localSpeed.y * scale);
                drag.z = (float)(double)(scale * localSpeed.z);
            }
            GmVec3 localAngular = ctx.dyna.GetLocalAngularSpeed();
            GmVec3 torque = GmVec3.Zero;
            scale = -ctx.tuning.aux.waterAngularDragLinear;
            torque.x = (float)(double)(scale * localAngular.x);
            torque.y = (float)(double)(localAngular.y * scale);
            torque.z = (float)(double)(scale * localAngular.z);
            float angularSpeed = Fp.Sqrt(WaterLengthSquared(ref localAngular));
            scale = (float)(double)((-angularSpeed) * ctx.tuning.aux.waterAngularDragQuadratic);
            torque.x = (float)(double)(((float)(double)(scale * localAngular.x)) + torque.x);
            torque.y = (float)(double)(((float)(double)(localAngular.y * scale)) + torque.y);
            torque.z = (float)(double)(((float)(double)(scale * localAngular.z)) + torque.z);
            GmVec3 buoyancy = new GmVec3(0.0f, -ctx.tuning.aux.waterBuoyancy, 0.0f);
            buoyancy.MultTranspose(ref rotation);
            GmVec3 force = GmVec3.Zero;
            force.x = (float)(double)(((float)(double)(buoyancy.x + drag.x)) - existingForce.x);
            force.y = (float)(double)(((float)(double)(buoyancy.y + drag.y)) - existingForce.y);
            force.z = (float)(double)(((float)(double)(buoyancy.z + drag.z)) - existingForce.z);
            VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
            VehicleBase.AddVehicleTorque(ref ctx, ref torque);
            return true;
        }

        // ------------------------------------------------------------------
        // integration
        // ------------------------------------------------------------------

        /// <summary>0x007C3900: integrates dry wheel, engine and steer state.</summary>
        public static void IntegrateVehicle(ref VehicleCtx ctx, float dt)
        {
            GmVec3 localSpeed = ctx.dyna.Live().linVel;
            GmMat3 liveRot = ctx.dyna.Live().rot;
            localSpeed.MultTranspose(ref liveRot);

            if ((ctx.vehicle.integrationFlags & 1u) != 0)
            {
                float absoluteSpeed = (float)Math.Abs((double)localSpeed.z);
                float steeringScale = (float)(double)(((float)(double)(absoluteSpeed * ctx.tuning.aux.steeringSpeedScale))
                    + ctx.tuning.aux.steeringSpeedBase);

                for (uint i = 0; i < ctx.vehicle.wheelCount; i++)
                {
                    CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[(int)i];
                    wheel.realTime.basis0 = wheel.surfaceSource.Rotation();
                    if (wheel.steerable == 0)
                    {
                        wheel.realTime.blendTarget = 0.0f;
                    }
                    else
                    {
                        float rotation = 0.0f;
                        float degrees = 30.0f;
                        float steeringValue = ctx.vehicle.steeringValue;

                        if (AuxEpsilon <= steeringScale)
                        {
                            rotation = (float)(double)((-steeringValue) / steeringScale);
                        }
                        Mat3RotateY(ref wheel.realTime.basis0, rotation);
                        if (ctx.tuning.aux.steeringAngleCurve != null)
                        {
                            float speedKmh = (float)((double)absoluteSpeed * Float36AsF64);
                            degrees = AuxCurveGetValue(ctx.tuning.aux.steeringAngleCurve, speedKmh);
                        }
                        // pi promoted to double, then / 180
                        float radians = (float)((double)degrees * FloatPiAsF64);
                        radians = (float)((double)radians / 180.0);
                        wheel.realTime.blendTarget = (float)(double)((-steeringValue) * radians);
                    }
                    ctx.vehicle.wheels[(int)i] = wheel;
                    VehicleBase.WheelUpdateSpeedFromVehicleSpeed(ref ctx, (int)i, localSpeed.z, dt);
                    CSceneVehicleCarWheelRealTimeState rt = ctx.vehicle.wheels[(int)i].realTime;
                    VehicleBase.WheelRealTimeStateIntegrate(ref rt, dt);
                    ctx.vehicle.wheels[(int)i].realTime = rt;
                }
            }
            if ((ctx.vehicle.integrationFlags & 2u) != 0)
            {
                for (uint i = 0; i < ctx.vehicle.wheelCount; i++)
                {
                    WheelIntegrate(ref ctx, (int)i, dt);
                }
            }
            if ((ctx.vehicle.integrationFlags & 4u) != 0)
            {
                if (ctx.vehicle.flag60c != 0)
                {
                    ctx.vehicle.engine.rpm = 0.0f;
                }
                else
                {
                    float throttle = ctx.vehicle.engine.reverse != 0
                        ? ctx.vehicle.inputBrake
                        : ctx.vehicle.inputGas;

                    VehicleBase.EngineIntegrate(ref ctx, throttle, dt);
                }
            }

            if (0.0 < (double)ctx.tuning.aux.steeringSlewRate)
            {
                float direction =
                    (0.0 <= (double)((float)(double)(ctx.vehicle.steeringValue - ctx.vehicle.inputSteer)))
                    ? -1.0f
                    : 1.0f;
                float next = (float)(double)(((float)(double)(((float)(double)(ctx.tuning.aux.steeringSlewRate * direction)) * dt))
                    + ctx.vehicle.steeringValue);

                if (ctx.vehicle.inputSteer <= ctx.vehicle.steeringValue)
                {
                    if (next < ctx.vehicle.inputSteer)
                    {
                        next = ctx.vehicle.inputSteer;
                    }
                }
                else if (ctx.vehicle.inputSteer < next)
                {
                    next = ctx.vehicle.inputSteer;
                }
                ctx.vehicle.steeringValue = next;
            }
            else
            {
                ctx.vehicle.steeringValue = ctx.vehicle.inputSteer;
            }

            // finish_integration (world.c callback): the model6 rollover
            // inputs read next tick's values here.
            ctx.vehicle.model6.contactBlockCount = ctx.vehicle.compute.state5d8;
            ctx.vehicle.model6.sideContact = ctx.vehicle.sideContact;
            ctx.vehicle.model6.rolloverAxis = ctx.vehicle.compute.normalizedForce;
        }

        // ------------------------------------------------------------------
        // fake contacts
        // ------------------------------------------------------------------

        /// <summary>0x007C3C00: deterministic bump contacts from the mask image.</summary>
        public static void CreateFakeContacts(ref VehicleCtx ctx, System.Func<uint, GmIso4> resolveBodyIso)
        {
            GmVec3 localSpeed = ctx.dyna.GetLocalLinearSpeed();
            for (uint i = 0; i < ctx.vehicle.wheelCount; i++)
            {
                CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[(int)i];

                if (wheel.realTime.hasGroundContact == 0)
                {
                    continue;
                }
                ushort materialId = (ushort)wheel.realTime.contactMaterialId;
                if (materialId >= ctx.tuning.groundMaterialIndices.Count)
                {
                    throw new InvalidOperationException("tmnf: wheel ground material id out of range");
                }
                TMNFVehicleGroundMaterial material =
                    ctx.tuning.groundMaterialIndices.Count > materialId
                        ? ctx.tuning.groundMaterials[(int)ctx.tuning.groundMaterialIndices[materialId]]
                        : null;
                // A material without a bump mask ends the whole pass; later
                // wheels are not visited.
                if (material == null || material.fakeContactMask == null)
                {
                    return;
                }
                GmVec3 localPosition = new GmVec3(
                    wheel.surfaceSource.t0,
                    wheel.surfaceSource.t1,
                    wheel.surfaceSource.t2);
                GmVec3 worldPosition = GmVec3.Zero;
                GmIso4 liveIso = ctx.dyna.Live().RotPosIso();
                worldPosition.SetMultIso4(ref localPosition, ref liveIso);
                uint x = FakeContactMaskCoordinate(
                    worldPosition.x, material.fakeContactPeriodX, TmnfFakeContactMaskWidth);
                uint y = FakeContactMaskCoordinate(
                    worldPosition.z, material.fakeContactPeriodZ, TmnfFakeContactMaskHeight);
                byte maskValue = material.fakeContactMask[(int)(y * TmnfFakeContactMaskWidth + x)];
                if (maskValue == 0)
                {
                    continue;
                }

                float impactSpeed = (float)((double)maskValue / 255.0);
                impactSpeed = (float)(double)(impactSpeed * localSpeed.z);
                impactSpeed = (float)(double)(impactSpeed * material.fakeContactImpulseScale);
                if (material.fakeContactImpulseLimit < impactSpeed)
                {
                    impactSpeed = material.fakeContactImpulseLimit;
                }

                CHmsPhysicalContact fakeContact = CHmsPhysicalContact.Create();
                fakeContact.normal.y = 1.0f;
                fakeContact.position = localPosition;
                fakeContact.relativeSpeed.y = -impactSpeed;
                fakeContact.otherSurfaceMaterial = materialId;
                VehicleContact.WheelAbsorbContact(ref ctx, (int)i, ref fakeContact, resolveBodyIso);
            }
        }

        /// <summary>Static-body iso resolver signature (wheel contact dispatch).</summary>
        public delegate GmIso4 ResolveBodyIsoFn(uint corpusRef);
    }

    /// <summary>The track's water map as the game holds it.</summary>
    [Serializable]
    public sealed class TmnfTrackWater
    {
        public float cellX;
        public float cellZ;
        public float originX;
        public float originZ;
        public uint width;
        public uint height;
        public byte defaultCell;
        /// <summary>width * height flags.</summary>
        public byte[] cells = new byte[0];
        /// <summary>Number of set cells.</summary>
        public uint cellCount;
        public float level;
        public float floor;
    }
}
