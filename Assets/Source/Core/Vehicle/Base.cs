using System;

namespace Tmnf.Sim
{
    /// <summary>vehicle.c: input mapper, wheel state, spring, impulse math, engine.</summary>
    public static class VehicleBase
    {
        // WHEEL_ANGLE_PERIOD 0x1.921fb6p+10f (1608.4954833984375)
        private static readonly float WheelAnglePeriod =
            BitConverter.Int32BitsToSingle(unchecked((int)0x44C9_0FDB));
        // NORMALIZE_EPSILON 0x1.b7cdfcp-34f
        private static readonly float NormalizeEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EDB_E6FE));
        // WHEEL_SPEED_EPSILON 0x1.4f8b58p-17f
        private static readonly float WheelSpeedEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3727_C5AC));
        // WHEEL_SPEED_DAMPING 0x1.fd70a4p-1 (0.99500000476837158203)
        private static readonly float WheelSpeedDamping =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F7E_B852));

        private static float GmModLocal(float value, float lower, float upper)
        {
            if ((lower < value) && (value < upper))
            {
                return value;
            }
            float period = (float)(double)(upper - lower);
            float relative = (float)(double)(value - lower);
            float result = Fp.X87Fmod(relative, period);
            if (result < 0.0f)
            {
                result = (float)(double)(result + period);
            }
            return (float)(double)(result + lower);
        }

        private static float VecLengthSquared(ref GmVec3 value)
        {
            return (float)(double)(((float)(double)(((float)(double)(value.x * value.x)) + ((float)(double)(value.y * value.y)))) + ((float)(double)(value.z * value.z)));
        }

        private static void NormalizeIfNonzero(ref GmVec3 value)
        {
            float lengthSquared = VecLengthSquared(ref value);
            if (NormalizeEpsilon < lengthSquared)
            {
                float length = Fp.Sqrt(lengthSquared);
                float inverseLength = (float)(double)(1.0f / length);
                value.x = (float)(double)(inverseLength * value.x);
                value.y = (float)(double)(value.y * inverseLength);
                value.z = (float)(double)(inverseLength * value.z);
            }
        }

        private static void Mat3SetUpVAndDov(ref GmMat3 matrix, ref GmVec3 upValue, ref GmVec3 dov)
        {
            GmVec3 line0 = GmVec3.Zero;
            GmVec3 line1 = upValue;
            GmVec3 line2 = GmVec3.Zero;

            line0.x = (float)(double)(((float)(double)(upValue.y * dov.z)) - ((float)(double)(upValue.z * dov.y)));
            line0.y = (float)(double)(((float)(double)(upValue.z * dov.x)) - ((float)(double)(upValue.x * dov.z)));
            line0.z = (float)(double)(((float)(double)(upValue.x * dov.y)) - ((float)(double)(dov.x * upValue.y)));
            NormalizeIfNonzero(ref line0);
            NormalizeIfNonzero(ref line1);
            line2.x = (float)(double)(((float)(double)(line0.y * line1.z)) - ((float)(double)(line0.z * line1.y)));
            line2.y = (float)(double)(((float)(double)(line1.x * line0.z)) - ((float)(double)(line0.x * line1.z)));
            line2.z = (float)(double)(((float)(double)(line0.x * line1.y)) - ((float)(double)(line0.y * line1.x)));
            matrix.m0 = line0.x;
            matrix.m3 = line0.y;
            matrix.m6 = line0.z;
            matrix.m1 = line1.x;
            matrix.m4 = line1.y;
            matrix.m7 = line1.z;
            matrix.m2 = line2.x;
            matrix.m5 = line2.y;
            matrix.m8 = line2.z;
        }

        private static void AddVec3(ref GmVec3 sum, ref GmVec3 value)
        {
            sum.x = (float)(double)(sum.x + value.x);
            sum.y = (float)(double)(value.y + sum.y);
            sum.z = (float)(double)(value.z + sum.z);
        }

        // ------------------------------------------------------------------
        // input pipeline
        // ------------------------------------------------------------------

        /// <summary>0x004FE500: maps timestamped inputs to car controls.</summary>
        public static void UpdateVehicleStateFromInputs(ref TMNFRaceInputs inputs, ref VehicleCtx ctx)
        {
            float gas;
            float brake;
            float steer;

            uint latest = inputs.accelerateTime;
            if (latest < inputs.brakeTime)
            {
                latest = inputs.brakeTime;
            }
            uint eventTime = inputs.gasAnalogTime;
            if (eventTime == latest
                && inputs.accelerate == 0
                && inputs.brake == 0
                && 0.01 < Math.Abs((double)inputs.gasAnalog))
            {
                eventTime = unchecked(latest + 1);
            }

            if (eventTime <= latest)
            {
                gas = inputs.accelerate != 0 ? 1.0f : 0.0f;
                brake = inputs.brake != 0 ? 1.0f : 0.0f;
            }
            else if (!float.IsNaN(inputs.gasAnalog) && 0.3 <= (double)inputs.gasAnalog)
            {
                gas = 1.0f;
                brake = 0.0f;
            }
            else if (!float.IsNaN(inputs.gasAnalog) && (double)inputs.gasAnalog <= -0.3)
            {
                gas = 0.0f;
                brake = 1.0f;
            }
            else
            {
                gas = 0.0f;
                brake = 0.0f;
            }
            ctx.vehicle.inputGas = gas;
            ctx.vehicle.inputBrake = brake;

            latest = inputs.steerLeftTime;
            if (latest < inputs.steerRightTime)
            {
                latest = inputs.steerRightTime;
            }
            eventTime = inputs.steerAnalogTime;
            if (eventTime == latest
                && inputs.steerLeft == 0
                && inputs.steerRight == 0
                && 0.01 < Math.Abs((double)inputs.steerAnalog))
            {
                eventTime = unchecked(latest + 1);
            }

            if (eventTime <= latest)
            {
                if (inputs.steerLeft != 0)
                {
                    steer = -1.0f;
                }
                else if (inputs.steerRight != 0)
                {
                    steer = 1.0f;
                }
                else
                {
                    steer = 0.0f;
                }
            }
            else
            {
                // steer = (float)-F(inputs->steer_analog): exact negation.
                steer = -inputs.steerAnalog;
            }
            ctx.vehicle.inputSteer = steer;
        }

        // ------------------------------------------------------------------
        // wheel real-time state
        // ------------------------------------------------------------------

        /// <summary>0x007C1060: integrates one wheel's angular + blend state.</summary>
        public static void WheelRealTimeStateIntegrate(ref CSceneVehicleCarWheelRealTimeState state, float dt)
        {
            float phaseInput = (float)(double)(((float)(double)(state.field6c * dt)) + state.rotationPhase);

            state.rotationPhase = GmModLocal(phaseInput, 0.0f, WheelAnglePeriod);
            float axisLengthSquared = VecLengthSquared(ref state.field90);
            if (NormalizeEpsilon < axisLengthSquared)
            {
                GmVec3 dov = GmVec3.Zero;

                NormalizeIfNonzero(ref state.field90);
                dov.x = (float)(double)(((float)(double)(state.field90.z * 0.0f)) - ((float)(double)(state.field90.y * 0.0f)));
                dov.y = (float)(double)(((float)(double)(state.field90.x * 0.0f)) - state.field90.z);
                dov.z = (float)(double)(state.field90.y - ((float)(double)(state.field90.x * 0.0f)));
                GmVec3 up = state.field90;
                Mat3SetUpVAndDov(ref state.basis1, ref up, ref dov);
            }
            if (state.blendTarget <= state.blendValue)
            {
                state.blendValue = (float)(double)(state.blendValue - dt);
                if (state.blendValue < state.blendTarget)
                {
                    state.blendValue = state.blendTarget;
                }
            }
            else
            {
                state.blendValue = (float)(double)(state.blendValue + dt);
                if (state.blendTarget < state.blendValue)
                {
                    state.blendValue = state.blendTarget;
                }
            }
        }

        /// <summary>0x007C0EC0: angular wheel speed from contact + driver inputs.</summary>
        public static void WheelUpdateSpeedFromVehicleSpeed(
            ref VehicleCtx ctx,
            int wheelIndex,
            float vehicleSpeed,
            float dt)
        {
            if (ctx.vehicle.wheels[wheelIndex].realTime.hasGroundContact != 0)
            {
                if (ctx.vehicle.forceWheelSpeed != 0 && ctx.vehicle.blockWheelSpeed == 0)
                {
                    ctx.vehicle.wheels[wheelIndex].realTime.field6c = ctx.vehicle.forcedWheelSpeed;
                }
                else
                {
                    ctx.vehicle.wheels[wheelIndex].realTime.field6c =
                        (float)(double)(vehicleSpeed / ctx.vehicle.wheels[wheelIndex].radius);
                }
                return;
            }

            float acceleration = 0.0f;
            float target = 0.0f;
            if (ctx.vehicle.inputBrake <= WheelSpeedEpsilon)
            {
                if (ctx.vehicle.inputGas <= WheelSpeedEpsilon
                    || ctx.vehicle.blockWheelSpeed != 0
                    || ctx.vehicle.flag60c != 0)
                {
                    ctx.vehicle.wheels[wheelIndex].realTime.field6c =
                        (float)((double)ctx.vehicle.wheels[wheelIndex].realTime.field6c * (double)WheelSpeedDamping);
                }
                else
                {
                    target = (float)((double)ctx.vehicle.inputGas * 200.0);
                    acceleration = 100.0f;
                }
            }
            else
            {
                target = (float)(double)(1.0f - ctx.vehicle.inputBrake);
                if (!float.IsNaN(target))
                {
                    if ((double)target <= 0.0)
                    {
                        target = 0.0f;
                    }
                    else if (1.0 <= (double)target)
                    {
                        target = 1.0f;
                    }
                }
                acceleration = -100.0f;
            }

            if ((double)WheelSpeedEpsilon <= Math.Abs((double)acceleration))
            {
                float speed = (float)(double)(((float)(double)(acceleration * dt)) + ctx.vehicle.wheels[wheelIndex].realTime.field6c);
                ctx.vehicle.wheels[wheelIndex].realTime.field6c = speed;
                if ((0.0 < (double)acceleration && (target < speed))
                    || (((double)acceleration < 0.0) && (speed < target)))
                {
                    ctx.vehicle.wheels[wheelIndex].realTime.field6c = target;
                }
            }
        }

        // ------------------------------------------------------------------
        // spring
        // ------------------------------------------------------------------

        /// <summary>0x008F46C0: integrates a scalar damped spring.</summary>
        public static void GmSpringFloatIntegrate(ref GmSpringFloat spring, float dt)
        {
            float displacement = (float)(double)(spring.target - spring.value);
            float springForce = (float)(double)(displacement * spring.stiffness);
            float dampingForce = (float)(double)(spring.damping * spring.velocity);
            float acceleration = (float)(double)(springForce - dampingForce);
            float velocity = (float)(double)(((float)(double)(acceleration * dt)) + spring.velocity);
            spring.velocity = velocity;
            spring.value = (float)(double)(((float)(double)(dt * velocity)) + spring.value);
        }

        // ------------------------------------------------------------------
        // impulse math
        // ------------------------------------------------------------------

        /// <summary>0x007BD090: one contact impulse from effective point mass.</summary>
        public static GmVec3 SdynaMathComputeImpulse(
            float mass,
            ref GmMat3 inverseInertia,
            float restitution,
            ref GmVec3 relativeSpeed,
            ref GmVec3 normal,
            ref GmVec3 leverArm)
        {
            GmVec3 angular = GmVec3.Zero;
            angular.x = (float)(double)(((float)(double)(normal.z * leverArm.y)) - ((float)(double)(normal.y * leverArm.z)));
            angular.y = (float)(double)(((float)(double)(normal.x * leverArm.z)) - ((float)(double)(leverArm.x * normal.z)));
            angular.z = (float)(double)(((float)(double)(normal.y * leverArm.x)) - ((float)(double)(normal.x * leverArm.y)));
            angular.MultMat3(ref inverseInertia);

            GmVec3 projected = GmVec3.Zero;
            projected.x = (float)(double)(((float)(double)(leverArm.z * angular.y)) - ((float)(double)(angular.z * leverArm.y)));
            projected.y = (float)(double)(((float)(double)(leverArm.x * angular.z)) - ((float)(double)(leverArm.z * angular.x)));
            projected.z = (float)(double)(((float)(double)(angular.x * leverArm.y)) - ((float)(double)(leverArm.x * angular.y)));
            float relativeNormalSpeed = (float)(double)(((float)(double)(((float)(double)(relativeSpeed.y * normal.y)) + ((float)(double)(relativeSpeed.x * normal.x))))
                + ((float)(double)(relativeSpeed.z * normal.z)));
            float rotationalMass = (float)(double)(((float)(double)(((float)(double)(normal.x * projected.x)) + ((float)(double)(normal.y * projected.y))))
                + ((float)(double)(projected.z * normal.z)));
            float denominator = (float)(double)(((float)(double)(1.0f / mass)) + rotationalMass);
            float numerator = (float)(double)(((float)(double)(restitution - 1.0f)) * relativeNormalSpeed);
            float scale = (float)(double)(numerator / denominator);
            return new GmVec3(
                (float)(double)(normal.x * scale),
                (float)(double)(normal.y * scale),
                (float)(double)(scale * normal.z));
        }

        // ------------------------------------------------------------------
        // local force pipeline
        // ------------------------------------------------------------------

        /// <summary>0x007BE310: adds a local central force.</summary>
        public static void AddVehicleCentralForce(ref VehicleCtx ctx, ref GmVec3 force)
        {
            GmVec3 worldForce = GmVec3.Zero;
            GmMat3 rot = ctx.dyna.Live().rot;
            worldForce.SetMultMat3(ref force, ref rot);
            ref CHmsStateDyna live = ref ctx.dyna.Live();
            AddVec3(ref live.force, ref worldForce);
            AddVec3(ref ctx.vehicle.totalForceAdded, ref force);
        }

        /// <summary>0x007BE360: adds a local torque.</summary>
        public static void AddVehicleTorque(ref VehicleCtx ctx, ref GmVec3 torque)
        {
            GmVec3 worldTorque = GmVec3.Zero;
            GmMat3 rot = ctx.dyna.Live().rot;
            worldTorque.SetMultMat3(ref torque, ref rot);
            ref CHmsStateDyna live = ref ctx.dyna.Live();
            AddVec3(ref live.torque, ref worldTorque);
        }

        /// <summary>0x007BE2C0: adds a local force at a local point.</summary>
        public static void AddVehicleForce(ref VehicleCtx ctx, ref GmVec3 force, ref GmVec3 point)
        {
            GmVec3 worldForce = GmVec3.Zero;
            GmVec3 worldPoint = GmVec3.Zero;
            GmVec3 centerOfMass = GmVec3.Zero;
            GmVec3 lever = GmVec3.Zero;
            GmVec3 torque = GmVec3.Zero;

            ref CHmsStateDyna live = ref ctx.dyna.Live();
            GmMat3 rot = live.rot;
            worldForce.SetMultMat3(ref force, ref rot);
            worldPoint.SetMultMat3(ref point, ref rot);
            worldPoint.x = (float)(double)(worldPoint.x + live.pos.x);
            worldPoint.y = (float)(double)(worldPoint.y + live.pos.y);
            worldPoint.z = (float)(double)(worldPoint.z + live.pos.z);
            ref CHmsDynaParams prms = ref ctx.DynaParams();
            centerOfMass.SetMultMat3(ref prms.comOffset, ref rot);
            centerOfMass.x = (float)(double)(centerOfMass.x + live.pos.x);
            centerOfMass.y = (float)(double)(centerOfMass.y + live.pos.y);
            centerOfMass.z = (float)(double)(centerOfMass.z + live.pos.z);
            lever.x = (float)(double)(worldPoint.x - centerOfMass.x);
            lever.y = (float)(double)(worldPoint.y - centerOfMass.y);
            lever.z = (float)(double)(worldPoint.z - centerOfMass.z);
            torque.x = (float)(double)(((float)(double)(lever.y * worldForce.z)) - ((float)(double)(lever.z * worldForce.y)));
            torque.y = (float)(double)(((float)(double)(lever.z * worldForce.x)) - ((float)(double)(lever.x * worldForce.z)));
            torque.z = (float)(double)(((float)(double)(lever.x * worldForce.y)) - ((float)(double)(lever.y * worldForce.x)));
            AddVec3(ref live.force, ref worldForce);
            AddVec3(ref live.torque, ref torque);
            AddVec3(ref ctx.vehicle.totalForceAdded, ref force);
        }

        // ------------------------------------------------------------------
        // engine / gearbox
        // ------------------------------------------------------------------

        /// <summary>0x007BE00B: the Steer01..Steer05 engine body.</summary>
        private static void EngineIntegrateOldModels(ref VehicleCtx ctx, float throttle, float dt, bool shiftReady)
        {
            Vehicle vehicle = ctx.vehicle;
            CSceneVehicleCarTuning tuning = ctx.tuning.@base;
            GmVec3 speed = vehicle.currentLocalSpeed;
            float limit = (float)(double)(tuning.forwardSpeedLimitScale * 0.2f);
            float magnitude = Math.Abs(throttle);
            bool canShift = !shiftReady && !(0.0 < (double)vehicle.engine.shiftTimer);
            uint index = vehicle.engine.gear > 1 ? (uint)(vehicle.engine.gear - 1) : 0;
            float upshift = tuning.gearUpshift[(int)index];
            float downshift = tuning.gearDownshift[(int)index];
            float ratio = tuning.gearRatios[(int)index];
            float sum = (float)(double)(((float)(double)(((float)(double)(((float)(double)(speed.x * speed.x)) * 0.3f)) + ((float)(double)(speed.z * speed.z))))
                + ((float)(double)(((float)(double)(speed.y * speed.y)) * 0.1f)));
            float ratioSpeed = (float)(double)(((float)(double)(Fp.Sqrt(sum) / limit)) * ratio);
            float drive;
            float target;

            if (canShift)
            {
                drive = ratioSpeed;
                if (vehicle.engine.reverse != 0)
                {
                    if (vehicle.engine.gear != 0)
                    {
                        vehicle.engine.gear = 0;
                        vehicle.engine.shiftTimer = 0.04f;
                    }
                }
                else if (vehicle.engine.gear == 0)
                {
                    vehicle.engine.gear = 1;
                    vehicle.engine.shiftTimer = 0.04f;
                }
                else if ((upshift < ratioSpeed) && vehicle.engine.gear < 5)
                {
                    vehicle.engine.gear += 1;
                    vehicle.engine.shiftTimer = 0.04f;
                }
                else if ((ratioSpeed < downshift) && vehicle.engine.gear > 1)
                {
                    vehicle.engine.gear -= 1;
                    vehicle.engine.shiftTimer = 0.04f;
                }
            }
            else
            {
                drive = magnitude;
                if (0.0 <= (double)vehicle.engine.shiftTimer
                    && !(((float)(double)(dt + dt)) < vehicle.engine.shiftTimer))
                {
                    vehicle.engine.rpm = (float)(double)(vehicle.engine.rpm - ((float)(double)(((float)(double)(vehicle.engine.maxRpm * dt)) * 1.9f)));
                }
            }
            target = (float)(double)(((float)(double)(vehicle.engine.maxRpm * drive)) - vehicle.engine.rpm);
            vehicle.engine.rpm =
                (float)(double)(((float)(double)(((float)(double)(target * dt)) * (canShift ? 12.0f : 3.5f))) + vehicle.engine.rpm);
        }

        /// <summary>0x007BD700: integrates the engine and gearbox state.</summary>
        public static void EngineIntegrate(ref VehicleCtx ctx, float throttle, float dt)
        {
            bool throttleOn = 0.1 < (double)throttle;
            bool allAirborne = true;
            bool shiftReady;
            float value;

            for (uint i = 0; i < ctx.vehicle.wheelCount; i++)
            {
                if (ctx.vehicle.wheels[(int)i].realTime.hasGroundContact != 0)
                {
                    allAirborne = false;
                    break;
                }
            }
            if (0.0 < (double)ctx.vehicle.engine.shiftTimer)
            {
                ctx.vehicle.engine.shiftTimer = (float)(double)(ctx.vehicle.engine.shiftTimer - dt);
            }
            shiftReady = allAirborne || (0.0 < (double)ctx.vehicle.engine.shiftTimer);

            // clamp_rpm / shift_checks blocks (goto structure flattened with
            // the same evaluation order):
            bool ranOldModels = false;
            bool brokeToClamp = false;
            bool brokeToShiftChecks = false;

            if (ctx.tuning.@base.engineModel != 5)
            {
                EngineIntegrateOldModels(ref ctx, throttle, dt, shiftReady);
                ranOldModels = true;
                brokeToClamp = true;
            }
            else if (shiftReady)
            {
                if (throttleOn)
                {
                    ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(ctx.tuning.@base.engineRpmAccel * dt)) + ctx.vehicle.engine.rpm);
                }
                else
                {
                    ctx.vehicle.engine.rpm = (float)(double)(ctx.vehicle.engine.rpm - ((float)(double)(ctx.tuning.@base.engineRpmDecel * dt)));
                }
                brokeToClamp = true;
            }
            else
            {
                if (ctx.vehicle.driveMode == 1 || ctx.vehicle.driveMode == 2)
                {
                    if (ctx.vehicle.engine.gear != 0)
                    {
                        ctx.vehicle.engineMode = 4;
                    }
                }
                else if (ctx.vehicle.engineMode == 4)
                {
                    ctx.vehicle.engineMode = 0;
                }

                if (ctx.vehicle.engineMode == 2)
                {
                    ctx.vehicle.engineLimitFlag =
                        ((ctx.tuning.@base.speed32c < ctx.vehicle.currentLocalSpeed.z)
                        || (ctx.vehicle.currentLocalSpeed.z < ctx.tuning.@base.speed330)) ? 1 : 0;
                    ctx.vehicle.engine.clutch = 1.0f;
                    ctx.vehicle.engine.targetRpm = (float)(double)(((float)(double)(ctx.tuning.@base.gearAux[1] * 0.0f))
                        + ((float)(double)(Math.Abs(ctx.vehicle.currentLocalSpeed.z)
                            * ctx.tuning.@base.gearRatios[1])));
                    if (ctx.vehicle.engineLimitFlag == 0 && throttleOn)
                    {
                        ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(ctx.tuning.@base.engineRpmLowAccel * dt))
                            + ctx.vehicle.engine.rpm);
                        brokeToShiftChecks = true;
                    }
                    else
                    {
                        value = (float)(double)(ctx.vehicle.engine.rpm - ((float)(double)(ctx.tuning.@base.engineRpmHighDecel * dt)));
                        ctx.vehicle.engine.rpm = value;
                        if (value <= ctx.vehicle.engine.targetRpm)
                        {
                            ctx.vehicle.engine.rpm = ctx.vehicle.engine.targetRpm;
                            ctx.vehicle.engineMode = 0;
                            ctx.vehicle.engineLimitFlag = 0;
                        }
                        brokeToShiftChecks = true;
                    }
                }
                else if (ctx.vehicle.engineMode == 3)
                {
                    ctx.vehicle.engineLimitFlag =
                        ((ctx.vehicle.currentLocalSpeed.z < ctx.tuning.@base.speed338)
                        || (ctx.tuning.@base.speed334 < ctx.vehicle.currentLocalSpeed.z)) ? 1 : 0;
                    ctx.vehicle.engine.clutch = 1.0f;
                    ctx.vehicle.engine.targetRpm = (float)(double)(((float)(double)(ctx.tuning.@base.gearAux[0] * 0.0f))
                        + ((float)(double)(Math.Abs(ctx.vehicle.currentLocalSpeed.z)
                            * ctx.tuning.@base.gearRatios[0])));
                    if (ctx.vehicle.engineLimitFlag == 0 && throttleOn)
                    {
                        ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(ctx.tuning.@base.engineRpmLowAccel * dt))
                            + ctx.vehicle.engine.rpm);
                        brokeToShiftChecks = true;
                    }
                    else
                    {
                        value = (float)(double)(ctx.vehicle.engine.rpm - ((float)(double)(ctx.tuning.@base.engineRpmHighDecel * dt)));
                        ctx.vehicle.engine.rpm = value;
                        if (!(ctx.vehicle.engine.targetRpm < value))
                        {
                            ctx.vehicle.engine.rpm = ctx.vehicle.engine.targetRpm;
                            ctx.vehicle.engineMode = 0;
                            ctx.vehicle.engineLimitFlag = 0;
                        }
                        brokeToShiftChecks = true;
                    }
                }
                else if (ctx.vehicle.engineMode == 4)
                {
                    ctx.vehicle.engine.targetRpm = ctx.vehicle.engine.maxRpm;
                    ctx.vehicle.engine.clutch = 1.15f;
                    if (ctx.vehicle.engine.maxRpm <= ctx.vehicle.engine.rpm)
                    {
                        if (ctx.vehicle.engine.rpm <= ctx.vehicle.engine.maxRpm)
                        {
                            brokeToShiftChecks = true;
                        }
                        else
                        {
                            value = ctx.tuning.@base.engineRpmDecel;
                            ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(value * dt)) + ctx.vehicle.engine.rpm);
                            brokeToShiftChecks = true;
                        }
                    }
                    else
                    {
                        value = ctx.tuning.@base.engineRpmReverseAccel;
                        ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(value * dt)) + ctx.vehicle.engine.rpm);
                        brokeToShiftChecks = true;
                    }
                }
                else
                {
                    if (ctx.vehicle.turboActive != 0 && throttleOn)
                    {
                        value = 1.15f;
                        if ((double)ctx.vehicle.engine.clutch < 1.15)
                        {
                            value = (float)(double)(((float)(double)(((float)(double)(((float)(double)(1.15f - ctx.vehicle.engine.clutch)) * 0.3f)) * dt))
                                + ctx.vehicle.engine.clutch);
                        }
                    }
                    else
                    {
                        value = 1.0f;
                    }
                    ctx.vehicle.engine.clutch = value;
                    uint gearIndex = (uint)ctx.vehicle.engine.gear;
                    float target = (float)(double)(((float)(double)(ctx.tuning.@base.gearAux[(int)gearIndex] * 0.0f))
                        + ((float)(double)(Math.Abs((float)(double)(ctx.vehicle.currentLocalSpeed.z * ctx.vehicle.engine.clutch))
                            * ctx.tuning.@base.gearRatios[(int)gearIndex])));
                    ctx.vehicle.engine.targetRpm = target;
                    if ((ctx.vehicle.engine.reverse == 0 && gearIndex == 0)
                        || (ctx.vehicle.engine.reverse != 0 && gearIndex != 0))
                    {
                        ctx.vehicle.engine.targetRpm = 0.0f;
                        target = 0.0f;
                    }
                    if (target <= ctx.vehicle.engine.rpm)
                    {
                        if (throttleOn)
                        {
                            if (ctx.vehicle.turboActive == 0
                                || ctx.vehicle.driveMode == 1
                                || ctx.vehicle.driveMode == 2)
                            {
                                value = ctx.tuning.@base.engineRpmTurboDecel;
                            }
                            else
                            {
                                value = ctx.tuning.@base.engineRpmDecel;
                            }
                        }
                        else
                        {
                            value = ctx.tuning.@base.engineRpmHighDecel;
                        }
                        ctx.vehicle.engine.rpm = (float)(double)(ctx.vehicle.engine.rpm - ((float)(double)(value * dt)));
                        if (ctx.vehicle.engine.rpm < ctx.vehicle.engine.targetRpm)
                        {
                            ctx.vehicle.engineMode = 0;
                        }
                    }
                    else
                    {
                        ctx.vehicle.engine.rpm = (float)(double)(((float)(double)(ctx.tuning.@base.engineRpmFollowAccel * dt))
                            + ctx.vehicle.engine.rpm);
                        if (ctx.vehicle.engine.targetRpm < ctx.vehicle.engine.rpm)
                        {
                            ctx.vehicle.engineMode = 0;
                        }
                    }
                }
            }

            if (!brokeToClamp)
            {
                // shift_checks:
                if (!throttleOn || ctx.vehicle.engine.reverse != 0 || ctx.vehicle.engineMode != 0)
                {
                    if ((ctx.tuning.@base.speed338 < ctx.vehicle.currentLocalSpeed.z)
                        && (ctx.vehicle.currentLocalSpeed.z < ctx.tuning.@base.speed334)
                        && throttleOn
                        && ctx.vehicle.engine.reverse != 0
                        && ctx.vehicle.engineMode == 0)
                    {
                        ctx.vehicle.engineMode = 3;
                        ctx.vehicle.engineLimitFlag = 0;
                        if (ctx.vehicle.engine.gear != 0)
                        {
                            ctx.vehicle.engine.gear = 0;
                            ctx.vehicle.engine.shiftTimer = 0.025f;
                        }
                    }
                }
                else if ((ctx.tuning.@base.speed330 < ctx.vehicle.currentLocalSpeed.z)
                    && (ctx.vehicle.currentLocalSpeed.z < ctx.tuning.@base.speed32c))
                {
                    ctx.vehicle.engineMode = 2;
                    ctx.vehicle.engineLimitFlag = 0;
                    if (ctx.vehicle.engine.gear == 0)
                    {
                        ctx.vehicle.engine.shiftTimer = 0.025f;
                        ctx.vehicle.engine.gear = 1;
                    }
                }

                if (ctx.vehicle.engineMode == 0 || ctx.vehicle.engineMode == 1)
                {
                    if (ctx.vehicle.engine.reverse == 0)
                    {
                        if (ctx.vehicle.engine.gear == 0)
                        {
                            ctx.vehicle.engineMode = 1;
                            ctx.vehicle.gearDownshiftFlag = 0;
                            if ((double)ctx.vehicle.engine.rpm < 1000.0)
                            {
                                ctx.vehicle.engine.shiftTimer = 0.025f;
                                ctx.vehicle.engine.gear = 1;
                            }
                        }
                        uint gearIndex = (uint)ctx.vehicle.engine.gear;
                        if (0 < ctx.vehicle.engine.gear)
                        {
                            if ((ctx.vehicle.engine.targetRpm
                                <= ((float)(double)(ctx.tuning.@base.gearUpshift[(int)gearIndex]
                                    * ctx.vehicle.engine.maxRpm)))
                                || 4 < ctx.vehicle.engine.gear)
                            {
                                if ((ctx.vehicle.engine.targetRpm
                                    < ((float)(double)(ctx.tuning.@base.gearDownshift[(int)gearIndex]
                                        * ctx.vehicle.engine.maxRpm)))
                                    && 1 < ctx.vehicle.engine.gear)
                                {
                                    ctx.vehicle.engine.shiftTimer = 0.025f;
                                    ctx.vehicle.engine.gear -= 1;
                                    ctx.vehicle.engineMode = 1;
                                    ctx.vehicle.gearDownshiftFlag = 1;
                                }
                            }
                            else
                            {
                                ctx.vehicle.engine.shiftTimer = 0.025f;
                                ctx.vehicle.engine.gear += 1;
                                ctx.vehicle.engineMode = 1;
                                ctx.vehicle.gearDownshiftFlag = 0;
                            }
                        }
                    }
                    else if (ctx.vehicle.engine.gear != 0)
                    {
                        ctx.vehicle.engineMode = 1;
                        ctx.vehicle.gearDownshiftFlag = 1;
                        if ((double)ctx.vehicle.engine.rpm < 1000.0)
                        {
                            ctx.vehicle.engine.gear = 0;
                            ctx.vehicle.engine.shiftTimer = allAirborne ? 0.002f : 0.025f;
                        }
                    }
                }
            }

            // clamp_rpm:
            value = ctx.vehicle.engine.rpm;
            if (0.0 < (double)value)
            {
                if (ctx.vehicle.engine.maxRpm < value)
                {
                    ctx.vehicle.engine.rpm = ctx.vehicle.engine.maxRpm;
                }
            }
            else
            {
                ctx.vehicle.engine.rpm = 0.0f;
            }
        }

        // ------------------------------------------------------------------
        // suspension force
        // ------------------------------------------------------------------

        /// <summary>0x007C1810: applies one wheel's suspension force.</summary>
        public static void WheelAddForceToVehicle(ref VehicleCtx ctx, int wheelIndex)
        {
            GmVec3 force = GmVec3.Zero;

            CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[wheelIndex];
            if (wheel.realTime.hasGroundContact == 0)
            {
                return;
            }
            float delta = (float)(double)(ctx.tuning.@base.suspensionRestLength - wheel.realTime.damperAbsorb);
            switch (ctx.tuning.@base.suspensionModel)
            {
                case 0:
                    force.y = (float)(double)(((float)(double)(ctx.tuning.@base.suspensionStiffness
                        * ctx.tuning.@base.suspensionScale))
                        * delta);
                    break;
                case 1:
                case 2:
                    force.y = (float)(double)(((float)(double)(delta * ctx.tuning.@base.suspensionStiffness))
                        - ((float)(double)(ctx.tuning.@base.suspensionDamping * wheel.realTime.field04)));
                    break;
                default:
                    return;
            }
            GmVec3 point = wheel.offsetFromVehicle;
            AddVehicleForce(ref ctx, ref force, ref point);
        }
    }
}
