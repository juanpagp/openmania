using System;

namespace Tmnf.Sim
{
    /// <summary>vehicle_model6.c: the Stadium car force model (+ donut, material6).</summary>
    public static class VehicleModel6
    {
        // MODEL6_LENGTH_EPSILON 0x1.b7cdfcp-34f
        private static readonly float Model6LengthEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EB7_CDFC));
        // MODEL6_CONTROL_EPSILON
        private static readonly float Model6ControlEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x374F_8B58));
        // MODEL6_POINT_ONE — 0.1 as a bare double literal
        private const double Model6PointOne = 0.1;
        // MODEL6_POINT_SEVEN — 0.7
        private const double Model6PointSeven = 0.7;
        // MODEL6_POINT_SEVEN_FIVE — 0.75
        private const double Model6PointSevenFive = 0.75;
        // MODEL6_PI — float pi promoted to double
        private static readonly double Model6Pi =
            BitConverter.Int64BitsToDouble(unchecked((long)0x4009_21FB_6000_0000));
        // METERS_PER_SECOND_TO_KMH — float 3.6 promoted
        private static readonly double MetersPerSecondToKmh =
            BitConverter.Int64BitsToDouble(unchecked((long)0x400C_CCCC_C000_0000));

        private static float X87MulDouble(float value, double multiplier)
        {
            return (float)((double)value * multiplier);
        }

        private static float X87SubDouble(float value, double subtrahend)
        {
            return (float)((double)value - subtrahend);
        }

        private static float X87AddDouble(float value, double addend)
        {
            return (float)((double)value + addend);
        }

        private static float Model6Abs(float value) => Math.Abs(value);

        private static float Model6CurveValue(CFuncKeysReal curve, float position)
        {
            uint lowerIndex = 0;
            return Curve.CfuncKeysRealGetValue(curve, position, ref lowerIndex);
        }

        private static float Model6SpeedPosition(float speed)
        {
            return X87MulDouble(speed, MetersPerSecondToKmh);
        }

        private static float Model6CurveFromSpeed(CFuncKeysReal curve, float speed)
        {
            return Model6CurveValue(curve, Model6SpeedPosition(speed));
        }

        private static float Model6LengthSquared(ref GmVec3 value)
        {
            float xy = (float)(double)(((float)(double)(value.x * value.x)) + ((float)(double)(value.y * value.y)));
            return (float)(double)(xy + ((float)(double)(value.z * value.z)));
        }

        private static float Model6Length(ref GmVec3 value)
        {
            return Fp.Sqrt(Model6LengthSquared(ref value));
        }

        private static bool Model6Normalize(ref GmVec3 value)
        {
            float lengthSquared = Model6LengthSquared(ref value);
            if (!((double)Model6LengthEpsilon < (double)lengthSquared))
            {
                return false;
            }
            float inverse = (float)(double)(1.0f / Fp.Sqrt(lengthSquared));
            value.x = (float)(double)(inverse * value.x);
            value.y = (float)(double)(value.y * inverse);
            value.z = (float)(double)(inverse * value.z);
            return true;
        }

        private static GmVec3 Model6SubVec3(ref GmVec3 left, ref GmVec3 right)
        {
            return new GmVec3((float)(double)(left.x - right.x), (float)(double)(left.y - right.y), (float)(double)(left.z - right.z));
        }

        private static GmVec3 Model6Cross(ref GmVec3 left, ref GmVec3 right)
        {
            return new GmVec3(
                (float)(double)(((float)(double)(left.y * right.z)) - ((float)(double)(left.z * right.y))),
                (float)(double)(((float)(double)(left.z * right.x)) - ((float)(double)(left.x * right.z))),
                (float)(double)(((float)(double)(left.x * right.y)) - ((float)(double)(left.y * right.x))));
        }

        // Schedule at 0x007C4B7B: x*x + y*y, then z*z.
        private static float Model6DotXyz(ref GmVec3 left, ref GmVec3 right)
        {
            float xy = (float)(double)(((float)(double)(left.x * right.x)) + ((float)(double)(left.y * right.y)));
            return (float)(double)(xy + ((float)(double)(left.z * right.z)));
        }

        // Schedule at 0x007C4378: y*y + x*x, then z*z.
        private static float Model6DotYxz(ref GmVec3 left, ref GmVec3 right)
        {
            float yx = (float)(double)(((float)(double)(left.y * right.y)) + ((float)(double)(left.x * right.x)));
            return (float)(double)(yx + ((float)(double)(left.z * right.z)));
        }

        /// <summary>
        /// Diagnostic hook (env-gated, non-behavioral): per-wheel
        /// lateral-force inputs when TMNF_DBG is set.
        /// </summary>
        private static void DbgWheel(
            ref VehicleCtx ctx, int index, ref GmVec3 axis,
            float velocityOnAxis, float damperModulation, float maxSide,
            float maximumForce, float requestedForce)
        {
            if (Environment.GetEnvironmentVariable("TMNF_DBG") == null)
            {
                return;
            }
            Console.Error.WriteLine(
                $"WHL {index} tick={ctx.vehicle.tickTime} " +
                $"axis=({BitConverter.SingleToInt32Bits(axis.x):X8},{BitConverter.SingleToInt32Bits(axis.y):X8},{BitConverter.SingleToInt32Bits(axis.z):X8}) " +
                $"von={BitConverter.SingleToInt32Bits(velocityOnAxis):X8} " +
                $"damp={BitConverter.SingleToInt32Bits(damperModulation):X8} " +
                $"maxside={BitConverter.SingleToInt32Bits(maxSide):X8} " +
                $"maxf={BitConverter.SingleToInt32Bits(maximumForce):X8} " +
                $"req={BitConverter.SingleToInt32Bits(requestedForce):X8}");
        }

        private static float[] Model6WheelMaterialValues(ref VehicleCtx ctx, int contactMaterialId)
        {
            ushort materialId = (ushort)contactMaterialId;
            uint materialIndex = ctx.tuning.groundMaterialIndices[materialId];
            return ctx.tuning.groundMaterials[(int)materialIndex].values;
        }

        private static float Model6EnterWave(ref VehicleCtx ctx, uint tick)
        {
            CSceneVehicleCarModel6Tuning tuning = ctx.tuning.model6;
            uint elapsed = unchecked(tick - ctx.vehicle.model6.burnoutStartTick);
            float angle = X87MulDouble(Fp.U32ToX87Float(elapsed), Model6Pi);
            float denominator = Fp.U32ToX87Float(tuning.burnoutEnterTicks * 2u);

            angle = (float)(double)(angle / denominator);
            return (float)(double)(((float)(double)(((float)(double)(tuning.burnoutEnterLateralScale - 1.0f)) * Fp.X87Cos(angle))) + 1.0f);
        }

        private static float Model6AccelWave(ref VehicleCtx ctx, uint tick)
        {
            CSceneVehicleCarModel6Tuning tuning = ctx.tuning.model6;
            uint elapsed = unchecked(tick - ctx.vehicle.model6.burnoutStartTick);
            float angle = X87MulDouble(Fp.U32ToX87Float(elapsed), Model6Pi);

            angle = (float)(double)(angle / Fp.U32ToX87Float(tuning.burnoutEnterTicks));
            return (float)(double)(((float)(double)(((float)(double)(tuning.burnoutEnterAccelScale - 1.0f)) * Fp.X87Sin(angle))) + 1.0f);
        }

        private static float Model6ExitWave(ref VehicleCtx ctx, uint tick, ref float extraAcceleration)
        {
            CSceneVehicleCarModel6Tuning tuning = ctx.tuning.model6;
            uint elapsed = unchecked(tick - ctx.vehicle.model6.burnoutTransitionTick);
            float angle = X87MulDouble(Fp.U32ToX87Float(elapsed), Model6Pi);
            uint quotient = elapsed / tuning.burnoutExitTicks;
            float phaseCount = Fp.U32ToX87Float(quotient);

            angle = (float)(double)(angle / Fp.U32ToX87Float(tuning.burnoutExitTicks));
            float phaseDelta = X87SubDouble(phaseCount, 1.0);
            extraAcceleration = (float)(double)(((float)(double)(phaseDelta * phaseDelta)) * tuning.burnoutExitExtraAccel);
            return (float)(double)(((float)(double)(((float)(double)(tuning.burnoutExitAccelScale - 1.0f)) * Fp.X87Sin(angle))) + 1.0f);
        }

        private static void Model6AddMaterial6Forces(
            ref VehicleCtx ctx,
            ref GmVec3 localSpeed,
            ref GmVec3 sideForce)
        {
            CSceneVehicleCarModel6Tuning tuning = ctx.tuning.model6;
            GmVec3 normalizedSpeed = localSpeed;
            GmVec3 frontForce = GmVec3.Zero;
            GmVec3 rearForce = GmVec3.Zero;
            float lengthSquared = Model6LengthSquared(ref normalizedSpeed);

            if ((double)Model6LengthEpsilon < (double)lengthSquared)
            {
                float inverse = (float)(double)(1.0f / Fp.Sqrt(lengthSquared));
                // 0x007C50C7 stores only the x component.
                normalizedSpeed.x = (float)(double)(inverse * normalizedSpeed.x);
            }
            if (Model6PointOne < (double)ctx.vehicle.inputBrake)
            {
                GmVec3 drag = new GmVec3(
                    X87MulDouble(localSpeed.x, -Model6PointOne),
                    X87MulDouble(localSpeed.y, -Model6PointOne),
                    X87MulDouble(localSpeed.z, -Model6PointOne));
                VehicleBase.AddVehicleCentralForce(ref ctx, ref drag);
            }
            if (ctx.vehicle.flag60c == 0
                && Model6PointOne < (double)ctx.vehicle.inputGas
                && (double)ctx.vehicle.inputBrake < Model6PointOne)
            {
                float absX = Model6Abs(normalizedSpeed.x);
                float speedShape = X87AddDouble(X87MulDouble(absX, 20.0), 1.0);
                float absoluteForward = X87AddDouble(Model6Abs(localSpeed.z), 1.0);
                float verticalDivisor = (float)(double)(absoluteForward * absoluteForward);
                float gasDivisor = X87AddDouble((float)(double)(tuning.material6GasDenominator * ctx.vehicle.inputGas), 1.0);
                float frontCommon = X87MulDouble((float)(double)(tuning.material6VerticalScale * ctx.vehicle.inputGas), 1.5);
                float rearCommon = (float)(double)(tuning.material6VerticalScale * ctx.vehicle.inputGas);

                frontCommon = (float)(double)(frontCommon * absX);
                frontCommon = (float)(double)(frontCommon * speedShape);
                frontCommon = (float)(double)(frontCommon * tuning.material6VerticalShape);
                rearCommon = (float)(double)(((float)(double)(((float)(double)(rearCommon * absX)) * speedShape))
                    * tuning.material6VerticalShape);

                frontForce.x = (float)(double)(((float)(double)(tuning.material6LongitudinalScale * normalizedSpeed.x))
                    / gasDivisor);
                frontForce.z = (float)(double)(frontCommon / verticalDivisor);
                rearForce.x = (float)(double)(((float)(double)(((float)(double)(normalizedSpeed.x * -1.0f))
                    * tuning.material6LongitudinalScale))
                    / gasDivisor);
                rearForce.z = (float)(double)(rearCommon / verticalDivisor);
            }

            int wheelCount = ctx.vehicle.wheels.Count;
            for (int index = 0; index < wheelCount; index++)
            {
                int isSliding = ctx.vehicle.wheels[index].realTime.isSliding;
                GmVec3 offsetFromVehicle = ctx.vehicle.wheels[index].offsetFromVehicle;

                if ((double)ctx.vehicle.inputBrake < Model6PointOne
                    && isSliding != 0)
                {
                    if (index == 0 || index == 1)
                    {
                        VehicleBase.AddVehicleForce(ref ctx, ref frontForce, ref offsetFromVehicle);
                    }
                    if (index == 2 || index == 3)
                    {
                        VehicleBase.AddVehicleForce(ref ctx, ref rearForce, ref offsetFromVehicle);
                    }
                }
            }
            VehicleBase.AddVehicleCentralForce(ref ctx, ref sideForce);
        }

        /// <summary>0x007C3E80's donut continuation branch.</summary>
        private static void Model6ContinueDonut(
            ref VehicleCtx ctx,
            ref Model6External ext,
            ref GmVec3 existingForce,
            ref GmVec3 localSpeed,
            ref GmVec3 localAngularSpeed,
            float steeringAngle,
            bool grounded,
            bool waterContact,
            float uprightValue)
        {
            CSceneVehicleCarTuningCurveSet curves = ctx.tuning.curves;

            bool brokeToTransition = false;

            if ((double)ctx.vehicle.inputGas < Model6PointOne
                || (double)ctx.vehicle.inputBrake < Model6PointOne
                || (double)Model6Abs(steeringAngle) < (double)Model6ControlEpsilon)
            {
                ctx.vehicle.driveMode = 0;
                brokeToTransition = true;
            }
            if (!brokeToTransition && (ctx.vehicle.model6.orbitSign != GmFunc.GmfuncSign(steeringAngle)
                || ctx.vehicle.model6.sideContact != 0
                || 0 < ctx.vehicle.model6.contactBlockCount
                || !grounded
                || waterContact
                || ctx.vehicle.flag60c != 0))
            {
                ctx.vehicle.driveMode = 0;
                brokeToTransition = true;
            }

            if (!brokeToTransition)
            {
                GmVec3 wheelNormal = GmVec3.Zero;
                for (int index = 0; index < ctx.vehicle.wheels.Count; index++)
                {
                    GmVec3 normal = ctx.vehicle.wheels[index].realTime.field90;
                    wheelNormal.x = (float)(double)(normal.x + wheelNormal.x);
                    wheelNormal.y = (float)(double)(normal.y + wheelNormal.y);
                    wheelNormal.z = (float)(double)(normal.z + wheelNormal.z);
                }
                if (!Model6Normalize(ref wheelNormal))
                {
                    wheelNormal = new GmVec3(0.0f, 1.0f, 0.0f);
                }
                GmVec3 worldNormal = GmVec3.Zero;
                GmMat3 modelIsoRot = ctx.vehicle.model6.modelIso.Rotation();
                worldNormal.SetMultMat3(ref wheelNormal, ref modelIsoRot);
                float normalAngle = Model6Abs(GmFunc.Gmvec3GetAngle(
                    ref worldNormal, ref ctx.vehicle.model6.orbitAxis));
                if ((double)ctx.tuning.model6.donutNormalAngleLimit < (double)normalAngle)
                {
                    ctx.vehicle.driveMode = 0;
                }

                GmIso4 inverse = default(GmIso4);
                GmIso4 modelIso = ctx.vehicle.model6.modelIso;
                inverse.SetInverse(ref modelIso);
                GmVec3 localRadius = GmVec3.Zero;
                GmVec3 orbitCenter = ctx.vehicle.model6.orbitCenter;
                localRadius.SetMultIso4(ref orbitCenter, ref inverse);
                localRadius = Model6SubVec3(ref localRadius, ref ext.bodyReferencePosition);
                float radius = Model6Length(ref localRadius);
                GmVec3 radialDirection = localRadius;
                _ = Model6Normalize(ref radialDirection);
                GmVec3 tangent = Model6Cross(ref wheelNormal, ref radialDirection);
                float lateralSpeed = Model6DotYxz(ref tangent, ref localSpeed);

                if ((double)ctx.tuning.model6.donutLateralSpeedLimit < (double)Model6Abs(lateralSpeed))
                {
                    ctx.vehicle.driveMode = 0;
                }
                if ((double)ctx.vehicle.model6.orbitInitialRadius < (double)radius
                    || (double)radius < (double)ctx.tuning.model6.donutRadiusMin)
                {
                    float radiusDelta = (float)(double)(radius - ctx.vehicle.model6.orbitRadius);
                    float radialSpeed = Model6DotYxz(ref radialDirection, ref localSpeed);
                    float exponent = (float)(double)(((float)(double)(radiusDelta * ctx.tuning.model6.donutRadiusExponent))
                        - ((float)(double)(radialSpeed * ctx.tuning.model6.donutRadialSpeedExponent)));
                    float magnitude = (float)(double)(((float)(double)(((float)(double)(lateralSpeed * lateralSpeed)) / radius))
                        * Fp.X87Exp(exponent));

                    GmVec3 force = new GmVec3(
                        (float)(double)(magnitude * radialDirection.x),
                        (float)(double)(magnitude * radialDirection.y),
                        (float)(double)(magnitude * radialDirection.z));
                    VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                }
                else
                {
                    ctx.vehicle.driveMode = 0;
                }

                for (int index = 0; index < ctx.vehicle.wheels.Count; index++)
                {
                    VehicleBase.WheelAddForceToVehicle(ref ctx, index);
                    ctx.vehicle.wheels[index].realTime.isSliding = 0;
                }

                {
                    float direction = GmFunc.GmfuncSign(steeringAngle);
                    // model6_lateral_speed_from_radius: the RADIUS is the
                    // raw curve position (no kmh transform), and the value
                    // is divided by the float-widened kmh constant.
                    float target = (float)(double)(Model6CurveValue(
                            ctx.tuning.curves.m6LateralSpeedFromBurnoutRadius, radius)
                        / (float)MetersPerSecondToKmh);
                    float difference = (float)(double)(((float)(double)(-direction * target)) - lateralSpeed);
                    float magnitude = (float)(double)(ctx.tuning.model6.donutLateralForceScale * difference);

                    GmVec3 force = new GmVec3(
                        (float)(double)(magnitude * tangent.x),
                        (float)(double)(magnitude * tangent.y),
                        (float)(double)(magnitude * tangent.z));
                    VehicleBase.AddVehicleCentralForce(ref ctx, ref force);

                    GmVec3 forwardAxis = new GmVec3(0.0f, 0.0f, 1.0f);
                    float signedAngle = (float)(double)(GmFunc.Gmvec3GetAngle(ref forwardAxis, ref radialDirection)
                        / (float)Model6Pi);
                    if (!GmFunc.GmfuncIsANumber(signedAngle))
                    {
                        ctx.vehicle.driveMode = 0;
                    }
                    else
                    {
                        float angleRadians = X87MulDouble((float)(double)(direction * signedAngle), Model6Pi);

                        if ((double)angleRadians < (double)(-(ctx.tuning.model6.donutAngleNegativeLimit))
                            || (double)ctx.tuning.model6.donutAnglePositiveLimit < (double)angleRadians)
                        {
                            ctx.vehicle.driveMode = 0;
                        }
                        else
                        {
                            float steerTorque;
                            float countersteer = 0.0f;
                            if ((double)signedAngle <= 0.0)
                            {
                                if ((double)localAngularSpeed.y <= 0.0)
                                {
                                    float shifted = (float)(double)(signedAngle + 1.0f);
                                    float shiftedSquared = (float)(double)(shifted * shifted);
                                    steerTorque = ((float)(double)(((float)(double)(-(ctx.tuning.model6.donutSteerQuadratic)
                                        * localAngularSpeed.y))
                                        * shiftedSquared));
                                }
                                else
                                {
                                    steerTorque = (float)(double)(-(ctx.tuning.model6.donutSteerLinear)
                                        * localAngularSpeed.y);
                                }
                            }
                            else if (0.0 <= (double)localAngularSpeed.y)
                            {
                                float shifted = (float)(double)(signedAngle - 1.0f);
                                float shiftedSquared = (float)(double)(shifted * shifted);
                                steerTorque = ((float)(double)(((float)(double)(-(ctx.tuning.model6.donutSteerQuadratic)
                                    * localAngularSpeed.y))
                                    * shiftedSquared));
                            }
                            else
                            {
                                steerTorque = (float)(double)(-(ctx.tuning.model6.donutSteerLinear)
                                    * localAngularSpeed.y);
                            }
                            float radialRatio = (float)(double)(lateralSpeed / radius);
                            if (((double)signedAngle <= 0.0 && 0.0 < (double)radialRatio)
                                || (0.0 < (double)signedAngle
                                && (double)radialRatio < 0.0))
                            {
                                countersteer = (float)(double)(-(ctx.tuning.model6.donutCountersteerScale)
                                    * radialRatio);
                            }
                            GmVec3 torque = new GmVec3(
                                (float)(double)(((float)(double)(((float)(double)(((float)(double)(ctx.tuning.model6.donutYawAngleScale * signedAngle))
                                    + steerTorque))
                                    + countersteer))
                                    * 0.0f),
                                (float)(double)(((float)(double)(((float)(double)(ctx.tuning.model6.donutYawAngleScale * signedAngle))
                                    + steerTorque))
                                    + countersteer),
                                0.0f);
                            float torqueY = torque.y;
                            GmVec3 torque2 = new GmVec3(
                                torque.x,
                                torqueY,
                                (float)(double)(torqueY * 0.0f));
                            VehicleBase.AddVehicleTorque(ref ctx, ref torque2);
                        }
                    }
                }

                {
                    float tz = (float)(double)(-(GmFunc.GmfuncSign(localSpeed.x))
                        * Model6CurveFromSpeed(curves.m6DonutRolloverFromSpeed, Model6Abs(localSpeed.x)));
                    GmVec3 torque = new GmVec3((float)(double)(tz * 0.0f), (float)(double)(tz * 0.0f), tz);
                    GmVec3 torque2 = new GmVec3(torque.x, torque.x, torque.z);
                    VehicleBase.AddVehicleTorque(ref ctx, ref torque2);
                    GmVec3 torque3 = new GmVec3(
                        Model6CurveFromSpeed(curves.m6BurnoutRolloverFromSpeed, localSpeed.z),
                        0.0f,
                        0.0f);
                    VehicleBase.AddVehicleTorque(ref ctx, ref torque3);
                }
            }

            // transition:
            if (ctx.vehicle.driveMode != 2)
            {
                uint tick = ctx.vehicle.tickTime;

                ctx.vehicle.driveMode = 1;
                ctx.vehicle.model6.burnoutStartTick = tick;
            }
        }

        /// <summary>0x007C3E80: the Stadium force model.</summary>
        public static void ComputeForcesModel6(
            ref VehicleCtx ctx,
            ref Model6External ext,
            TmnfTrackWater water,
            float modelValue,
            ref GmVec3 existingForce,
            float lateralForceFactor,
            float longitudinalForceFactor,
            ref GmVec3 localSpeed,
            ref GmVec3 localAngularSpeed,
            float steeringAngle,
            bool grounded,
            ref CSceneVehicleMaterialBlendableVals material,
            ref int sliding,
            ref float brakeForce)
        {
            ctx.vehicle.model6.modelIso = ext.modelIsoSource;
            float uprightValue = ctx.vehicle.model6.modelIso.m4;
            bool waterContact = VehicleAux.ApplyWaterForces(ref ctx, water, ref existingForce);
            // 0x007C3ED8 stores car +0x5e4: both the air-control lock and the
            // airborne friction gate.
            ctx.vehicle.airControlLocked = waterContact ? 1 : 0;
            ctx.vehicle.airborneFrictionGate = waterContact ? 1 : 0;

            int wheelCount = ctx.vehicle.wheels.Count;
            bool allMaterial6 = true;
            bool anySliding = false;
            bool wasSliding = ctx.vehicle.turboActive != 0;
            bool jumpedToFinish = false;

            for (int index = 0; index < wheelCount; index++)
            {
                CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[index];
                if (wheel.realTime.hasGroundContact == 0
                    || (ushort)wheel.realTime.contactMaterialId != 6)
                {
                    allMaterial6 = false;
                }
            }

            if (ctx.vehicle.driveMode == 2)
            {
                Model6ContinueDonut(
                    ref ctx, ref ext, ref existingForce, ref localSpeed,
                    ref localAngularSpeed, steeringAngle, grounded, waterContact,
                    uprightValue);
                jumpedToFinish = true; // goto finish
            }
            else
            {
                uint tick = ctx.vehicle.tickTime;
                if (ctx.vehicle.driveMode == 1)
                {
                    uint start = ctx.vehicle.model6.burnoutStartTick;

                    if (tick < start
                        || ctx.tuning.model6.burnoutEnterTicks
                        <= unchecked(tick - start))
                    {
                        ctx.vehicle.model6.burnoutTransitionTick = tick;
                        ctx.vehicle.driveMode = 3;
                    }
                    else
                    {
                        anySliding = true;
                    }
                }
                if (ctx.vehicle.driveMode == 3)
                {
                    uint start = ctx.vehicle.model6.burnoutTransitionTick;

                    if (tick < start
                        || ctx.tuning.model6.burnoutExitTicks <= unchecked(tick - start))
                    {
                        ctx.vehicle.driveMode = 0;
                        ctx.vehicle.forceWheelSpeed = 0;
                    }
                    else
                    {
                        for (int index = 0; index < wheelCount; index++)
                        {
                            ctx.vehicle.wheels[index].realTime.isSliding = 1;
                        }
                    }
                }

                for (int index = 0; index < wheelCount; index++)
                {
                    VehicleBase.WheelAddForceToVehicle(ref ctx, index);
                    CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[index];
                    int hasGroundContact = wheel.realTime.hasGroundContact;
                    int contactMaterialId = wheel.realTime.contactMaterialId;
                    if (hasGroundContact == 0
                        || !((double)ctx.tuning.model6.lateralForceScale > 0.0))
                    {
                        continue;
                    }
                    GmVec3 field54 = wheel.realTime.field54;
                    GmVec3 field90 = wheel.realTime.field90;
                    float damperAbsorb = wheel.realTime.damperAbsorb;
                    int isSlidingR = wheel.realTime.isSliding;
                    int steerable = wheel.steerable;

                    GmVec3 relativeContact =
                        Model6SubVec3(ref field54, ref ext.bodyReferencePosition);
                    GmVec3 wheelAxis = new GmVec3(
                        (float)(double)(field90.y - ((float)(double)(field90.z * 0.0f))),
                        (float)(double)(((float)(double)(field90.z * 0.0f)) - field90.x),
                        (float)(double)(((float)(double)(field90.x * 0.0f)) - ((float)(double)(field90.y * 0.0f))));
                    float axisLengthSquared = Model6LengthSquared(ref wheelAxis);
                    if ((double)axisLengthSquared <= (double)Model6LengthEpsilon)
                    {
                        wheelAxis = new GmVec3(1.0f, 0.0f, 0.0f);
                    }
                    else
                    {
                        float inverse = (float)(double)(1.0f / Fp.Sqrt(axisLengthSquared));
                        wheelAxis.x = (float)(double)(inverse * wheelAxis.x);
                        wheelAxis.y = (float)(double)(inverse * wheelAxis.y);
                        wheelAxis.z = (float)(double)(inverse * wheelAxis.z);
                    }
                    if (steerable != 0)
                    {
                        float cosine = Fp.X87Cos(steeringAngle);
                        float negativeSine = -Fp.X87Sin(steeringAngle);

                        GmVec3 rotated = new GmVec3(
                            (float)(double)(((float)(double)(cosine * wheelAxis.x)) + ((float)(double)(negativeSine * 0.0f))),
                            (float)(double)(((float)(double)(negativeSine * 0.0f)) + ((float)(double)(cosine * wheelAxis.y))),
                            (float)(double)(negativeSine + ((float)(double)(cosine * wheelAxis.z))));
                        wheelAxis = rotated;
                    }
                    float velocityOnAxis = Model6DotXyz(ref localSpeed, ref wheelAxis);

                    {
                        float axisScale = -ctx.tuning.model6.burnoutTriggerScale;
                        GmVec3 rolloverAxis = new GmVec3(
                            (float)(double)(axisScale * ctx.vehicle.model6.rolloverAxis.x),
                            (float)(double)(ctx.vehicle.model6.rolloverAxis.y * axisScale),
                            (float)(double)(axisScale * ctx.vehicle.model6.rolloverAxis.z));
                        float axisLength = Model6Length(ref rolloverAxis);
                        if ((double)axisLength
                            < (double)ctx.tuning.model6.rolloverAxisMinLength)
                        {
                            rolloverAxis = GmVec3.Zero;
                        }
                        GmVec3 torque = Model6Cross(ref relativeContact, ref rolloverAxis);
                        torque.x = (float)(double)(((float)(double)(torque.x * -1.0f))
                            * ctx.tuning.model6.rolloverTorqueXScale);
                        torque.y = 0.0f;
                        torque.z = (float)(double)(((float)(double)(torque.z * -1.0f))
                            * ctx.tuning.model6.rolloverTorqueZScale);
                        VehicleBase.AddVehicleTorque(ref ctx, ref torque);
                    }
                    float enterScale = 1.0f;
                    if (ctx.vehicle.driveMode == 1)
                    {
                        float tx = Model6CurveFromSpeed(ctx.tuning.curves.m6BurnoutRolloverFromSpeed, localSpeed.z);
                        GmVec3 torque = new GmVec3(tx, 0.0f, 0.0f);
                        VehicleBase.AddVehicleTorque(ref ctx, ref torque);
                        enterScale = Model6EnterWave(ref ctx, tick);
                    }

                    float damperModulation =
                        Curve.CsceneVehicleCarTuningM6GetModulationFromDamperAbsorbVal(
                            ctx.tuning.curves, damperAbsorb);
                    float slidingScale = isSlidingR == 0
                        ? 1.0f
                        : ctx.tuning.model6.slidingLateralLimitScale;
                    float brakingScale = isSlidingR != 0
                        && Model6PointOne < (double)ctx.vehicle.inputBrake
                        ? ctx.tuning.model6.brakingLateralLimitScale
                        : 1.0f;
                    float[] materialValues = Model6WheelMaterialValues(ref ctx, contactMaterialId);
                    float maxSide = Model6CurveFromSpeed(ctx.tuning.curves.maxSideFrictionFromSpeed, localSpeed.z);
                    float maximumForce = (float)(double)((float)(double)(((float)(double)(((float)(double)(((float)(double)(materialValues[3] * lateralForceFactor))
                        * maxSide))
                        * slidingScale))
                        * brakingScale)
                        * damperModulation);
                    float requestedForce = (float)(double)(((float)(double)(((float)(double)(-(ctx.tuning.model6.lateralForceScale)
                        * 0.5f))
                        * velocityOnAxis))
                        * enterScale);
                    if ((double)Model6Abs(requestedForce) <= (double)maximumForce)
                    {
                        ctx.vehicle.wheels[index].realTime.isSliding = 0;
                    }
                    else
                    {
                        float signedLimit = (double)requestedForce <= 0.0
                            ? -maximumForce
                            : maximumForce;

                        ctx.vehicle.wheels[index].realTime.isSliding = 1;
                        requestedForce = (float)(double)(((float)(double)(((float)(double)(1.0f - ctx.tuning.model6.lateralOverflowBlend))
                            * signedLimit))
                            + ((float)(double)(ctx.tuning.model6.lateralOverflowBlend * requestedForce)));
                    }
                    if (ctx.vehicle.wheels[index].realTime.isSliding != 0)
                    {
                        sliding = 1;
                    }
                    GmVec3 force = new GmVec3(
                        (float)(double)(requestedForce * wheelAxis.x),
                        (float)(double)(requestedForce * wheelAxis.y),
                        (float)(double)(wheelAxis.z * requestedForce));
                    DbgWheel(ref ctx, index, ref wheelAxis, velocityOnAxis,
                        damperModulation, maxSide, maximumForce, requestedForce);
                    if (allMaterial6 && 6.0 < (double)localSpeed.z)
                    {
                        Model6AddMaterial6Forces(ref ctx, ref localSpeed, ref force);
                        continue;
                    }
                    VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                }

                if (!grounded)
                {
                    if (ctx.vehicle.driveMode == 1)
                    {
                        ctx.vehicle.model6.burnoutTransitionTick = tick;
                        ctx.vehicle.driveMode = 3;
                    }
                    // goto store_sliding
                }
                else
                {
                    float speedLength = Model6Length(ref localSpeed);

                    if (ctx.vehicle.flag60c == 0
                        && Model6PointOne < (double)ctx.vehicle.inputBrake
                        && (double)ctx.vehicle.inputGas < Model6PointOne
                        && ctx.vehicle.driveMode == 1)
                    {
                        ctx.vehicle.model6.burnoutTransitionTick = tick;
                        ctx.vehicle.driveMode = 3;
                    }
                    if (ctx.vehicle.flag60c == 0)
                    {
                        if (Model6PointOne < (double)ctx.vehicle.inputGas
                            && Model6PointOne < (double)ctx.vehicle.inputBrake
                            && (double)localSpeed.z
                            < (double)ctx.tuning.model6.burnoutSpeedMin
                            && Model6PointSevenFive
                            < (double)uprightValue)
                        {
                            ctx.vehicle.driveMode = 1;
                            ctx.vehicle.forceWheelSpeed = 1;
                            ctx.vehicle.model6.burnoutStartTick = tick;
                        }
                        if (Model6PointOne < (double)ctx.vehicle.inputGas
                            && Model6PointOne < (double)ctx.vehicle.inputBrake
                            && (double)localSpeed.z
                            < (double)ctx.tuning.model6.burnoutSpeedMax
                            && (double)ctx.tuning.model6.burnoutSpeedMin
                            < (double)localSpeed.z
                            && (double)Model6ControlEpsilon
                            <= (double)Model6Abs(steeringAngle))
                        {
                            GmVec3 accumulated = GmVec3.Zero;
                            float direction = GmFunc.GmfuncSign(steeringAngle);

                            ctx.vehicle.driveMode = 2;
                            ctx.vehicle.model6.orbitSign = direction;
                            for (int index = 0; index < wheelCount; index++)
                            {
                                CSceneVehicleCarWheel wheel = ctx.vehicle.wheels[index];
                                if (wheel.realTime.hasGroundContact != 0)
                                {
                                    accumulated.x =
                                        (float)(double)(wheel.realTime.field90.x + accumulated.x);
                                    accumulated.y =
                                        (float)(double)(wheel.realTime.field90.y + accumulated.y);
                                    accumulated.z =
                                        (float)(double)(wheel.realTime.field90.z + accumulated.z);
                                }
                            }
                            if (!Model6Normalize(ref accumulated))
                            {
                                ctx.vehicle.driveMode = 0;
                                accumulated = new GmVec3(0.0f, 1.0f, 0.0f);
                            }
                            ctx.vehicle.model6.orbitAxis = accumulated;
                            GmVec3 pivotDelta = Model6SubVec3(
                                ref ctx.vehicle.model6.pivotPosition,
                                ref ext.bodyReferencePosition);
                            GmVec3 pivotAxis = new GmVec3(
                                (float)(double)(ctx.vehicle.model6.pivotAxis.y * 0.0f),
                                (float)(double)(ctx.vehicle.model6.pivotAxis.x * 0.0f),
                                ctx.vehicle.model6.pivotAxis.z);
                            pivotDelta.x = (float)(double)(pivotAxis.x + pivotDelta.x);
                            pivotDelta.y = (float)(double)(pivotAxis.y + pivotDelta.y);
                            pivotDelta.z = (float)(double)(pivotAxis.z + pivotDelta.z);
                            ctx.vehicle.model6.orbitInitialRadius =
                                Model6Length(ref pivotDelta);
                            GmVec3 tangent =
                                Model6Cross(ref ctx.vehicle.model6.orbitAxis, ref localSpeed);
                            tangent.x = (float)(double)(direction * tangent.x);
                            tangent.y = (float)(double)(direction * tangent.y);
                            tangent.z = (float)(double)(direction * tangent.z);
                            _ = Model6Normalize(ref tangent);
                            GmMat3 modelIsoRot = ctx.vehicle.model6.modelIso.Rotation();
                            ctx.vehicle.model6.orbitAxis.MultMat3(ref modelIsoRot);
                            if ((double)ctx.vehicle.model6.orbitAxis.y
                                < Model6PointSevenFive)
                            {
                                ctx.vehicle.driveMode = 0;
                                ctx.vehicle.model6.orbitAxis = new GmVec3(0.0f, 1.0f, 0.0f);
                            }
                            GmVec3 forwardAxis = new GmVec3(0.0f, 0.0f, 1.0f);
                            GmVec3 tangentArg = tangent;
                            float angle = GmFunc.Gmvec3GetAngle(ref forwardAxis, ref tangentArg);
                            angle = (float)(double)(direction * angle);
                            if ((double)ctx.tuning.model6.donutAnglePositiveLimit < (double)angle
                                || (double)angle
                                < (double)(-(ctx.tuning.model6.donutAngleNegativeLimit)))
                            {
                                ctx.vehicle.driveMode = 0;
                            }
                            else
                            {
                                float radius = Model6CurveFromSpeed(
                                    ctx.tuning.curves.m6BurnoutRadiusFromSpeed, localSpeed.z);

                                GmVec3 localOrbitDirection = GmVec3.Zero;
                                localOrbitDirection.SetMultMat3(ref tangent, ref modelIsoRot);
                                GmVec3 transformedReference = GmVec3.Zero;
                                transformedReference.SetMultIso4(
                                    ref ext.bodyReferencePosition, ref ctx.vehicle.model6.modelIso);
                                radius = (float)(double)(radius + ctx.vehicle.model6.orbitInitialRadius);
                                ctx.vehicle.model6.orbitRadius = radius;
                                ctx.vehicle.model6.orbitCenter = new GmVec3(
                                    (float)(double)(transformedReference.x
                                        + ((float)(double)(radius * localOrbitDirection.x))),
                                    (float)(double)(((float)(double)(radius * localOrbitDirection.y))
                                        + transformedReference.y),
                                    (float)(double)(transformedReference.z
                                        + ((float)(double)(radius * localOrbitDirection.z))));
                            }
                            ctx.vehicle.forceWheelSpeed =
                                (ctx.vehicle.driveMode == 2) ? 1 : 0;
                        }
                    }

                    if (ctx.vehicle.driveMode == 0)
                    {
                        if (Model6PointOne < (double)ctx.vehicle.inputBrake
                            && (double)localSpeed.z
                            < (double)ctx.vehicle.model6.reverseSpeedThreshold
                            && (double)Model6Abs(localSpeed.x) < 2.0)
                        {
                            ctx.vehicle.model6.reverseMode = 1;
                        }
                        if (Model6PointOne < (double)ctx.vehicle.inputGas
                            && (0.0 < (double)localSpeed.z
                            || 2.0 < (double)Model6Abs(localSpeed.x)))
                        {
                            ctx.vehicle.model6.reverseMode = 0;
                        }
                        if ((double)ctx.vehicle.inputGas < Model6PointOne
                            && (double)ctx.vehicle.inputBrake < Model6PointOne)
                        {
                            if (0.0 < (double)localSpeed.z
                                || (double)Model6Abs(localSpeed.z) < 2.0)
                            {
                                ctx.vehicle.model6.reverseMode = 0;
                            }
                            else
                            {
                                ctx.vehicle.model6.reverseMode = 1;
                            }
                        }
                        if (0.0 < (double)localSpeed.z
                            && ctx.vehicle.turboType != TMNFVehicleTurboType.None)
                        {
                            ctx.vehicle.model6.reverseMode = 0;
                        }
                    }
                    else
                    {
                        ctx.vehicle.model6.reverseMode = 0;
                    }

                    {
                        float speedRatio = (float)(double)(((float)(double)(localSpeed.x * localSpeed.x))
                            / X87AddDouble(Model6Abs(localSpeed.z), 1.0));
                        float direction = GmFunc.GmfuncSign(-localSpeed.x);
                        float ratio = Model6CurveFromSpeed(
                            ctx.tuning.curves.m6RolloverLateralFromSpeedRatio, speedRatio);
                        GmVec3 torque = new GmVec3(0.0f, 0.0f, (float)(double)(ratio * direction));
                        VehicleBase.AddVehicleTorque(ref ctx, ref torque);
                    }

                    {
                        float totalRequested = 0.0f;
                        float totalLimited = 0.0f;
                        for (int index = 0; index < wheelCount; index++)
                        {
                            int steerable = ctx.vehicle.wheels[index].steerable;
                            int isSlidingW = ctx.vehicle.wheels[index].realTime.isSliding;
                            float halfWidth = X87MulDouble(
                                steerable != 0 ? ctx.vehicle.model6.axleWidth : -ctx.vehicle.model6.axleWidth,
                                0.5d);

                            GmVec3 offsetSpeed = new GmVec3(
                                (float)(double)(((float)(double)(localAngularSpeed.y * halfWidth)) + localSpeed.x),
                                (float)(double)(localSpeed.y + 0.0f),
                                (float)(double)(localSpeed.z + 0.0f));
                            float steerModulation;
                            if ((double)speedLength < Model6PointSeven)
                            {
                                steerModulation = 0.0f;
                            }
                            else if ((double)speedLength
                                <= (double)ctx.tuning.model6.wheelSteerSineLimit)
                            {
                                float angle = X87MulDouble(
                                    (float)(double)(speedLength / ctx.tuning.model6.wheelSteerSineLimit),
                                    Model6Pi);
                                steerModulation = Fp.X87Sin(X87MulDouble(angle, 0.5));
                            }
                            else
                            {
                                steerModulation = 1.0f;
                            }
                            float maximum = (float)(double)(Model6CurveFromSpeed(
                                ctx.tuning.curves.maxSideFrictionFromSpeed, localSpeed.z)
                                * material.lateralGrip);
                            float requested = (float)(double)(((float)(double)(-(ctx.tuning.model6.lateralForceScale)
                                * 0.5f))
                                * offsetSpeed.x);
                            if ((double)maximum < (double)Model6Abs(requested))
                            {
                                float absoluteRequest = Model6Abs(requested);
                                float blended = (float)(double)(((float)(double)(maximum
                                    * ((float)(double)(1.0f - ctx.tuning.model6.wheelOverflowBlend))))
                                    + ((float)(double)(absoluteRequest
                                        * ctx.tuning.model6.wheelOverflowBlend)));

                                totalLimited = (float)(double)(totalLimited + maximum);
                                totalRequested = (float)(double)(absoluteRequest + totalRequested);
                                requested = (float)(double)(GmFunc.GmfuncSign(requested) * blended);
                                anySliding = true;
                            }
                            float requested2 =
                                (float)(double)(ctx.tuning.model6.wheelTorqueScale * requested);
                            if (steerable != 0)
                            {
                                float reverseDirection = ctx.vehicle.model6.reverseMode == 0
                                    ? 1.0f
                                    : -1.0f;
                                float wheelSlideScale = isSlidingW == 0
                                    ? 1.0f
                                    : ctx.tuning.model6.slidingSteerTorqueScale;
                                float driveTorque = Model6CurveFromSpeed(
                                    ctx.tuning.curves.steerDriveTorqueFromSpeed, localSpeed.z);
                                float correction = (float)(double)(((float)(double)(((float)(double)(((float)(double)(reverseDirection * steerModulation))
                                    * ctx.vehicle.steeringValue))
                                    * driveTorque))
                                    * wheelSlideScale);

                                requested2 = (float)(double)(requested2 - correction);
                            }
                            GmVec3 torque = new GmVec3(
                                (float)(double)(((float)(double)(0.0f * 0.0f)) - ((float)(double)(0.0f * halfWidth))),
                                (float)(double)(((float)(double)(requested2 * halfWidth)) - ((float)(double)(0.0f * 0.0f))),
                                (float)(double)(((float)(double)(0.0f * 0.0f)) - ((float)(double)(requested2 * 0.0f))));
                            VehicleBase.AddVehicleTorque(ref ctx, ref torque);
                        }

                        if (anySliding)
                        {
                            ctx.vehicle.model6.lastSlidingTick = tick;
                            if (!wasSliding)
                            {
                                ctx.vehicle.model6.slidingStartTick = tick;
                            }
                            ctx.vehicle.model6.slidingElapsedTicks = unchecked(
                                tick - ctx.vehicle.model6.slidingStartTick);
                        }

                        {
                            float traction = 1.0f;
                            float reverseBias = ctx.vehicle.model6.reverseMode == 0
                                ? 0.0f
                                : -1.0f;
                            float turboFactor = ctx.vehicle.turboType
                                == TMNFVehicleTurboType.None
                                ? 0.0f
                                : ctx.vehicle.turboFactor;
                            float direction = ctx.vehicle.model6.reverseMode == 0
                                ? 1.0f
                                : -1.0f;
                            float brake = 0.0f;

                            if (tick == ctx.vehicle.model6.lastSlidingTick
                                && (double)Model6ControlEpsilon
                                < (double)totalLimited)
                            {
                                float loss = (float)(double)(((float)(double)(((float)(double)(totalRequested - totalLimited))
                                    / totalLimited))
                                    / ctx.tuning.model6.tractionLossScale);

                                if ((double)loss < 0.0)
                                {
                                    loss = 0.0f;
                                }
                                else if (1.0 < (double)loss)
                                {
                                    loss = 1.0f;
                                }
                                traction = (float)(double)(1.0f - loss);
                            }
                            // The slipping-accel lookup carries its own
                            // scale (m5_slipping_accel_scale, car +0x1e4) —
                            // the reference multiplies it in at this call
                            // site (model6_slipping_accel), NOT inside the
                            // tuning getter.
                            float slippingAcceleration = (float)(double)(ctx.tuning.curves.m5SlippingAccelScale
                                * Model6CurveFromSpeed(
                                    ctx.tuning.curves.m5SlippingAccelFromSpeed, localSpeed.z));
                            float normalAcceleration = ctx.vehicle.model6.reverseMode == 0
                                ? Model6CurveFromSpeed(ctx.tuning.curves.accelFromSpeed, localSpeed.z)
                                : Model6CurveFromSpeed(ctx.tuning.curves.m6RearGearAccelFromSpeed, localSpeed.z);
                            float acceleration;
                            if (ctx.vehicle.engineMode == 1)
                            {
                                acceleration = 0.0f;
                            }
                            else
                            {
                                acceleration = (float)(double)(((float)(double)(((float)(double)(1.0f - traction)) * slippingAcceleration))
                                    + ((float)(double)(normalAcceleration * traction)));
                            }
                            float steerSlowdown = (float)(double)(((float)(double)(ctx.tuning.model6.steerSlowdownScale
                                * Model6Abs(ctx.vehicle.steeringValue)))
                                * Model6CurveFromSpeed(ctx.tuning.curves.steerSlowdownFromSpeed, localSpeed.z));
                            float modeScale = 1.0f;
                            float modeExtra = 0.0f;
                            if (ctx.vehicle.driveMode == 1)
                            {
                                modeScale = Model6AccelWave(ref ctx, tick);
                            }
                            if (ctx.vehicle.driveMode == 3)
                            {
                                modeScale = Model6ExitWave(ref ctx, tick, ref modeExtra);
                            }
                            // The (gas*braking + rev_bias*braking*brake) sum
                            // MULTIPLIES the blended acceleration.
                            float acceleration2 = (float)(double)(modeExtra
                                + ((float)(double)(((float)(double)(modeScale
                                    * ((float)(double)(((float)(double)(normalAcceleration * turboFactor))
                                        + ((float)(double)(((float)(double)(((float)(double)(ctx.vehicle.inputGas * material.braking))
                                            + ((float)(double)(((float)(double)(reverseBias * material.braking))
                                                * ctx.vehicle.inputBrake))))
                                            * acceleration))))))
                                    - ((float)(double)(steerSlowdown * direction)))));
                            float acceleration3 = waterContact
                                ? X87MulDouble(acceleration2, 0.5)
                                : acceleration2;
                            float acceleration4 = ctx.vehicle.flag60c != 0
                                ? (ctx.vehicle.turboType == TMNFVehicleTurboType.None
                                    ? (float)(double)(normalAcceleration * 0.0f)
                                    : (float)(double)(normalAcceleration * ctx.vehicle.turboFactor))
                                : acceleration3;

                            if (0.0 < (double)localSpeed.z)
                            {
                                float product = 1.0f;

                                for (int index = 0; index < wheelCount; index++)
                                {
                                    if (ctx.vehicle.wheels[index].realTime.isSliding != 0)
                                    {
                                        product = (float)(double)(ctx.tuning.model6.slidingBrakeScale
                                            * product);
                                    }
                                }
                                brake = (float)(double)(((float)(double)(((float)(double)(((float)(double)(ctx.tuning.model6.brakeSpeedScale
                                    * localSpeed.z))
                                    + ctx.tuning.model6.brakeBase))
                                    * ctx.vehicle.inputBrake))
                                    * product);
                                float limit = (float)(double)(material.steering
                                    * (sliding != 0
                                        ? ctx.tuning.model6.forwardBrakeLimitSliding
                                        : ctx.tuning.model6.forwardBrakeLimit));
                                if ((double)limit < (double)brake)
                                {
                                    brake = limit;
                                    anySliding = true;
                                    for (int index = 0; index < wheelCount; index++)
                                    {
                                        ctx.vehicle.wheels[index].realTime.isSliding = 1;
                                    }
                                }
                            }
                            if ((double)localSpeed.z < 0.0
                                && Model6PointOne < (double)ctx.vehicle.inputGas)
                            {
                                float product = 1.0f;

                                if (ctx.vehicle.flag60c == 0)
                                {
                                    float trigger = ((float)(double)(((float)(double)(-acceleration4
                                        * ctx.tuning.model6.burnoutTriggerScale))
                                        * localSpeed.z));

                                    if ((double)ctx.tuning.model6.burnoutTriggerLimit < (double)trigger
                                        && Model6PointSevenFive
                                        < (double)uprightValue)
                                    {
                                        ctx.vehicle.model6.burnoutStartTick = tick;
                                        ctx.vehicle.driveMode = 1;
                                        ctx.vehicle.forceWheelSpeed = 1;
                                    }
                                }
                                for (int index = 0; index < wheelCount; index++)
                                {
                                    if (ctx.vehicle.wheels[index].realTime.isSliding != 0)
                                    {
                                        product = (float)(double)(ctx.tuning.model6.slidingBrakeScale
                                            * product);
                                    }
                                }
                                brake = (float)(double)(((float)(double)(((float)(double)(ctx.tuning.model6.brakeBase
                                    - ((float)(double)(ctx.tuning.model6.brakeSpeedScale * localSpeed.z))))
                                    * ctx.vehicle.inputGas))
                                    * product);
                                float limit = (float)(double)(material.steering
                                    * (sliding != 0
                                        ? ctx.tuning.model6.reverseBrakeLimitSliding
                                        : ctx.tuning.model6.reverseBrakeLimit));
                                if ((double)limit < (double)brake)
                                {
                                    brake = limit;
                                    anySliding = true;
                                    for (int index = 0; index < wheelCount; index++)
                                    {
                                        ctx.vehicle.wheels[index].realTime.isSliding = 1;
                                    }
                                }
                            }

                            brakeForce = brake;
                            float net = (float)(double)(acceleration4
                                - ((float)(double)(GmFunc.GmfuncSign(localSpeed.z) * brake)));
                            float net2;
                            if ((double)(((float)(double)(ctx.tuning.model6.forwardSpeedLimitScale
                                * material.acceleration)))
                                < (double)localSpeed.z)
                            {
                                if (0.0 <= (double)net)
                                {
                                    net2 = -ctx.tuning.model6.speedLimitForce;
                                }
                                else
                                {
                                    net2 = (float)(double)(net - ctx.tuning.model6.speedLimitForce);
                                }
                            }
                            else
                            {
                                net2 = net;
                            }
                            float net3;
                            if ((double)localSpeed.z
                                < (double)(-((float)(double)(ctx.tuning.model6.reverseSpeedLimitScale
                                * material.acceleration))))
                            {
                                if ((double)net2 <= 0.0)
                                {
                                    net3 = ctx.tuning.model6.speedLimitForce;
                                }
                                else
                                {
                                    net3 = (float)(double)(ctx.tuning.model6.speedLimitForce + net2);
                                }
                            }
                            else
                            {
                                net3 = net2;
                            }
                            if (DbgHook.Enabled)
                            {
                                Console.Error.WriteLine(
                                    $"NET tick={ctx.vehicle.tickTime} " +
                                    $"speedz={BitConverter.SingleToInt32Bits(localSpeed.z):X8} " +
                                    $"acc={BitConverter.SingleToInt32Bits(acceleration4):X8} " +
                                    $"brake={BitConverter.SingleToInt32Bits(brake):X8} " +
                                    $"net={BitConverter.SingleToInt32Bits(net3):X8} " +
                                    $"longf={BitConverter.SingleToInt32Bits(longitudinalForceFactor):X8} " +
                                    $"exfz={BitConverter.SingleToInt32Bits(existingForce.z):X8}");
                            }
                            GmVec3 force = new GmVec3(
                                0.0f,
                                0.0f,
                                (float)(double)(net3 * longitudinalForceFactor));
                            VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                            GmVec3 force2 = new GmVec3(
                                0.0f,
                                0.0f,
                                (float)(double)(((float)(double)(-(ctx.tuning.model6.verticalForceScale)
                                * existingForce.z))
                                / ctx.tuning.model6.verticalForceDivisor));
                            VehicleBase.AddVehicleCentralForce(ref ctx, ref force2);
                        }
                    }
                }
            }

            // store_sliding: (skipped by the donut path's goto finish)
            if (!jumpedToFinish)
            {
                ctx.vehicle.turboActive = anySliding ? 1 : 0;
            }

            // finish:
            ctx.vehicle.engine.reverse = ctx.vehicle.model6.reverseMode;
            ctx.vehicle.frictionInputSelector = ctx.vehicle.model6.reverseMode;
            ctx.vehicle.currentLocalSpeed = localSpeed;
        }
    }
}
