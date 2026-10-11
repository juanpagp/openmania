using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The curve key interpolation layer (vehicle_curve.c): bounds compilation,
    /// adjacent-key search (forward and backward), normalized blend, and the
    /// 21 tuning getters.
    /// </summary>
    public static class Curve
    {
        // CURVE_BOUNDARY_EPSILON: 0x1.4f8b588e368f1p-17 (an f64 constant —
        // NOT the f32 blend epsilon)
        private static readonly double CurveBoundaryEpsilon =
            BitConverter.Int64BitsToDouble(unchecked((long)0x3EE4_F8B5_88E3_68F1));
        // CURVE_BLEND_EPSILON: 0x1.4f8b6p-17f
        private static readonly float CurveBlendEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x3727_C5B0));
        // METERS_PER_SECOND_TO_KMH: 0x1.ccccccp+1 as f64 == (double)3.6f32
        private static readonly double MetersPerSecondToKmh =
            BitConverter.Int64BitsToDouble(unchecked((long)0x400C_CCCC_C000_0000));

        private static float CurveLowerBoundary(float position)
        {
            return (float)((double)position - CurveBoundaryEpsilon);
        }

        private static float CurveUpperBoundary(float position)
        {
            return (float)((double)position + CurveBoundaryEpsilon);
        }

        private static float TuningSpeedPosition(float speed)
        {
            return (float)((double)speed * MetersPerSecondToKmh);
        }

        /// <summary>
        /// Throws when a required curve is absent (the original would
        /// dereference NULL).
        /// </summary>
        private static CFuncKeysReal ExpectCurve(CFuncKeysReal curve, string field)
        {
            if (curve == null)
            {
                throw new InvalidOperationException($"tmnf vehicle curve: {field} is NULL (the original would dereference it)");
            }
            return curve;
        }

        /// <summary>CFuncKeys_Compile: precompute the bounds from positions.</summary>
        public static void CfuncKeysCompile(CFuncKeys keys)
        {
            float[] lower = new float[keys.count];
            float[] upper = new float[keys.count];
            for (uint i = 0; i < keys.count; i++)
            {
                lower[i] = CurveLowerBoundary(keys.positions[i]);
                upper[i] = CurveUpperBoundary(keys.positions[i]);
            }
            keys.lowerBounds = lower;
            keys.upperBounds = upper;
        }

        /// <summary>
        /// 0x005914C0: locate the adjacent position keys in the caller's
        /// direction. u32 index arithmetic wraps exactly like the original.
        /// The BACKWARD direction indexes out of bounds when no pair at or
        /// below the cached start matches — that panic is part of the
        /// contract (the original reads heap garbage there).
        /// </summary>
        public static void CfuncKeysGetBoundingIndices(
            CFuncKeys keys,
            float position,
            ref uint lowerIndex,
            ref uint upperIndex,
            bool forward)
        {
            uint count = keys.count;
            uint lower;
            uint upper;

            if (count == 0)
            {
                lowerIndex = uint.MaxValue;
                upperIndex = uint.MaxValue;
                return;
            }
            if (count == 1)
            {
                lowerIndex = 0;
                upperIndex = 0;
                return;
            }
            float[] lowerBounds = keys.lowerBounds;
            float[] upperBounds = keys.upperBounds;
            if (position < lowerBounds[0])
            {
                lowerIndex = 0;
                upperIndex = 0;
                return;
            }

            lower = count - 1;
            if (upperBounds[lower] < position)
            {
                lowerIndex = lower;
                upperIndex = lower;
                return;
            }

            // u32 index arithmetic wraps exactly as the original's does.
            uint searchIndex = lowerIndex;
            uint attempts = 0;
            if (forward)
            {
                if (searchIndex >= count)
                {
                    searchIndex = 0;
                }
                while (true)
                {
                    lower = searchIndex;
                    searchIndex = unchecked(searchIndex + 1);
                    if (searchIndex >= count)
                    {
                        searchIndex = 0;
                    }
                    upper = searchIndex;
                    attempts = unchecked(attempts + 1);
                    if (attempts > count)
                    {
                        break;
                    }
                    if (!(lowerBounds[lower] <= position)
                        || !(position <= upperBounds[upper]))
                    {
                        continue;
                    }
                    break;
                }
            }
            else
            {
                searchIndex = unchecked(searchIndex + 1);
                if ((searchIndex & 0x8000_0000) != 0)
                {
                    searchIndex = 0;
                }
                while (true)
                {
                    upper = searchIndex;
                    searchIndex = unchecked(searchIndex - 1);
                    if ((searchIndex & 0x8000_0000) != 0)
                    {
                        searchIndex = count;
                    }
                    lower = searchIndex;
                    attempts = unchecked(attempts + 1);
                    if (attempts > count)
                    {
                        break;
                    }
                    if (!(lowerBounds[lower] <= position)
                        || !(position <= upperBounds[upper]))
                    {
                        continue;
                    }
                    break;
                }
            }

            lowerIndex = lower;
            upperIndex = upper;
        }

        /// <summary>0x00591670: locate the adjacent keys + normalized blend.</summary>
        public static bool CfuncKeysComputeBlendCoef(
            CFuncKeys keys,
            float position,
            ref uint lowerIndex,
            ref uint upperIndex,
            ref float blend,
            bool forward)
        {
            CfuncKeysGetBoundingIndices(keys, position, ref lowerIndex, ref upperIndex, forward);
            if (lowerIndex == uint.MaxValue)
            {
                return false;
            }
            if (lowerIndex == upperIndex)
            {
                blend = 0.0f;
                return true;
            }

            float lowerPosition = keys.positions[lowerIndex];
            float difference = (float)(double)(keys.positions[upperIndex] - lowerPosition);
            float magnitude = Math.Abs(difference);
            if (magnitude < CurveBlendEpsilon)
            {
                blend = 0.0f;
                return true;
            }

            float numerator = (float)(double)(position - lowerPosition);
            blend = (float)(double)(numerator / difference);
            return true;
        }

        /// <summary>0x00585E70: evaluate with the selected interpolation mode.</summary>
        public static void CfuncKeysRealGetRealAt(
            CFuncKeysReal curve,
            float position,
            ref float value,
            ref uint lowerIndex,
            ref uint upperIndex,
            ref float blend,
            int interpolation,
            bool forward)
        {
            if (!CfuncKeysComputeBlendCoef(
                curve.keys, position, ref lowerIndex, ref upperIndex, ref blend, forward))
            {
                value = 0.0f;
                return;
            }
            if (interpolation == 1)
            {
                value = curve.values[lowerIndex];
                return;
            }

            float lowerWeight = (float)(1.0 - (double)blend);
            float lowerValue = (float)(double)(lowerWeight * curve.values[lowerIndex]);
            float upperValue = (float)(double)(blend * curve.values[upperIndex]);
            value = (float)(double)(lowerValue + upperValue);
        }

        private static void CfuncKeysRealGetValueOutInterp(
            CFuncKeysReal curve,
            float position,
            ref float value,
            ref uint lowerIndex,
            int interpolation)
        {
            uint upperIndex = unchecked(lowerIndex + 1);
            float blend = 0.0f;

            CfuncKeysRealGetRealAt(
                curve, position, ref value, ref lowerIndex, ref upperIndex,
                ref blend, interpolation, true);
        }

        /// <summary>0x00586200: evaluate and update the cached lower index.</summary>
        public static void CfuncKeysRealGetValueOut(
            CFuncKeysReal curve,
            float position,
            ref float value,
            ref uint lowerIndex)
        {
            CfuncKeysRealGetValueOutInterp(curve, position, ref value, ref lowerIndex, curve.interpolation);
        }

        /// <summary>0x00586240: return a curve value with a cached lower index.</summary>
        public static float CfuncKeysRealGetValue(CFuncKeysReal curve, float position, ref uint lowerIndex)
        {
            uint upperIndex = unchecked(lowerIndex + 1);
            float blend = 0.0f;
            float value = 0.0f;

            CfuncKeysRealGetRealAt(
                curve, position, ref value, ref lowerIndex, ref upperIndex,
                ref blend, curve.interpolation, true);
            return value;
        }

        // ------------------------------------------------------------------
        // tuning getters (0x007F3BE0 .. 0x007F4130)
        // ------------------------------------------------------------------

        private static float TuningGetValue(CFuncKeysReal curve, float position)
        {
            uint lowerIndex = 0;
            float value = 0.0f;
            CfuncKeysRealGetValueOut(curve, position, ref value, ref lowerIndex);
            return value;
        }

        private static float TuningGetStepValue(CFuncKeysReal curve, float position)
        {
            uint lowerIndex = 0;
            float value = 0.0f;
            CfuncKeysRealGetValueOutInterp(curve, position, ref value, ref lowerIndex, 1);
            return value;
        }

        /// <summary>0x007F3BE0: acceleration curve after km/h conversion.</summary>
        public static float CsceneVehicleCarTuningGetAccelFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetStepValue(
                ExpectCurve(curves.accelFromSpeed, "accel_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3C30: lateral rollover against km/h.</summary>
        public static float CsceneVehicleCarTuningGetRolloverLateralFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.rolloverLateralFromSpeed, "rollover_lateral_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3C70: maximum side friction against km/h.</summary>
        public static float CsceneVehicleCarTuningGetMaxSideFrictionFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.maxSideFrictionFromSpeed, "max_side_friction_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3CB0: lateral contact slowdown against km/h.</summary>
        public static float CsceneVehicleCarTuningGetLateralContactSlowDownFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetStepValue(
                ExpectCurve(curves.lateralContactSlowdownFromSpeed, "lateral_contact_slowdown_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3D00: steering slowdown against km/h.</summary>
        public static float CsceneVehicleCarTuningGetSteerSlowDownFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetStepValue(
                ExpectCurve(curves.steerSlowdownFromSpeed, "steer_slowdown_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3D50: lateral rollover against angle.</summary>
        public static float CsceneVehicleCarTuningGetRolloverLateralCoefFromAngle(CSceneVehicleCarTuningCurveSet curves, float angle)
        {
            return TuningGetValue(
                ExpectCurve(curves.rolloverLateralCoefFromAngle, "rollover_lateral_coef_from_angle"),
                angle);
        }

        /// <summary>0x007F3D80: steering drive torque against km/h.</summary>
        public static float CsceneVehicleCarTuningGetSteerDriveTorqueFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.steerDriveTorqueFromSpeed, "steer_drive_torque_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3DC0: Model 4 steering radius.</summary>
        public static float CsceneVehicleCarTuningM4GetSteerRadiusFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m4SteerRadiusFromSpeed, "m4_steer_radius_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3E00: Model 4 maximum friction.</summary>
        public static float CsceneVehicleCarTuningM4GetMaxFrictionForceFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m4MaxFrictionForceFromSpeed, "m4_max_friction_force_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3E40: Model 5 acceleration.</summary>
        public static float CsceneVehicleCarTuningM5GetAccelFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.accelFromSpeed, "accel_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3E80: Model 5 slipping acceleration (scaled).</summary>
        public static float CsceneVehicleCarTuningM5GetSlippingAccelFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            float value = TuningGetValue(
                ExpectCurve(curves.m5SlippingAccelFromSpeed, "m5_slipping_accel_from_speed"),
                TuningSpeedPosition(speed));
            return (float)(double)(curves.m5SlippingAccelScale * value);
        }

        /// <summary>0x007F3ED0: Model 5 steering slowdown.</summary>
        public static float CsceneVehicleCarTuningM5GetSteerSlowDownFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.steerSlowdownFromSpeed, "steer_slowdown_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3F10: Model 5 lateral contact slowdown (NULL cached index).</summary>
        public static float CsceneVehicleCarTuningM5GetLateralContactSlowDownFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            uint lowerIndex = 0;
            return CfuncKeysRealGetValue(
                ExpectCurve(curves.lateralContactSlowdownFromSpeed, "lateral_contact_slowdown_from_speed"),
                TuningSpeedPosition(speed),
                ref lowerIndex);
        }

        /// <summary>0x007F3F40: water friction against km/h.</summary>
        public static float CsceneVehicleCarTuningGetWaterFrictionFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.waterFrictionFromSpeed, "water_friction_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F3F80: damper modulation after normalization.</summary>
        public static float CsceneVehicleCarTuningM6GetModulationFromDamperAbsorbVal(CSceneVehicleCarTuningCurveSet curves, float damperAbsorb)
        {
            float position;
            if (curves.damperMin == curves.damperMax)
            {
                position = 0.0f;
            }
            else
            {
                float numerator = (float)(double)(damperAbsorb - curves.damperMin);
                float denominator = (float)(double)(curves.damperMax - curves.damperMin);
                position = (float)(double)(numerator / denominator);
            }
            return TuningGetValue(
                ExpectCurve(curves.m6DamperModulation, "m6_damper_modulation"),
                position);
        }

        /// <summary>0x007F3FF0: rear-gear acceleration against km/h.</summary>
        public static float CsceneVehicleCarTuningM6GetRearGearAccelFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m6RearGearAccelFromSpeed, "m6_rear_gear_accel_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F4030: burnout radius against km/h.</summary>
        public static float CsceneVehicleCarTuningM6GetBurnoutRadiusFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m6BurnoutRadiusFromSpeed, "m6_burnout_radius_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F4070: burnout lateral speed (km/h to m/s).</summary>
        public static float CsceneVehicleCarTuningM6GetLateralSpeedFromBurnoutRadius(CSceneVehicleCarTuningCurveSet curves, float radius)
        {
            float value = TuningGetValue(
                ExpectCurve(curves.m6LateralSpeedFromBurnoutRadius, "m6_lateral_speed_from_burnout_radius"),
                radius);
            return (float)((double)value / MetersPerSecondToKmh);
        }

        /// <summary>0x007F40B0: burnout rollover against km/h.</summary>
        public static float CsceneVehicleCarTuningM6GetBurnoutRolloverFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m6BurnoutRolloverFromSpeed, "m6_burnout_rollover_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F40F0: donut rollover against km/h.</summary>
        public static float CsceneVehicleCarTuningM6GetDonutRolloverFromSpeed(CSceneVehicleCarTuningCurveSet curves, float speed)
        {
            return TuningGetValue(
                ExpectCurve(curves.m6DonutRolloverFromSpeed, "m6_donut_rollover_from_speed"),
                TuningSpeedPosition(speed));
        }

        /// <summary>0x007F4130: lateral rollover against speed ratio.</summary>
        public static float CsceneVehicleCarTuningM6GetRolloverLateralFromSpeedRatio(CSceneVehicleCarTuningCurveSet curves, float speedRatio)
        {
            return TuningGetValue(
                ExpectCurve(curves.m6RolloverLateralFromSpeedRatio, "m6_rollover_lateral_from_speed_ratio"),
                TuningSpeedPosition(speedRatio));
        }
    }
}
