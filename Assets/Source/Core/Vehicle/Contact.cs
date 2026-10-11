using System;

namespace Tmnf.Sim
{
    /// <summary>vehicle_contact.c: wheel/body contact absorption, friction, materials.</summary>
    public static class VehicleContact
    {
        // CONTACT_LENGTH_EPSILON 0x1.b7cdfcp-34f
        private static readonly float ContactLengthEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EB7_CDFC));
        // FRICTION_SPEED_EPSILON 0x1.4f8b58p-17f
        private static readonly float FrictionSpeedEpsilon =
            BitConverter.Int32BitsToSingle(unchecked((int)0x374F_8B58));
        // CONTACT_ANGLE_RADIANS 0x1.921fb60000000p-1 — float pi/2 promoted
        private static readonly double ContactAngleRadians =
            BitConverter.Int64BitsToDouble(unchecked((long)0x3FE9_21FB_6000_0000));
        // TMNF_PI 0x1.921fb60000000p+1 — float pi promoted
        private static readonly double TmnfPi =
            BitConverter.Int64BitsToDouble(unchecked((long)0x4009_21FB_6000_0000));

        private static float VecDotYxz(ref GmVec3 left, ref GmVec3 right)
        {
            return (float)(double)(((float)(double)(((float)(double)(left.y * right.y)) + ((float)(double)(left.x * right.x)))) + ((float)(double)(left.z * right.z)));
        }

        private static float VecLengthSquaredYxz(ref GmVec3 value)
        {
            return VecDotYxz(ref value, ref value);
        }

        private static float VecLengthSquaredXyz(ref GmVec3 value)
        {
            return (float)(double)(((float)(double)(value.z * value.z)) + ((float)(double)(((float)(double)(value.x * value.x)) + ((float)(double)(value.y * value.y)))));
        }

        private static GmIso4 DynaIso(ref CHmsStateDyna state)
        {
            return state.RotPosIso();
        }

        private static float SlopeCurve(float ratio, float lower, float upper)
        {
            float numerator = (float)(double)(ratio - lower);
            float denominator = (float)(double)(upper - lower);
            float normalized = (float)(double)(numerator / denominator);
            float radians = (float)((double)normalized * TmnfPi);
            float halfRadians = (float)((double)radians * 0.5);
            float cosine = Fp.X87Cos(halfRadians);
            return (float)(double)(1.0f - cosine);
        }

        /// <summary>0x0093A4A0: the timer's current simulation tick.</summary>
        public static uint CmwTimerAdapterGetTickTime(ref VehicleCtx ctx)
        {
            return ctx.vehicle.tickTime;
        }

        /// <summary>0x007BE390: local impulse at a local point + speed clamps.</summary>
        public static void AddVehicleImpulse(ref VehicleCtx ctx, ref GmVec3 impulse, ref GmVec3 point)
        {
            float inverseMass = (float)(double)(1.0f / ctx.dyna.@params.mass);
            GmMat3 rot = ctx.dyna.Live().rot;
            GmIso4 iso = DynaIso(ref ctx.dyna.Live());
            GmVec3 worldImpulse = GmVec3.Zero;
            worldImpulse.SetMultMat3(ref impulse, ref rot);
            GmVec3 worldPoint = GmVec3.Zero;
            worldPoint.SetMultIso4(ref point, ref iso);
            GmVec3 linearDelta = new GmVec3(
                (float)(double)(worldImpulse.x * inverseMass),
                (float)(double)(worldImpulse.y * inverseMass),
                (float)(double)(inverseMass * worldImpulse.z));
            GmVec3 newLinearSpeed;
            {
                ref CHmsStateDyna state = ref ctx.dyna.Live();
                newLinearSpeed = new GmVec3(
                    (float)(double)(state.linVel.x + linearDelta.x),
                    (float)(double)(state.linVel.y + linearDelta.y),
                    (float)(double)(state.linVel.z + linearDelta.z));
            }
            // The old-vs-new speed compare is f64 (single comparison).
            float oldSpeedSquared;
            {
                ref CHmsStateDyna state = ref ctx.dyna.Live();
                oldSpeedSquared = VecLengthSquaredYxz(ref state.linVel);
            }
            float newSpeedSquared = VecLengthSquaredYxz(ref newLinearSpeed);
            if ((double)oldSpeedSquared < (double)newSpeedSquared
                && (double)ctx.tuning.contact.maxLinearSpeedDelta
                < (double)((float)(double)(newSpeedSquared - oldSpeedSquared)))
            {
                newLinearSpeed = GmVec3.Zero;
            }
            ctx.dyna.Live().linVel = newLinearSpeed;

            GmVec3 worldCenter = GmVec3.Zero;
            GmVec3 comOffset = ctx.dyna.@params.comOffset;
            worldCenter.SetMultIso4(ref comOffset, ref iso);
            GmVec3 lever = new GmVec3(
                (float)(double)(worldPoint.x - worldCenter.x),
                (float)(double)(worldPoint.y - worldCenter.y),
                (float)(double)(worldPoint.z - worldCenter.z));
            GmVec3 angularDelta = new GmVec3(
                (float)(double)(((float)(double)(worldImpulse.z * lever.y)) - ((float)(double)(worldImpulse.y * lever.z))),
                (float)(double)(((float)(double)(worldImpulse.x * lever.z)) - ((float)(double)(lever.x * worldImpulse.z))),
                (float)(double)(((float)(double)(lever.x * worldImpulse.y)) - ((float)(double)(worldImpulse.x * lever.y))));
            {
                ref CHmsStateDyna state = ref ctx.dyna.Live();
                GmMat3 invInertiaWorld = state.invInertiaWorld;
                angularDelta.MultMat3(ref invInertiaWorld);
            }
            angularDelta.x = (float)(double)(ctx.tuning.contact.angularXzScale * angularDelta.x);
            angularDelta.y = (float)(double)(ctx.tuning.contact.angularXzScale * angularDelta.y);
            angularDelta.z = (float)(double)(ctx.tuning.contact.angularXzScale * angularDelta.z);
            angularDelta.y = (float)(double)(ctx.tuning.contact.angularYScale * angularDelta.y);
            {
                ref CHmsStateDyna state = ref ctx.dyna.Live();
                state.angVel.x = (float)(double)(state.angVel.x + angularDelta.x);
                state.angVel.y = (float)(double)(state.angVel.y + angularDelta.y);
                state.angVel.z = (float)(double)(state.angVel.z + angularDelta.z);
            }

            float angularSpeedSquared;
            {
                ref CHmsStateDyna state = ref ctx.dyna.Live();
                angularSpeedSquared = VecLengthSquaredYxz(ref state.angVel);
            }
            float maxAngularSquared =
                (float)(double)(ctx.tuning.contact.maxAngularSpeed * ctx.tuning.contact.maxAngularSpeed);
            if ((double)maxAngularSquared < (double)angularSpeedSquared)
            {
                float angularSpeed = Fp.Sqrt(angularSpeedSquared);
                float ratio = (float)(double)(ctx.tuning.contact.maxAngularSpeed / angularSpeed);

                ref CHmsStateDyna state = ref ctx.dyna.Live();
                state.angVel.x = (float)(double)(state.angVel.x * ratio);
                state.angVel.y = (float)(double)(state.angVel.y * ratio);
                state.angVel.z = (float)(double)(ratio * state.angVel.z);
            }

            ctx.vehicle.totalImpulseAdded.x = (float)(double)(impulse.x + ctx.vehicle.totalImpulseAdded.x);
            ctx.vehicle.totalImpulseAdded.y = (float)(double)(impulse.y + ctx.vehicle.totalImpulseAdded.y);
            ctx.vehicle.totalImpulseAdded.z = (float)(double)(impulse.z + ctx.vehicle.totalImpulseAdded.z);
        }

        private static void ApplyWheelImpulse(
            ref VehicleCtx ctx,
            ref GmVec3 relativeSpeed,
            ref GmVec3 normal,
            ref GmVec3 point,
            float restitution)
        {
            ref CHmsDynaParams prms = ref ctx.dyna.@params;
            GmVec3 lever = new GmVec3(
                (float)(double)(point.x - prms.comOffset.x),
                (float)(double)(point.y - prms.comOffset.y),
                (float)(double)(point.z - prms.comOffset.z));
            GmMat3 invInertiaBody = prms.invInertiaBody;
            GmVec3 impulse = VehicleBase.SdynaMathComputeImpulse(
                prms.mass, ref invInertiaBody, -restitution,
                ref relativeSpeed, ref normal, ref lever);
            AddVehicleImpulse(ref ctx, ref impulse, ref point);
        }

        /// <summary>0x007C11D0: classifies a wheel contact, absorbs, impulses.</summary>
        public static void WheelAbsorbContact(
            ref VehicleCtx ctx,
            int wheelIndex,
            ref CHmsPhysicalContact contact,
            System.Func<uint, GmIso4> resolveBodyIso)
        {
            // The contact rotation is the COMMITTED state's rotation
            // (world->committed_state.rot), not the live one.
            GmMat3 contactRotation = ctx.dyna.StateB().rot;
            float groundThreshold = Fp.X87Sin((float)ContactAngleRadians);
            bool groundContact =
                (double)Math.Abs(contact.normal.x) < (double)groundThreshold;

            ctx.vehicle.wheels[wheelIndex].realTime.hasGroundContact = groundContact ? 1 : 0;
            if (!groundContact)
            {
                ctx.vehicle.sideContact = 1;
                ctx.vehicle.wheels[wheelIndex].contactRelativeLocalDistance = contact.position;
                ctx.vehicle.wheels[wheelIndex].field15c = 1;
            }
            if (groundContact)
            {
                CSceneVehicleCarWheelRealTimeState rt = ctx.vehicle.wheels[wheelIndex].realTime;
                rt.groundContactCount += 1;
                rt.field90.x = (float)(double)(rt.field90.x + contact.normal.x);
                rt.field90.y = (float)(double)(contact.normal.y + rt.field90.y);
                rt.field90.z = (float)(double)(contact.normal.z + rt.field90.z);
                // set_contact_material_id: u16 store into the low half.
                rt.contactMaterialId = (int)(((uint)rt.contactMaterialId & 0xFFFF0000u)
                    | (uint)contact.otherSurfaceMaterial);
                ctx.vehicle.wheels[wheelIndex].realTime = rt;
            }
            contact.accepted = 0;

            if (contact.otherBodyCorpusRef.HasValue)
            {
                uint otherCorpus = contact.otherBodyCorpusRef.Value;
                GmIso4 otherIso = resolveBodyIso(otherCorpus);
                GmVec3 otherUp = new GmVec3(otherIso.m2, otherIso.m5, otherIso.m8);
                otherUp.MultTranspose(ref contactRotation);
                ctx.vehicle.wheels[wheelIndex].realTime.relativeRotzAxis = otherUp;
                ctx.vehicle.wheels[wheelIndex].contactBody = otherCorpus;
            }
            ctx.vehicle.wheels[wheelIndex].realTime.field54 = contact.position;

            if (ctx.tuning.contact.wheelContactModel == 2)
            {
                float replacementY = (float)(double)(((float)(double)(((float)(double)(contact.replacement.x * 0.0f))
                    + contact.replacement.y))
                    + ((float)(double)(0.0f * contact.replacement.z)));
                bool replacementAccepted;

                if (!(0.0 < (double)replacementY))
                {
                    replacementAccepted = true;
                }
                else
                {
                    bool accepted = false;
                    if (!((double)ctx.tuning.contact.damperMax
                        < (double)(-FrictionSpeedEpsilon)))
                    {
                        float damperReplacement =
                            (float)(double)(ctx.vehicle.wheels[wheelIndex].realTime.damperAbsorb
                            - ctx.tuning.contact.damperMax);

                        if (!((double)replacementY < (double)damperReplacement))
                        {
                            accepted = true;
                            replacementY = damperReplacement;
                        }
                    }
                    replacementAccepted = accepted;
                    float contactReplacement = replacementY;
                    if ((double)replacementY
                        <= (double)ctx.vehicle.wheels[wheelIndex].realTime.field08)
                    {
                        replacementY = ctx.vehicle.wheels[wheelIndex].realTime.field08;
                    }
                    ctx.vehicle.wheels[wheelIndex].realTime.field08 = replacementY;
                    {
                        float zeroReplacement = (float)(double)(0.0f * contactReplacement);
                        contact.replacement.x = (float)(double)(contact.replacement.x - zeroReplacement);
                        contact.replacement.y = (float)(double)(contact.replacement.y - contactReplacement);
                        contact.replacement.z = (float)(double)(contact.replacement.z - zeroReplacement);
                    }
                }

                if (!groundContact)
                {
                    float restitution = contact.otherSurfaceMaterial == 4
                        ? ctx.tuning.contact.restitutionAirMaterial4
                        : ctx.tuning.contact.restitutionAir;
                    float normalSpeed = VecDotYxz(ref contact.normal, ref contact.relativeSpeed);

                    if ((double)normalSpeed < 0.0)
                    {
                        GmVec3 impulsePoint = contact.position;
                        impulsePoint.y = ctx.dyna.@params.comOffset.y;
                        ApplyWheelImpulse(
                            ref ctx, ref contact.relativeSpeed, ref contact.normal,
                            ref impulsePoint, restitution);
                    }
                }
                else
                {
                    float restitution = contact.otherSurfaceMaterial == 4
                        ? ctx.tuning.contact.restitutionGroundMaterial4
                        : ctx.tuning.contact.restitutionGround;
                    float normalSpeed = VecDotYxz(ref contact.normal, ref contact.relativeSpeed);

                    if ((double)normalSpeed < 0.0)
                    {
                        GmVec3 projected = new GmVec3(
                            (float)(double)(contact.normal.x * normalSpeed),
                            (float)(double)(contact.normal.y * normalSpeed),
                            (float)(double)(normalSpeed * contact.normal.z));
                        float verticalProjection = (float)(double)(((float)(double)(((float)(double)(projected.x * 0.0f)) + projected.y))
                            + ((float)(double)(projected.z * 0.0f)));
                        if (replacementAccepted
                            || !((double)verticalProjection < 0.0))
                        {
                            GmVec3 contactPosition = contact.position;
                            ApplyWheelImpulse(
                                ref ctx, ref contact.relativeSpeed, ref contact.normal,
                                ref contactPosition, restitution);
                        }
                        else
                        {
                            float horizontalProjection = (float)(double)(verticalProjection * 0.0f);

                            GmVec3 adjustedSpeed = new GmVec3(
                                (float)(double)(contact.relativeSpeed.x - horizontalProjection),
                                (float)(double)(contact.relativeSpeed.y - verticalProjection),
                                (float)(double)(contact.relativeSpeed.z - horizontalProjection));
                            if ((double)VecDotYxz(ref contact.normal, ref adjustedSpeed) < 0.0)
                            {
                                GmVec3 impulsePoint =
                                    ctx.vehicle.wheels[wheelIndex].impulsePoint;
                                ApplyWheelImpulse(
                                    ref ctx, ref adjustedSpeed, ref contact.normal,
                                    ref impulsePoint, restitution);
                            }
                        }
                    }
                }
            }
        }

        private static int WheelIndexFromTree(ref VehicleCtx ctx, uint treeRef)
        {
            for (int i = 0; i < ctx.tuning.wheelTreeRefs.Count; i++)
            {
                if (ctx.tuning.wheelTreeRefs[i] == treeRef)
                {
                    return i;
                }
            }
            foreach (uint r in ctx.tuning.bodyTreeRefs)
            {
                if (r == treeRef)
                {
                    return -1;
                }
            }
            throw new InvalidOperationException($"tmnf: contact tree ref {treeRef} is neither wheel nor body");
        }

        private static void AbsorbBodyContact(ref VehicleCtx ctx, ref CHmsPhysicalContact contact)
        {
            // reject: label — fallthrough semantics preserved with a flag.
            bool rejected = false;
            do
            {
                if ((double)contact.normal.y < -0.75
                    && ctx.tuning.contact.frictionModel == 5)
                {
                    float replacementProjection =
                        VecDotYxz(ref contact.normal, ref contact.replacement);

                    contact.replacement.x = (float)(double)(contact.normal.x * replacementProjection);
                    contact.replacement.y = (float)(double)(contact.normal.y * replacementProjection);
                    contact.replacement.z = (float)(double)(replacementProjection * contact.normal.z);
                }

                ctx.vehicle.bodyContactPositionSum.x = (float)(double)(ctx
                    .vehicle.bodyContactPositionSum.x + contact.position.x);
                ctx.vehicle.bodyContactPositionSum.y =
                    (float)(double)(contact.position.y + ctx.vehicle.bodyContactPositionSum.y);
                ctx.vehicle.bodyContactPositionSum.z = (float)(double)(ctx
                    .vehicle.bodyContactPositionSum.z + contact.position.z);
                ctx.vehicle.bodyContactNormalSum.x =
                    (float)(double)(contact.normal.x + ctx.vehicle.bodyContactNormalSum.x);
                ctx.vehicle.bodyContactNormalSum.y =
                    (float)(double)(contact.normal.y + ctx.vehicle.bodyContactNormalSum.y);
                ctx.vehicle.bodyContactNormalSum.z =
                    (float)(double)(contact.normal.z + ctx.vehicle.bodyContactNormalSum.z);
                ctx.vehicle.bodyContactCount += 1;
                ctx.vehicle.model6.contactBlockCount = 1;
                ctx.vehicle.compute.eventSourceC = (byte)contact.otherSurfaceMaterial;

                if (ctx.tuning.contact.wheelContactModel != 2)
                {
                    rejected = true;
                    break;
                }
                float normalSpeed = VecDotYxz(ref contact.normal, ref contact.relativeSpeed);
                if (!((double)normalSpeed < 0.0))
                {
                    rejected = true;
                    break;
                }

                float tangentRatio;
                float restitution;
                if (contact.otherSurfaceMaterial == 4)
                {
                    tangentRatio = ctx.tuning.contact.bodyTangentRatioMaterial4;
                    restitution = ctx.tuning.contact.restitutionAirMaterial4;
                }
                else
                {
                    tangentRatio = ctx.tuning.contact.bodyTangentRatio;
                    restitution = ctx.tuning.contact.restitutionAir;
                }
                GmVec3 normalProjection = new GmVec3(
                    (float)(double)(contact.normal.x * normalSpeed),
                    (float)(double)(contact.normal.y * normalSpeed),
                    (float)(double)(normalSpeed * contact.normal.z));
                GmVec3 tangent = new GmVec3(
                    (float)(double)(contact.relativeSpeed.x - normalProjection.x),
                    (float)(double)(contact.relativeSpeed.y - normalProjection.y),
                    (float)(double)(contact.relativeSpeed.z - normalProjection.z));
                float normalLength = Fp.Sqrt(VecLengthSquaredXyz(ref normalProjection));
                float tangentLength = Fp.Sqrt(VecLengthSquaredXyz(ref tangent));
                {
                    float tangentLimit = (float)(double)(normalLength * tangentRatio);

                    if ((double)tangentLimit < (double)tangentLength)
                    {
                        float scale = (float)(double)(tangentLimit / tangentLength);

                        tangent.x = (float)(double)(scale * tangent.x);
                        tangent.y = (float)(double)(tangent.y * scale);
                        tangent.z = (float)(double)(scale * tangent.z);
                    }
                }
                GmVec3 impulseDirection = new GmVec3(
                    -((float)(double)(tangent.x + normalProjection.x)),
                    -((float)(double)(tangent.y + normalProjection.y)),
                    -((float)(double)(tangent.z + normalProjection.z)));
                float impulseLength = Fp.Sqrt(VecLengthSquaredXyz(ref impulseDirection));
                if (!((double)FrictionSpeedEpsilon < (double)impulseLength))
                {
                    rejected = true;
                    break;
                }
                {
                    float inverseLength = (float)(double)(1.0f / impulseLength);

                    impulseDirection.x = (float)(double)(inverseLength * impulseDirection.x);
                    impulseDirection.y = (float)(double)(impulseDirection.y * inverseLength);
                    impulseDirection.z = (float)(double)(impulseDirection.z * inverseLength);
                }
                ref CHmsDynaParams prms = ref ctx.dyna.@params;
                GmVec3 lever = new GmVec3(
                    (float)(double)(contact.position.x - prms.comOffset.x),
                    (float)(double)(contact.position.y - prms.comOffset.y),
                    (float)(double)(contact.position.z - prms.comOffset.z));
                GmMat3 invInertiaBody = prms.invInertiaBody;
                GmVec3 impulse = VehicleBase.SdynaMathComputeImpulse(
                    prms.mass, ref invInertiaBody, -restitution,
                    ref contact.relativeSpeed, ref impulseDirection, ref lever);
                GmVec3 contactPosition = contact.position;
                AddVehicleImpulse(ref ctx, ref impulse, ref contactPosition);
            } while (false);
            _ = rejected;

            // reject:
            contact.accepted = 0;
        }

        /// <summary>0x007C3410: filters scene contacts and dispatches response.</summary>
        public static void AbsorbContact(
            ref VehicleCtx ctx,
            ref CHmsPhysicalContact contact,
            System.Func<uint, GmIso4> resolveBodyIso)
        {
            if (contact.otherSurfaceMaterial == 13
                || contact.otherSurfaceMaterial == 23)
            {
                contact.replacement = GmVec3.Zero;
                contact.accepted = 0;
                return;
            }

            ctx.vehicle.airControlImmediate = 1;
            int wheelIndex = WheelIndexFromTree(ref ctx, contact.treeRef);
            float impact = Math.Abs(VecDotYxz(ref contact.normal, ref contact.relativeSpeed));
            if (wheelIndex >= 0 && !((double)contact.normal.y < 0.2))
            {
                if (ctx.vehicle.wheels[wheelIndex].steerable != 0)
                {
                    ctx.vehicle.compute.eventMetricA = (float)(double)(impact + ctx.vehicle.compute.eventMetricA);
                }
                else
                {
                    ctx.vehicle.compute.eventMetricB = (float)(double)(ctx.vehicle.compute.eventMetricB + impact);
                }
            }
            else
            {
                ctx.vehicle.compute.eventMetricC = (float)(double)(ctx.vehicle.compute.eventMetricC + impact);
            }

            if (wheelIndex < 0)
            {
                AbsorbBodyContact(ref ctx, ref contact);
                return;
            }
            ctx.vehicle.compute.eventSourceAb = (byte)contact.otherSurfaceMaterial;
            WheelAbsorbContact(ref ctx, wheelIndex, ref contact, resolveBodyIso);
            ctx.vehicle.wheelContactAbsorbCount += 1;
        }

        /// <summary>0x007BEB40: contact-normal slope to adherence ramps.</summary>
        public static (float, float) GetSlopeAdherence(ref VehicleCtx ctx, ref GmVec3 normal)
        {
            float lengthSquared = VecLengthSquaredYxz(ref normal);

            if ((double)ContactLengthEpsilon < (double)lengthSquared)
            {
                float length = Fp.Sqrt(lengthSquared);
                float ratio = (float)Math.Abs((double)((float)(double)(normal.y / length)));

                float adherence = ratio;
                if ((double)ratio < (double)ctx.tuning.contact.slopeAdherenceMin)
                {
                    adherence = 0.0f;
                }
                else if ((double)ctx.tuning.contact.slopeAdherenceMax < (double)ratio)
                {
                    adherence = 1.0f;
                }
                else
                {
                    adherence = SlopeCurve(
                        ratio,
                        ctx.tuning.contact.slopeAdherenceMin,
                        ctx.tuning.contact.slopeAdherenceMax);
                }

                ratio = (float)Math.Abs((double)((float)(double)(normal.y / length)));
                float secondary = ratio;
                if ((double)ratio < (double)ctx.tuning.contact.slopeSecondaryMin)
                {
                    secondary = 0.0f;
                    return (adherence, secondary);
                }
                if ((double)ctx.tuning.contact.slopeSecondaryMax < (double)ratio)
                {
                    secondary = 1.0f;
                    return (adherence, secondary);
                }
                secondary = SlopeCurve(
                    ratio,
                    ctx.tuning.contact.slopeSecondaryMin,
                    ctx.tuning.contact.slopeSecondaryMax);
                return (adherence, secondary);
            }
            return (0.0f, 0.0f);
        }

        /// <summary>0x007BD1E0: whether any wheel has ground contact.</summary>
        public static bool IsGroundContact(ref VehicleCtx ctx)
        {
            foreach (CSceneVehicleCarWheel wheel in ctx.vehicle.wheels)
            {
                if (wheel.realTime.hasGroundContact != 0)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>0x007BF5C0: every contacting wheel has the requested material.</summary>
        public static bool IsAllWheelGroundContactId(ref VehicleCtx ctx, byte materialId)
        {
            uint inactiveCount = 0;

            foreach (CSceneVehicleCarWheel wheel in ctx.vehicle.wheels)
            {
                if (wheel.realTime.hasGroundContact == 0)
                {
                    inactiveCount += 1;
                }
                else if ((ushort)wheel.realTime.contactMaterialId != materialId)
                {
                    return false;
                }
            }
            return inactiveCount < (uint)ctx.vehicle.wheels.Count;
        }

        /// <summary>0x007BF620: first contacting wheel with the requested material.</summary>
        public static (GmVec3, uint?)? IsGroundContactId(ref VehicleCtx ctx, byte materialId)
        {
            foreach (CSceneVehicleCarWheel wheel in ctx.vehicle.wheels)
            {
                if (wheel.realTime.hasGroundContact != 0
                    && (ushort)wheel.realTime.contactMaterialId == materialId)
                {
                    return (wheel.realTime.relativeRotzAxis, wheel.contactBody);
                }
            }
            return null;
        }

        /// <summary>0x007BED10: central drag + timed lateral slowdown.</summary>
        public static void ApplyFrictionForces(ref VehicleCtx ctx, ref GmVec3 velocity)
        {
            if ((ctx.tuning.contact.frictionModel == 4
                || ctx.tuning.contact.frictionModel == 5)
                && ctx.vehicle.airborneFrictionGate != 0
                && !IsGroundContact(ref ctx))
            {
                return;
            }

            float selectedInput = ctx.vehicle.frictionInputSelector == 0
                ? ctx.vehicle.inputGas
                : ctx.vehicle.inputBrake;
            if ((double)selectedInput < (double)FrictionSpeedEpsilon
                || ctx.vehicle.flag60c != 0)
            {
                GmVec3 force = velocity;
                float lengthSquared = VecLengthSquaredYxz(ref force);

                if ((double)ContactLengthEpsilon < (double)lengthSquared)
                {
                    float length = Fp.Sqrt(lengthSquared);
                    float inverseLength = (float)(double)(1.0f / length);
                    float scale;

                    force.x = (float)(double)(inverseLength * force.x);
                    force.y = (float)(double)(force.y * inverseLength);
                    force.z = (float)(double)(inverseLength * force.z);
                    scale = -(ctx.tuning.contact.frictionForce);
                    force.x = (float)(double)(scale * force.x);
                    force.y = (float)(double)(force.y * scale);
                    force.z = (float)(double)(scale * force.z);
                    if (ctx.vehicle.flag60c == 0)
                    {
                        GmVec3 extra;
                        scale = -(ctx.tuning.contact.extraFrictionForce);
                        extra = new GmVec3(
                            (float)(double)(velocity.x * scale),
                            (float)(double)(velocity.y * scale),
                            (float)(double)(scale * velocity.z));
                        force.x = (float)(double)(extra.x + force.x);
                        force.y = (float)(double)(extra.y + force.y);
                        force.z = (float)(double)(extra.z + force.z);
                    }
                    VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                }
            }

            if (ctx.tuning.contact.frictionModel < 4)
            {
                if (ctx.vehicle.sideContact != 0)
                {
                    float scale = -(Curve.CsceneVehicleCarTuningGetLateralContactSlowDownFromSpeed(
                        ctx.tuning.curves, velocity.z));
                    GmVec3 force = new GmVec3(
                        (float)(double)(scale * velocity.x),
                        (float)(double)(velocity.y * scale),
                        (float)(double)(scale * velocity.z));
                    VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                }
                return;
            }

            {
                uint tick = ctx.vehicle.tickTime;

                if (ctx.vehicle.sideContact != 0)
                {
                    ctx.vehicle.lastSideContactTick = tick;
                }
                if (ctx.vehicle.lastSideContactTick <= tick
                    && unchecked(tick - ctx.vehicle.lastSideContactTick)
                    < ctx.tuning.contact.lateralContactDurationTicks)
                {
                    float lengthSquared = VecLengthSquaredXyz(ref velocity);
                    float length = Fp.Sqrt(lengthSquared);

                    if ((double)FrictionSpeedEpsilon < (double)length)
                    {
                        float inverseLength = (float)(double)(1.0f / length);
                        GmVec3 normalized = new GmVec3(
                            (float)(double)(velocity.x * inverseLength),
                            (float)(double)(velocity.y * inverseLength),
                            (float)(double)(inverseLength * velocity.z));
                        float scale = -(Curve.CsceneVehicleCarTuningM5GetLateralContactSlowDownFromSpeed(
                            ctx.tuning.curves, length));
                        GmVec3 force = new GmVec3(
                            (float)(double)(scale * normalized.x),
                            (float)(double)(normalized.y * scale),
                            (float)(double)(scale * normalized.z));
                        VehicleBase.AddVehicleCentralForce(ref ctx, ref force);
                    }
                }
            }
        }

        /// <summary>0x007BF080: Model 4 lateral friction for one material blend.</summary>
        public static (float, bool) GetLateralFriction(
            ref VehicleCtx ctx,
            ref GmVec3 velocity,
            ref GmVec3 lateralAxis,
            ref CSceneVehicleMaterialBlendableVals material,
            float load,
            bool grounded)
        {
            float lateralSpeed = VecDotYxz(ref velocity, ref lateralAxis);
            float absoluteSpeed = Math.Abs(lateralSpeed);
            float linear = (float)(double)(ctx.tuning.contact.lateralLinear * (-lateralSpeed));
            float quadratic = (float)(double)(((float)(double)(lateralSpeed * absoluteSpeed))
                * ctx.tuning.contact.lateralQuadratic);
            float requested = (float)(double)(linear - quadratic);
            float groundScale = !grounded ? 1.0f : ctx.tuning.contact.lateralGroundScale;
            float maximum = Curve.CsceneVehicleCarTuningM4GetMaxFrictionForceFromSpeed(
                ctx.tuning.curves, absoluteSpeed);
            float limit = (float)(double)(material.lateralGrip * load);

            limit = (float)(double)(limit * maximum);
            limit = (float)(double)(limit * groundScale);
            if ((double)limit < (double)Math.Abs(requested))
            {
                float sign = GmFunc.GmfuncSign(requested);
                return ((float)(double)(limit * sign), true);
            }
            return (requested, false);
        }

        /// <summary>0x007C2800: averages the material blend for contacting wheels.</summary>
        public static (CSceneVehicleMaterialBlendableVals, bool) ComputeVehicleGroundMaterialVals(
            ref VehicleCtx ctx)
        {
            float[] output = { 0.0f, 0.0f, 0.0f, 0.0f };
            uint contactCount = 0;
            bool hasMaterial = false;

            for (int index = 0; index < ctx.vehicle.wheels.Count; index++)
            {
                if (ctx.vehicle.wheels[index].realTime.hasGroundContact != 0)
                {
                    ushort materialId =
                        (ushort)ctx.vehicle.wheels[0].realTime.contactMaterialId;
                    uint materialIndex = ctx.tuning.groundMaterialIndices[materialId];
                    TMNFVehicleGroundMaterial material =
                        ctx.tuning.groundMaterials[(int)materialIndex];

                    contactCount += 1;
                    output[0] = (float)(double)(material.values[0] + output[0]);
                    output[1] = (float)(double)(material.values[1] + output[1]);
                    output[2] = (float)(double)(material.values[2] + output[2]);
                    output[3] = (float)(double)(material.values[3] + output[3]);
                    hasMaterial = true;
                }
            }
            if (contactCount != 0)
            {
                float floatCount = Fp.U32ToX87Float(contactCount);
                float inverseCount = (float)(double)(1.0f / floatCount);
                output[0] = (float)(double)(inverseCount * output[0]);
                output[1] = (float)(double)(output[1] * inverseCount);
                output[2] = (float)(double)(inverseCount * output[2]);
                output[3] = (float)(double)(inverseCount * output[3]);
            }
            CSceneVehicleMaterialBlendableVals vals = default(CSceneVehicleMaterialBlendableVals);
            vals.acceleration = output[0];
            vals.braking = output[1];
            vals.steering = output[2];
            vals.lateralGrip = output[3];
            return (vals, hasMaterial);
        }
    }
}
