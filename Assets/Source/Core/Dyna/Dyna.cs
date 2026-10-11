using System;

namespace Tmnf.Sim
{
    /// <summary>
    /// The 180-byte integrated rigid-body state (CHmsStateDyna). Value type:
    /// the reference Copy semantics map to C# struct assignment. The tail is
    /// five expanded floats (no arrays in structs).
    /// </summary>
    [Serializable]
    public struct CHmsStateDyna
    {
        public GmQuat quat;            /* 0x00 orientation, bytes (w,x,y,z) */
        public GmMat3 rot;             /* 0x10 rotation matrix */
        public GmVec3 pos;             /* 0x34 */
        public GmVec3 linVel;          /* 0x40 */
        public GmVec3 linVelAdded;     /* 0x4C */
        public GmVec3 angVel;          /* 0x58 */
        public GmVec3 force;           /* 0x64 */
        public GmVec3 torque;          /* 0x70 */
        public GmMat3 invInertiaWorld; /* 0x7C */
        public float tail0;            /* 0xA0 */
        public float tail1;
        public float tail2;
        public float tail3;
        public float tail4;            /* 0xB0 */

        /// <summary>
        /// The reference's Default: zeroed bytes, IDENTITY rotation and
        /// quaternion — never use <c>default(T)</c> for this type.
        /// </summary>
        public static CHmsStateDyna Create()
        {
            CHmsStateDyna s = default(CHmsStateDyna);
            s.quat = new GmQuat { x = 1.0f, y = 0.0f, z = 0.0f, w = 0.0f };
            s.rot = GmMat3.Identity;
            s.invInertiaWorld = GmMat3.Identity;
            return s;
        }

        /// <summary>The rot+pos 48-byte overlay as an isometry.</summary>
        public GmIso4 RotPosIso()
        {
            GmIso4 r = default(GmIso4);
            r.m0 = rot.m0; r.m1 = rot.m1; r.m2 = rot.m2;
            r.m3 = rot.m3; r.m4 = rot.m4; r.m5 = rot.m5;
            r.m6 = rot.m6; r.m7 = rot.m7; r.m8 = rot.m8;
            r.t0 = pos.x;
            r.t1 = pos.y;
            r.t2 = pos.z;
            return r;
        }

        /// <summary>Raw little-endian bytes (the golden record / diff form).</summary>
        public byte[] ToBytes()
        {
            byte[] b = new byte[180];
            int o = 0;
            BitsUtil.PutF32(b, o, quat.x); BitsUtil.PutF32(b, o + 4, quat.y);
            BitsUtil.PutF32(b, o + 8, quat.z); BitsUtil.PutF32(b, o + 12, quat.w);
            o += 16;
            for (int i = 0; i < 9; i++) BitsUtil.PutF32(b, o + 4 * i, rot[i]);
            o += 36;
            pos.ToBytes(b, o); o += 12;
            linVel.ToBytes(b, o); o += 12;
            linVelAdded.ToBytes(b, o); o += 12;
            angVel.ToBytes(b, o); o += 12;
            force.ToBytes(b, o); o += 12;
            torque.ToBytes(b, o); o += 12;
            for (int i = 0; i < 9; i++) BitsUtil.PutF32(b, o + 4 * i, invInertiaWorld[i]);
            o += 36;
            BitsUtil.PutF32(b, o, tail0);
            BitsUtil.PutF32(b, o + 4, tail1);
            BitsUtil.PutF32(b, o + 8, tail2);
            BitsUtil.PutF32(b, o + 12, tail3);
            BitsUtil.PutF32(b, o + 16, tail4);
            return b;
        }

        /// <summary>Reads the 180-byte game state.</summary>
        public static CHmsStateDyna FromBytes(byte[] b)
        {
            CHmsStateDyna s = default(CHmsStateDyna);
            s.quat = new GmQuat
            {
                x = BitsUtil.GetF32(b, 0x00),
                y = BitsUtil.GetF32(b, 0x04),
                z = BitsUtil.GetF32(b, 0x08),
                w = BitsUtil.GetF32(b, 0x0c),
            };
            for (int i = 0; i < 9; i++) s.rot[i] = BitsUtil.GetF32(b, 0x10 + 4 * i);
            s.pos = GmVec3.FromBytes(b, 0x34);
            s.linVel = GmVec3.FromBytes(b, 0x40);
            s.linVelAdded = GmVec3.FromBytes(b, 0x4c);
            s.angVel = GmVec3.FromBytes(b, 0x58);
            s.force = GmVec3.FromBytes(b, 0x64);
            s.torque = GmVec3.FromBytes(b, 0x70);
            for (int i = 0; i < 9; i++) s.invInertiaWorld[i] = BitsUtil.GetF32(b, 0x7c + 4 * i);
            s.tail0 = BitsUtil.GetF32(b, 0xa0);
            s.tail1 = BitsUtil.GetF32(b, 0xa4);
            s.tail2 = BitsUtil.GetF32(b, 0xa8);
            s.tail3 = BitsUtil.GetF32(b, 0xac);
            s.tail4 = BitsUtil.GetF32(b, 0xb0);
            return s;
        }
    }

    /// <summary>The 68-byte params block (mass, body inertia, drags, COM).</summary>
    [Serializable]
    public struct CHmsDynaParams
    {
        public float mass;                /* 0x00 */
        public GmMat3 invInertiaBody;     /* 0x04 */
        public float dragLinear;          /* 0x28 */
        public float dragAngular;         /* 0x2C */
        public float substepLen;          /* 0x30 */
        public float forceFieldScale;     /* 0x34 */
        public GmVec3 comOffset;          /* 0x38 */

        public byte[] ToBytes()
        {
            byte[] b = new byte[0x5c];
            BitsUtil.PutF32(b, 0x00, mass);
            for (int i = 0; i < 9; i++) BitsUtil.PutF32(b, 0x04 + 4 * i, invInertiaBody[i]);
            BitsUtil.PutF32(b, 0x28, dragLinear);
            BitsUtil.PutF32(b, 0x2c, dragAngular);
            BitsUtil.PutF32(b, 0x30, substepLen);
            BitsUtil.PutF32(b, 0x34, forceFieldScale);
            comOffset.ToBytes(b, 0x38);
            return b;
        }

        public static CHmsDynaParams FromBytes(byte[] b)
        {
            CHmsDynaParams p = default(CHmsDynaParams);
            p.mass = BitsUtil.GetF32(b, 0x00);
            for (int i = 0; i < 9; i++) p.invInertiaBody[i] = BitsUtil.GetF32(b, 0x04 + 4 * i);
            p.dragLinear = BitsUtil.GetF32(b, 0x28);
            p.dragAngular = BitsUtil.GetF32(b, 0x2c);
            p.substepLen = BitsUtil.GetF32(b, 0x30);
            p.forceFieldScale = BitsUtil.GetF32(b, 0x34);
            p.comOffset = GmVec3.FromBytes(b, 0x38);
            return p;
        }
    }

    /// <summary>
    /// CHmsDyna: the rigid-body integrator and state manager. Owns three
    /// state slots (live, stateB, temp) by value and the replacement buffer.
    /// </summary>
    [Serializable]
    public sealed class CHmsDyna
    {
        public int clampAngular;
        public float maxAngularSpeed;
        public CHmsDynaParams @params;
        public CHmsStateDyna tempState;
        private CHmsStateDyna stateB;
        private CHmsStateDyna liveState;
        public System.Collections.Generic.List<GmVec3> replacementBuf = new System.Collections.Generic.List<GmVec3>();
        public int dirtyFlag;
        public int mode;

        // magic constants (from bits):
        // DAT_00CDB690: the Rust oracle writes the decimal literal
        // 9.999999439624929e-11, which rounds to f32 bits 0x2EDBE6FE
        // (NOT 0x2EB7CDFC — that is a different number, 8.358e-11).
        private static readonly float Dat00Cdb690 = // 9.999999439624929e-11
            BitConverter.Int32BitsToSingle(unchecked((int)0x2EDB_E6FE));
        private static readonly float Dat00Cdb67c = // 0.009999999776482582
            BitConverter.Int32BitsToSingle(unchecked((int)0x3C23_D70A));

        public CHmsDyna(CHmsDynaParams parameters, int modeValue, int clampAngularValue, float maxAngularSpeedValue)
        {
            clampAngular = clampAngularValue;
            maxAngularSpeed = maxAngularSpeedValue;
            @params = parameters;
            tempState = CHmsStateDyna.Create();
            stateB = CHmsStateDyna.Create();
            liveState = CHmsStateDyna.Create();
            replacementBuf = new System.Collections.Generic.List<GmVec3>();
            dirtyFlag = 0;
            mode = modeValue;
        }

        public ref CHmsStateDyna Live() => ref liveState;

        public ref CHmsStateDyna StateB() => ref stateB;

        public void SetLive(CHmsStateDyna s) => liveState = s;

        public void SetStateB(CHmsStateDyna s) => stateB = s;

        private static float LengthSq(float x, float y, float z) => (float)(double)(((float)(double)(((float)(double)(y * y)) + ((float)(double)(x * x)))) + ((float)(double)(z * z)));

        /// <summary>0x00532D40: temp = live.</summary>
        public void CopyStateToTemp() => tempState = liveState;

        /// <summary>0x00532D60: stateB = temp.</summary>
        public void CopyTempToState() => stateB = tempState;

        /// <summary>0x00532D20: stateB = live.</summary>
        public void ValidateDynamicState() => stateB = liveState;

        /// <summary>0x00533EE0.</summary>
        public GmVec3 GetLinearSpeed() => liveState.linVel;

        /// <summary>0x00533F30.</summary>
        public GmVec3 GetAngularSpeed() => liveState.angVel;

        /// <summary>0x00533A30.</summary>
        public void SetForce(ref GmVec3 f) => liveState.force = f;

        /// <summary>0x00533A50.</summary>
        public void SetTorque(ref GmVec3 t) => liveState.torque = t;

        /// <summary>0x00533B80. Operand orders preserved.</summary>
        public void AddForce(ref GmVec3 force)
        {
            liveState.force.x = (float)(double)(liveState.force.x + force.x);
            liveState.force.y = (float)(double)(force.y + liveState.force.y);
            liveState.force.z = (float)(double)(force.z + liveState.force.z);
        }

        /// <summary>0x00533BB0.</summary>
        public GmVec3 GetForce() => liveState.force;

        /// <summary>0x00533BE0. Operand orders preserved.</summary>
        public void AddTorque(ref GmVec3 torque)
        {
            liveState.torque.x = (float)(double)(liveState.torque.x + torque.x);
            liveState.torque.y = (float)(double)(torque.y + liveState.torque.y);
            liveState.torque.z = (float)(double)(torque.z + liveState.torque.z);
        }

        /// <summary>0x00533EC0.</summary>
        public void SetLinearSpeed(ref GmVec3 speed) => liveState.linVel = speed;

        /// <summary>0x00533F10.</summary>
        public void SetAngularSpeed(ref GmVec3 speed) => liveState.angVel = speed;

        /// <summary>0x00533A70: world force + moment about world COM.</summary>
        public void AddForceAt(ref GmVec3 force, ref GmVec3 worldPoint)
        {
            AddForce(ref force);

            GmVec3 worldCom = GmVec3.Zero;
            GmVec3 comOffset = @params.comOffset;
            GmIso4 liveIso = liveState.RotPosIso();
            worldCom.SetMultIso4(ref comOffset, ref liveIso);
            GmVec3 lever = new GmVec3(
                (float)(double)(worldPoint.x - worldCom.x),
                (float)(double)(worldPoint.y - worldCom.y),
                (float)(double)(worldPoint.z - worldCom.z));
            GmVec3 torque = new GmVec3(
                (float)(double)(((float)(double)(lever.y * force.z)) - ((float)(double)(lever.z * force.y))),
                (float)(double)(((float)(double)(force.x * lever.z)) - ((float)(double)(lever.x * force.z))),
                (float)(double)(((float)(double)(force.y * lever.x)) - ((float)(double)(lever.y * force.x))));
            AddTorque(ref torque);
        }

        private GmVec3 LocalToWorldVector(ref GmVec3 local)
        {
            GmVec3 world = GmVec3.Zero;
            GmMat3 rot = liveState.rot;
            world.SetMultMat3(ref local, ref rot);
            return world;
        }

        /// <summary>0x00533F60.</summary>
        public void AddLocalForceAt(ref GmVec3 force, ref GmVec3 localPoint)
        {
            GmVec3 worldForce = LocalToWorldVector(ref force);
            GmVec3 worldPoint = GmVec3.Zero;
            GmIso4 liveIso = liveState.RotPosIso();
            worldPoint.SetMultIso4(ref localPoint, ref liveIso);
            AddForceAt(ref worldForce, ref worldPoint);
        }

        /// <summary>0x00534030.</summary>
        public void SetLocalForce(ref GmVec3 force)
        {
            GmVec3 world = LocalToWorldVector(ref force);
            SetForce(ref world);
        }

        /// <summary>0x005340B0.</summary>
        public void AddLocalForce(ref GmVec3 force)
        {
            GmVec3 world = LocalToWorldVector(ref force);
            AddForce(ref world);
        }

        /// <summary>0x00534120.</summary>
        public GmVec3 GetLocalForce()
        {
            GmVec3 force = GetForce();
            GmMat3 rot = liveState.rot;
            force.MultTranspose(ref rot);
            return force;
        }

        /// <summary>0x00534140.</summary>
        public void SetLocalTorque(ref GmVec3 torque)
        {
            GmVec3 world = LocalToWorldVector(ref torque);
            SetTorque(ref world);
        }

        /// <summary>0x005341C0.</summary>
        public void AddLocalTorque(ref GmVec3 torque)
        {
            GmVec3 world = LocalToWorldVector(ref torque);
            AddTorque(ref world);
        }

        /// <summary>0x00534300.</summary>
        public void AddLocalImpulse(ref GmVec3 impulse)
        {
            GmVec3 world = LocalToWorldVector(ref impulse);
            AddImpulse(ref world);
        }

        /// <summary>0x00534370.</summary>
        public void SetLocalLinearSpeed(ref GmVec3 speed)
        {
            GmVec3 world = LocalToWorldVector(ref speed);
            SetLinearSpeed(ref world);
        }

        /// <summary>0x005343E0.</summary>
        public GmVec3 GetLocalLinearSpeed()
        {
            GmVec3 speed = GetLinearSpeed();
            GmMat3 rot = liveState.rot;
            speed.MultTranspose(ref rot);
            return speed;
        }

        /// <summary>0x00534400.</summary>
        public void SetLocalAngularSpeed(ref GmVec3 speed)
        {
            GmVec3 world = LocalToWorldVector(ref speed);
            SetAngularSpeed(ref world);
        }

        /// <summary>0x00534470.</summary>
        public GmVec3 GetLocalAngularSpeed()
        {
            GmVec3 speed = GetAngularSpeed();
            GmMat3 rot = liveState.rot;
            speed.MultTranspose(ref rot);
            return speed;
        }

        /// <summary>0x005334E0: live.pos += d. Operand orders preserved.</summary>
        public void ApplyReplacement(ref GmVec3 d)
        {
            liveState.pos.x = (float)(double)(liveState.pos.x + d.x);
            liveState.pos.y = (float)(double)(d.y + liveState.pos.y);
            liveState.pos.z = (float)(double)(d.z + liveState.pos.z);
        }

        /// <summary>
        /// 0x00533DD0: velocity at a world point. The angular term is a
        /// LOAD-BEARING f64 single-rounding expression: the whole term is
        /// computed in double and rounded once at the store.
        /// </summary>
        public GmVec3 GetSpeed(ref GmVec3 at)
        {
            if (mode == 2)
            {
                return GmVec3.Zero;
            }
            GmVec3 result = liveState.linVel;
            if (mode == 1)
            {
                GmVec3 comWorld = GmVec3.Zero;
                GmVec3 comOffset = @params.comOffset;
                GmIso4 liveIso = liveState.RotPosIso();
                comWorld.SetMultIso4(ref comOffset, ref liveIso);
                double rx = (double)at.x - (double)comWorld.x;
                double ry = (double)at.y - (double)comWorld.y;
                double rz = (double)at.z - (double)comWorld.z;
                result.x = (float)((double)result.x
                    + ((double)liveState.angVel.y * rz - ry * (double)liveState.angVel.z));
                result.y = (float)((double)result.y
                    + ((double)liveState.angVel.z * rx - rz * (double)liveState.angVel.x));
                result.z = (float)((double)result.z
                    + ((double)liveState.angVel.x * ry - rx * (double)liveState.angVel.y));
            }
            return result;
        }

        /// <summary>0x00533D50: central linear impulse.</summary>
        public void AddImpulse(ref GmVec3 j)
        {
            if (mode == 2)
            {
                return;
            }
            if (dirtyFlag == 0)
            {
                dirtyFlag = 1;
            }
            float invMass = (float)(double)(1.0f / @params.mass);
            liveState.linVel.x = (float)(double)(((float)(double)(invMass * j.x)) + liveState.linVel.x);
            liveState.linVel.y = (float)(double)(liveState.linVel.y + ((float)(double)(j.y * invMass)));
            liveState.linVel.z = (float)(double)(((float)(double)(invMass * j.z)) + liveState.linVel.z);
        }

        /// <summary>0x00533C10: impulse at a world point (linear + angular).</summary>
        public void AddImpulseAt(ref GmVec3 j, ref GmVec3 at)
        {
            if (mode == 2)
            {
                return;
            }
            if (dirtyFlag == 0)
            {
                dirtyFlag = 1;
            }
            float invMass = (float)(double)(1.0f / @params.mass);
            liveState.linVel.x = (float)(double)(liveState.linVel.x + ((float)(double)(invMass * j.x)));
            liveState.linVel.y = (float)(double)(liveState.linVel.y + ((float)(double)(j.y * invMass)));
            liveState.linVel.z = (float)(double)(((float)(double)(invMass * j.z)) + liveState.linVel.z);
            if (mode == 1)
            {
                GmVec3 comWorld = GmVec3.Zero;
                GmVec3 comOffset = @params.comOffset;
                GmIso4 liveIso = liveState.RotPosIso();
                comWorld.SetMultIso4(ref comOffset, ref liveIso);
                float rx = (float)(double)(at.x - comWorld.x);
                float ry = (float)(double)(at.y - comWorld.y);
                float rz = (float)(double)(at.z - comWorld.z);
                GmVec3 tau = new GmVec3(
                    (float)(double)(((float)(double)(ry * j.z)) - ((float)(double)(rz * j.y))),
                    (float)(double)(((float)(double)(rz * j.x)) - ((float)(double)(rx * j.z))),
                    (float)(double)(((float)(double)(rx * j.y)) - ((float)(double)(j.x * ry))));
                GmMat3 invInertiaWorld = liveState.invInertiaWorld;
                tau.MultMat3(ref invInertiaWorld);
                // Single-op F() adds: identical to f32 addition.
                liveState.angVel.x = (float)(double)(liveState.angVel.x + tau.x);
                liveState.angVel.y = (float)(double)(liveState.angVel.y + tau.y);
                liveState.angVel.z = (float)(double)(tau.z + liveState.angVel.z);
            }
        }

        /// <summary>0x00533510: one integration substep.</summary>
        public void IntegrateStep(ref CHmsStateDyna input, ref CHmsStateDyna output, float dt)
        {
            if (mode == 2)
            {
                output = input;
                return;
            }

            float invMass = (float)(double)(1.0f / @params.mass);
            float forceX = (float)(double)(input.force.x * invMass);
            float forceY = (float)(double)(input.force.y * invMass);
            float forceZ = (float)(double)(invMass * input.force.z);
            float posDx = (float)(double)(input.linVel.x * dt);
            float posDy = (float)(double)(input.linVel.y * dt);
            float posDz = (float)(double)(input.linVel.z * dt);

            output.pos.x = (float)(double)(posDx + input.pos.x);
            output.pos.y = (float)(double)(input.pos.y + posDy);
            output.pos.z = (float)(double)(input.pos.z + posDz);
            float addedDx = (float)(double)(input.linVelAdded.x * dt);
            float addedDy = (float)(double)(input.linVelAdded.y * dt);
            float addedDz = (float)(double)(input.linVelAdded.z * dt);
            output.pos.x = (float)(double)(output.pos.x + addedDx);
            output.pos.y = (float)(double)(output.pos.y + addedDy);
            output.pos.z = (float)(double)(output.pos.z + addedDz);
            output.linVelAdded.z = 0.0f;
            output.linVelAdded.y = 0.0f;
            output.linVelAdded.x = 0.0f;
            output.linVel.x = (float)(double)(input.linVel.x + ((float)(double)(forceX * dt)));
            output.linVel.y = (float)(double)(input.linVel.y + ((float)(double)(forceY * dt)));
            output.linVel.z = (float)(double)(input.linVel.z + ((float)(double)(dt * forceZ)));

            if (mode != 0)
            {
                // Iinv_world * torque
                GmVec3 angAcc = GmVec3.Zero;
                GmVec3 torque = input.torque;
                GmMat3 invInertiaWorld = input.invInertiaWorld;
                angAcc.SetMultMat3(ref torque, ref invInertiaWorld);

                float wx = input.angVel.x, wy = input.angVel.y, wz = input.angVel.z;
                if (LengthSq(wx, wy, wz) <= Dat00Cdb690)
                {
                    output.rot = input.rot;
                    output.quat = input.quat;
                }
                else
                {
                    // Quaternion bytes are (w,x,y,z) scalar-first in fields
                    // named (x,y,z,w): field .x = w, .y = x, .z = y, .w = z.
                    float qx = input.quat.x; // w
                    float qy = input.quat.y; // x
                    float qz = input.quat.z; // y
                    float qw = input.quat.w; // z
                    float dqx = (float)(double)(((float)(double)(((float)(double)(((float)(double)(-wx * qy)) - ((float)(double)(wy * qz)))) - ((float)(double)(qw * wz)))) * 0.5f);
                    float dqy = (float)(double)(((float)(double)(((float)(double)(((float)(double)(qx * wx)) + ((float)(double)(wy * qw)))) - ((float)(double)(qz * wz)))) * 0.5f);
                    float dqz = (float)(double)(((float)(double)(((float)(double)(((float)(double)(wy * qx)) - ((float)(double)(qw * wx)))) + ((float)(double)(wz * qy)))) * 0.5f);
                    float dqw = (float)(double)(((float)(double)(((float)(double)(((float)(double)(qz * wx)) - ((float)(double)(wy * qy)))) + ((float)(double)(qx * wz)))) * 0.5f);
                    output.quat = input.quat;
                    output.quat.x = (float)(double)(((float)(double)(dqx * dt)) + output.quat.x);
                    output.quat.y = (float)(double)(((float)(double)(dqy * dt)) + output.quat.y);
                    output.quat.z = (float)(double)(((float)(double)(dqz * dt)) + output.quat.z);
                    output.quat.w = (float)(double)(((float)(double)(dt * dqw)) + output.quat.w);
                    output.quat.Normalize();
                    // SetFromQuat takes (w,x,y,z): pass the byte order.
                    GmQuat oq = output.quat;
                    output.rot.SetFromQuat(oq.x, oq.y, oq.z, oq.w);
                    GmVec3 comOld = GmVec3.Zero;
                    GmVec3 comOffset = @params.comOffset;
                    GmMat3 inputRot = input.rot;
                    comOld.SetMultMat3(ref comOffset, ref inputRot);
                    GmVec3 comNew = GmVec3.Zero;
                    GmMat3 outputRot = output.rot;
                    comNew.SetMultMat3(ref comOffset, ref outputRot);
                    float comDx = (float)(double)(comNew.x - comOld.x);
                    float comDy = (float)(double)(comNew.y - comOld.y);
                    float comDz = (float)(double)(comNew.z - comOld.z);
                    output.pos.x = (float)(double)(output.pos.x - comDx);
                    output.pos.y = (float)(double)(output.pos.y - comDy);
                    output.pos.z = (float)(double)(output.pos.z - comDz);
                }
                output.angVel.x = (float)(double)(input.angVel.x + ((float)(double)(dt * angAcc.x)));
                output.angVel.y = (float)(double)(input.angVel.y + ((float)(double)(angAcc.y * dt)));
                output.angVel.z = (float)(double)(input.angVel.z + ((float)(double)(dt * angAcc.z)));

                if (clampAngular != 0)
                {
                    float mx = maxAngularSpeed;
                    float nsq = LengthSq(output.angVel.x, output.angVel.y, output.angVel.z);
                    if (((float)(double)(mx * mx)) < nsq)
                    {
                        float s = Fp.Sqrt(nsq);
                        float ratio = (float)(double)(mx / s);
                        output.angVel.x = (float)(double)(output.angVel.x * ratio);
                        output.angVel.y = (float)(double)(ratio * output.angVel.y);
                        output.angVel.z = (float)(double)(ratio * output.angVel.z);
                    }
                }

                GmMat3 rotT = default(GmMat3);
                GmMat3 outputRot2 = output.rot;
                rotT.SetTranspose(ref outputRot2);
                output.invInertiaWorld = rotT;
                // GmMat3_Mult(self, B): self = self * B.
                GmMat3 iw = default(GmMat3);
                GmMat3 oi = output.invInertiaWorld;
                GmMat3 invInertiaBody = @params.invInertiaBody;
                iw.SetMult(ref oi, ref invInertiaBody);
                GmMat3 outputRot3 = output.rot;
                iw.Mult(ref outputRot3);
                output.invInertiaWorld = iw;
                return;
            }

            output.rot = input.rot;
        }

        /// <summary>0x00535D00: integrate live one step, clear replacements.</summary>
        public void DoPreCollisionDynamic(float dt)
        {
            CHmsStateDyna local = liveState;
            // `out` starts as a copy of live: unwritten fields (force,
            // torque, tail) keep live's values, exactly like the aliasing.
            CHmsStateDyna output = liveState;
            IntegrateStep(ref local, ref output, dt);
            liveState = output;
            replacementBuf.Clear();
            if (DbgHook.Enabled)
            {
                CHmsStateDyna s = liveState;
                Console.Error.WriteLine(
                    $"PRE dt={BitConverter.SingleToInt32Bits(dt):X8} " +
                    $"pos=({BitConverter.SingleToInt32Bits(s.pos.x):X8},{BitConverter.SingleToInt32Bits(s.pos.y):X8},{BitConverter.SingleToInt32Bits(s.pos.z):X8}) " +
                    $"quat=({BitConverter.SingleToInt32Bits(s.quat.x):X8},{BitConverter.SingleToInt32Bits(s.quat.y):X8},{BitConverter.SingleToInt32Bits(s.quat.z):X8},{BitConverter.SingleToInt32Bits(s.quat.w):X8}) " +
                    $"lv=({BitConverter.SingleToInt32Bits(s.linVel.x):X8},{BitConverter.SingleToInt32Bits(s.linVel.y):X8},{BitConverter.SingleToInt32Bits(s.linVel.z):X8}) " +
                    $"av=({BitConverter.SingleToInt32Bits(s.angVel.x):X8},{BitConverter.SingleToInt32Bits(s.angVel.y):X8},{BitConverter.SingleToInt32Bits(s.angVel.z):X8})");
            }
        }

        /// <summary>0x00535FC0: append a penetration-correction vector.</summary>
        public void AddReplacement(ref GmVec3 d)
        {
            if (DbgHook.Enabled)
            {
                Console.Error.WriteLine(
                    $"ADDR d=({BitConverter.SingleToInt32Bits(d.x):X8},{BitConverter.SingleToInt32Bits(d.y):X8},{BitConverter.SingleToInt32Bits(d.z):X8})");
            }
            if (dirtyFlag == 0)
            {
                dirtyFlag = 1;
            }
            replacementBuf.Add(d);
        }

        /// <summary>0x00535D50: reduce replacements into one clamped vector.</summary>
        public GmVec3 ComputeSynthetizedReplacement()
        {
            var b = replacementBuf;
            if (b.Count == 0)
            {
                return GmVec3.Zero;
            }
            float ax = b[0].x, ay = b[0].y, az = b[0].z;
            for (int i = 1; i < b.Count; i++)
            {
                float px = b[i].x, py = b[i].y, pz = b[i].z;
                float dot = (float)(double)(((float)(double)(((float)(double)(ax * px)) + ((float)(double)(ay * py)))) + ((float)(double)(az * pz)));
                float mag = LengthSq(ax, ay, az);
                if (dot > 0.0f && mag > Dat00Cdb690)
                {
                    if (mag < dot)
                    {
                        dot = mag;
                    }
                    dot = (float)(double)(dot / mag);
                    ax = (float)(double)(ax - ((float)(double)(dot * ax)));
                    ay = (float)(double)(ay - ((float)(double)(dot * ay)));
                    az = (float)(double)(az - ((float)(double)(dot * az)));
                }
                ax = (float)(double)(px + ax);
                ay = (float)(double)(ay + py);
                az = (float)(double)(az + pz);
            }
            float mag2 = LengthSq(ax, ay, az);
            if (mag2 <= (float)(double)(Dat00Cdb67c * Dat00Cdb67c))
            {
                return GmVec3.Zero;
            }
            float s = Fp.Sqrt(mag2);
            float inv = (float)(double)(1.0f / s);
            float nx = (float)(double)(ax * inv);
            float ny = (float)(double)(ay * inv);
            float nz = (float)(double)(az * inv);
            return new GmVec3(
                (float)(double)(ax - ((float)(double)(Dat00Cdb67c * nx))),
                (float)(double)(ay - ((float)(double)(Dat00Cdb67c * ny))),
                (float)(double)(az - ((float)(double)(Dat00Cdb67c * nz))));
        }

        /// <summary>0x00536400: apply the synthesized replacement.</summary>
        public void DoPostCollisionDynamic()
        {
            GmVec3 repl = ComputeSynthetizedReplacement();
            if (DbgHook.Enabled)
            {
                Console.Error.WriteLine(
                    $"REPL pos=({BitConverter.SingleToInt32Bits(liveState.pos.x):X8},{BitConverter.SingleToInt32Bits(liveState.pos.y):X8},{BitConverter.SingleToInt32Bits(liveState.pos.z):X8}) " +
                    $"repl=({BitConverter.SingleToInt32Bits(repl.x):X8},{BitConverter.SingleToInt32Bits(repl.y):X8},{BitConverter.SingleToInt32Bits(repl.z):X8}) " +
                    $"n={replacementBuf.Count}");
            }
            ApplyReplacement(ref repl);
        }
    }
}
