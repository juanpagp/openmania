using System;

namespace Tmnf.Sim
{
    /// <summary>The 8-byte material data pair (friction, restitution).</summary>
    [Serializable]
    public struct CPlugSurfaceMaterialData
    {
        public float friction;
        public float restitution;
    }

    /// <summary>The response material (category / mode / sides).</summary>
    [Serializable]
    public struct CHmsResponseMaterial
    {
        public uint category;
        public uint responseMode;
        public uint[] sideEnabled;

        public static CHmsResponseMaterial Create()
        {
            CHmsResponseMaterial m = default(CHmsResponseMaterial);
            m.sideEnabled = new uint[] { 0, 0 };
            return m;
        }
    }

    /// <summary>What a body's contact sink dispatches to.</summary>
    public enum AbsorbSink
    {
        None = 0,
        VehicleBody = 1,
        VehicleWheel = 2,
    }

    /// <summary>A response body (dyna is an index into the world arena or null).</summary>
    [Serializable]
    public struct CHmsResponseBody
    {
        public uint corpusRef;
        public uint classificationFlags;
        public uint responseFlags;
        public float responseWeight;
        public GmIso4 iso;
        public int? dyna;             // index into the world's dyna arena
        public bool hasContactSink;
        public AbsorbSink sink;

        public uint LocalContactMode() => (classificationFlags >> 17) & 3;

        public uint Rank() => (classificationFlags >> 11) & 3;

        public uint Category() => (classificationFlags >> 13) & 0xf;
    }

    /// <summary>One absorbed contact (corpus refs instead of pointers).</summary>
    [Serializable]
    public struct CHmsPhysicalContact
    {
        public uint bodyCorpusRef;
        public uint treeRef;
        public ushort surfaceMaterial;
        public GmVec3 normal;
        public GmVec3 position;
        public GmVec3 relativeSpeed;
        public GmVec3 replacement;
        public uint accepted;
        public uint? otherBodyCorpusRef;
        public uint otherTreeRef;
        public ushort otherSurfaceMaterial;

        public static CHmsPhysicalContact Create()
        {
            CHmsPhysicalContact c = default(CHmsPhysicalContact);
            c.normal = GmVec3.Zero;
            c.position = GmVec3.Zero;
            c.relativeSpeed = GmVec3.Zero;
            c.replacement = GmVec3.Zero;
            return c;
        }
    }

    /// <summary>
    /// The seam the world implements (the C's resolver function pointers) —
    /// the C# port uses an interface with ref-friendly mutators.
    /// </summary>
    public interface IResponseCtx
    {
        /// <summary>Resolves a corpus ref to its body (throws if unknown).</summary>
        CHmsResponseBody Body(uint corpusRef);

        /// <summary>The body's world iso, by value.</summary>
        GmIso4 BodyIso(uint corpusRef);

        /// <summary>0x00533DD0 inlined with its PC=24 schedule (per-op f32).</summary>
        GmVec3 GetSpeed(uint corpusRef, ref GmVec3 position);

        /// <summary>AddReplacement routed to the body's dyna (no-op static).</summary>
        void AddReplacement(uint corpusRef, ref GmVec3 d);

        /// <summary>SolveBodyImpulse routed to the body's dyna (no-op static).</summary>
        void SolveBodyImpulse(uint corpusRef, ref GmCollision collision, ref GmVec3 speed, float frictionProduct, float restitution);

        /// <summary>The body's contact sink (no-op when absent).</summary>
        void AbsorbContact(uint corpusRef, ref CHmsPhysicalContact contact);

        /// <summary>The response materials table.</summary>
        CHmsResponseMaterial ResponseMaterial(uint materialRef);

        /// <summary>The 31-entry surface material data.</summary>
        CPlugSurfaceMaterialData[] SurfaceMaterials();

        /// <summary>The collision buffer being responded to (sorted in place).</summary>
        CFastBufferShmsPhysicalCollision Collisions();
    }

    /// <summary>
    /// 0x00533DD0, inlined with its PC=24 operation schedule (per-op f32 —
    /// deliberately different from CHmsDyna.GetSpeed's f64 intermediates).
    /// </summary>
    public static class ResponseGetSpeed
    {
        public static GmVec3 Compute(ref CHmsResponseBody body, CHmsDyna dyna, ref GmVec3 position)
        {
            if (dyna == null)
            {
                return GmVec3.Zero;
            }
            if (dyna.mode == 2)
            {
                return GmVec3.Zero;
            }
            ref CHmsStateDyna state = ref dyna.Live();
            GmVec3 speed = state.linVel;
            if (dyna.mode == 1)
            {
                GmVec3 centerOfMass = GmVec3.Zero;
                GmVec3 comOffset = dyna.@params.comOffset;
                GmIso4 liveIso = state.RotPosIso();
                centerOfMass.SetMultIso4(ref comOffset, ref liveIso);
                float radiusX = (float)(double)(position.x - centerOfMass.x);
                float radiusY = (float)(double)(position.y - centerOfMass.y);
                float radiusZ = (float)(double)(position.z - centerOfMass.z);
                float crossX = (float)(double)(((float)(double)(radiusZ * state.angVel.y)) - ((float)(double)(radiusY * state.angVel.z)));
                float crossY = (float)(double)(((float)(double)(radiusX * state.angVel.z)) - ((float)(double)(radiusZ * state.angVel.x)));
                float crossZ = (float)(double)(((float)(double)(radiusY * state.angVel.x)) - ((float)(double)(radiusX * state.angVel.y)));
                speed.x = (float)(double)(speed.x + crossX);
                speed.y = (float)(double)(speed.y + crossY);
                speed.z = (float)(double)(speed.z + crossZ);
            }
            return speed;
        }
    }

    /// <summary>The impulse solver + contact sink dispatch (collision_response.c).</summary>
    public static class Response
    {
        private const float ResponseEpsilon = 1.0e-5f;
        private const uint CentralImpulseFlag = 0x1000;

        private static float LengthSquared(ref GmVec3 v)
        {
            return (float)(double)(((float)(double)(((float)(double)(v.x * v.x)) + ((float)(double)(v.y * v.y)))) + ((float)(double)(v.z * v.z)));
        }

        private static float DotCollisionNormal(ref GmVec3 normal, ref GmVec3 v)
        {
            return (float)(double)(((float)(double)(normal.z * v.z)) + ((float)(double)(((float)(double)(normal.x * v.x)) + ((float)(double)(normal.y * v.y)))));
        }

        private static GmVec3 Cross(ref GmVec3 a, ref GmVec3 b)
        {
            return new GmVec3(
                (float)(double)(((float)(double)(a.y * b.z)) - ((float)(double)(a.z * b.y))),
                (float)(double)(((float)(double)(a.z * b.x)) - ((float)(double)(a.x * b.z))),
                (float)(double)(((float)(double)(a.x * b.y)) - ((float)(double)(a.y * b.x))));
        }

        /// <summary>0x0087D9A0: combines two material restitution coefficients.</summary>
        public static float RestitutionCoefWith(ref CPlugSurfaceMaterialData a, ref CPlugSurfaceMaterialData b)
        {
            if (a.restitution <= 0.0f)
            {
                if (0.0f < b.restitution)
                {
                    return a.restitution;
                }
                return (float)(double)(b.restitution + a.restitution);
            }
            if (0.0f < b.restitution)
            {
                return (float)(double)(b.restitution * a.restitution);
            }
            return b.restitution;
        }

        /// <summary>make_contact: builds a contact for the body when enabled.</summary>
        public static CHmsPhysicalContact? MakeContact(
            ref CHmsResponseBody body,
            ref GmIso4 bodyIso,
            uint treeRef,
            ushort surfaceMaterialIndex,
            ref CHmsResponseBody otherBody,
            uint otherTreeRef,
            ushort otherSurfaceMaterial,
            ref GmCollision collision,
            uint sideEnabled)
        {
            if (sideEnabled == 0 || !body.hasContactSink)
            {
                return null;
            }
            CHmsPhysicalContact contact = CHmsPhysicalContact.Create();
            contact.bodyCorpusRef = body.corpusRef;
            contact.treeRef = treeRef;
            contact.surfaceMaterial = surfaceMaterialIndex;
            contact.otherBodyCorpusRef = otherBody.corpusRef;
            contact.otherTreeRef = otherTreeRef;
            contact.otherSurfaceMaterial = otherSurfaceMaterial;
            if (body.LocalContactMode() > 1)
            {
                GmVec3 normal = collision.normal;
                GmMat3 rot = default(GmMat3);
                rot.m0 = bodyIso.m0; rot.m1 = bodyIso.m1; rot.m2 = bodyIso.m2;
                rot.m3 = bodyIso.m3; rot.m4 = bodyIso.m4; rot.m5 = bodyIso.m5;
                rot.m6 = bodyIso.m6; rot.m7 = bodyIso.m7; rot.m8 = bodyIso.m8;
                normal.MultTranspose(ref rot);
                contact.normal = normal;
                GmVec3 position = new GmVec3(
                    (float)(double)(collision.position.x - bodyIso.t0),
                    (float)(double)(collision.position.y - bodyIso.t1),
                    (float)(double)(collision.position.z - bodyIso.t2));
                position.MultTranspose(ref rot);
                contact.position = position;
            }
            return contact;
        }

        /// <summary>solve_body_impulse (transcribed). Mutates the dyna.</summary>
        public static void SolveBodyImpulse(
            ref CHmsResponseBody body,
            CHmsDyna dyna,
            ref GmCollision collision,
            ref GmVec3 speed,
            float frictionProduct,
            float restitution)
        {
            if (dyna == null)
            {
                return;
            }

            float normalSpeed = DotCollisionNormal(ref collision.normal, ref speed);
            GmVec3 projection = new GmVec3(
                (float)(double)(collision.normal.x * normalSpeed),
                (float)(double)(collision.normal.y * normalSpeed),
                (float)(double)(normalSpeed * collision.normal.z));
            GmVec3 tangent = new GmVec3(
                (float)(double)(speed.x - projection.x),
                (float)(double)(speed.y - projection.y),
                (float)(double)(speed.z - projection.z));

            float projectionLength = Fp.Sqrt(LengthSquared(ref projection));
            float tangentLength = Fp.Sqrt(LengthSquared(ref tangent));
            float tangentLimit = (float)(double)(projectionLength * frictionProduct);
            if (tangentLimit < tangentLength)
            {
                float scale = (float)(double)(tangentLimit / tangentLength);
                tangent.x = (float)(double)(scale * tangent.x);
                tangent.y = (float)(double)(tangent.y * scale);
                tangent.z = (float)(double)(scale * tangent.z);
            }

            GmVec3 impulseDirection = new GmVec3(
                -((float)(double)(tangent.x + projection.x)),
                -((float)(double)(tangent.y + projection.y)),
                -((float)(double)(tangent.z + projection.z)));
            float speedLength = Fp.Sqrt(LengthSquared(ref impulseDirection));
            if (!(ResponseEpsilon < speedLength))
            {
                return;
            }

            float inverseSpeed = (float)(double)(1.0f / speedLength);
            impulseDirection.x = (float)(double)(inverseSpeed * impulseDirection.x);
            impulseDirection.y = (float)(double)(impulseDirection.y * inverseSpeed);
            impulseDirection.z = (float)(double)(impulseDirection.z * inverseSpeed);

            float numerator = (float)(double)(((float)(double)(restitution + 1.0f)) * speedLength);
            float denominator = (float)(double)(1.0f / dyna.@params.mass);

            if (dyna.mode == 1 && (body.responseFlags & CentralImpulseFlag) == 0)
            {
                ref CHmsStateDyna state = ref dyna.Live();
                GmVec3 centerOfMass = GmVec3.Zero;
                GmVec3 comOffset = dyna.@params.comOffset;
                GmIso4 liveIso = state.RotPosIso();
                centerOfMass.SetMultIso4(ref comOffset, ref liveIso);
                GmVec3 radius = new GmVec3(
                    (float)(double)(collision.position.x - centerOfMass.x),
                    (float)(double)(collision.position.y - centerOfMass.y),
                    (float)(double)(collision.position.z - centerOfMass.z));
                GmVec3 radiusCrossNormal = Cross(ref radius, ref impulseDirection);
                GmVec3 inertiaCross = GmVec3.Zero;
                GmMat3 invInertiaWorld = state.invInertiaWorld;
                inertiaCross.SetMultMat3(ref radiusCrossNormal, ref invInertiaWorld);
                GmVec3 angular = Cross(ref inertiaCross, ref radius);
                float rotational = (float)(double)(((float)(double)(((float)(double)(angular.x * impulseDirection.x))
                    + ((float)(double)(angular.y * impulseDirection.y))))
                    + ((float)(double)(angular.z * impulseDirection.z)));
                denominator = (float)(double)(rotational + denominator);
            }

            float scale2 = (float)(double)(numerator / denominator);
            GmVec3 impulse = new GmVec3(
                (float)(double)(impulseDirection.x * scale2),
                (float)(double)(impulseDirection.y * scale2),
                (float)(double)(scale2 * impulseDirection.z));
            if ((body.responseFlags & CentralImpulseFlag) == 0)
            {
                GmVec3 collisionPosition = collision.position;
                dyna.AddImpulseAt(ref impulse, ref collisionPosition);
            }
            else
            {
                dyna.AddImpulse(ref impulse);
            }
        }

        /// <summary>compute_replacements (transcribed).</summary>
        public static (GmVec3, GmVec3) ComputeReplacements(
            ref CHmsResponseBody body1,
            ref CHmsResponseBody body2,
            ref GmVec3 normal)
        {
            uint rank1 = body1.Rank();
            uint rank2 = body2.Rank();
            if (rank2 < rank1)
            {
                return (new GmVec3(-normal.x, -normal.y, -normal.z), GmVec3.Zero);
            }
            if (rank1 < rank2)
            {
                return (GmVec3.Zero, normal);
            }

            float inverseTotal = (float)(double)(1.0f / ((float)(double)(body2.responseWeight + body1.responseWeight)));
            float scale1 = (float)(double)(-body2.responseWeight * inverseTotal);
            GmVec3 replacement1 = new GmVec3(
                (float)(double)(normal.x * scale1),
                (float)(double)(scale1 * normal.y),
                (float)(double)(scale1 * normal.z));
            float scale2 = (float)(double)(body1.responseWeight * inverseTotal);
            GmVec3 replacement2 = new GmVec3(
                (float)(double)(normal.x * scale2),
                (float)(double)(scale2 * normal.y),
                (float)(double)(scale2 * normal.z));
            return (replacement1, replacement2);
        }

        /// <summary>prepare_solve_contact (transcribed): returns the world replacement.</summary>
        public static GmVec3 PrepareSolveContact(
            IResponseCtx ctx,
            ref CHmsPhysicalContact contact,
            ref GmVec3 replacement,
            ref GmVec3 relativeSpeed,
            bool firstBody)
        {
            contact.accepted = 1;
            GmIso4 bodyIso = ctx.BodyIso(contact.bodyCorpusRef);
            GmMat3 rotation = default(GmMat3);
            rotation.m0 = bodyIso.m0; rotation.m1 = bodyIso.m1; rotation.m2 = bodyIso.m2;
            rotation.m3 = bodyIso.m3; rotation.m4 = bodyIso.m4; rotation.m5 = bodyIso.m5;
            rotation.m6 = bodyIso.m6; rotation.m7 = bodyIso.m7; rotation.m8 = bodyIso.m8;
            GmVec3 contactReplacement = replacement;
            contactReplacement.MultTranspose(ref rotation);
            contact.replacement = contactReplacement;
            if (firstBody)
            {
                contact.relativeSpeed = new GmVec3(
                    -relativeSpeed.x, -relativeSpeed.y, -relativeSpeed.z);
            }
            else
            {
                contact.relativeSpeed = relativeSpeed;
            }
            contact.relativeSpeed.MultTranspose(ref rotation);
            ctx.AbsorbContact(contact.bodyCorpusRef, ref contact);
            contact.relativeSpeed.MultMat3(ref rotation);
            if (firstBody)
            {
                relativeSpeed.x = -contact.relativeSpeed.x;
                relativeSpeed.y = -contact.relativeSpeed.y;
                relativeSpeed.z = -contact.relativeSpeed.z;
            }
            GmVec3 replacementWorld = GmVec3.Zero;
            replacementWorld.SetMultMat3(ref contact.replacement, ref rotation);
            return replacementWorld;
        }

        /// <summary>
        /// 0x00548BF0: replacement, contact callbacks, friction, impulse. The
        /// record is mutated in place (GmCollision_Neg before body2's
        /// impulse) exactly as the original does inside the buffer.
        /// </summary>
        public static void SolveImpulse(
            IResponseCtx ctx,
            ref SHmsPhysicalCollision physical,
            ref CHmsPhysicalContact? contact1,
            ref CHmsPhysicalContact? contact2)
        {
            CHmsResponseBody body1 = ctx.Body(physical.corpus1);
            CHmsResponseBody body2 = ctx.Body(physical.corpus2);
            CPlugSurfaceMaterialData[] surfaceMaterials = ctx.SurfaceMaterials();
            CPlugSurfaceMaterialData surface1 = surfaceMaterials[physical.collision.material1];
            CPlugSurfaceMaterialData surface2 = surfaceMaterials[physical.collision.material2];
            float restitution = RestitutionCoefWith(ref surface1, ref surface2);

            var (replacement1, replacement2) =
                ComputeReplacements(ref body1, ref body2, ref physical.collision.separation);

            GmVec3 speed1 = ctx.GetSpeed(physical.corpus1, ref physical.collision.position);
            GmVec3 speed2 = ctx.GetSpeed(physical.corpus2, ref physical.collision.position);
            GmVec3 relativeSpeed = new GmVec3(
                (float)(double)(speed2.x - speed1.x),
                (float)(double)(speed2.y - speed1.y),
                (float)(double)(speed2.z - speed1.z));

            GmVec3 replacement1World = replacement1;
            if (contact1.HasValue)
            {
                CHmsPhysicalContact c1 = contact1.Value;
                replacement1World = PrepareSolveContact(
                    ctx, ref c1, ref replacement1, ref relativeSpeed, true);
                contact1 = c1;
            }
            ctx.AddReplacement(physical.corpus1, ref replacement1World);
            GmVec3 replacement2World = replacement2;
            if (contact2.HasValue)
            {
                CHmsPhysicalContact c2 = contact2.Value;
                replacement2World = PrepareSolveContact(
                    ctx, ref c2, ref replacement2, ref relativeSpeed, false);
                contact2 = c2;
            }
            ctx.AddReplacement(physical.corpus2, ref replacement2World);

            if ((contact1.HasValue && contact1.Value.accepted == 0)
                || (contact2.HasValue && contact2.Value.accepted == 0))
            {
                return;
            }

            float frictionProduct = (float)(double)(surface1.friction * surface2.friction);
            ctx.SolveBodyImpulse(physical.corpus1, ref physical.collision, ref speed1,
                frictionProduct, restitution);
            // GmCollision_Neg in place, as the original does.
            Narrowphase.GmCollisionNeg(ref physical.collision);
            ctx.SolveBodyImpulse(physical.corpus2, ref physical.collision, ref speed2,
                frictionProduct, restitution);
        }

        /// <summary>fill_external_contact_speed (transcribed).</summary>
        public static void FillExternalContactSpeed(
            IResponseCtx ctx,
            ref CHmsPhysicalContact contact,
            ref GmVec3 relativeSpeed,
            bool firstBody)
        {
            CHmsResponseBody body = ctx.Body(contact.bodyCorpusRef);
            if (body.LocalContactMode() > 1)
            {
                if (firstBody)
                {
                    contact.relativeSpeed = new GmVec3(
                        -relativeSpeed.x, -relativeSpeed.y, -relativeSpeed.z);
                }
                else
                {
                    contact.relativeSpeed = relativeSpeed;
                }
                GmIso4 bodyIso = ctx.BodyIso(contact.bodyCorpusRef);
                GmMat3 rot = default(GmMat3);
                rot.m0 = bodyIso.m0; rot.m1 = bodyIso.m1; rot.m2 = bodyIso.m2;
                rot.m3 = bodyIso.m3; rot.m4 = bodyIso.m4; rot.m5 = bodyIso.m5;
                rot.m6 = bodyIso.m6; rot.m7 = bodyIso.m7; rot.m8 = bodyIso.m8;
                contact.relativeSpeed.MultTranspose(ref rot);
            }
            ctx.AbsorbContact(contact.bodyCorpusRef, ref contact);
        }

        /// <summary>0x005497C0: sorts and dispatches all detected collisions.</summary>
        public static void ComputeCollisionResponse(IResponseCtx ctx)
        {
            ctx.Collisions().Qsort(BufferCompare.ShmsPhysicalCollisionCompare);

            int count = (int)ctx.Collisions().Count();
            if (DbgHook.Enabled)
            {
                for (int i = 0; i < count; i++)
                {
                    SHmsPhysicalCollision c = ctx.Collisions().At(i);
                    Console.Error.WriteLine(
                        $"COL {i} c1={c.corpus1} t1={c.tree1} c2={c.corpus2} t2={c.tree2} mat={c.material} " +
                        $"pos=({BitConverter.SingleToInt32Bits(c.collision.position.x):X8},{BitConverter.SingleToInt32Bits(c.collision.position.y):X8},{BitConverter.SingleToInt32Bits(c.collision.position.z):X8}) " +
                        $"nrm=({BitConverter.SingleToInt32Bits(c.collision.normal.x):X8},{BitConverter.SingleToInt32Bits(c.collision.normal.y):X8},{BitConverter.SingleToInt32Bits(c.collision.normal.z):X8}) " +
                        $"sep=({BitConverter.SingleToInt32Bits(c.collision.separation.x):X8},{BitConverter.SingleToInt32Bits(c.collision.separation.y):X8},{BitConverter.SingleToInt32Bits(c.collision.separation.z):X8}) " +
                        $"m1={c.collision.material1} m2={c.collision.material2}");
                }
            }
            for (int i = 0; i < count; i++)
            {
                SHmsPhysicalCollision physical = ctx.Collisions().At(i);
                CHmsResponseBody body1 = ctx.Body(physical.corpus1);
                CHmsResponseBody body2 = ctx.Body(physical.corpus2);
                CHmsResponseMaterial material = ctx.ResponseMaterial(physical.material);

                uint side1 = (body1.Category() != material.category) ? 1u : 0u;
                GmIso4 iso1 = ctx.BodyIso(physical.corpus1);
                GmIso4 iso2 = ctx.BodyIso(physical.corpus2);
                CHmsPhysicalContact? contact1 = MakeContact(
                    ref body1, ref iso1,
                    physical.tree1,
                    physical.collision.material1,
                    ref body2,
                    physical.tree2,
                    physical.collision.material2,
                    ref physical.collision,
                    material.sideEnabled[side1]);
                CHmsPhysicalContact? contact2 = MakeContact(
                    ref body2, ref iso2,
                    physical.tree2,
                    physical.collision.material2,
                    ref body1,
                    // NB: the original passes collision->tree2 here as well —
                    // a quirk of the transcription, preserved verbatim.
                    physical.tree2,
                    physical.collision.material1,
                    ref physical.collision,
                    material.sideEnabled[1 - side1]);

                if (material.responseMode != 0)
                {
                    SolveImpulse(ctx, ref physical, ref contact1, ref contact2);
                    ctx.Collisions().At(i) = physical;
                    continue;
                }

                GmVec3 speed1 = ctx.GetSpeed(physical.corpus1, ref physical.collision.position);
                GmVec3 speed2 = ctx.GetSpeed(physical.corpus2, ref physical.collision.position);
                GmVec3 relativeSpeed = new GmVec3(
                    (float)(double)(speed2.x - speed1.x),
                    (float)(double)(speed2.y - speed1.y),
                    (float)(double)(speed2.z - speed1.z));
                if (contact1.HasValue)
                {
                    CHmsPhysicalContact c1 = contact1.Value;
                    FillExternalContactSpeed(ctx, ref c1, ref relativeSpeed, true);
                }
                if (contact2.HasValue)
                {
                    CHmsPhysicalContact c2 = contact2.Value;
                    FillExternalContactSpeed(ctx, ref c2, ref relativeSpeed, false);
                }
            }
        }
    }
}
