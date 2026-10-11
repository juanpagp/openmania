using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>The TMNFROU1 image loader + the projection BVH.</summary>
    public static class RouteLoader
    {
        public const uint TmnfRouteVersion = 3;
        public const int TmnfRouteSectionCount = 5;

        /// <summary>Section indices (module constants).</summary>
        public const int SectionMetadata = 0;
        public const int SectionStart = 1;
        public const int SectionCheckpoints = 2;
        public const int SectionFinish = 3;
        public const int SectionReference = 4;

        public const uint TmnfRouteWaypointStart = 0;
        public const uint TmnfRouteWaypointFinish = 1;
        public const uint TmnfRouteWaypointCheckpoint = 2;
        public const uint TmnfRouteWaypointNone = 3;
        public const uint TmnfRouteWaypointStartFinish = 4;

        /// <summary>TMNF_ROUTE_MULTILAP (1u &lt;&lt; 0).</summary>
        public const uint TmnfRouteMultilap = 1u << 0;

        /// <summary>0x10 bytes.</summary>
        [Serializable]
        public struct TmnfRouteSection
        {
            public ulong offset;
            public uint count;
            public uint stride;
        }

        /// <summary>0xE0 bytes.</summary>
        [Serializable]
        public sealed class TmnfRouteHeader
        {
            public byte[] magic = new byte[8]; // "TMNFROU1"
            public uint version;
            public uint endian;       // 0x12345678
            public uint headerSize;   // 0xE0
            public uint sectionCount; // 5
            public ulong fileSize;
            public byte[] exeSha256 = new byte[32];
            public byte[] trackSha256 = new byte[32];
            public TmnfRouteSection[] sections = new TmnfRouteSection[TmnfRouteSectionCount];
            public byte[] payloadSha256 = new byte[32];
            public byte[] reserved = new byte[16];
        }

        /// <summary>0x40 bytes.</summary>
        [Serializable]
        public sealed class TmnfRouteMetadata
        {
            public uint lapCount;
            public uint checkpointCount;
            public uint finishCount;
            public uint referenceCount;
            public uint totalRaceCheckpoints;
            public uint raceCheckpointLimit;
            public uint flags;
            public uint centerlineCount;
            public ulong startRel;
            public ulong checkpointsRel;
            public ulong finishRel;
            public ulong referenceRel;
        }

        /// <summary>0xAC bytes: the pointer-free CHmsStateDyna prefix.</summary>
        [Serializable]
        public struct TmnfRouteInitialState
        {
            public GmQuat quat;
            public GmMat3 rot;
            public GmVec3 pos;
            public GmVec3 linVel;
            public GmVec3 linVelAdded;
            public GmVec3 angVel;
            public GmVec3 force;
            public GmVec3 torque;
            public GmMat3 invInertiaWorld;
            public GmVec3 notTweakedLinVel;
        }

        /// <summary>0x114 bytes.</summary>
        [Serializable]
        public sealed class TmnfRouteStart
        {
            public GmIso4 transform;
            public TmnfRouteInitialState initialState;
            public uint blockIndex;
            public uint waypointType;
            public GmIso4 spawn;
        }

        /// <summary>0x90 bytes.</summary>
        [Serializable]
        public sealed class TmnfRouteTrigger
        {
            public uint raceIndex;
            public uint blockIndex;
            public uint waypointType;
            public uint treeFlags;
            public GmBoxAligned boxAligned;
            public GmIso4 transform;
            public GmIso4 spawn;
            public uint noRespawn;
            public uint reserved;
        }

        /// <summary>0x18 bytes.</summary>
        [Serializable]
        public struct TmnfRouteReferencePoint
        {
            public GmVec3 position;
            public float arcLength;
            public float halfWidth;
            public uint legIndex;
        }

        /// <summary>TmnfRouteProjection.</summary>
        [Serializable]
        public struct TmnfRouteProjection
        {
            public float arcLength;
            public float lateralOffset;
            public float halfWidth;
            public uint segmentIndex;
            public uint centerlineSegmentIndex;
            public uint segmentsTested;
        }

        /// <summary>36 bytes: projection BVH node.</summary>
        [Serializable]
        public struct TmnfRouteBvhNode
        {
            public float[] minimum;
            public float[] maximum;
            public uint left;
            public uint right;
            public uint segment;
        }

        /// <summary>The loaded route snapshot.</summary>
        [Serializable]
        public sealed class TmnfRoute
        {
            public TmnfRouteHeader header = new TmnfRouteHeader();
            public TmnfRouteMetadata metadata = new TmnfRouteMetadata();
            public TmnfRouteStart start = new TmnfRouteStart();
            public List<TmnfRouteTrigger> checkpoints = new List<TmnfRouteTrigger>();
            public List<TmnfRouteTrigger> finish = new List<TmnfRouteTrigger>();
            public List<TmnfRouteReferencePoint> centerline = new List<TmnfRouteReferencePoint>();
            public List<TmnfRouteBvhNode> projectionBvh = new List<TmnfRouteBvhNode>();
            public uint projectionBvhCount;
            public uint projectionBvhHeight;
        }

        // ------------------------------------------------------------------
        // parsing
        // ------------------------------------------------------------------

        private static TmnfRouteHeader ParseRouteHeader(byte[] data)
        {
            Reader r = new Reader(data);
            TmnfRouteHeader h = new TmnfRouteHeader();
            h.magic = r.Bytes(8);
            h.version = r.U32();
            h.endian = r.U32();
            h.headerSize = r.U32();
            h.sectionCount = r.U32();
            h.fileSize = r.U64();
            h.exeSha256 = r.Bytes(32);
            h.trackSha256 = r.Bytes(32);
            for (int i = 0; i < TmnfRouteSectionCount; i++)
            {
                h.sections[i].offset = r.U64();
                h.sections[i].count = r.U32();
                h.sections[i].stride = r.U32();
            }
            h.payloadSha256 = r.Bytes(32);
            h.reserved = r.Bytes(16);
            return h;
        }

        private static GmQuat ReadQuat(Reader r)
        {
            GmQuat q = default(GmQuat);
            q.x = r.F32();
            q.y = r.F32();
            q.z = r.F32();
            q.w = r.F32();
            return q;
        }

        private static TmnfRouteMetadata ParseRouteMetadata(Reader r)
        {
            TmnfRouteMetadata m = new TmnfRouteMetadata();
            m.lapCount = r.U32();
            m.checkpointCount = r.U32();
            m.finishCount = r.U32();
            m.referenceCount = r.U32();
            m.totalRaceCheckpoints = r.U32();
            m.raceCheckpointLimit = r.U32();
            m.flags = r.U32();
            m.centerlineCount = r.U32();
            m.startRel = r.U64();
            m.checkpointsRel = r.U64();
            m.finishRel = r.U64();
            m.referenceRel = r.U64();
            return m;
        }

        private static TmnfRouteInitialState ParseRouteInitialState(Reader r)
        {
            TmnfRouteInitialState s = default(TmnfRouteInitialState);
            s.quat = ReadQuat(r);
            for (int i = 0; i < 9; i++) s.rot[i] = r.F32();
            s.pos = r.GmVec3();
            s.linVel = r.GmVec3();
            s.linVelAdded = r.GmVec3();
            s.angVel = r.GmVec3();
            s.force = r.GmVec3();
            s.torque = r.GmVec3();
            for (int i = 0; i < 9; i++) s.invInertiaWorld[i] = r.F32();
            s.notTweakedLinVel = r.GmVec3();
            return s;
        }

        private static TmnfRouteStart ParseRouteStart(Reader r)
        {
            TmnfRouteStart s = new TmnfRouteStart();
            s.transform = r.GmIso4();
            s.initialState = ParseRouteInitialState(r);
            s.blockIndex = r.U32();
            s.waypointType = r.U32();
            s.spawn = r.GmIso4();
            return s;
        }

        private static TmnfRouteTrigger ParseRouteTrigger(Reader r)
        {
            TmnfRouteTrigger t = new TmnfRouteTrigger();
            t.raceIndex = r.U32();
            t.blockIndex = r.U32();
            t.waypointType = r.U32();
            t.treeFlags = r.U32();
            GmVec3 center = r.GmVec3();
            GmVec3 half = r.GmVec3();
            t.boxAligned = default(GmBoxAligned);
            t.boxAligned.center = center;
            t.boxAligned.halfExtent = half;
            t.transform = r.GmIso4();
            t.spawn = r.GmIso4();
            t.noRespawn = r.U32();
            t.reserved = r.U32();
            return t;
        }

        // ------------------------------------------------------------------
        // validation
        // ------------------------------------------------------------------

        private static ulong SectionBytes(ref TmnfRouteSection section)
        {
            ulong bytes = section.count * section.stride;
            if (section.stride != 0 && bytes / section.stride != section.count)
            {
                throw new InvalidOperationException("tmnf route: section size overflow");
            }
            return bytes;
        }

        private static void ValidateSections(TmnfRouteHeader header, int mappingSize)
        {
            uint[] expectedStride = { 0x40, 0x114, 0x90, 0x90, 0x18 };
            for (int i = 0; i < TmnfRouteSectionCount; i++)
            {
                TmnfRouteSection section = header.sections[i];
                if (section.stride != expectedStride[i]
                    || (section.count == 0 && i != SectionCheckpoints)
                    || (section.offset & 7) != 0
                    || section.offset < 0xE0)
                {
                    throw new InvalidOperationException("tmnf route: invalid section descriptor");
                }
                ulong bytes = SectionBytes(ref section);
                if (section.offset > (ulong)mappingSize
                    || bytes > (ulong)mappingSize - section.offset)
                {
                    throw new InvalidOperationException("tmnf route: section outside file");
                }
            }
            for (int i = 0; i < TmnfRouteSectionCount; i++)
            {
                ulong a0 = header.sections[i].offset;
                ulong a1 = a0 + SectionBytes(ref header.sections[i]);
                for (int j = i + 1; j < TmnfRouteSectionCount; j++)
                {
                    ulong b0 = header.sections[j].offset;
                    ulong b1 = b0 + SectionBytes(ref header.sections[j]);
                    if (a0 < b1 && b0 < a1)
                    {
                        throw new InvalidOperationException("tmnf route: overlapping sections");
                    }
                }
            }
        }

        private static bool FiniteFloats(float[] values)
        {
            foreach (float v in values)
            {
                if (!float.IsFinite(v))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool FiniteVec3(ref GmVec3 v) => FiniteFloats(new[] { v.x, v.y, v.z });

        private static bool FiniteIso4(ref GmIso4 iso)
        {
            float[] all = new float[12];
            for (int i = 0; i < 9; i++) all[i] = iso[i];
            all[9] = iso.t0; all[10] = iso.t1; all[11] = iso.t2;
            return FiniteFloats(all);
        }

        private static bool FiniteBox(ref GmBoxAligned box)
        {
            return FiniteVec3(ref box.center) && FiniteVec3(ref box.halfExtent);
        }

        private static bool FiniteInitialState(ref TmnfRouteInitialState s)
        {
            return FiniteFloats(new[] { s.quat.x, s.quat.y, s.quat.z, s.quat.w })
                && FiniteFloats(new[]
                {
                    s.rot.m0, s.rot.m1, s.rot.m2, s.rot.m3, s.rot.m4, s.rot.m5,
                    s.rot.m6, s.rot.m7, s.rot.m8,
                })
                && FiniteVec3(ref s.pos)
                && FiniteVec3(ref s.linVel)
                && FiniteVec3(ref s.linVelAdded)
                && FiniteVec3(ref s.angVel)
                && FiniteVec3(ref s.force)
                && FiniteVec3(ref s.torque)
                && FiniteFloats(new[]
                {
                    s.invInertiaWorld.m0, s.invInertiaWorld.m1, s.invInertiaWorld.m2,
                    s.invInertiaWorld.m3, s.invInertiaWorld.m4, s.invInertiaWorld.m5,
                    s.invInertiaWorld.m6, s.invInertiaWorld.m7, s.invInertiaWorld.m8,
                })
                && FiniteVec3(ref s.notTweakedLinVel);
        }

        private static bool F32BitsEqual(float[] a, float[] b)
        {
            for (int i = 0; i < a.Length && i < b.Length; i++)
            {
                if (BitConverter.SingleToInt32Bits(a[i]) != BitConverter.SingleToInt32Bits(b[i]))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool ValidSpawn(ref GmIso4 spawn)
        {
            if (!FiniteIso4(ref spawn))
            {
                return false;
            }
            for (int row = 0; row < 3; row++)
            {
                float r0 = spawn[row * 3 + 0];
                float r1 = spawn[row * 3 + 1];
                float r2 = spawn[row * 3 + 2];
                float length = (float)(double)((float)(double)(((float)(double)(r0 * r0)) + ((float)(double)(r1 * r1))) + ((float)(double)(r2 * r2)));
                if (Math.Abs((double)((float)(double)(length - 1.0f))) > 1e-3)
                {
                    return false;
                }
            }
            return true;
        }

        private static void ValidateTrigger(TmnfRouteTrigger trigger, uint index, uint waypointType)
        {
            if (trigger.raceIndex != index
                || trigger.waypointType != waypointType
                || trigger.treeFlags == 0
                || !FiniteBox(ref trigger.boxAligned)
                || !FiniteIso4(ref trigger.transform)
                || !(trigger.boxAligned.halfExtent.x > 0.0f)
                || !(trigger.boxAligned.halfExtent.y > 0.0f)
                || !(trigger.boxAligned.halfExtent.z > 0.0f)
                || !ValidSpawn(ref trigger.spawn)
                || trigger.noRespawn > 1
                || trigger.reserved != 0)
            {
                throw new InvalidOperationException("tmnf route: invalid route trigger");
            }
        }

        // ------------------------------------------------------------------
        // projection BVH
        // ------------------------------------------------------------------

        private static uint BuildProjectionBvh(
            List<TmnfRouteReferencePoint> centerline,
            List<TmnfRouteBvhNode> bvh,
            ref uint nodeCount,
            ref uint height,
            uint first,
            uint count,
            uint depth)
        {
            if (count == 0 || depth >= 64)
            {
                throw new InvalidOperationException("tmnf route: reference line exceeds BVH depth bound");
            }
            uint nodeIndex = nodeCount;
            nodeCount += 1;
            {
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                node.minimum = new[] { float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity };
                node.maximum = new[] { float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity };
                bvh[(int)nodeIndex] = node;
            }
            for (uint segment = first; segment < first + count; segment++)
            {
                GmVec3 a = centerline[(int)segment].position;
                GmVec3 b = centerline[(int)(segment + 1)].position;
                float[] pa = { a.x, a.y, a.z };
                float[] pb = { b.x, b.y, b.z };
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                for (int axis = 0; axis < 3; axis++)
                {
                    node.minimum[axis] = Fp.RustMin(node.minimum[axis], Fp.RustMin(pa[axis], pb[axis]));
                    node.maximum[axis] = Fp.RustMax(node.maximum[axis], Fp.RustMax(pa[axis], pb[axis]));
                }
                bvh[(int)nodeIndex] = node;
            }
            if (depth + 1 > height)
            {
                height = depth + 1;
            }
            if (count == 1)
            {
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                node.left = uint.MaxValue;
                node.right = uint.MaxValue;
                node.segment = first;
                bvh[(int)nodeIndex] = node;
                return nodeIndex;
            }
            uint leftCount = count / 2;
            {
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                node.segment = uint.MaxValue;
                bvh[(int)nodeIndex] = node;
            }
            uint left = BuildProjectionBvh(centerline, bvh, ref nodeCount, ref height, first, leftCount, depth + 1);
            {
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                node.left = left;
                bvh[(int)nodeIndex] = node;
            }
            uint right = BuildProjectionBvh(centerline, bvh, ref nodeCount, ref height, first + leftCount, count - leftCount, depth + 1);
            {
                TmnfRouteBvhNode node = bvh[(int)nodeIndex];
                node.right = right;
                bvh[(int)nodeIndex] = node;
            }
            return nodeIndex;
        }

        private static (List<TmnfRouteBvhNode>, uint, uint) InitializeProjectionBvh(
            TmnfRouteMetadata metadata,
            List<TmnfRouteReferencePoint> centerline)
        {
            uint segmentCount = metadata.centerlineCount - 1;
            if (segmentCount > (uint.MaxValue - 1) / 2)
            {
                throw new InvalidOperationException("tmnf route: reference line is too large for its BVH");
            }
            uint nodeCount = segmentCount * 2 - 1;
            var bvh = new List<TmnfRouteBvhNode>((int)nodeCount);
            for (uint i = 0; i < nodeCount; i++)
            {
                TmnfRouteBvhNode n = default(TmnfRouteBvhNode);
                n.minimum = new float[3];
                n.maximum = new float[3];
                bvh.Add(n);
            }
            uint count = 0;
            uint height = 0;
            if (BuildProjectionBvh(centerline, bvh, ref count, ref height, 0, segmentCount, 0)
                != 0
                || count != nodeCount)
            {
                throw new InvalidOperationException("tmnf route: projection BVH construction failed");
            }
            return (bvh, count, height);
        }

        // ------------------------------------------------------------------
        // loader
        // ------------------------------------------------------------------

        /// <summary>TmnfRoute_Load: reads and fully validates a TMNFROU1 snapshot.</summary>
        public static TmnfRoute TmnfRouteLoad(string path, byte[] expectedTrackSha256)
        {
            byte[] data;
            try
            {
                data = System.IO.File.ReadAllBytes(path);
            }
            catch (Exception)
            {
                throw new InvalidOperationException("tmnf route: cannot open snapshot");
            }
            return TmnfRouteLoadBytes(data, expectedTrackSha256);
        }

        /// <summary>
        /// TmnfRoute_Load over an in-memory image — the same validation and
        /// projection pipeline as the path entry point, for callers that
        /// build the bytes themselves (see Tmnf.Sim.Fixtures.TmnfFixtures).
        /// </summary>
        public static TmnfRoute TmnfRouteLoadBytes(byte[] data, byte[] expectedTrackSha256)
        {
            if (data.Length < 0xE0)
            {
                throw new InvalidOperationException("tmnf route: invalid snapshot size");
            }
            int size = data.Length;

            TmnfRouteHeader header = ParseRouteHeader(data);
            if (!IsMagic(header.magic, "TMNFROU1")
                || header.version != TmnfRouteVersion
                || header.endian != 0x1234_5678
                || header.headerSize != 0xE0
                || header.sectionCount != TmnfRouteSectionCount
                || header.fileSize != (ulong)size)
            {
                throw new InvalidOperationException("tmnf route: invalid snapshot header");
            }
            if (!BytesEqual(header.exeSha256, TrackLoader.Tmnf21126ExeSha256))
            {
                throw new InvalidOperationException("tmnf route: snapshot targets a different executable");
            }
            if (expectedTrackSha256 != null && !BytesEqual(header.trackSha256, expectedTrackSha256))
            {
                throw new InvalidOperationException("tmnf route: snapshot is for a different track");
            }
            ValidateSections(header, size);

            byte[] payload = new byte[size - 0xE0];
            Buffer.BlockCopy(data, 0xE0, payload, 0, payload.Length);
            if (!BytesEqual(Sha256.Digest(payload), header.payloadSha256))
            {
                throw new InvalidOperationException("tmnf route: payload SHA-256 mismatch");
            }

            // fix_and_validate
            Reader mr = Reader.At(data, (int)header.sections[SectionMetadata].offset);
            TmnfRouteMetadata metadata = ParseRouteMetadata(mr);
            if (header.sections[SectionMetadata].count != 1
                || header.sections[SectionStart].count != 1
                || header.sections[SectionFinish].count != metadata.finishCount
                || metadata.lapCount == 0
                || metadata.finishCount == 0
                || (ulong)metadata.finishCount + metadata.checkpointCount > 64
                || metadata.checkpointCount != header.sections[SectionCheckpoints].count
                || metadata.centerlineCount != header.sections[SectionReference].count
                || metadata.referenceCount != metadata.checkpointCount + 2
                || metadata.centerlineCount < metadata.referenceCount
                || (metadata.flags & ~TmnfRouteMultilap) != 0
                || ((metadata.flags & TmnfRouteMultilap) != 0) != (metadata.lapCount > 1))
            {
                throw new InvalidOperationException("tmnf route: invalid route metadata");
            }

            ulong startRel = ResolveArray(
                ref header.sections[SectionStart], metadata.startRel, 1);
            ulong checkpointsRel = ResolveArray(
                ref header.sections[SectionCheckpoints], metadata.checkpointsRel, metadata.checkpointCount);
            ulong finishRel = ResolveArray(
                ref header.sections[SectionFinish], metadata.finishRel, metadata.finishCount);
            ulong referenceRel = ResolveArray(
                ref header.sections[SectionReference], metadata.referenceRel, metadata.centerlineCount);

            Reader sr = Reader.At(data, (int)startRel);
            TmnfRouteStart start = ParseRouteStart(sr);

            var checkpoints = new List<TmnfRouteTrigger>((int)metadata.checkpointCount);
            {
                Reader r = Reader.At(data, (int)checkpointsRel);
                for (uint i = 0; i < metadata.checkpointCount; i++)
                {
                    checkpoints.Add(ParseRouteTrigger(r));
                }
            }
            var finish = new List<TmnfRouteTrigger>((int)metadata.finishCount);
            {
                Reader r = Reader.At(data, (int)finishRel);
                for (uint i = 0; i < metadata.finishCount; i++)
                {
                    finish.Add(ParseRouteTrigger(r));
                }
            }
            var centerline = new List<TmnfRouteReferencePoint>((int)metadata.centerlineCount);
            {
                Reader r = Reader.At(data, (int)referenceRel);
                for (uint i = 0; i < metadata.centerlineCount; i++)
                {
                    TmnfRouteReferencePoint p = default(TmnfRouteReferencePoint);
                    p.position = r.GmVec3();
                    p.arcLength = r.F32();
                    p.halfWidth = r.F32();
                    p.legIndex = r.U32();
                    centerline.Add(p);
                }
            }

            if ((start.waypointType != TmnfRouteWaypointStart
                && start.waypointType != TmnfRouteWaypointStartFinish)
                || !FiniteIso4(ref start.transform)
                || !FiniteInitialState(ref start.initialState)
                || !F32BitsEqual(IsoMatrix(ref start.transform), MatToArray(ref start.initialState.rot))
                || !F32BitsEqual(IsoTranslation(ref start.transform),
                    new[] { start.initialState.pos.x, start.initialState.pos.y, start.initialState.pos.z })
                || !ValidSpawn(ref start.spawn))
            {
                throw new InvalidOperationException("tmnf route: invalid route start");
            }
            for (int i = 0; i < (int)metadata.checkpointCount; i++)
            {
                ValidateTrigger(checkpoints[i], (uint)i, TmnfRouteWaypointCheckpoint);
            }
            for (int i = 0; i < (int)metadata.finishCount; i++)
            {
                if (finish[i].waypointType == TmnfRouteWaypointFinish)
                {
                    ValidateTrigger(finish[i], (uint)i, TmnfRouteWaypointFinish);
                    if (start.waypointType != TmnfRouteWaypointStart)
                    {
                        throw new InvalidOperationException("tmnf route: dedicated finish has a shared start");
                    }
                }
                else
                {
                    ValidateTrigger(finish[i], (uint)i, TmnfRouteWaypointStartFinish);
                    if (metadata.finishCount != 1
                        || start.waypointType != TmnfRouteWaypointStartFinish
                        || start.blockIndex != finish[i].blockIndex
                        || metadata.lapCount <= 1)
                    {
                        throw new InvalidOperationException("tmnf route: invalid shared start/finish");
                    }
                }
            }
            if (centerline[0].arcLength != 0.0f
                || centerline[0].legIndex != 0
                || !F32BitsEqual(
                    new[] { centerline[0].position.x, centerline[0].position.y, centerline[0].position.z },
                    IsoTranslation(ref start.transform)))
            {
                throw new InvalidOperationException("tmnf route: invalid reference-line origin");
            }
            for (int i = 0; i < (int)metadata.centerlineCount; i++)
            {
                TmnfRouteReferencePoint point = centerline[i];
                if (!FiniteVec3(ref point.position)
                    || !float.IsFinite(point.arcLength)
                    || !float.IsFinite(point.halfWidth)
                    || !(point.halfWidth > 0.0f)
                    || point.legIndex > metadata.checkpointCount
                    || (i != 0
                        && (point.legIndex < centerline[i - 1].legIndex
                            || point.legIndex > centerline[i - 1].legIndex + 1))
                    || (i != 0 && !(point.arcLength > centerline[i - 1].arcLength)))
                {
                    throw new InvalidOperationException("tmnf route: invalid reference line");
                }
                if (i != 0)
                {
                    float dx = (float)(double)(point.position.x - centerline[i - 1].position.x);
                    float dy = (float)(double)(point.position.y - centerline[i - 1].position.y);
                    float dz = (float)(double)(point.position.z - centerline[i - 1].position.z);
                    float length = Fp.Sqrt((float)(double)((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy))) + ((float)(double)(dz * dz))));
                    float arcDelta = (float)(double)(point.arcLength - centerline[i - 1].arcLength);
                    float tolerance = Fp.RustMax(0.001f, (float)(double)(length * 0.0001f));
                    if (!(length > 0.0f) || Math.Abs((double)((float)(double)(arcDelta - length))) > (double)tolerance)
                    {
                        throw new InvalidOperationException("tmnf route: reference arc length does not match geometry");
                    }
                }
            }
            if (centerline[(int)metadata.centerlineCount - 1].legIndex != metadata.checkpointCount)
            {
                throw new InvalidOperationException("tmnf route: reference line does not contain every route leg");
            }

            var (projectionBvh, projectionBvhCount, projectionBvhHeight) =
                InitializeProjectionBvh(metadata, centerline);

            TmnfRoute route = new TmnfRoute();
            route.header = header;
            route.metadata = metadata;
            route.start = start;
            route.checkpoints = checkpoints;
            route.finish = finish;
            route.centerline = centerline;
            route.projectionBvh = projectionBvh;
            route.projectionBvhCount = projectionBvhCount;
            route.projectionBvhHeight = projectionBvhHeight;
            return route;
        }

        private static float[] IsoMatrix(ref GmIso4 iso)
        {
            return new[] { iso.m0, iso.m1, iso.m2, iso.m3, iso.m4, iso.m5, iso.m6, iso.m7, iso.m8 };
        }

        private static float[] MatToArray(ref GmMat3 m)
        {
            return new[] { m.m0, m.m1, m.m2, m.m3, m.m4, m.m5, m.m6, m.m7, m.m8 };
        }

        private static float[] IsoTranslation(ref GmIso4 iso)
        {
            return new[] { iso.t0, iso.t1, iso.t2 };
        }

        private static ulong ResolveArray(ref TmnfRouteSection section, ulong relative, uint count)
        {
            ulong bytes = count * section.stride;
            ulong sectionEnd = section.offset + SectionBytes(ref section);
            if (relative < section.offset
                || relative > sectionEnd
                || bytes > sectionEnd - relative
                || (relative - section.offset) % section.stride != 0)
            {
                throw new InvalidOperationException("tmnf route: relative pointer outside target section");
            }
            return relative;
        }

        private static bool IsMagic(byte[] magic, string expected)
        {
            for (int i = 0; i < 8; i++)
            {
                if (magic[i] != (byte)expected[i])
                {
                    return false;
                }
            }
            return true;
        }

        private static bool BytesEqual(byte[] a, byte[] b)
        {
            if (a.Length != b.Length)
            {
                return false;
            }
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i])
                {
                    return false;
                }
            }
            return true;
        }
    }

    /// <summary>Route getters + projection.</summary>
    public static class RouteQueries
    {
        /// <summary>TmnfRoute_Project: nearest finite 3D polyline segment.</summary>
        public static RouteLoader.TmnfRouteProjection Project(RouteLoader.TmnfRoute route, ref GmVec3 worldPosition)
        {
            if (!float.IsFinite(worldPosition.x)
                || !float.IsFinite(worldPosition.y)
                || !float.IsFinite(worldPosition.z))
            {
                throw new InvalidOperationException("tmnf route: cannot project a non-finite position");
            }
            float bestDistanceSq = float.PositiveInfinity;
            RouteLoader.TmnfRouteProjection result = default(RouteLoader.TmnfRouteProjection);
            result.segmentIndex = uint.MaxValue;
            result.centerlineSegmentIndex = uint.MaxValue;
            uint[] stack = new uint[64];
            int stackSize = 1;
            stack[0] = 0;
            float[] position = { worldPosition.x, worldPosition.y, worldPosition.z };
            while (stackSize != 0)
            {
                stackSize -= 1;
                RouteLoader.TmnfRouteBvhNode node = route.projectionBvh[(int)stack[stackSize]];
                float nodeDistanceSq = 0.0f;
                for (int axis = 0; axis < 3; axis++)
                {
                    float offset = 0.0f;
                    if (position[axis] < node.minimum[axis])
                    {
                        offset = (float)(double)(node.minimum[axis] - position[axis]);
                    }
                    else if (position[axis] > node.maximum[axis])
                    {
                        offset = (float)(double)(position[axis] - node.maximum[axis]);
                    }
                    nodeDistanceSq += (float)(double)(offset * offset);
                }
                if (nodeDistanceSq > bestDistanceSq)
                {
                    continue;
                }
                if (node.segment == uint.MaxValue)
                {
                    RouteLoader.TmnfRouteBvhNode left = route.projectionBvh[(int)node.left];
                    RouteLoader.TmnfRouteBvhNode right = route.projectionBvh[(int)node.right];
                    float leftDistanceSq = 0.0f;
                    float rightDistanceSq = 0.0f;
                    for (int axis = 0; axis < 3; axis++)
                    {
                        float leftOffset = 0.0f;
                        float rightOffset = 0.0f;
                        if (position[axis] < left.minimum[axis])
                        {
                            leftOffset = (float)(double)(left.minimum[axis] - position[axis]);
                        }
                        else if (position[axis] > left.maximum[axis])
                        {
                            leftOffset = (float)(double)(position[axis] - left.maximum[axis]);
                        }
                        if (position[axis] < right.minimum[axis])
                        {
                            rightOffset = (float)(double)(right.minimum[axis] - position[axis]);
                        }
                        else if (position[axis] > right.maximum[axis])
                        {
                            rightOffset = (float)(double)(position[axis] - right.maximum[axis]);
                        }
                        leftDistanceSq += (float)(double)(leftOffset * leftOffset);
                        rightDistanceSq += (float)(double)(rightOffset * rightOffset);
                    }
                    if (stackSize + 2 > 64)
                    {
                        throw new InvalidOperationException("tmnf route: projection BVH traversal stack overflow");
                    }
                    if (leftDistanceSq <= rightDistanceSq)
                    {
                        stack[stackSize] = node.right;
                        stackSize += 1;
                        stack[stackSize] = node.left;
                        stackSize += 1;
                    }
                    else
                    {
                        stack[stackSize] = node.left;
                        stackSize += 1;
                        stack[stackSize] = node.right;
                        stackSize += 1;
                    }
                    continue;
                }
                uint i = node.segment;
                RouteLoader.TmnfRouteReferencePoint a = route.centerline[(int)i];
                RouteLoader.TmnfRouteReferencePoint b = route.centerline[(int)(i + 1)];
                float dx = (float)(double)(b.position.x - a.position.x);
                float dy = (float)(double)(b.position.y - a.position.y);
                float dz = (float)(double)(b.position.z - a.position.z);
                float px = (float)(double)(worldPosition.x - a.position.x);
                float py = (float)(double)(worldPosition.y - a.position.y);
                float pz = (float)(double)(worldPosition.z - a.position.z);
                float lengthSq = (float)(double)((float)(double)(((float)(double)(dx * dx)) + ((float)(double)(dy * dy))) + ((float)(double)(dz * dz)));
                float t = (float)(double)(((float)(double)((float)(double)((float)(double)(px * dx) + (float)(double)(py * dy)) + (float)(double)(pz * dz))) / lengthSq);
                if (t < 0.0f)
                {
                    t = 0.0f;
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                }
                float ox = (float)(double)(px - (float)(double)(t * dx));
                float oy = (float)(double)(py - (float)(double)(t * dy));
                float oz = (float)(double)(pz - (float)(double)(t * dz));
                float distanceSq = (float)(double)((float)(double)(((float)(double)(ox * ox)) + ((float)(double)(oy * oy))) + ((float)(double)(oz * oz)));
                result.segmentsTested += 1;
                if (distanceSq < bestDistanceSq
                    || (distanceSq == bestDistanceSq && i < result.centerlineSegmentIndex))
                {
                    bestDistanceSq = distanceSq;
                    result.arcLength = (float)(double)(a.arcLength + (float)(double)(t * ((float)(double)(b.arcLength - a.arcLength))));
                    result.halfWidth = (float)(double)(a.halfWidth + (float)(double)(t * ((float)(double)(b.halfWidth - a.halfWidth))));
                    result.segmentIndex = a.legIndex;
                    result.centerlineSegmentIndex = i;
                }
            }
            if (result.centerlineSegmentIndex == uint.MaxValue)
            {
                throw new InvalidOperationException("tmnf route: projection BVH returned no segment");
            }
            result.lateralOffset = Fp.Sqrt(bestDistanceSq);
            return result;
        }
    }
}
