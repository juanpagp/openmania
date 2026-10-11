using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>The immutable half of a world.</summary>
    [Serializable]
    public sealed class WorldCold
    {
        public byte[] vehicleBlob;
        public World.VehicleSnapshotHeader header;
        public VehicleTuning tuning;
        public CHmsResponseBody playerResponseBody;
        public CHmsResponseMaterial responseMaterial;
        /// <summary>0x0055F3B0-style uniform gravity field.</summary>
        public int gravityActive;
        public GmVec3 gravityValue;
        /// <summary>The body reference position (image data).</summary>
        public GmVec3 bodyReferencePosition;
    }

    /// <summary>Creation options (TmnfWorldOptions).</summary>
    [Serializable]
    public sealed class WorldOptions
    {
        /// <summary>128x128 fake-contact mask; REQUIRED at runtime.</summary>
        public byte[] fakeContactMask;
        /// <summary>Require the image to declare the canonical hashes.</summary>
        public bool pinProvenance;
    }

    /// <summary>The mutable simulation world: one car + one track.</summary>
    public sealed partial class World
    {
        public WorldCold cold;
        public TrackLoader.TmnfTrack track;
        public RouteLoader.TmnfRoute route;

        public CHmsDyna dyna;
        public Vehicle vehicle;

        /// <summary>Collision trees: pre-order nodes (wheels included).</summary>
        public List<VehicleTree> nodes = new List<VehicleTree>();
        public List<int> rootChildren = new List<int>();
        public uint rootObjectRef;
        public uint rootFlags;
        public GmBoxAligned rootBox;
        public GmIso4 rootLocalIso;
        /// <summary>Node index of each wheel leaf.</summary>
        public int[] wheelNodeOf = new int[4];
        /// <summary>One contact buffer per node (root has none).</summary>
        public List<SHmsSphereBufferContact> contactBuffers = new List<SHmsSphereBufferContact>();

        // Zone state (CHmsCollisionManager_SZone mutable fields)
        public uint currentMaterial;
        public uint currentCorpus1;
        public uint currentCorpus2;
        public bool staticGroupBound;
        public List<int> mergeBuffers = new List<int>();
        public CHmsCollisionBuffer collisionBuffer = new CHmsCollisionBuffer();

        // Physics world state (TmnfPhysicsWorld)
        public float linearDragScale;
        public float angularDragScale;
        public ulong triggerContacts;
        /// <summary>Game scene object +0x18 of the car corpus.</summary>
        public uint sceneFlags;
        public uint tickTime;

        /// <summary>The car corpus' object_ref (== track.staticResponseCount).</summary>
        public uint playerCorpusRef;

        /// <summary>One node of the vehicle collision tree (wheel set on leaves).</summary>
        [Serializable]
        public sealed class VehicleTree
        {
            public uint objectRef;
            public uint flags;
            public GmBoxAligned boxAligned;
            public GmIso4 localIso;
            public CPlugSurface surface; // null = none
            public List<int> children = new List<int>();
            public uint? wheel;
        }

        // ------------------------------------------------------------------
        // constants (image layout)
        // ------------------------------------------------------------------

        public const int CarSize = 0x878;
        public const int VehicleStructSize = 0x50;
        public const int TuningSize = 0x3ac;
        public const int WheelSize = 0x2fc;
        public const int DynaSize = 0x344;
        public const int ParamsSize = 0x5c;
        public const int StateSize = 0xb4;
        public const int ActiveCurveCount = 21;
        public const int PrimaryCurveCount = 2;
        public const int GearboxCount = 4;
        public const int GearboxValueCount = 6;
        public const int VehicleCollisionMaxNodes = 8;

        private const int CurveAccel = 0;
        private const int CurveLateralContactSlowdown = 1;
        private const int CurveSteerSlowdown = 2;
        private const int CurveSteerDriveTorque = 3;
        private const int CurveMaxSideFriction = 4;
        private const int CurveRolloverLateral = 5;
        private const int CurveRolloverAngle = 6;
        private const int CurveM4SteerRadius = 7;
        private const int CurveM4MaxFriction = 8;
        private const int CurveM5SlippingAccel = 9;
        private const int CurveWaterFriction = 10;
        private const int CurveM6DamperModulation = 11;
        private const int CurveM6RearGearAccel = 12;
        private const int CurveM6RolloverRatio = 13;
        private const int CurveM6BurnoutRadius = 14;
        private const int CurveM6BurnoutLateralSpeed = 15;
        private const int CurveM6DonutRollover = 16;
        private const int CurveM6BurnoutRollover = 17;
        private const int CurveAirVertical = 18;
        private const int CurveSteeringAngle = 19;
        private const int CurveEffect = 20;

        private static readonly uint[] ActiveCurveOffsets =
        {
            0x034, 0x068, 0x078, 0x0a0, 0x0ac, 0x0b8, 0x0bc,
            0x1b4, 0x1bc, 0x1e0, 0x218, 0x224, 0x230, 0x250,
            0x25c, 0x260, 0x288, 0x2a4, 0x36c, 0x378, 0x380,
        };
        private static readonly uint[] PrimaryCurveOffsets = { 0x044, 0x048 };
        private static readonly uint[] GearboxOffsets = { 0x2c4, 0x2d4, 0x2e0, 0x304 };

        private const uint VehicleCurveOwnerStruct = 0;
        private const uint VehicleCurveOwnerTuning = 1;

        /// <summary>The snapshot magic ("TMNFM6G1") as bytes.</summary>
        public static readonly byte[] TmnfVehicleSnapshotMagic = { (byte)'T', (byte)'M', (byte)'N', (byte)'F', (byte)'M', (byte)'6', (byte)'G', (byte)'1' };
        public const uint TmnfVehicleSnapshotVersion = 4;

        private const uint VehicleCollisionTreeBody = 0;
        private const uint VehicleCollisionTreeWheel = 1;
        private const uint VehicleCollisionTreeRoot = 2;
        private const uint VehicleCollisionKindMask = 0xff;
        private const uint VehicleCollisionChildShift = 8;

        /// <summary>TMNF_WATER_IMPULSE_POSITIONS (float bits in the comments).</summary>
        public static readonly float[] TmnfWaterImpulsePositions =
        {
            0.0f,
            BitConverter.Int32BitsToSingle(unchecked((int)0x3E99_999A)), /* 0.3f */
            0.5f,
            1.0f,
        };
        /// <summary>TMNF_WATER_IMPULSE_VERTICAL_VALUES.</summary>
        public static readonly float[] TmnfWaterImpulseVerticalValues =
        {
            BitConverter.Int32BitsToSingle(unchecked((int)0x3FCC_CCCC)), /* 0.9f */
            BitConverter.Int32BitsToSingle(unchecked((int)0x3FCC_CCCC)),
            1.5f,
            1.5f,
        };
        /// <summary>TMNF_WATER_IMPULSE_HORIZONTAL_VALUES.</summary>
        public static readonly float[] TmnfWaterImpulseHorizontalValues =
        {
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F66_6666)), /* 0.7f */
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F66_6666)),
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F33_3334)), /* 0.6f */
            BitConverter.Int32BitsToSingle(unchecked((int)0x3F33_3334)),
        };

        /// <summary>SHA-256 of the canonical TMNF 2.11.26 executable.</summary>
        public static readonly byte[] Tmnf21126ExeSha256 = TrackLoader.Tmnf21126ExeSha256;

        // ------------------------------------------------------------------
        // image structures
        // ------------------------------------------------------------------

        /// <summary>The 320-byte vehicle snapshot header.</summary>
        [Serializable]
        public sealed class VehicleSnapshotHeader
        {
            public byte[] magic = new byte[8];
            public uint version;
            public uint totalSize;
            public uint headerSize;
            public uint phase;
            public uint tick;
            public uint carId;
            public uint vehicleStructId;
            public uint tuningContainerId;
            public uint wheelsId;
            public uint itemId;
            public uint corpusId;
            public uint dynaId;
            public uint paramsId;
            public uint stateId;
            public uint modelIsoId;
            public uint bodyReferenceId;
            public uint wheelCount;
            public uint groundIdCount;
            public uint groundMaterialCount;
            public uint tuningCount;
            public uint activeTuningKey;
            public uint curveCount;
            public uint gearboxCount;
            public uint carOffset;
            public uint vehicleStructOffset;
            public uint tuningDescriptorsOffset;
            public uint wheelsOffset;
            public uint dynaOffset;
            public uint paramsOffset;
            public uint stateOffset;
            public uint curveDescriptorsOffset;
            public uint gearboxDescriptorsOffset;
            public uint groundIdsOffset;
            public uint groundMaterialsOffset;
            public uint modelIsoOffset;
            public uint bodyReferenceOffset;
            public uint curveDataOffset;
            public uint gearboxDataOffset;
            public uint collisionChildCount;
            public uint collisionRootOffset;
            public uint collisionChildrenOffset;
            public uint collisionMaterialIdsOffset;
            public float modelValue;
            public float lateralForceFactor;
            public float longitudinalForceFactor;
            public float steeringAngle;
            public int grounded;
            public GmVec3 existingForce;
            public GmVec3 localSpeed;
            public GmVec3 localAngularSpeed;
            public float[] material = new float[4];
            public int sliding;
            public float brakeForce;
            public byte[] sourceExeSha256 = new byte[32];
            public byte[] sourceTrackSha256 = new byte[32];

            public static VehicleSnapshotHeader FromBytes(byte[] b)
            {
                VehicleSnapshotHeader h = new VehicleSnapshotHeader();
                Array.Copy(b, 0, h.magic, 0, 8);
                uint g(uint o) => BitsUtil.GetU32(b, (int)o);
                for (int i = 0; i < 4; i++)
                {
                    h.material[i] = BitsUtil.GetF32(b, 0xe8 + 4 * i);
                }
                Array.Copy(b, 0x100, h.sourceExeSha256, 0, 32);
                Array.Copy(b, 0x120, h.sourceTrackSha256, 0, 32);
                h.version = g(0x08);
                h.totalSize = g(0x0c);
                h.headerSize = g(0x10);
                h.phase = g(0x14);
                h.tick = g(0x18);
                h.carId = g(0x1c);
                h.vehicleStructId = g(0x20);
                h.tuningContainerId = g(0x24);
                h.wheelsId = g(0x28);
                h.itemId = g(0x2c);
                h.corpusId = g(0x30);
                h.dynaId = g(0x34);
                h.paramsId = g(0x38);
                h.stateId = g(0x3c);
                h.modelIsoId = g(0x40);
                h.bodyReferenceId = g(0x44);
                h.wheelCount = g(0x48);
                h.groundIdCount = g(0x4c);
                h.groundMaterialCount = g(0x50);
                h.tuningCount = g(0x54);
                h.activeTuningKey = g(0x58);
                h.curveCount = g(0x5c);
                h.gearboxCount = g(0x60);
                h.carOffset = g(0x64);
                h.vehicleStructOffset = g(0x68);
                h.tuningDescriptorsOffset = g(0x6c);
                h.wheelsOffset = g(0x70);
                h.dynaOffset = g(0x74);
                h.paramsOffset = g(0x78);
                h.stateOffset = g(0x7c);
                h.curveDescriptorsOffset = g(0x80);
                h.gearboxDescriptorsOffset = g(0x84);
                h.groundIdsOffset = g(0x88);
                h.groundMaterialsOffset = g(0x8c);
                h.modelIsoOffset = g(0x90);
                h.bodyReferenceOffset = g(0x94);
                h.curveDataOffset = g(0x98);
                h.gearboxDataOffset = g(0x9c);
                h.collisionChildCount = g(0xa0);
                h.collisionRootOffset = g(0xa4);
                h.collisionChildrenOffset = g(0xa8);
                h.collisionMaterialIdsOffset = g(0xac);
                h.modelValue = BitsUtil.GetF32(b, 0xb0);
                h.lateralForceFactor = BitsUtil.GetF32(b, 0xb4);
                h.longitudinalForceFactor = BitsUtil.GetF32(b, 0xb8);
                h.steeringAngle = BitsUtil.GetF32(b, 0xbc);
                h.grounded = BitsUtil.GetI32(b, 0xc0);
                h.existingForce = GmVec3.FromBytes(b, 0xc4);
                h.localSpeed = GmVec3.FromBytes(b, 0xd0);
                h.localAngularSpeed = GmVec3.FromBytes(b, 0xdc);
                h.sliding = BitsUtil.GetI32(b, 0xf8);
                h.brakeForce = BitsUtil.GetF32(b, 0xfc);
                return h;
            }
        }

        /// <summary>16-byte tuning descriptor.</summary>
        [Serializable]
        public struct VehicleTuningDescriptor
        {
            public uint key;
            public uint objectId;
            public uint rawOffset;
            public uint rawSize;
        }

        /// <summary>40-byte curve descriptor.</summary>
        [Serializable]
        public struct VehicleCurveDescriptor
        {
            public uint ownerKind;
            public uint ownerKey;
            public uint fieldOffset;
            public uint objectId;
            public uint positionsId;
            public uint valuesId;
            public uint count;
            public int interpolation;
            public uint positionsOffset;
            public uint valuesOffset;
        }

        /// <summary>24-byte gearbox descriptor.</summary>
        [Serializable]
        public struct VehicleGearboxDescriptor
        {
            public uint ownerKey;
            public uint fieldOffset;
            public uint bufferId;
            public uint dataId;
            public uint count;
            public uint dataOffset;
        }

        /// <summary>40-byte ground material record.</summary>
        [Serializable]
        public struct VehicleMaterialRecord
        {
            public uint objectId;
            public float[] values; // 4
            public uint fakeContactMask;
            public float fakeContactPeriodX;
            public float fakeContactPeriodZ;
            public float fakeContactImpulseScale;
            public float fakeContactImpulseLimit;

            // Note: no parameterless constructor on purpose — C# 9 (Unity's
            // level) does not allow them on structs. The ONLY construction
            // site is ReadMaterialRecord below, which always assigns the
            // values array in its initializer; default(VehicleMaterialRecord)
            // leaves values null and must never be dereferenced.
        }

        /// <summary>132-byte collision tree record.</summary>
        [Serializable]
        public sealed class VehicleCollisionTreeRecord
        {
            public uint objectId;
            public uint flags;
            public uint surfaceId;
            public uint geometryId;
            public uint kind;
            public uint wheelIndex;
            public uint materialIndex;
            public uint materialCount;
            public ushort geometryMaterialIndex;
            public byte geometryType;
            public byte geometryReserved;
            public GmBoxAligned boxAligned;
            public GmIso4 localIso;
            public byte[] shape = new byte[24];
        }

        /// <summary>The validated graph: header + descriptor views into the blob.</summary>
        public sealed class VehicleGraph
        {
            public VehicleSnapshotHeader header;
            public List<VehicleTuningDescriptor> tuningDescriptors = new List<VehicleTuningDescriptor>();
            public List<VehicleCurveDescriptor> curveDescriptors = new List<VehicleCurveDescriptor>();
            public List<VehicleGearboxDescriptor> gearboxDescriptors = new List<VehicleGearboxDescriptor>();
            public List<VehicleMaterialRecord> materials = new List<VehicleMaterialRecord>();
            public VehicleCollisionTreeRecord collisionRoot;
            public List<VehicleCollisionTreeRecord> collisionChildren = new List<VehicleCollisionTreeRecord>();
            public byte[] blob;
        }

        // ------------------------------------------------------------------
        // graph validation
        // ------------------------------------------------------------------

        private static int GraphSection(byte[] blob, uint offset, uint length, string what)
        {
            uint size = (uint)blob.Length;
            if (offset > size || length > size - offset)
            {
                throw new InvalidOperationException($"tmnf world: vehicle graph section is out of bounds: {what}");
            }
            return (int)offset;
        }

        private static bool ContainsOffset(uint[] offsets, uint offset)
        {
            foreach (uint o in offsets)
            {
                if (o == offset)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>validate_vehicle_graph: the full header + graph check.</summary>
        public static VehicleGraph ValidateVehicleGraph(byte[] blob, byte[] expectedTrackSha256, bool pinProvenance)
        {
            if (blob.Length < 320)
            {
                throw new InvalidOperationException("tmnf world: truncated vehicle graph");
            }
            VehicleSnapshotHeader header = VehicleSnapshotHeader.FromBytes(blob);
            if (!BytesEqual(header.magic, TmnfVehicleSnapshotMagic)
                || header.version != TmnfVehicleSnapshotVersion
                || header.totalSize != (uint)blob.Length
                || header.headerSize != 320
                || header.phase != 0
                || header.carId == 0
                || header.vehicleStructId == 0
                || header.tuningContainerId == 0
                || header.wheelsId == 0
                || header.itemId == 0
                || header.corpusId == 0
                || header.dynaId == 0
                || header.paramsId == 0
                || header.stateId == 0
                || header.modelIsoId == 0
                || header.bodyReferenceId == 0
                || header.wheelCount != 4
                || header.groundIdCount != 31
                || header.groundMaterialCount == 0
                || header.groundMaterialCount > SurfaceMaterials.TmnfMaxGroundMaterials
                || header.tuningCount == 0
                || header.tuningCount > 64
                || header.activeTuningKey >= header.tuningCount
                || header.curveCount == 0
                || header.curveCount > 4096
                || header.collisionChildCount == 0
                || header.collisionChildCount > VehicleCollisionMaxNodes
                || header.gearboxCount != header.tuningCount * GearboxCount
                || (pinProvenance
                    && (!BytesEqual(header.sourceExeSha256, Tmnf21126ExeSha256)
                        || (expectedTrackSha256 != null
                            && !BytesEqual(header.sourceTrackSha256, expectedTrackSha256)))))
            {
                throw new InvalidOperationException("tmnf world: invalid vehicle graph header");
            }
            GraphSection(blob, header.carOffset, CarSize, "car");
            GraphSection(blob, header.vehicleStructOffset, VehicleStructSize, "vehicle struct");
            GraphSection(blob, header.wheelsOffset, 4 * WheelSize, "wheels");
            GraphSection(blob, header.dynaOffset, DynaSize, "dyna");
            GraphSection(blob, header.paramsOffset, ParamsSize, "params");
            GraphSection(blob, header.stateOffset, StateSize, "state");
            GraphSection(blob, header.groundIdsOffset, 31 * 4, "ground ids");
            GraphSection(blob, header.groundMaterialsOffset, header.groundMaterialCount * 40, "ground materials");
            GraphSection(blob, header.modelIsoOffset, 48, "model iso");
            GraphSection(blob, header.bodyReferenceOffset, 12, "body reference");
            GraphSection(blob, header.curveDataOffset, 0, "curve data");
            GraphSection(blob, header.gearboxDataOffset, 0, "gearbox data");
            int rootOff = GraphSection(blob, header.collisionRootOffset, 132, "collision root");
            int childrenOff = GraphSection(blob, header.collisionChildrenOffset,
                header.collisionChildCount * 132, "collision children");
            VehicleCollisionTreeRecord collisionRoot = ReadCollisionTreeRecord(blob, rootOff);
            var collisionChildren = new List<VehicleCollisionTreeRecord>();
            for (uint i = 0; i < header.collisionChildCount; i++)
            {
                collisionChildren.Add(ReadCollisionTreeRecord(blob, childrenOff + (int)(i * 132)));
            }
            if (collisionRoot.objectId == 0
                || (collisionRoot.flags & 0x80) == 0
                || collisionRoot.surfaceId != 0
                || collisionRoot.geometryId != 0
                || collisionRoot.kind != VehicleCollisionTreeRoot
                || collisionRoot.wheelIndex != uint.MaxValue
                || collisionRoot.materialCount != 0)
            {
                throw new InvalidOperationException("tmnf world: invalid vehicle collision root");
            }
            uint wheelMask = 0;
            uint collisionMaterialCount = 0;
            uint openChildren = 0;
            foreach (VehicleCollisionTreeRecord tree in collisionChildren)
            {
                uint kind = tree.kind & VehicleCollisionKindMask;
                uint childCount = tree.kind >> (int)VehicleCollisionChildShift;
                if (tree.objectId == 0
                    || (tree.flags & 0x80) == 0
                    || childCount > 0xff
                    || tree.materialIndex != collisionMaterialCount)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle collision node");
                }
                // Pre-order: a record either fills a slot opened by an
                // earlier parent or is a direct child of the root.
                if (openChildren != 0)
                {
                    openChildren -= 1;
                }
                openChildren += childCount;
                if (tree.surfaceId == 0)
                {
                    if (kind != VehicleCollisionTreeBody
                        || tree.geometryId != 0
                        || tree.wheelIndex != uint.MaxValue
                        || tree.materialCount != 0
                        || childCount == 0)
                    {
                        throw new InvalidOperationException("tmnf world: invalid surface-less vehicle collision node");
                    }
                    continue;
                }
                if (tree.geometryId == 0
                    || (tree.flags & 0x04) == 0
                    || (tree.geometryType != GmSurfTypes.GmSurfEllipsoid
                        && tree.geometryType != GmSurfTypes.GmSurfSphere)
                    || tree.geometryMaterialIndex != 0
                    || tree.materialCount != 1)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle collision shape");
                }
                int material = GraphSection(blob,
                    unchecked(header.collisionMaterialIdsOffset + tree.materialIndex),
                    tree.materialCount, "collision material");
                if (blob[material] >= 31)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle collision material");
                }
                if (kind == VehicleCollisionTreeBody)
                {
                    if (tree.wheelIndex != uint.MaxValue)
                    {
                        throw new InvalidOperationException("tmnf world: invalid vehicle body collision node");
                    }
                }
                else if (kind == VehicleCollisionTreeWheel)
                {
                    if (tree.wheelIndex >= 4
                        || childCount != 0
                        || (wheelMask & (1u << (int)tree.wheelIndex)) != 0)
                    {
                        throw new InvalidOperationException("tmnf world: invalid vehicle wheel collision node");
                    }
                    wheelMask |= 1u << (int)tree.wheelIndex;
                }
                else
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle collision node kind");
                }
                collisionMaterialCount += tree.materialCount;
            }
            if (wheelMask != 0x0f || openChildren != 0)
            {
                throw new InvalidOperationException("tmnf world: incomplete vehicle collision tree");
            }
            GraphSection(blob, header.collisionMaterialIdsOffset,
                collisionMaterialCount, "collision material ids");

            int tuningsOff = GraphSection(blob, header.tuningDescriptorsOffset,
                header.tuningCount * 16, "tuning descriptors");
            var tuningDescriptors = new List<VehicleTuningDescriptor>();
            for (uint i = 0; i < header.tuningCount; i++)
            {
                VehicleTuningDescriptor d = ReadTuningDescriptor(blob, tuningsOff + (int)(i * 16));
                if (d.key != i || d.objectId == 0 || d.rawSize != TuningSize)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle tuning descriptor");
                }
                GraphSection(blob, d.rawOffset, TuningSize, "tuning raw");
                tuningDescriptors.Add(d);
            }

            int curvesOff = GraphSection(blob, header.curveDescriptorsOffset,
                header.curveCount * 40, "curve descriptors");
            var curveDescriptors = new List<VehicleCurveDescriptor>();
            for (uint i = 0; i < header.curveCount; i++)
            {
                VehicleCurveDescriptor c = ReadCurveDescriptor(blob, curvesOff + (int)(i * 40));
                byte[] owner;
                if (c.ownerKind == VehicleCurveOwnerStruct)
                {
                    if (c.ownerKey != 0
                        || !ContainsOffset(PrimaryCurveOffsets, c.fieldOffset))
                    {
                        throw new InvalidOperationException("tmnf world: invalid vehicle-struct curve owner");
                    }
                    owner = blob;
                }
                else if (c.ownerKind == VehicleCurveOwnerTuning)
                {
                    if (c.ownerKey >= header.tuningCount
                        || !ContainsOffset(ActiveCurveOffsets, c.fieldOffset))
                    {
                        throw new InvalidOperationException("tmnf world: invalid tuning curve owner");
                    }
                    uint rawOff = tuningDescriptors[(int)c.ownerKey].rawOffset;
                    owner = blob;
                    // (the u32-at-owner check below uses the raw offset)
                    if (BitsUtil.GetU32(blob, (int)(rawOff + c.fieldOffset)) == 0)
                    {
                        throw new InvalidOperationException("tmnf world: invalid vehicle curve descriptor");
                    }
                }
                else
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle curve owner kind");
                }
                if (c.ownerKind == VehicleCurveOwnerStruct
                    && BitsUtil.GetU32(blob, (int)(header.vehicleStructOffset + c.fieldOffset)) == 0)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle curve descriptor");
                }
                if (c.objectId == 0 || c.positionsId == 0 || c.valuesId == 0
                    || c.count == 0 || c.count > uint.MaxValue / 4)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle curve descriptor");
                }
                GraphSection(blob, c.positionsOffset, c.count * 4, "curve positions");
                GraphSection(blob, c.valuesOffset, c.count * 4, "curve values");
                for (int j = 0; j < (int)i; j++)
                {
                    VehicleCurveDescriptor other = curveDescriptors[j];
                    if (other.ownerKind == c.ownerKind
                        && other.ownerKey == c.ownerKey
                        && other.fieldOffset == c.fieldOffset)
                    {
                        throw new InvalidOperationException("tmnf world: duplicate vehicle curve descriptor");
                    }
                }
                curveDescriptors.Add(c);
            }

            int gearboxesOff = GraphSection(blob, header.gearboxDescriptorsOffset,
                header.gearboxCount * 24, "gearbox descriptors");
            var gearboxDescriptors = new List<VehicleGearboxDescriptor>();
            for (uint i = 0; i < header.gearboxCount; i++)
            {
                VehicleGearboxDescriptor g = ReadGearboxDescriptor(blob, gearboxesOff + (int)(i * 24));
                if (g.ownerKey >= header.tuningCount
                    || !ContainsOffset(GearboxOffsets, g.fieldOffset)
                    || g.bufferId == 0
                    || g.dataId == 0
                    || g.count != GearboxValueCount)
                {
                    throw new InvalidOperationException("tmnf world: invalid vehicle gearbox descriptor");
                }
                uint rawOff = tuningDescriptors[(int)g.ownerKey].rawOffset;
                if (BitsUtil.GetU32(blob, (int)(rawOff + g.fieldOffset)) != GearboxValueCount
                    || BitsUtil.GetU32(blob, (int)(rawOff + g.fieldOffset + 4)) == 0)
                {
                    throw new InvalidOperationException("tmnf world: vehicle gearbox does not match raw tuning");
                }
                GraphSection(blob, g.dataOffset, 24, "gearbox data");
                for (int j = 0; j < (int)i; j++)
                {
                    VehicleGearboxDescriptor other = gearboxDescriptors[j];
                    if (other.ownerKey == g.ownerKey
                        && other.fieldOffset == g.fieldOffset)
                    {
                        throw new InvalidOperationException("tmnf world: duplicate vehicle gearbox descriptor");
                    }
                }
                gearboxDescriptors.Add(g);
            }

            int materialsOff = GraphSection(blob, header.groundMaterialsOffset,
                header.groundMaterialCount * 40, "ground materials");
            var materials = new List<VehicleMaterialRecord>();
            for (uint i = 0; i < header.groundMaterialCount; i++)
            {
                VehicleMaterialRecord m = ReadMaterialRecord(blob, materialsOff + (int)(i * 40));
                if (m.objectId == 0)
                {
                    throw new InvalidOperationException("tmnf world: vehicle ground material has null identity");
                }
                if (m.fakeContactMask > 1)
                {
                    throw new InvalidOperationException("tmnf world: vehicle ground material mask is not a flag");
                }
                materials.Add(m);
            }

            VehicleGraph graph = new VehicleGraph();
            graph.header = header;
            graph.tuningDescriptors = tuningDescriptors;
            graph.curveDescriptors = curveDescriptors;
            graph.gearboxDescriptors = gearboxDescriptors;
            graph.materials = materials;
            graph.collisionRoot = collisionRoot;
            graph.collisionChildren = collisionChildren;
            graph.blob = blob;
            return graph;
        }

        private static VehicleTuningDescriptor ReadTuningDescriptor(byte[] b, int o)
        {
            VehicleTuningDescriptor d = default(VehicleTuningDescriptor);
            d.key = BitsUtil.GetU32(b, o);
            d.objectId = BitsUtil.GetU32(b, o + 4);
            d.rawOffset = BitsUtil.GetU32(b, o + 8);
            d.rawSize = BitsUtil.GetU32(b, o + 12);
            return d;
        }

        private static VehicleCurveDescriptor ReadCurveDescriptor(byte[] b, int o)
        {
            VehicleCurveDescriptor d = default(VehicleCurveDescriptor);
            d.ownerKind = BitsUtil.GetU32(b, o);
            d.ownerKey = BitsUtil.GetU32(b, o + 4);
            d.fieldOffset = BitsUtil.GetU32(b, o + 8);
            d.objectId = BitsUtil.GetU32(b, o + 12);
            d.positionsId = BitsUtil.GetU32(b, o + 16);
            d.valuesId = BitsUtil.GetU32(b, o + 20);
            d.count = BitsUtil.GetU32(b, o + 24);
            d.interpolation = BitsUtil.GetI32(b, o + 28);
            d.positionsOffset = BitsUtil.GetU32(b, o + 32);
            d.valuesOffset = BitsUtil.GetU32(b, o + 36);
            return d;
        }

        private static VehicleGearboxDescriptor ReadGearboxDescriptor(byte[] b, int o)
        {
            VehicleGearboxDescriptor d = default(VehicleGearboxDescriptor);
            d.ownerKey = BitsUtil.GetU32(b, o);
            d.fieldOffset = BitsUtil.GetU32(b, o + 4);
            d.bufferId = BitsUtil.GetU32(b, o + 8);
            d.dataId = BitsUtil.GetU32(b, o + 12);
            d.count = BitsUtil.GetU32(b, o + 16);
            d.dataOffset = BitsUtil.GetU32(b, o + 20);
            return d;
        }

        private static VehicleMaterialRecord ReadMaterialRecord(byte[] b, int o)
        {
            VehicleMaterialRecord m = new VehicleMaterialRecord
            {
                values = new float[4],
            };
            for (int i = 0; i < 4; i++)
            {
                m.values[i] = BitsUtil.GetF32(b, o + 4 + 4 * i);
            }
            m.objectId = BitsUtil.GetU32(b, o);
            m.fakeContactMask = BitsUtil.GetU32(b, o + 20);
            m.fakeContactPeriodX = BitsUtil.GetF32(b, o + 24);
            m.fakeContactPeriodZ = BitsUtil.GetF32(b, o + 28);
            m.fakeContactImpulseScale = BitsUtil.GetF32(b, o + 32);
            m.fakeContactImpulseLimit = BitsUtil.GetF32(b, o + 36);
            return m;
        }

        private static VehicleCollisionTreeRecord ReadCollisionTreeRecord(byte[] b, int o)
        {
            VehicleCollisionTreeRecord r = new VehicleCollisionTreeRecord();
            byte[] shape = new byte[24];
            Array.Copy(b, o + 0x6c, shape, 0, 24);
            r.objectId = BitsUtil.GetU32(b, o);
            r.flags = BitsUtil.GetU32(b, o + 4);
            r.surfaceId = BitsUtil.GetU32(b, o + 8);
            r.geometryId = BitsUtil.GetU32(b, o + 12);
            r.kind = BitsUtil.GetU32(b, o + 16);
            r.wheelIndex = BitsUtil.GetU32(b, o + 20);
            r.materialIndex = BitsUtil.GetU32(b, o + 24);
            r.materialCount = BitsUtil.GetU32(b, o + 28);
            r.geometryMaterialIndex = BitsUtil.GetU16(b, o + 32);
            r.geometryType = b[o + 34];
            r.geometryReserved = b[o + 35];
            r.boxAligned = GmBoxAligned.FromBytes(b, o + 36);
            r.localIso = GmIso4.FromBytes(b, o + 60);
            r.shape = shape;
            return r;
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

        // ... construction, decode, tick, response ctx in WorldTick.cs
    }
}
