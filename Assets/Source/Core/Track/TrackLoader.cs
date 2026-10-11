using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>The TMNFTRK1 image loader + the static grid builder.</summary>
    public static class TrackLoader
    {
        public const uint TmnfTrackVersion = 3;
        public const int TmnfTrackSectionCount = 12;
        public const uint TmnfTrackMaterialCount = 31;

        public const int TmnfTrackEntries = 0;
        public const int TmnfTrackSurfaces = 1;
        public const int TmnfTrackMeshes = 2;
        public const int TmnfTrackVertices = 3;
        public const int TmnfTrackFaces = 4;
        public const int TmnfTrackNodes = 5;
        public const int TmnfTrackMaterialIds = 6;
        public const int TmnfTrackMaterialData = 7;
        public const int TmnfTrackCollisionPairs = 8;
        public const int TmnfTrackCorpusIsos = 9;
        public const int TmnfTrackWater = 10;
        public const int TmnfTrackWaterCells = 11;

        public const uint TmnfWaterMaterial = 13;

        /// <summary>The canonical TMNF 2.11.26 executable SHA-256.</summary>
        public static readonly byte[] Tmnf21126ExeSha256 =
        {
            0x38, 0x47, 0xcf, 0x9f, 0x20, 0xbf, 0xc6, 0x39,
            0x14, 0x45, 0x00, 0x60, 0xed, 0x52, 0x8c, 0x12,
            0x10, 0x4f, 0x74, 0x3d, 0x96, 0xad, 0x23, 0xd6,
            0xe7, 0x6a, 0xbd, 0x17, 0x8d, 0xe8, 0xc8, 0x4f,
        };

        /// <summary>0x10 bytes; one file section.</summary>
        [Serializable]
        public struct TmnfTrackSection
        {
            public ulong offset;
            public uint count;
            public uint stride;
        }

        /// <summary>0x150 bytes.</summary>
        [Serializable]
        public sealed class TmnfTrackHeader
        {
            public byte[] magic = new byte[8]; // "TMNFTRK1"
            public uint version;
            public uint endian;       // 0x12345678
            public uint headerSize;   // 0x150
            public uint sectionCount; // 12
            public ulong fileSize;
            public byte[] exeSha256 = new byte[32];
            public byte[] trackSha256 = new byte[32];
            public TmnfTrackSection[] sections = new TmnfTrackSection[TmnfTrackSectionCount];
            public byte[] payloadSha256 = new byte[32];
            public byte[] reserved = new byte[16];
        }

        /// <summary>0x14 bytes (5 raw words, content unused by the sim).</summary>
        [Serializable]
        public struct TmnfTrackCollisionPair
        {
            public uint[] raw;
        }

        /// <summary>The loaded static collision snapshot.</summary>
        [Serializable]
        public sealed class TmnfTrack
        {
            public TmnfTrackHeader header = new TmnfTrackHeader();
            public List<HmsStaticCollisionEntry> entries = new List<HmsStaticCollisionEntry>();
            public uint entryCount;
            public List<CPlugSurface> surfaces = new List<CPlugSurface>();
            public uint surfaceCount;
            public List<GmSurfMesh> meshes = new List<GmSurfMesh>();
            public uint meshCount;
            public List<CPlugSurfaceMaterialData> materials = new List<CPlugSurfaceMaterialData>();
            public List<TmnfTrackCollisionPair> collisionPairs = new List<TmnfTrackCollisionPair>();
            public uint collisionPairCount;
            /// <summary>Indexed by corpus_ref - 1.</summary>
            public List<GmIso4> corpusIsos = new List<GmIso4>();
            public uint corpusCount;
            public TmnfTrackWater water = new TmnfTrackWater();
            /// <summary>Region-restricted static tree copies per grid cell.</summary>
            public TmnfStaticGrid grid = new TmnfStaticGrid();
            public int gridNodeCount;
            public int gridMeshSlotCount;
            public List<CHmsResponseBody> staticResponseBodies = new List<CHmsResponseBody>();
            public List<byte> staticResponsePresent = new List<byte>();
            public uint staticResponseCount;
        }

        // ------------------------------------------------------------------
        // parsing
        // ------------------------------------------------------------------

        private static TmnfTrackHeader ParseTrackHeader(byte[] data)
        {
            Reader r = new Reader(data);
            TmnfTrackHeader h = new TmnfTrackHeader();
            h.magic = r.Bytes(8);
            h.version = r.U32();
            h.endian = r.U32();
            h.headerSize = r.U32();
            h.sectionCount = r.U32();
            h.fileSize = r.U64();
            h.exeSha256 = r.Bytes(32);
            h.trackSha256 = r.Bytes(32);
            for (int i = 0; i < TmnfTrackSectionCount; i++)
            {
                h.sections[i].offset = r.U64();
                h.sections[i].count = r.U32();
                h.sections[i].stride = r.U32();
            }
            h.payloadSha256 = r.Bytes(32);
            h.reserved = r.Bytes(16);
            return h;
        }

        private static GmSurfMeshFace ParseMeshFace(Reader r)
        {
            GmSurfMeshFace f = default(GmSurfMeshFace);
            f.normal = r.GmVec3();
            f.reserved0c = r.U32();
            f.vertex0 = r.U32();
            f.vertex1 = r.U32();
            f.vertex2 = r.U32();
            f.materialIndex = r.U16();
            f.reserved1e = r.U16();
            return f;
        }

        private static GmSurfMeshNode ParseMeshNode(Reader r)
        {
            GmSurfMeshNode n = default(GmSurfMeshNode);
            n.skipCount = r.U32();
            GmVec3 center = r.GmVec3();
            GmVec3 half = r.GmVec3();
            n.boxAligned = default(GmBoxAligned);
            n.boxAligned.center = center;
            n.boxAligned.halfExtent = half;
            n.faceIndex = r.U32();
            return n;
        }

        private static ulong SectionBytes(ref TmnfTrackSection section)
        {
            ulong bytes = section.count * section.stride;
            if (section.stride != 0 && bytes / section.stride != section.count)
            {
                throw new InvalidOperationException("tmnf track: section size overflow");
            }
            return bytes;
        }

        private static void ValidateSections(TmnfTrackHeader header, int mappingSize)
        {
            uint[] expectedStride =
            {
                0x60, 0x18, 0x38, 12, 0x20, 0x20, 1, 8, 0x14, 48, 0x28, 1,
            };

            for (int i = 0; i < TmnfTrackSectionCount; i++)
            {
                TmnfTrackSection section = header.sections[i];
                if (section.stride != expectedStride[i]
                    || section.count == 0
                    || (section.offset & 7) != 0
                    || section.offset < 0x150)
                {
                    throw new InvalidOperationException("tmnf track: invalid section descriptor");
                }
                ulong bytes = SectionBytes(ref section);
                if (section.offset > (ulong)mappingSize
                    || bytes > (ulong)mappingSize - section.offset)
                {
                    throw new InvalidOperationException("tmnf track: section outside file");
                }
            }
            for (int i = 0; i < TmnfTrackSectionCount; i++)
            {
                ulong a0 = header.sections[i].offset;
                ulong a1 = a0 + SectionBytes(ref header.sections[i]);
                for (int j = i + 1; j < TmnfTrackSectionCount; j++)
                {
                    ulong b0 = header.sections[j].offset;
                    ulong b1 = b0 + SectionBytes(ref header.sections[j]);
                    if (a0 < b1 && b0 < a1)
                    {
                        throw new InvalidOperationException("tmnf track: overlapping sections");
                    }
                }
            }
        }

        private static ulong ResolveArray(ref TmnfTrackSection section, ulong relative, uint count)
        {
            ulong bytes = count * section.stride;
            ulong sectionEnd = section.offset + SectionBytes(ref section);
            if (relative < section.offset
                || relative > sectionEnd
                || bytes > sectionEnd - relative
                || (relative - section.offset) % section.stride != 0)
            {
                throw new InvalidOperationException("tmnf track: relative pointer outside target section");
            }
            return relative;
        }

        private static List<GmSurfMesh> FixMeshes(byte[] data, TmnfTrackHeader header)
        {
            var meshSection = header.sections[TmnfTrackMeshes];
            var meshes = new List<GmSurfMesh>((int)meshSection.count);
            for (uint i = 0; i < meshSection.count; i++)
            {
                Reader r = Reader.At(data, (int)(meshSection.offset + i * 0x38));
                GmSurf surf = default(GmSurf);
                surf.vtable = r.U32();
                surf.materialIndex = r.U16();
                surf.surfType = r.U8();
                surf.reserved = r.U8();
                uint vertexCount = r.U32();
                r.U32(); // pad0c
                ulong verticesRel = r.U64();
                uint faceCount = r.U32();
                r.U32(); // pad1c
                ulong facesRel = r.U64();
                uint nodeCount = r.U32();
                r.U32(); // pad2c
                ulong nodesRel = r.U64();
                if (surf.surfType != GmSurfTypes.GmSurfMesh
                    || vertexCount == 0
                    || faceCount == 0
                    || nodeCount == 0)
                {
                    throw new InvalidOperationException("tmnf track: invalid mesh header");
                }
                ulong verticesOff = ResolveArray(
                    ref header.sections[TmnfTrackVertices], verticesRel, vertexCount);
                ulong facesOff = ResolveArray(
                    ref header.sections[TmnfTrackFaces], facesRel, faceCount);
                ulong nodesOff = ResolveArray(
                    ref header.sections[TmnfTrackNodes], nodesRel, nodeCount);

                var vertices = new GmVec3[vertexCount];
                {
                    Reader vr = Reader.At(data, (int)verticesOff);
                    for (uint v = 0; v < vertexCount; v++)
                    {
                        vertices[v] = vr.GmVec3();
                    }
                }
                var faces = new GmSurfMeshFace[faceCount];
                {
                    Reader fr = Reader.At(data, (int)facesOff);
                    for (uint f = 0; f < faceCount; f++)
                    {
                        faces[f] = ParseMeshFace(fr);
                        fr.Pos += 0x20 - 0x20; // (already exact)
                    }
                }
                var nodes = new GmSurfMeshNode[nodeCount];
                {
                    Reader nr = Reader.At(data, (int)nodesOff);
                    for (uint n = 0; n < nodeCount; n++)
                    {
                        nodes[n] = ParseMeshNode(nr);
                    }
                }
                GmSurfMesh mesh = new GmSurfMesh();
                mesh.@base = surf;
                mesh.vertexCount = vertexCount;
                mesh.vertices = vertices;
                mesh.faceCount = faceCount;
                mesh.faces = faces;
                mesh.nodeCount = nodeCount;
                mesh.nodes = nodes;

                for (uint f = 0; f < mesh.faceCount; f++)
                {
                    for (int v = 0; v < 3; v++)
                    {
                        if (mesh.faces[f].Vertex(v) >= mesh.vertexCount)
                        {
                            throw new InvalidOperationException("tmnf track: mesh face vertex outside vertex array");
                        }
                    }
                }
                for (uint n = 0; n < mesh.nodeCount; n++)
                {
                    GmSurfMeshNode node = mesh.nodes[n];
                    if (node.skipCount == 0
                        || unchecked(n + node.skipCount) > mesh.nodeCount
                        || (node.faceIndex != uint.MaxValue && node.faceIndex >= mesh.faceCount))
                    {
                        throw new InvalidOperationException("tmnf track: invalid mesh octree node");
                    }
                }
                meshes.Add(mesh);
            }
            return meshes;
        }

        private static (List<CPlugSurface>, List<int>) FixSurfaces(
            byte[] data, TmnfTrackHeader header, List<GmSurfMesh> meshes)
        {
            var section = header.sections[TmnfTrackSurfaces];
            var meshSection = header.sections[TmnfTrackMeshes];
            var surfaces = new List<CPlugSurface>((int)section.count);
            var surfaceMesh = new List<int>((int)section.count);
            for (uint i = 0; i < section.count; i++)
            {
                Reader r = Reader.At(data, (int)(section.offset + i * 0x18));
                ulong meshRel = r.U64();
                ulong materialIdsRel = r.U64();
                uint materialCount = r.U32();
                r.U32(); // reserved
                if (materialCount == 0)
                {
                    throw new InvalidOperationException("tmnf track: surface has no material remap");
                }
                ulong meshOff = ResolveArray(ref meshSection, meshRel, 1);
                ulong materialIdsOff = ResolveArray(
                    ref header.sections[TmnfTrackMaterialIds], materialIdsRel, materialCount);

                int meshIndex = (int)((meshOff - meshSection.offset) / 0x38);
                byte[] materialIds = new byte[materialCount];
                Buffer.BlockCopy(data, (int)materialIdsOff, materialIds, 0, (int)materialCount);

                GmSurfMesh mesh = meshes[meshIndex];
                for (uint f = 0; f < mesh.faceCount; f++)
                {
                    if (mesh.faces[f].materialIndex >= materialCount)
                    {
                        throw new InvalidOperationException("tmnf track: face material outside surface remap");
                    }
                }
                // The original shares one mesh between surfaces via pointers;
                // each referencing surface owns a clone (identical bytes).
                CPlugSurface surface = new CPlugSurface();
                surface.geom = new GmSurfGeomMesh { mesh = mesh.Clone() };
                surface.materialIds = materialIds;
                surface.materialCount = materialCount;
                surfaces.Add(surface);
                surfaceMesh.Add(meshIndex);
            }
            return (surfaces, surfaceMesh);
        }

        private static List<HmsStaticCollisionEntry> FixEntries(byte[] data, TmnfTrackHeader header)
        {
            var section = header.sections[TmnfTrackEntries];
            var surfacesSection = header.sections[TmnfTrackSurfaces];
            var entries = new List<HmsStaticCollisionEntry>((int)section.count);
            for (uint i = 0; i < section.count; i++)
            {
                Reader r = Reader.At(data, (int)(section.offset + i * 0x60));
                uint skipCount = r.U32();
                GmVec3 center = r.GmVec3();
                GmVec3 half = r.GmVec3();
                GmIso4 iso = r.GmIso4();
                uint treeFlags = r.U32();
                ulong surfaceRel = r.U64();
                uint treeId = r.U32();
                uint corpusId = r.U32();
                if (skipCount == 0 || unchecked(i + skipCount) > section.count)
                {
                    throw new InvalidOperationException("tmnf track: invalid flattened static entry");
                }
                ulong surfaceOff = ResolveArray(ref surfacesSection, surfaceRel, 1);
                int surfaceIndex = (int)((surfaceOff - surfacesSection.offset) / 0x18);
                HmsStaticCollisionEntry entry = default(HmsStaticCollisionEntry);
                entry.skipCount = skipCount;
                entry.boxAligned = default(GmBoxAligned);
                entry.boxAligned.center = center;
                entry.boxAligned.halfExtent = half;
                entry.iso = iso;
                entry.treeFlags = treeFlags;
                entry.surface = surfaceIndex;
                entry.treeRef = treeId;
                entry.corpusRef = corpusId;
                entries.Add(entry);
            }
            return entries;
        }

        private static TmnfTrackWater DeriveWater(byte[] data, TmnfTrackHeader header)
        {
            Reader r = Reader.At(data, (int)header.sections[TmnfTrackWater].offset);
            float cellX = r.F32();
            float cellZ = r.F32();
            float originX = r.F32();
            float originZ = r.F32();
            uint width = r.U32();
            uint height = r.U32();
            uint defaultCell = r.U32();
            float level = r.F32();
            float floor = r.F32();
            uint reserved = r.U32();
            int cellsOff = (int)header.sections[TmnfTrackWaterCells].offset;
            if (header.sections[TmnfTrackWater].count != 1
                || cellX != cellZ
                || !(cellX > 0.0f)
                || originX != 0.0f
                || originZ != 0.0f
                || width == 0
                || width > 256
                || height == 0
                || height > 256
                || defaultCell > 1
                || reserved != 0
                || !(floor <= level)
                || header.sections[TmnfTrackWaterCells].count != unchecked(width * height))
            {
                throw new InvalidOperationException("tmnf track: invalid water map");
            }
            int len = (int)(width * height);
            byte[] cells = new byte[len];
            Buffer.BlockCopy(data, cellsOff, cells, 0, len);
            TmnfTrackWater water = new TmnfTrackWater();
            water.cellX = cellX;
            water.cellZ = cellZ;
            water.originX = originX;
            water.originZ = originZ;
            water.width = width;
            water.height = height;
            water.defaultCell = (byte)defaultCell;
            water.cells = cells;
            water.cellCount = 0;
            water.level = level;
            water.floor = floor;
            for (int i = 0; i < len; i++)
            {
                if (cells[i] > 1)
                {
                    throw new InvalidOperationException("tmnf track: water map cell is not a flag");
                }
                water.cellCount += cells[i];
            }
            return water;
        }

        // ------------------------------------------------------------------
        // static grid
        // ------------------------------------------------------------------

        private const double GridCell = 16.0;
        private const double GridExpand = 2.5;
        private const double GridMeshExpand = GridExpand + 1.0;
        private const double GridMeshRegionMargin = 0.5;
        private const double GridBoundsLeafHalf = 64.0;
        private const ulong GridMaxCells = 1UL << 20;
        private const double GridOrthonormalTolerance = 1e-5;
        private static readonly double[] MeshGridCell = { 1.0, 2.0 };
        private static readonly double[] MeshGridExpand = { 0.75, 2.25 };
        private const ulong MeshGridMaxCells = 1UL << 16;

        [Serializable]
        private struct DBox
        {
            public double[] lo;
            public double[] hi;
        }

        private static DBox NewDBox()
        {
            DBox b = default(DBox);
            b.lo = new double[3];
            b.hi = new double[3];
            return b;
        }

        private const ulong FnvOffset = 0xcbf29ce484222325UL;
        private const ulong FnvPrime = 0x100000001b3UL;

        private static ulong FnvBytes(ulong hash, byte[] bytes)
        {
            ulong h = hash;
            foreach (byte b in bytes)
            {
                h ^= b;
                h = unchecked(h * FnvPrime);
            }
            return h;
        }

        /// <summary>Pool element interface: FNV + bit-level equality.</summary>
        private interface IPoolElement<T>
        {
            void FnvUpdate(ByteArrayWriter w);
            bool BytesEqual(T other);
        }

        /// <summary>Streaming byte writer for hashing pool elements.</summary>
        private sealed class ByteArrayWriter
        {
            public System.Collections.Generic.List<byte> b = new System.Collections.Generic.List<byte>();
            public void U32(uint v) { b.AddRange(BitConverter.GetBytes(v)); }
            public void F32(float v) { b.AddRange(BitConverter.GetBytes(v)); }
        }

        private static void PoolFnvFloat(ByteArrayWriter w, float f)
        {
            w.U32(unchecked((uint)BitConverter.SingleToInt32Bits(f)));
        }

        private static bool FloatBitsEqual(float a, float b)
        {
            return BitConverter.SingleToInt32Bits(a) == BitConverter.SingleToInt32Bits(b);
        }

        /// <summary>The content-addressed list store (open addressing, FNV-1a).</summary>
        private sealed class ListIndex<T> where T : IPoolElement<T>
        {
            public List<uint> slots = new List<uint>();
            public List<uint> lengths = new List<uint>();
            public int count;

            private const uint Empty = uint.MaxValue;

            public void Grow(List<T> pool)
            {
                int capacity = slots.Count == 0 ? 4096 : slots.Count * 2;
                List<uint> newSlots = new List<uint>(capacity);
                List<uint> newLengths = new List<uint>(capacity);
                for (int i = 0; i < capacity; i++)
                {
                    newSlots.Add(Empty);
                    newLengths.Add(0);
                }
                for (int i = 0; i < slots.Count; i++)
                {
                    if (slots[i] == Empty)
                    {
                        continue;
                    }
                    int start = (int)slots[i];
                    int length = (int)lengths[i];
                    ByteArrayWriter w = new ByteArrayWriter();
                    for (int k = start; k < start + length; k++)
                    {
                        pool[k].FnvUpdate(w);
                    }
                    ulong hash = FnvBytes(FnvOffset, w.b.ToArray());
                    int at = (int)(hash & (ulong)(capacity - 1));
                    while (newSlots[at] != Empty)
                    {
                        at = (at + 1) & (capacity - 1);
                    }
                    newSlots[at] = slots[i];
                    newLengths[at] = lengths[i];
                }
                slots = newSlots;
                lengths = newLengths;
            }

            /// <summary>Interns the candidate list at pool[start..] and truncates.</summary>
            public uint Intern(List<T> pool, int start)
            {
                int length = pool.Count - start;
                if (count * 2 >= slots.Count)
                {
                    Grow(pool);
                }
                ByteArrayWriter w = new ByteArrayWriter();
                for (int k = start; k < pool.Count; k++)
                {
                    pool[k].FnvUpdate(w);
                }
                ulong hash = FnvBytes(FnvOffset, w.b.ToArray());
                int at = (int)(hash & (ulong)(slots.Count - 1));
                while (slots[at] != Empty)
                {
                    if (lengths[at] == (uint)length)
                    {
                        int existing = (int)slots[at];
                        bool identical = true;
                        for (int k = 0; k < length; k++)
                        {
                            if (!pool[existing + k].BytesEqual(pool[start + k]))
                            {
                                identical = false;
                                break;
                            }
                        }
                        if (identical)
                        {
                            pool.RemoveRange(start, length);
                            return slots[at];
                        }
                    }
                    at = (at + 1) & (slots.Count - 1);
                }
                if (start > int.MaxValue || length > int.MaxValue)
                {
                    throw new InvalidOperationException("tmnf track: static grid too large");
                }
                slots[at] = (uint)start;
                lengths[at] = (uint)length;
                count += 1;
                return (uint)start;
            }
        }

        private struct PoolStaticNode : IPoolElement<PoolStaticNode>
        {
            public TmnfStaticCellNode node;

            public void FnvUpdate(ByteArrayWriter w)
            {
                PoolFnvFloat(w, node.boxAligned.center.x);
                PoolFnvFloat(w, node.boxAligned.center.y);
                PoolFnvFloat(w, node.boxAligned.center.z);
                PoolFnvFloat(w, node.boxAligned.halfExtent.x);
                PoolFnvFloat(w, node.boxAligned.halfExtent.y);
                PoolFnvFloat(w, node.boxAligned.halfExtent.z);
                w.U32(node.skip);
                w.U32(node.entryIndex);
            }

            public bool BytesEqual(PoolStaticNode other)
            {
                return FloatBitsEqual(node.boxAligned.center.x, other.node.boxAligned.center.x)
                    && FloatBitsEqual(node.boxAligned.center.y, other.node.boxAligned.center.y)
                    && FloatBitsEqual(node.boxAligned.center.z, other.node.boxAligned.center.z)
                    && FloatBitsEqual(node.boxAligned.halfExtent.x, other.node.boxAligned.halfExtent.x)
                    && FloatBitsEqual(node.boxAligned.halfExtent.y, other.node.boxAligned.halfExtent.y)
                    && FloatBitsEqual(node.boxAligned.halfExtent.z, other.node.boxAligned.halfExtent.z)
                    && node.skip == other.node.skip
                    && node.entryIndex == other.node.entryIndex;
            }
        }

        private struct PoolMeshSlot : IPoolElement<PoolMeshSlot>
        {
            public TmnfMeshPoolSlot slot;

            public void FnvUpdate(ByteArrayWriter w)
            {
                w.U32(slot.word0);
                w.U32(slot.word1);
            }

            public bool BytesEqual(PoolMeshSlot other)
            {
                return slot.word0 == other.slot.word0 && slot.word1 == other.slot.word1;
            }
        }

        private static bool BoxOverlaps(ref GmBoxAligned box, ref DBox region)
        {
            float[] c = { box.center.x, box.center.y, box.center.z };
            float[] h = { box.halfExtent.x, box.halfExtent.y, box.halfExtent.z };
            for (int a = 0; a < 3; a++)
            {
                if (!(((double)c[a] - (double)h[a] <= region.hi[a])
                    && (region.lo[a] <= (double)c[a] + (double)h[a])))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool BoxContains(ref GmBoxAligned box, ref DBox region, double margin)
        {
            float[] c = { box.center.x, box.center.y, box.center.z };
            float[] h = { box.halfExtent.x, box.halfExtent.y, box.halfExtent.z };
            for (int a = 0; a < 3; a++)
            {
                if (!((((double)c[a] - (double)h[a]) + margin <= region.lo[a])
                    && (region.hi[a] <= ((double)c[a] + (double)h[a]) - margin)))
                {
                    return false;
                }
            }
            return true;
        }

        private static double BoxReach(ref GmBoxAligned box)
        {
            float[] c = { box.center.x, box.center.y, box.center.z };
            float[] h = { box.halfExtent.x, box.halfExtent.y, box.halfExtent.z };
            double reach = 0.0;
            for (int a = 0; a < 3; a++)
            {
                double value = Math.Abs((double)c[a]) + (double)h[a];
                if (value > reach)
                {
                    reach = value;
                }
            }
            return reach;
        }

        private static double RoundingMargin(double reach)
        {
            return 0.05d + 1e-6d * reach;
        }

        [Serializable]
        private struct SkipFrame
        {
            public uint end;
            public int listIndex; // -1 = dropped
        }

        private static void CloseFrames<T>(
            SkipFrame[] frames, ref uint depth, uint sourceIndex,
            List<T> list, Action<int, uint> setSkip)
        {
            while (depth > 0 && sourceIndex >= frames[depth - 1].end)
            {
                SkipFrame frame = frames[depth - 1];
                depth -= 1;
                if (frame.listIndex != -1)
                {
                    int total = list.Count;
                    setSkip(frame.listIndex, (uint)(total - frame.listIndex));
                }
            }
        }

        private static void PushFrame(SkipFrame[] frames, ref uint depth, uint end, int listIndex)
        {
            if (depth >= 256)
            {
                throw new InvalidOperationException("tmnf track: collision tree deeper than 256");
            }
            frames[depth] = new SkipFrame { end = end, listIndex = listIndex };
            depth += 1;
        }

        private static bool IsoIsOrthonormal(ref GmIso4 iso)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double dot = 0.0;
                    for (int k = 0; k < 3; k++)
                    {
                        dot += (double)iso[k * 3 + i] * (double)iso[k * 3 + j];
                    }
                    if (Math.Abs(dot - (i == j ? 1.0d : 0.0d)) > GridOrthonormalTolerance)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        private static uint BuildMeshList(
            List<PoolMeshSlot> meshPool,
            ListIndex<PoolMeshSlot> meshLists,
            GmSurfMesh mesh,
            ref DBox region,
            double margin)
        {
            int start = meshPool.Count;
            meshPool.Add(default(PoolMeshSlot));

            SkipFrame[] frames = new SkipFrame[256];
            uint depth = 0;
            uint i = 0;
            while (i < mesh.nodeCount)
            {
                CloseFrames(frames, ref depth, i, meshPool, (idx, v) => { var e = meshPool[idx]; e.slot.word1 = v; meshPool[idx] = e; });
                GmSurfMeshNode source = mesh.nodes[(int)i];
                if (!BoxOverlaps(ref source.boxAligned, ref region))
                {
                    i = unchecked(i + source.skipCount);
                    continue;
                }
                bool isLeaf = source.faceIndex != uint.MaxValue;
                int listIndex = -1;
                if (isLeaf || !BoxContains(ref source.boxAligned, ref region, margin))
                {
                    PoolMeshSlot entry = default(PoolMeshSlot);
                    entry.slot.word0 = i;      // entry.node
                    entry.slot.word1 = 1;      // entry.skip
                    meshPool.Add(entry);
                    listIndex = meshPool.Count - 1;
                }
                if (source.skipCount > 1)
                {
                    PushFrame(frames, ref depth, unchecked(i + source.skipCount), listIndex);
                }
                i += 1;
            }
            CloseFrames(frames, ref depth, uint.MaxValue, meshPool, (idx, v) => { var e = meshPool[idx]; e.slot.word1 = v; meshPool[idx] = e; });

            uint nodeCount = (uint)(meshPool.Count - start - 1);
            PoolMeshSlot header = meshPool[start];
            header.slot.word0 = nodeCount; // header->node_count
            meshPool[start] = header;
            return meshLists.Intern(meshPool, start);
        }

        private static bool MeshReachesRegion(GmSurfMesh mesh, ref DBox region)
        {
            uint i = 0;
            while (i < mesh.nodeCount)
            {
                GmSurfMeshNode node = mesh.nodes[(int)i];
                if (!BoxOverlaps(ref node.boxAligned, ref region))
                {
                    i = unchecked(i + node.skipCount);
                    continue;
                }
                if (node.faceIndex != uint.MaxValue)
                {
                    return true;
                }
                i += 1;
            }
            return false;
        }

        private static TmnfMeshPoolSlot[] ToArray(List<PoolMeshSlot> list)
        {
            var r = new TmnfMeshPoolSlot[list.Count];
            for (int i = 0; i < list.Count; i++)
            {
                r[i] = list[i].slot;
            }
            return r;
        }

        private static TmnfMeshGrid BuildMeshGrid(
            List<List<PoolMeshSlot>> meshPools,
            ListIndex<PoolMeshSlot>[] meshLists,
            int meshIndex,
            GmSurfMesh mesh,
            double margin)
        {
            DBox bounds = NewDBox();
            bounds.lo[0] = double.PositiveInfinity; bounds.lo[1] = double.PositiveInfinity; bounds.lo[2] = double.PositiveInfinity;
            bounds.hi[0] = double.NegativeInfinity; bounds.hi[1] = double.NegativeInfinity; bounds.hi[2] = double.NegativeInfinity;
            for (int i = 0; i < (int)mesh.nodeCount; i++)
            {
                float[] c = { mesh.nodes[i].boxAligned.center.x, mesh.nodes[i].boxAligned.center.y, mesh.nodes[i].boxAligned.center.z };
                float[] h = { mesh.nodes[i].boxAligned.halfExtent.x, mesh.nodes[i].boxAligned.halfExtent.y, mesh.nodes[i].boxAligned.halfExtent.z };
                for (int a = 0; a < 3; a++)
                {
                    if ((double)c[a] - (double)h[a] < bounds.lo[a])
                    {
                        bounds.lo[a] = (double)c[a] - (double)h[a];
                    }
                    if ((double)c[a] + (double)h[a] > bounds.hi[a])
                    {
                        bounds.hi[a] = (double)c[a] + (double)h[a];
                    }
                }
            }

            TmnfMeshGrid grid = new TmnfMeshGrid();
            if (mesh.faceCount != 0)
            {
                var edges = new List<TmnfSphereFaceEdges>((int)mesh.faceCount);
                for (int i = 0; i < (int)mesh.faceCount; i++)
                {
                    GmSurfMeshFace face = mesh.faces[i];
                    TmnfSphereFaceEdges e = new TmnfSphereFaceEdges();
                    for (int v = 0; v < 4; v++)
                    {
                        GmVec3 vertex = mesh.vertices[(int)face.Vertex(v % 3)];
                        e.xyz[0][v] = vertex.x;
                        e.xyz[1][v] = vertex.y;
                        e.xyz[2][v] = vertex.z;
                    }
                    e.padding[0] = 0.0f;
                    e.padding[1] = 0.0f;
                    for (int ed = 0; ed < 3; ed++)
                    {
                        GmVec3 v0 = mesh.vertices[(int)face.Vertex(ed)];
                        GmVec3 v1 = mesh.vertices[(int)face.Vertex((ed + 1) % 3)];
                        GmVec3 normal = face.normal;
                        e.edges[ed] = CollisionHelpers.TmnfSphereMeshEdgePrepare(ref v0, ref v1, ref normal);
                    }
                    edges.Add(e);
                }
                grid.sphereEdges = edges.ToArray();
            }
            for (int l = 0; l < MeshGridConstants.TmnfMeshGridLevels; l++)
            {
                double expand = MeshGridExpand[l];
                double cellSize = MeshGridCell[l];
                uint[] dims = new uint[3];
                while (true)
                {
                    ulong total = 1;
                    for (int a = 0; a < 3; a++)
                    {
                        double extent = bounds.hi[a] - bounds.lo[a] + 2.0 * (expand + cellSize);
                        dims[a] = (uint)(Math.Floor(extent / cellSize)) + 1;
                        total = unchecked(total * dims[a]);
                    }
                    if (total <= MeshGridMaxCells)
                    {
                        break;
                    }
                    cellSize *= 2.0d;
                }
                TmnfMeshGridLevel level = new TmnfMeshGridLevel();
                level.cellSize = cellSize;
                level.invCellSize = 1.0d / cellSize;
                level.maxHalf = expand - margin;
                level.dims = (uint[])dims.Clone();
                level.cellCount = 0;
                uint cellCount = 1;
                for (int a = 0; a < 3; a++)
                {
                    level.origin[a] = bounds.lo[a] - expand - cellSize;
                    level.dims[a] = dims[a];
                    cellCount = unchecked(cellCount * dims[a]);
                }
                level.cellCount = cellCount;
                var cells = new List<uint>((int)cellCount);
                for (uint cell = 0; cell < cellCount; cell++)
                {
                    uint[] index =
                    {
                        cell % dims[0],
                        (cell / dims[0]) % dims[1],
                        cell / (dims[0] * dims[1]),
                    };
                    DBox region = NewDBox();
                    for (int a = 0; a < 3; a++)
                    {
                        region.lo[a] = level.origin[a] + index[a] * cellSize - expand;
                        region.hi[a] = region.lo[a] + cellSize + 2.0d * expand;
                    }
                    uint header = BuildMeshList(
                        meshPools[meshIndex], meshLists[meshIndex], mesh, ref region, margin);
                    cells.Add(header);
                }
                level.cells = cells.ToArray();
                grid.levels[l] = level;
            }
            grid.pool = ToArray(meshPools[meshIndex]);
            return grid;
        }

        private static int? EntryMeshGrid(
            List<List<PoolMeshSlot>> meshPools,
            ListIndex<PoolMeshSlot>[] meshLists,
            TmnfMeshGrid[] meshGrids,
            bool[] meshGridBuilt,
            double[] meshMargins,
            TmnfTrack track,
            List<int> surfaceMesh,
            HmsStaticCollisionEntry entry)
        {
            if (entry.surface < 0)
            {
                return null; // entry->surface == NULL cannot happen post-fixup
            }
            CPlugSurface surface = track.surfaces[entry.surface];
            if (!(surface.geom is GmSurfGeomMesh m))
            {
                return null; // surface->geom->type != GM_SURF_MESH
            }
            GmIso4 entryIso = entry.iso;
            if (!IsoIsOrthonormal(ref entryIso))
            {
                return null;
            }
            int meshIndex = surfaceMesh[entry.surface];
            if (!meshGridBuilt[meshIndex])
            {
                double margin = meshMargins[meshIndex];
                meshGrids[meshIndex] = BuildMeshGrid(meshPools, meshLists, meshIndex, m.mesh, margin);
                meshGridBuilt[meshIndex] = true;
            }
            return meshIndex;
        }

        private static bool EntryReachesRegion(
            TmnfTrack track, HmsStaticCollisionEntry entry, ref DBox worldRegion)
        {
            if (entry.surface < 0)
            {
                return true;
            }
            CPlugSurface surface = track.surfaces[entry.surface];
            if (!(surface.geom is GmSurfGeomMesh m))
            {
                return true;
            }
            GmIso4 iso = entry.iso;
            DBox region = NewDBox();
            region.lo[0] = double.PositiveInfinity; region.lo[1] = double.PositiveInfinity; region.lo[2] = double.PositiveInfinity;
            region.hi[0] = double.NegativeInfinity; region.hi[1] = double.NegativeInfinity; region.hi[2] = double.NegativeInfinity;
            for (uint corner = 0; corner < 8; corner++)
            {
                double[] w = new double[3];
                for (int a = 0; a < 3; a++)
                {
                    w[a] = ((corner >> a) & 1) != 0 ? worldRegion.hi[a] : worldRegion.lo[a];
                    w[a] -= (double)iso[9 + a];
                }
                for (int a = 0; a < 3; a++)
                {
                    double mm = (double)iso[0 * 3 + a] * w[0]
                        + (double)iso[1 * 3 + a] * w[1]
                        + (double)iso[2 * 3 + a] * w[2];
                    if (mm < region.lo[a])
                    {
                        region.lo[a] = mm;
                    }
                    if (mm > region.hi[a])
                    {
                        region.hi[a] = mm;
                    }
                }
            }
            for (int a = 0; a < 3; a++)
            {
                region.lo[a] -= GridMeshRegionMargin;
                region.hi[a] += GridMeshRegionMargin;
            }
            return MeshReachesRegion(m.mesh, ref region);
        }

        private static void PruneEmptyCellNodes(List<PoolStaticNode> nodes, int start)
        {
            uint count = (uint)(nodes.Count - start);
            uint[] keptBelow = new uint[count + 1];
            keptBelow[count] = 0;
            uint j = count;
            while (j > 0)
            {
                j -= 1;
                int skip = (int)nodes[start + (int)j].node.skip;
                bool keep;
                if (nodes[start + (int)j].node.entryIndex == CellNodeConstants.TmnfCellNodeInternal)
                {
                    keep = keptBelow[j + 1] - keptBelow[j + skip] > 0;
                }
                else
                {
                    keep = nodes[start + (int)j].node.entryIndex != CellNodeConstants.TmnfCellNodeEmpty;
                }
                keptBelow[j] = keptBelow[j + 1] + (keep ? 1u : 0u);
            }
            uint @out = 0;
            for (uint jj = 0; jj < count; jj++)
            {
                int j2 = (int)jj;
                if (keptBelow[j2] == keptBelow[j2 + 1])
                {
                    continue;
                }
                PoolStaticNode node = nodes[start + j2];
                int skip = (int)node.node.skip;
                node.node.skip = keptBelow[j2] - keptBelow[j2 + skip];
                nodes[start + (int)@out] = node;
                @out += 1;
            }
            nodes.RemoveRange(start + (int)@out, (int)(count - @out));
        }

        private static (uint, uint) BuildCellList(
            List<PoolStaticNode> nodes,
            ListIndex<PoolStaticNode> nodeLists,
            TmnfTrack track,
            ref DBox cell,
            double margin)
        {
            DBox region = NewDBox();
            DBox meshRegion = NewDBox();
            for (int a = 0; a < 3; a++)
            {
                region.lo[a] = cell.lo[a] - GridExpand;
                region.hi[a] = cell.hi[a] + GridExpand;
                meshRegion.lo[a] = cell.lo[a] - GridMeshExpand;
                meshRegion.hi[a] = cell.hi[a] + GridMeshExpand;
            }

            int start = nodes.Count;
            SkipFrame[] frames = new SkipFrame[256];
            uint depth = 0;
            uint i = 0;
            while (i < track.entryCount)
            {
                CloseFrames(frames, ref depth, i, nodes, (idx, v) => { var e = nodes[idx]; e.node.skip = v; nodes[idx] = e; });
                HmsStaticCollisionEntry entry = track.entries[(int)i];
                if (!BoxOverlaps(ref entry.boxAligned, ref region))
                {
                    i = unchecked(i + entry.skipCount);
                    continue;
                }
                bool isLeaf = entry.surface >= 0 && (entry.treeFlags & 0x80u) != 0;
                int listIndex = -1;
                if (isLeaf || !BoxContains(ref entry.boxAligned, ref region, margin))
                {
                    uint entryIndex;
                    if (isLeaf)
                    {
                        if (EntryReachesRegion(track, entry, ref meshRegion))
                        {
                            entryIndex = i;
                        }
                        else
                        {
                            entryIndex = CellNodeConstants.TmnfCellNodeEmpty;
                        }
                    }
                    else
                    {
                        entryIndex = CellNodeConstants.TmnfCellNodeInternal;
                    }
                    PoolStaticNode node = default(PoolStaticNode);
                    node.node.boxAligned = entry.boxAligned;
                    node.node.skip = 1;
                    node.node.entryIndex = entryIndex;
                    nodes.Add(node);
                    listIndex = nodes.Count - 1;
                }
                if (entry.skipCount > 1)
                {
                    PushFrame(frames, ref depth, unchecked(i + entry.skipCount), listIndex);
                }
                i += 1;
            }
            CloseFrames(frames, ref depth, uint.MaxValue, nodes, (idx, v) => { var e = nodes[idx]; e.node.skip = v; nodes[idx] = e; });
            PruneEmptyCellNodes(nodes, start);
            uint count = (uint)(nodes.Count - start);
            uint offset = nodeLists.Intern(nodes, start);
            return (offset, count);
        }

        private static void BuildStaticGrid(TmnfTrack track, List<int> surfaceMesh)
        {
            DBox bounds = NewDBox();
            bounds.lo[0] = double.PositiveInfinity; bounds.lo[1] = double.PositiveInfinity; bounds.lo[2] = double.PositiveInfinity;
            bounds.hi[0] = double.NegativeInfinity; bounds.hi[1] = double.NegativeInfinity; bounds.hi[2] = double.NegativeInfinity;
            double reach = 0.0;
            uint boundedLeaves = 0;
            for (int ei = 0; ei < track.entries.Count; ei++)
            {
                HmsStaticCollisionEntry entry = track.entries[ei];
                double value = BoxReach(ref entry.boxAligned);
                if (value > reach)
                {
                    reach = value;
                }
                if (entry.surface < 0 || (entry.treeFlags & 0x80u) == 0)
                {
                    continue;
                }
                float[] c = { entry.boxAligned.center.x, entry.boxAligned.center.y, entry.boxAligned.center.z };
                float[] h = { entry.boxAligned.halfExtent.x, entry.boxAligned.halfExtent.y, entry.boxAligned.halfExtent.z };
                if ((double)h[0] > GridBoundsLeafHalf
                    || (double)h[1] > GridBoundsLeafHalf
                    || (double)h[2] > GridBoundsLeafHalf)
                {
                    continue;
                }
                boundedLeaves += 1;
                for (int a = 0; a < 3; a++)
                {
                    if ((double)c[a] - (double)h[a] < bounds.lo[a])
                    {
                        bounds.lo[a] = (double)c[a] - (double)h[a];
                    }
                    if ((double)c[a] + (double)h[a] > bounds.hi[a])
                    {
                        bounds.hi[a] = (double)c[a] + (double)h[a];
                    }
                }
            }
            if (boundedLeaves == 0)
            {
                return; // the grid stays default
            }
            if (!double.IsFinite(reach))
            {
                throw new InvalidOperationException("tmnf track: static collision tree has non-finite boxes");
            }

            double cellSize = GridCell;
            uint[] dims = new uint[3];
            while (true)
            {
                ulong total = 1;
                for (int a = 0; a < 3; a++)
                {
                    double extent = bounds.hi[a] - bounds.lo[a];
                    dims[a] = (uint)(Math.Floor(extent / cellSize)) + 1;
                    total = unchecked(total * dims[a]);
                }
                if (total <= GridMaxCells)
                {
                    break;
                }
                cellSize *= 2.0d;
            }

            TmnfStaticGrid grid = new TmnfStaticGrid();
            grid.cellSize = cellSize;
            grid.invCellSize = 1.0d / cellSize;
            grid.expand = GridExpand;
            grid.margin = RoundingMargin(reach);
            grid.dims = dims;
            grid.cellCount = unchecked(dims[0] * dims[1] * dims[2]);
            for (int a = 0; a < 3; a++)
            {
                grid.origin[a] = bounds.lo[a];
                grid.dims[a] = dims[a];
            }

            int meshCount = track.meshes.Count;
            var nodes = new List<PoolStaticNode>();
            var nodeLists = new ListIndex<PoolStaticNode>();
            var meshPools = new List<List<PoolMeshSlot>>();
            var meshLists = new ListIndex<PoolMeshSlot>[meshCount];
            for (int m = 0; m < meshCount; m++)
            {
                meshPools.Add(new List<PoolMeshSlot>());
                meshLists[m] = new ListIndex<PoolMeshSlot>();
            }
            var meshMargins = new double[meshCount];
            var meshGrids = new TmnfMeshGrid[meshCount];
            var meshGridBuilt = new bool[meshCount];

            for (int m = 0; m < meshCount; m++)
            {
                GmSurfMesh mesh = track.meshes[m];
                double meshReach = 0.0;
                for (int i = 0; i < (int)mesh.nodeCount; i++)
                {
                    double value = BoxReach(ref mesh.nodes[i].boxAligned);
                    if (value > meshReach)
                    {
                        meshReach = value;
                    }
                }
                if (!double.IsFinite(meshReach))
                {
                    throw new InvalidOperationException("tmnf track: mesh collision tree has non-finite boxes");
                }
                meshMargins[m] = RoundingMargin(meshReach);
            }

            var entryMeshGrids = new List<int?>(track.entries.Count);
            var entryInverseIsos = new List<GmIso4>(track.entries.Count);
            for (int ei = 0; ei < track.entries.Count; ei++)
            {
                HmsStaticCollisionEntry entry = track.entries[ei];
                GmIso4 inverse = default(GmIso4);
                GmIso4 entryIso = entry.iso;
                inverse.SetInverse(ref entryIso);
                entryInverseIsos.Add(inverse);
                int? mg = entry.surface >= 0 && (entry.treeFlags & 0x80u) != 0
                    ? EntryMeshGrid(meshPools, meshLists, meshGrids, meshGridBuilt, meshMargins, track, surfaceMesh, entry)
                    : null;
                entryMeshGrids.Add(mg);
            }

            var cellOffsets = new uint[grid.cellCount];
            var cellCounts = new uint[grid.cellCount];
            for (uint cell = 0; cell < grid.cellCount; cell++)
            {
                uint[] index =
                {
                    cell % dims[0],
                    (cell / dims[0]) % dims[1],
                    cell / (dims[0] * dims[1]),
                };
                DBox cellBox = NewDBox();
                for (int a = 0; a < 3; a++)
                {
                    cellBox.lo[a] = grid.origin[a] + index[a] * cellSize;
                    cellBox.hi[a] = cellBox.lo[a] + cellSize;
                }
                var (offset, count) = BuildCellList(nodes, nodeLists, track, ref cellBox, grid.margin);
                cellOffsets[cell] = offset;
                cellCounts[cell] = count;
            }

            int gridMeshSlotCount = 0;
            for (int m = 0; m < meshCount; m++)
            {
                gridMeshSlotCount += meshPools[m].Count;
            }
            grid.cellOffsets = cellOffsets;
            grid.cellCounts = cellCounts;
            var nodeArray = new TmnfStaticCellNode[nodes.Count];
            for (int i = 0; i < nodes.Count; i++)
            {
                nodeArray[i] = nodes[i].node;
            }
            grid.nodes = nodeArray;
            grid.meshPool = new TmnfMeshPoolSlot[0]; // per-mesh pools live in each grid
            grid.meshGrids = meshGrids;
            grid.entryMeshGrids = entryMeshGrids.ToArray();
            grid.entryInverseIsos = entryInverseIsos.ToArray();
            track.gridNodeCount = nodeArray.Length;
            track.gridMeshSlotCount = gridMeshSlotCount;
            track.grid = grid;
        }

        private static void BuildStaticResponseBodies(TmnfTrack track)
        {
            uint maximum = 0;
            for (int i = 0; i < track.entries.Count; i++)
            {
                if (maximum < track.entries[i].corpusRef)
                {
                    maximum = track.entries[i].corpusRef;
                }
            }
            track.staticResponseCount = unchecked(maximum + 1);
            int count = (int)track.staticResponseCount;
            var bodies = new List<CHmsResponseBody>(count);
            var present = new List<byte>(count);
            for (int i = 0; i < count; i++)
            {
                // calloc-zeroed; NULL dyna / no contact sink.
                CHmsResponseBody b = default(CHmsResponseBody);
                b.iso = default(GmIso4);
                bodies.Add(b);
                present.Add(0);
            }
            for (int i = 0; i < track.entries.Count; i++)
            {
                uint id = track.entries[i].corpusRef;
                if (id == 0 || present[(int)id] != 0)
                {
                    continue;
                }
                CHmsResponseBody body = bodies[(int)id];
                present[(int)id] = 1;
                body.corpusRef = id;
                body.classificationFlags = 0x11888101u;
                body.responseFlags = 0xfff18002u;
                body.responseWeight = 1.0f;
                body.iso = track.corpusIsos[(int)id - 1];
                bodies[(int)id] = body;
            }
            track.staticResponseBodies = bodies;
            track.staticResponsePresent = present;
        }

        /// <summary>TmnfTrack_Load: loads and validates a TMNFTRK1 snapshot.</summary>
        public static TmnfTrack TmnfTrackLoad(string path, byte[] expectedTrackSha256)
        {
            byte[] data;
            try
            {
                data = System.IO.File.ReadAllBytes(path);
            }
            catch (Exception)
            {
                throw new InvalidOperationException("tmnf track: cannot open snapshot");
            }
            return TmnfTrackLoadBytes(data, expectedTrackSha256);
        }

        /// <summary>
        /// TmnfTrack_Load over an in-memory image — the same validation and
        /// fixup pipeline as the path entry point, for callers that build
        /// the bytes themselves (see Tmnf.Sim.Fixtures.TmnfFixtures).
        /// </summary>
        public static TmnfTrack TmnfTrackLoadBytes(byte[] data, byte[] expectedTrackSha256)
        {
            if (data.Length < 0x150)
            {
                throw new InvalidOperationException("tmnf track: invalid snapshot size");
            }
            int size = data.Length;

            TmnfTrackHeader header = ParseTrackHeader(data);
            if (!IsMagic(header.magic, "TMNFTRK1")
                || header.version != TmnfTrackVersion
                || header.endian != 0x1234_5678
                || header.headerSize != 0x150
                || header.sectionCount != TmnfTrackSectionCount
                || header.fileSize != (ulong)size)
            {
                throw new InvalidOperationException("tmnf track: invalid snapshot header");
            }
            if (!BytesEqual(header.exeSha256, Tmnf21126ExeSha256))
            {
                throw new InvalidOperationException("tmnf track: snapshot targets a different executable");
            }
            if (expectedTrackSha256 != null && !BytesEqual(header.trackSha256, expectedTrackSha256))
            {
                throw new InvalidOperationException("tmnf track: snapshot is for a different track");
            }
            ValidateSections(header, size);
            if (header.sections[TmnfTrackMaterialData].count != TmnfTrackMaterialCount)
            {
                throw new InvalidOperationException("tmnf track: physical material table must contain 31 entries");
            }

            byte[] payload = new byte[size - 0x150];
            Buffer.BlockCopy(data, 0x150, payload, 0, payload.Length);
            if (!BytesEqual(Sha256.Digest(payload), header.payloadSha256))
            {
                throw new InvalidOperationException("tmnf track: payload SHA-256 mismatch");
            }

            List<GmSurfMesh> meshes = FixMeshes(data, header);
            var (surfaces, surfaceMesh) = FixSurfaces(data, header, meshes);
            List<HmsStaticCollisionEntry> entries = FixEntries(data, header);

            var materialsSection = header.sections[TmnfTrackMaterialData];
            var materials = new List<CPlugSurfaceMaterialData>((int)materialsSection.count);
            {
                Reader r = Reader.At(data, (int)materialsSection.offset);
                for (uint i = 0; i < materialsSection.count; i++)
                {
                    CPlugSurfaceMaterialData d = default(CPlugSurfaceMaterialData);
                    d.friction = r.F32();
                    d.restitution = r.F32();
                    materials.Add(d);
                }
            }
            var pairsSection = header.sections[TmnfTrackCollisionPairs];
            var collisionPairs = new List<TmnfTrackCollisionPair>((int)pairsSection.count);
            {
                Reader r = Reader.At(data, (int)pairsSection.offset);
                for (uint i = 0; i < pairsSection.count; i++)
                {
                    TmnfTrackCollisionPair p = default(TmnfTrackCollisionPair);
                    p.raw = new[] { r.U32(), r.U32(), r.U32(), r.U32(), r.U32() };
                    collisionPairs.Add(p);
                }
            }
            var corpusSection = header.sections[TmnfTrackCorpusIsos];
            var corpusIsos = new List<GmIso4>((int)corpusSection.count);
            {
                Reader r = Reader.At(data, (int)corpusSection.offset);
                for (uint i = 0; i < corpusSection.count; i++)
                {
                    corpusIsos.Add(r.GmIso4());
                }
            }

            TmnfTrack track = new TmnfTrack();
            track.entryCount = header.sections[TmnfTrackEntries].count;
            track.surfaceCount = header.sections[TmnfTrackSurfaces].count;
            track.meshCount = header.sections[TmnfTrackMeshes].count;
            track.collisionPairCount = header.sections[TmnfTrackCollisionPairs].count;
            track.corpusCount = header.sections[TmnfTrackCorpusIsos].count;
            track.header = header;
            track.entries = entries;
            track.surfaces = surfaces;
            track.meshes = meshes;
            track.materials = materials;
            track.collisionPairs = collisionPairs;
            track.corpusIsos = corpusIsos;

            for (uint i = 0; i < track.entryCount; i++)
            {
                if (track.entries[(int)i].corpusRef > track.corpusCount)
                {
                    throw new InvalidOperationException("tmnf track: static entry references a missing corpus location");
                }
            }

            track.water = DeriveWater(data, track.header);
            BuildStaticGrid(track, surfaceMesh);
            BuildStaticResponseBodies(track);
            return track;
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
}
