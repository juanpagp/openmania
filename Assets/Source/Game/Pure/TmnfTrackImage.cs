using System;
using System.Collections.Generic;

namespace Tmnf.Game.Pure
{
    /// <summary>
    /// Builds a TMNFTRK1 track image (the snapshot format
    /// TrackLoader.TmnfTrackLoadBytes consumes) from a <see cref="GameTrackSpec"/>.
    ///
    /// This is the collision bridge between Unity scene geometry and the
    /// custom simulation: every mesh becomes one GmSurfMesh (with a binary
    /// BVH over its faces), one CPlugSurface, and one static collision entry
    /// with a world-space AABB (identity entry iso — the vertices are
    /// pre-transformed, which also keeps the per-mesh query grids valid:
    /// they require orthonormal entry isos). A top-level BVH over the entry
    /// list (pre-order skip-count layout, exactly the traversal contract the
    /// static scan implements) accelerates broad-phase culling, and the
    /// loader's own static grid builder runs on top of the produced image —
    /// so imported meshes get the same acceleration structure the original
    /// game's tracks use, without any manual conversion step.
    ///
    /// Pure data in, bytes out: no UnityEngine dependency, no file IO. The
    /// layout follows the on-disk contract of the reference loader exactly
    /// (section table, strides, hashes, validation rules) — see
    /// TmnfFixtures.BuildTrack in the core, which builds the golden fixture
    /// with the same layout and is verified byte-for-byte by the test suite.
    /// </summary>
    public static class TmnfTrackImage
    {
        /// <summary>The physical material table size (loader-fixed).</summary>
        public const int MaterialCount = 31;

        /// <summary>The canonical TMNF 2.11.26 executable SHA-256.</summary>
        private static readonly byte[] ExeSha256 = Tmnf.Sim.TrackLoader.Tmnf21126ExeSha256;

        // ------------------------------------------------------------------
        // public API
        // ------------------------------------------------------------------

        /// <summary>
        /// Builds the TMNFTRK1 bytes. Throws <see cref="InvalidOperationException"/>
        /// with a precise message when the spec cannot produce a valid image
        /// (empty track, non-finite vertices, degenerate triangles only, ...).
        /// </summary>
        public static byte[] Build(GameTrackSpec spec)
        {
            if (spec == null)
            {
                throw new InvalidOperationException("track image: no track spec");
            }
            if (spec.meshes == null || spec.meshes.Count == 0)
            {
                throw new InvalidOperationException(
                    "track image: the track has no collidable meshes");
            }
            if (spec.meshes.Count > 100000)
            {
                throw new InvalidOperationException(
                    "track image: unreasonable mesh count (> 100000)");
            }
            foreach (GameMeshSpec mesh in spec.meshes)
            {
                ValidateMesh(mesh);
            }

            List<BakedMesh> baked = new List<BakedMesh>(spec.meshes.Count);
            foreach (GameMeshSpec mesh in spec.meshes)
            {
                baked.Add(BakeMesh(mesh));
            }

            // The static entry list: one BVH over one leaf per mesh.
            List<EntryNode> entries = BuildEntryBvh(baked);

            return WriteImage(baked, entries, spec.friction, spec.restitution);
        }

        /// <summary>
        /// Fills the 31-entry physical material table with the reference
        /// values (same table the golden fixtures use).
        /// </summary>
        public static void DefaultMaterialTable(float[] friction, float[] restitution)
        {
            if (friction == null || restitution == null
                || friction.Length != MaterialCount || restitution.Length != MaterialCount)
            {
                throw new InvalidOperationException(
                    "track image: material tables must hold 31 entries");
            }
            for (int i = 0; i < MaterialCount; i++)
            {
                friction[i] = 1.0f;
                restitution[i] = 0.5f;
            }
            friction[0] = 1.0f; restitution[0] = 0.5f;       // concrete
            friction[1] = 1.0f; restitution[1] = 0.4f;       // pavement
            friction[2] = 0.6f; restitution[2] = 0.5f;      // grass
            friction[3] = 0.15f; restitution[3] = 0.1f;      // ice
            friction[4] = 1.0f; restitution[4] = 0.3f;      // metal
            friction[5] = 0.7f; restitution[5] = 0.4f;      // sand
            friction[6] = 0.8f; restitution[6] = 0.4f;      // dirt
            friction[7] = 1.0f; restitution[7] = 0.5f;      // turbo
            friction[8] = 0.9f; restitution[8] = 0.4f;       // dirt road
            friction[9] = 1.2f; restitution[9] = 0.2f;       // rubber
            friction[10] = 0.9f; restitution[10] = 0.2f;     // sliding rubber
            friction[11] = 1.0f; restitution[11] = 0.5f;     // test
            friction[12] = 1.0f; restitution[12] = 0.3f;     // rock
            friction[13] = 0.5f; restitution[13] = 0.0f;     // water
            friction[14] = 0.9f; restitution[14] = 0.4f;     // wood
            friction[15] = 1.0f; restitution[15] = 1.0f;    // danger
            friction[16] = 1.0f; restitution[16] = 0.5f;     // asphalt
            friction[17] = 0.8f; restitution[17] = 0.4f;     // wet dirt road
            friction[18] = 0.9f; restitution[18] = 0.4f;      // wet asphalt
            friction[19] = 0.9f; restitution[19] = 0.4f;     // wet pavement
            friction[20] = 0.55f; restitution[20] = 0.45f;   // wet grass
            friction[21] = 0.4f; restitution[21] = 0.3f;    // snow
            friction[22] = 1.0f; restitution[22] = 0.3f;     // resonant metal
            friction[23] = 1.0f; restitution[23] = 0.9f;    // golf ball
            friction[24] = 1.0f; restitution[24] = 0.5f;     // golf wall
            friction[25] = 1.0f; restitution[25] = 0.5f;     // golf ground
            friction[26] = 1.0f; restitution[26] = 0.5f;     // turbo2
            friction[27] = 1.3f; restitution[27] = 0.8f;     // bumper
            friction[28] = 1.0f; restitution[28] = 0.5f;    // not collidable
            friction[29] = 1.0f; restitution[29] = 0.5f;     // free wheeling
            friction[30] = 1.0f; restitution[30] = 0.5f;    // turbo roulette
        }

        // ------------------------------------------------------------------
        // validation + mesh bake
        // ------------------------------------------------------------------

        private static void ValidateMesh(GameMeshSpec mesh)
        {
            if (mesh == null)
            {
                throw new InvalidOperationException("track image: null mesh in spec");
            }
            int vertexCount = mesh.VertexCount;
            int faceCount = mesh.FaceCount;
            if (vertexCount == 0 || faceCount == 0)
            {
                throw new InvalidOperationException($"track image: mesh '{mesh.name}' has no triangles");
            }
            if (mesh.faceNormals == null || mesh.faceNormals.Length != faceCount * 3)
            {
                throw new InvalidOperationException(
                    $"track image: mesh '{mesh.name}' is missing one unit normal per face");
            }
            if (mesh.materialId >= MaterialCount)
            {
                throw new InvalidOperationException(
                    $"track image: mesh '{mesh.name}' has material id {mesh.materialId} outside 0..30");
            }
            for (int i = 0; i < mesh.vertices.Length; i++)
            {
                if (!float.IsFinite(mesh.vertices[i]))
                {
                    throw new InvalidOperationException(
                        $"track image: mesh '{mesh.name}' has a non-finite vertex");
                }
            }
            for (int f = 0; f < faceCount; f++)
            {
                for (int k = 0; k < 3; k++)
                {
                    if (mesh.triangles[f * 3 + k] >= vertexCount)
                    {
                        throw new InvalidOperationException(
                            $"track image: mesh '{mesh.name}' face {f} references vertex "
                            + $"{mesh.triangles[f * 3 + k]} outside the vertex array");
                    }
                }
            }
        }

        /// <summary>A mesh with its face BVH baked and its AABB computed.</summary>
        private sealed class BakedMesh
        {
            public GameMeshSpec source;
            public float boxCx;
            public float boxCy;
            public float boxCz;
            public float boxHx;
            public float boxHy;
            public float boxHz;
            public MeshBvhNode[] bvh;
            public int bvhNodeCount;
        }

        private struct MeshBvhNode
        {
            public float cx;
            public float cy;
            public float cz;
            public float hx;
            public float hy;
            public float hz;
            public uint faceIndex; // 0xFFFFFFFF = internal
            public uint skipCount;
        }

        private static BakedMesh BakeMesh(GameMeshSpec mesh)
        {
            int faceCount = mesh.FaceCount;
            // Face boxes feed both the BVH and the leaf node records.
            float[] faceBox = new float[faceCount * 6];
            float[] centroid = new float[faceCount * 3];
            for (int f = 0; f < faceCount; f++)
            {
                uint a = mesh.triangles[f * 3 + 0];
                uint b = mesh.triangles[f * 3 + 1];
                uint c = mesh.triangles[f * 3 + 2];
                float ax = mesh.vertices[a * 3 + 0];
                float ay = mesh.vertices[a * 3 + 1];
                float az = mesh.vertices[a * 3 + 2];
                float bx = mesh.vertices[b * 3 + 0];
                float by = mesh.vertices[b * 3 + 1];
                float bz = mesh.vertices[b * 3 + 2];
                float cx = mesh.vertices[c * 3 + 0];
                float cy = mesh.vertices[c * 3 + 1];
                float cz = mesh.vertices[c * 3 + 2];
                float lox = Math.Min(ax, Math.Min(bx, cx));
                float loy = Math.Min(ay, Math.Min(by, cy));
                float loz = Math.Min(az, Math.Min(bz, cz));
                float hix = Math.Max(ax, Math.Max(bx, cx));
                float hiy = Math.Max(ay, Math.Max(by, cy));
                float hiz = Math.Max(az, Math.Max(bz, cz));
                faceBox[f * 6 + 0] = lox;
                faceBox[f * 6 + 1] = loy;
                faceBox[f * 6 + 2] = loz;
                faceBox[f * 6 + 3] = hix;
                faceBox[f * 6 + 4] = hiy;
                faceBox[f * 6 + 5] = hiz;
                centroid[f * 3 + 0] = (lox + hix) * 0.5f;
                centroid[f * 3 + 1] = (loy + hiy) * 0.5f;
                centroid[f * 3 + 2] = (loz + hiz) * 0.5f;
            }

            List<MeshBvhNode> nodes = new List<MeshBvhNode>(faceCount * 2);
            int[] faces = new int[faceCount];
            for (int f = 0; f < faceCount; f++)
            {
                faces[f] = f;
            }
            BuildMeshBvh(mesh, faceBox, centroid, faces, 0, faceCount, nodes);

            BakedMesh baked = new BakedMesh();
            baked.source = mesh;
            baked.bvh = nodes.ToArray();
            baked.bvhNodeCount = nodes.Count;
            float loX = float.PositiveInfinity;
            float loY = float.PositiveInfinity;
            float loZ = float.PositiveInfinity;
            float hiX = float.NegativeInfinity;
            float hiY = float.NegativeInfinity;
            float hiZ = float.NegativeInfinity;
            for (int f = 0; f < faceCount; f++)
            {
                loX = Math.Min(loX, faceBox[f * 6 + 0]);
                loY = Math.Min(loY, faceBox[f * 6 + 1]);
                loZ = Math.Min(loZ, faceBox[f * 6 + 2]);
                hiX = Math.Max(hiX, faceBox[f * 6 + 3]);
                hiY = Math.Max(hiY, faceBox[f * 6 + 4]);
                hiZ = Math.Max(hiZ, faceBox[f * 6 + 5]);
            }
            baked.boxCx = (loX + hiX) * 0.5f;
            baked.boxCy = (loY + hiY) * 0.5f;
            baked.boxCz = (loZ + hiZ) * 0.5f;
            baked.boxHx = (hiX - loX) * 0.5f;
            baked.boxHy = (hiY - loY) * 0.5f;
            baked.boxHz = (hiZ - loZ) * 0.5f;
            return baked;
        }

        /// <summary>
        /// Median-split binary BVH over [from, to) of the face list, emitted
        /// pre-order with skip counts (the loader's mesh octree contract:
        /// no overlap -> advance by skipCount, internal -> descend).
        /// </summary>
        private static void BuildMeshBvh(
            GameMeshSpec mesh, float[] faceBox, float[] centroid,
            int[] faces, int from, int to, List<MeshBvhNode> nodes)
        {
            int nodeIndex = nodes.Count;
            nodes.Add(default(MeshBvhNode));

            float loX = float.PositiveInfinity;
            float loY = float.PositiveInfinity;
            float loZ = float.PositiveInfinity;
            float hiX = float.NegativeInfinity;
            float hiY = float.NegativeInfinity;
            float hiZ = float.NegativeInfinity;
            for (int i = from; i < to; i++)
            {
                int f = faces[i];
                loX = Math.Min(loX, faceBox[f * 6 + 0]);
                loY = Math.Min(loY, faceBox[f * 6 + 1]);
                loZ = Math.Min(loZ, faceBox[f * 6 + 2]);
                hiX = Math.Max(hiX, faceBox[f * 6 + 3]);
                hiY = Math.Max(hiY, faceBox[f * 6 + 4]);
                hiZ = Math.Max(hiZ, faceBox[f * 6 + 5]);
            }
            MeshBvhNode node = new MeshBvhNode();
            node.cx = (loX + hiX) * 0.5f;
            node.cy = (loY + hiY) * 0.5f;
            node.cz = (loZ + hiZ) * 0.5f;
            node.hx = (hiX - loX) * 0.5f;
            node.hy = (hiY - loY) * 0.5f;
            node.hz = (hiZ - loZ) * 0.5f;

            int count = to - from;
            if (count == 1)
            {
                node.faceIndex = (uint)faces[from];
                node.skipCount = 1;
                nodes[nodeIndex] = node;
                return;
            }

            node.faceIndex = uint.MaxValue;
            float ex = hiX - loX;
            float ey = hiY - loY;
            float ez = hiZ - loZ;
            int axis = ex >= ey ? (ex >= ez ? 0 : 2) : (ey >= ez ? 1 : 2);
            Array.Sort(faces, from, to - from, Comparer<int>.Create((a, b) =>
            {
                float delta = centroid[a * 3 + axis] - centroid[b * 3 + axis];
                return delta < 0.0f ? -1 : (delta > 0.0f ? 1 : 0);
            }));
            int mid = from + count / 2;
            BuildMeshBvh(mesh, faceBox, centroid, faces, from, mid, nodes);
            BuildMeshBvh(mesh, faceBox, centroid, faces, mid, to, nodes);

            // skipCount = this node's whole subtree size in pre-order.
            node.skipCount = (uint)(nodes.Count - nodeIndex);
            nodes[nodeIndex] = node;
        }

        // ------------------------------------------------------------------
        // static entry BVH
        // ------------------------------------------------------------------

        private struct EntryNode
        {
            public float cx;
            public float cy;
            public float cz;
            public float hx;
            public float hy;
            public float hz;
            public int surfaceIndex; // -1 = internal BVH node
            public uint skipCount;
        }

        private static List<EntryNode> BuildEntryBvh(List<BakedMesh> meshes)
        {
            // Entry boxes are the EXACT mesh AABBs — no skin. The reference
            // ground tiles are razor-thin on Y by design: the static scan
            // tests entry boxes against the vehicle nodes' boxes, and a
            // thin ground box is what keeps the car's tall front body
            // ellipsoid from being tested against flat ground at normal
            // ride height (it only touches when the nose is genuinely
            // pitched in — the TMNF nose-scrape). Inflating the boxes
            // "for safety" lets that query through and the resulting
            // deep penetration launches the car.
            const float skin = 0.0f;

            List<EntryNode> nodes = new List<EntryNode>(meshes.Count * 2 + 1);
            int[] leaves = new int[meshes.Count];
            for (int i = 0; i < meshes.Count; i++)
            {
                leaves[i] = i;
            }
            if (meshes.Count == 1)
            {
                // Keep the canonical shape even for a single mesh: a root
                // entry (no tree flags) wrapping one collidable leaf. The
                // static scan and the grid cell builder both expect a
                // leading root that is not itself a leaf.
                BakedMesh m = meshes[0];
                EntryNode root = new EntryNode();
                root.cx = m.boxCx;
                root.cy = m.boxCy;
                root.cz = m.boxCz;
                root.hx = m.boxHx + skin;
                root.hy = m.boxHy + skin;
                root.hz = m.boxHz + skin;
                root.surfaceIndex = -1;
                root.skipCount = 2;
                nodes.Add(root);
                AddEntryLeaf(nodes, meshes, 0, skin);
                return nodes;
            }

            BuildEntryBvhRecursive(meshes, leaves, 0, meshes.Count, nodes, skin);
            return nodes;
        }

        private static void AddEntryLeaf(
            List<EntryNode> nodes, List<BakedMesh> meshes, int index, float skin)
        {
            BakedMesh m = meshes[index];
            EntryNode leaf = new EntryNode();
            leaf.cx = m.boxCx;
            leaf.cy = m.boxCy;
            leaf.cz = m.boxCz;
            leaf.hx = m.boxHx + skin;
            leaf.hy = m.boxHy + skin;
            leaf.hz = m.boxHz + skin;
            leaf.surfaceIndex = index;
            leaf.skipCount = 1;
            nodes.Add(leaf);
        }

        private static void BuildEntryBvhRecursive(
            List<BakedMesh> meshes, int[] leaves, int from, int to,
            List<EntryNode> nodes, float skin)
        {
            int nodeIndex = nodes.Count;
            nodes.Add(default(EntryNode));

            float loX = float.PositiveInfinity;
            float loY = float.PositiveInfinity;
            float loZ = float.PositiveInfinity;
            float hiX = float.NegativeInfinity;
            float hiY = float.NegativeInfinity;
            float hiZ = float.NegativeInfinity;
            for (int i = from; i < to; i++)
            {
                BakedMesh m = meshes[leaves[i]];
                loX = Math.Min(loX, m.boxCx - m.boxHx);
                loY = Math.Min(loY, m.boxCy - m.boxHy);
                loZ = Math.Min(loZ, m.boxCz - m.boxHz);
                hiX = Math.Max(hiX, m.boxCx + m.boxHx);
                hiY = Math.Max(hiY, m.boxCy + m.boxHy);
                hiZ = Math.Max(hiZ, m.boxCz + m.boxHz);
            }
            EntryNode node = new EntryNode();
            node.cx = (loX + hiX) * 0.5f;
            node.cy = (loY + hiY) * 0.5f;
            node.cz = (loZ + hiZ) * 0.5f;
            node.hx = (hiX - loX) * 0.5f;
            node.hy = (hiY - loY) * 0.5f;
            node.hz = (hiZ - loZ) * 0.5f;

            int count = to - from;
            if (count == 1)
            {
                BakedMesh m = meshes[leaves[from]];
                node.cx = m.boxCx;
                node.cy = m.boxCy;
                node.cz = m.boxCz;
                node.hx = m.boxHx + skin;
                node.hy = m.boxHy + skin;
                node.hz = m.boxHz + skin;
                node.surfaceIndex = leaves[from];
                node.skipCount = 1;
                nodes[nodeIndex] = node;
                return;
            }

            node.surfaceIndex = -1;
            float ex = hiX - loX;
            float ey = hiY - loY;
            float ez = hiZ - loZ;
            int axis = ex >= ey ? (ex >= ez ? 0 : 2) : (ey >= ez ? 1 : 2);
            Array.Sort(leaves, from, to - from, Comparer<int>.Create((a, b) =>
            {
                BakedMesh ma = meshes[a];
                BakedMesh mb = meshes[b];
                float ca = axis == 0 ? ma.boxCx : axis == 1 ? ma.boxCy : ma.boxCz;
                float cb = axis == 0 ? mb.boxCx : axis == 1 ? mb.boxCy : mb.boxCz;
                float delta = ca - cb;
                return delta < 0.0f ? -1 : (delta > 0.0f ? 1 : 0);
            }));
            int mid = from + count / 2;
            BuildEntryBvhRecursive(meshes, leaves, from, mid, nodes, skin);
            BuildEntryBvhRecursive(meshes, leaves, mid, to, nodes, skin);

            node.skipCount = (uint)(nodes.Count - nodeIndex);
            nodes[nodeIndex] = node;
        }

        // ------------------------------------------------------------------
        // serialization
        // ------------------------------------------------------------------

        private const int HeaderSize = 0x150;
        private const int SectionCount = 12;

        private static byte[] WriteImage(
            List<BakedMesh> meshes, List<EntryNode> entries,
            float[] friction, float[] restitution)
        {
            int meshCount = meshes.Count;

            // -- section layout (all offsets 8-byte aligned, after 0x150) --
            int cursor = HeaderSize;
            int entriesOff = Align(ref cursor);
            cursor += entries.Count * 0x60;
            int surfacesOff = Align(ref cursor);
            cursor += meshCount * 0x18;
            int meshesOff = Align(ref cursor);
            cursor += meshCount * 0x38;
            int verticesOff = Align(ref cursor);
            int vertexTotal = 0;
            foreach (BakedMesh m in meshes)
            {
                vertexTotal += m.source.VertexCount;
            }
            cursor += vertexTotal * 12;
            int facesOff = Align(ref cursor);
            int faceTotal = 0;
            foreach (BakedMesh m in meshes)
            {
                faceTotal += m.source.FaceCount;
            }
            cursor += faceTotal * 0x20;
            int nodesOff = Align(ref cursor);
            int nodeTotal = 0;
            foreach (BakedMesh m in meshes)
            {
                nodeTotal += m.bvhNodeCount;
            }
            cursor += nodeTotal * 0x20;
            int materialIdsOff = Align(ref cursor);
            cursor += meshCount; // one remapped material id per surface
            int materialDataOff = Align(ref cursor);
            cursor += MaterialCount * 8;
            int pairsOff = Align(ref cursor);
            cursor += 3 * 0x14;
            int corpusOff = Align(ref cursor);
            cursor += 1 * 48; // one corpus iso (identity)
            int waterOff = Align(ref cursor);
            cursor += 0x28;
            int cellsOff = Align(ref cursor);
            cursor += 4 * 4; // 4x4 all-dry cells
            int fileSize = cursor;

            byte[] buf = new byte[fileSize];

            // -- header --
            PutBytes(buf, 0, System.Text.Encoding.ASCII.GetBytes("TMNFTRK1"));
            PutU32(buf, 0x08, 3);                 // version
            PutU32(buf, 0x0c, 0x1234_5678);       // endian
            PutU32(buf, 0x10, (uint)HeaderSize);  // header size
            PutU32(buf, 0x14, SectionCount);
            PutU64(buf, 0x18, (ulong)fileSize);
            PutBytes(buf, 0x20, ExeSha256);
            PutBytes(buf, 0x40, new byte[32]);    // track sha (checked only vs expectation)
            (ulong, uint, uint)[] sections =
            {
                ((ulong)entriesOff, (uint)entries.Count, 0x60u),
                ((ulong)surfacesOff, (uint)meshCount, 0x18u),
                ((ulong)meshesOff, (uint)meshCount, 0x38u),
                ((ulong)verticesOff, (uint)vertexTotal, 12u),
                ((ulong)facesOff, (uint)faceTotal, 0x20u),
                ((ulong)nodesOff, (uint)nodeTotal, 0x20u),
                ((ulong)materialIdsOff, (uint)meshCount, 1u),
                ((ulong)materialDataOff, (uint)MaterialCount, 8u),
                ((ulong)pairsOff, 3, 0x14u),
                ((ulong)corpusOff, 1, 48u),
                ((ulong)waterOff, 1, 0x28u),
                ((ulong)cellsOff, 16, 1u),
            };
            for (int i = 0; i < sections.Length; i++)
            {
                int at = 0x60 + i * 16;
                PutU64(buf, at, sections[i].Item1);
                PutU32(buf, at + 8, sections[i].Item2);
                PutU32(buf, at + 12, sections[i].Item3);
            }

            // -- mesh array cursor positions --
            int vertCursor = verticesOff;
            int faceCursor = facesOff;
            int nodeCursor = nodesOff;
            int[] meshVertexOff = new int[meshCount];
            int[] meshFaceOff = new int[meshCount];
            int[] meshNodeOff = new int[meshCount];
            for (int i = 0; i < meshCount; i++)
            {
                BakedMesh m = meshes[i];
                meshVertexOff[i] = vertCursor;
                meshFaceOff[i] = faceCursor;
                meshNodeOff[i] = nodeCursor;
                vertCursor += m.source.VertexCount * 12;
                faceCursor += m.source.FaceCount * 0x20;
                nodeCursor += m.bvhNodeCount * 0x20;
            }

            // -- entries --
            for (int i = 0; i < entries.Count; i++)
            {
                EntryNode e = entries[i];
                int at = entriesOff + i * 0x60;
                PutU32(buf, at, e.skipCount);
                PutF32(buf, at + 4, e.cx);
                PutF32(buf, at + 8, e.cy);
                PutF32(buf, at + 12, e.cz);
                PutF32(buf, at + 16, e.hx);
                PutF32(buf, at + 20, e.hy);
                PutF32(buf, at + 24, e.hz);
                // identity iso (vertices are world-space)
                PutF32(buf, at + 28, 1.0f);
                PutF32(buf, at + 44, 1.0f);
                PutF32(buf, at + 60, 1.0f);
                uint treeFlags = e.surfaceIndex >= 0 ? 0x80u : 0u;
                PutU32(buf, at + 76, treeFlags);
                int surface = e.surfaceIndex >= 0 ? e.surfaceIndex : 0;
                PutU64(buf, at + 80, (ulong)(surfacesOff + surface * 0x18));
                PutU32(buf, at + 88, 9000u + (uint)i); // tree id (provenance only)
                PutU32(buf, at + 92, 1u);               // corpus
            }

            // -- surfaces (one per mesh) --
            for (int i = 0; i < meshCount; i++)
            {
                int at = surfacesOff + i * 0x18;
                PutU64(buf, at, (ulong)(meshesOff + i * 0x38));
                PutU64(buf, at + 8, (ulong)(materialIdsOff + i));
                PutU32(buf, at + 16, 1); // one material id in the remap
                PutU32(buf, at + 20, 0);
                buf[materialIdsOff + i] = meshes[i].source.materialId;
            }

            // -- mesh records --
            for (int i = 0; i < meshCount; i++)
            {
                BakedMesh m = meshes[i];
                int at = meshesOff + i * 0x38;
                PutU32(buf, at + 0x00, 0); // vtable
                PutU32(buf, at + 0x04, 0); // material index + type below
                buf[at + 0x06] = 7;       // surf_type = GM_SURF_MESH
                buf[at + 0x07] = 0;
                PutU32(buf, at + 0x08, (uint)m.source.VertexCount);
                PutU32(buf, at + 0x0c, 0);
                PutU64(buf, at + 0x10, (ulong)meshVertexOff[i]);
                PutU32(buf, at + 0x18, (uint)m.source.FaceCount);
                PutU32(buf, at + 0x1c, 0);
                PutU64(buf, at + 0x20, (ulong)meshFaceOff[i]);
                PutU32(buf, at + 0x28, (uint)m.bvhNodeCount);
                PutU32(buf, at + 0x2c, 0);
                PutU64(buf, at + 0x30, (ulong)meshNodeOff[i]);
            }

            // -- vertices --
            for (int i = 0; i < meshCount; i++)
            {
                float[] v = meshes[i].source.vertices;
                Buffer.BlockCopy(v, 0, buf, meshVertexOff[i], v.Length * 4);
            }

            // -- faces --
            for (int i = 0; i < meshCount; i++)
            {
                GameMeshSpec mesh = meshes[i].source;
                int at = meshFaceOff[i];
                for (int f = 0; f < mesh.FaceCount; f++)
                {
                    PutF32(buf, at + 0, mesh.faceNormals[f * 3 + 0]);
                    PutF32(buf, at + 4, mesh.faceNormals[f * 3 + 1]);
                    PutF32(buf, at + 8, mesh.faceNormals[f * 3 + 2]);
                    PutU32(buf, at + 12, 0); // reserved
                    PutU32(buf, at + 16, mesh.triangles[f * 3 + 0]);
                    PutU32(buf, at + 20, mesh.triangles[f * 3 + 1]);
                    PutU32(buf, at + 24, mesh.triangles[f * 3 + 2]);
                    buf[at + 28] = 0; // face material index (into surface remap)
                    buf[at + 29] = 0;
                    buf[at + 30] = 0;
                    buf[at + 31] = 0;
                    at += 0x20;
                }
            }

            // -- mesh BVH nodes --
            for (int i = 0; i < meshCount; i++)
            {
                BakedMesh m = meshes[i];
                int at = meshNodeOff[i];
                for (int n = 0; n < m.bvhNodeCount; n++)
                {
                    MeshBvhNode node = m.bvh[n];
                    PutU32(buf, at, node.skipCount);
                    PutF32(buf, at + 4, node.cx);
                    PutF32(buf, at + 8, node.cy);
                    PutF32(buf, at + 12, node.cz);
                    PutF32(buf, at + 16, node.hx);
                    PutF32(buf, at + 20, node.hy);
                    PutF32(buf, at + 24, node.hz);
                    PutU32(buf, at + 28, node.faceIndex);
                    at += 0x20;
                }
            }

            // -- material data --
            for (int i = 0; i < MaterialCount; i++)
            {
                int at = materialDataOff + i * 8;
                PutF32(buf, at, friction[i]);
                PutF32(buf, at + 4, restitution[i]);
            }

            // -- collision pairs (payload only; content unused by the sim) --
            for (int i = 0; i < 3; i++)
            {
                int at = pairsOff + i * 0x14;
                PutU32(buf, at, 0);
                PutU32(buf, at + 4, 1);
                PutU32(buf, at + 8, 1);
                PutU32(buf, at + 12, 0);
                PutU32(buf, at + 16, 0);
            }

            // -- corpus iso (identity) --
            PutF32(buf, corpusOff + 0, 1.0f);
            PutF32(buf, corpusOff + 16, 1.0f);
            PutF32(buf, corpusOff + 32, 1.0f);

            // -- water: an inert map far below any plausible track --
            {
                int at = waterOff;
                PutF32(buf, at + 0x00, 32.0f);  // cell_x
                PutF32(buf, at + 0x04, 32.0f);  // cell_z
                PutF32(buf, at + 0x08, 0.0f);  // origin_x
                PutF32(buf, at + 0x0c, 0.0f);  // origin_z
                PutU32(buf, at + 0x10, 4);     // width
                PutU32(buf, at + 0x14, 4);     // height
                PutU32(buf, at + 0x18, 0);     // default cell = dry
                PutF32(buf, at + 0x1c, -1000.0f); // level
                PutF32(buf, at + 0x20, -1001.0f); // floor (floor <= level)
                PutU32(buf, at + 0x24, 0);     // reserved
                // cells: all zero (dry) — the buffer already is.
            }

            // -- payload digest --
            byte[] payload = new byte[fileSize - HeaderSize];
            Buffer.BlockCopy(buf, HeaderSize, payload, 0, payload.Length);
            byte[] digest = Tmnf.Sim.Sha256.Digest(payload);
            PutBytes(buf, 0x120, digest);
            PutBytes(buf, 0x40, digest); // track sha == payload sha (like the fixtures)
            return buf;
        }

        private static int Align(ref int cursor)
        {
            while (cursor % 8 != 0)
            {
                cursor += 1;
            }
            return cursor;
        }

        private static void PutU32(byte[] b, int off, uint v)
        {
            b[off] = (byte)v;
            b[off + 1] = (byte)(v >> 8);
            b[off + 2] = (byte)(v >> 16);
            b[off + 3] = (byte)(v >> 24);
        }

        private static void PutU64(byte[] b, int off, ulong v)
        {
            for (int i = 0; i < 8; i++)
            {
                b[off + i] = (byte)(v >> (8 * i));
            }
        }

        private static void PutF32(byte[] b, int off, float v)
        {
            byte[] bytes = BitConverter.GetBytes(v);
            Buffer.BlockCopy(bytes, 0, b, off, 4);
        }

        private static void PutBytes(byte[] b, int off, byte[] src)
        {
            Buffer.BlockCopy(src, 0, b, off, src.Length);
        }
    }
}
