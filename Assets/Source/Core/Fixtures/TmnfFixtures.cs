using System;
using System.Collections.Generic;

namespace Tmnf.Sim.Fixtures
{
    /// <summary>
    /// Pure-data construction of every input the simulation consumes: a
    /// TMNFTRK1 track, a TMNFROU1 route, a TMNFM6G1 vehicle image and the
    /// 128x128 fake-contact mask. Nothing is read from disk — calling these
    /// builders produces the exact byte images the golden fixtures use, so
    /// a scene can run the whole pipeline (loaders, world, physics, race)
    /// without shipping any asset files.
    ///
    /// The layouts follow the on-disk contracts of the loaders exactly
    /// (section tables, strides, hashes, validation rules); the car uses
    /// the real Stadium tuning/structure values baked into
    /// <see cref="TmnfFixturesData"/> so the physics runs at realistic
    /// magnitudes. The builders are verified byte-for-byte against the
    /// checked-in golden fixtures by the test suite, which also replays
    /// the 400-tick facade drive schedule from pure data and checks its
    /// hash against the golden trace.
    /// </summary>
    public static class TmnfFixtures
    {
        // ------------------------------------------------------------------
        // byte helpers
        // ------------------------------------------------------------------

        private static void PutU32(byte[] b, int off, uint v)
        {
            b[off] = (byte)v;
            b[off + 1] = (byte)(v >> 8);
            b[off + 2] = (byte)(v >> 16);
            b[off + 3] = (byte)(v >> 24);
        }

        private static void PutI32(byte[] b, int off, int v)
        {
            PutU32(b, off, unchecked((uint)v));
        }

        private static void PutF32(byte[] b, int off, float v)
        {
            PutU32(b, off, unchecked((uint)BitConverter.SingleToInt32Bits(v)));
        }

        private static void PutU64(byte[] b, int off, ulong v)
        {
            PutU32(b, off, (uint)v);
            PutU32(b, off + 4, (uint)(v >> 32));
        }

        /// <summary>
        /// Advances the cursor up to the next 8-byte boundary and returns
        /// the aligned value (the section offsets align AND consume the
        /// padding — mirrors the builder layout arithmetic exactly).
        /// </summary>
        private static int AlignTo(ref int cursor)
        {
            while (cursor % 8 != 0)
            {
                cursor += 1;
            }
            return cursor;
        }

        /// <summary>
        /// The canonical TMNF 2.11.26 executable SHA-256 every image header
        /// must carry (the loaders reject anything else).
        /// </summary>
        private static readonly byte[] ExeSha256 = new byte[]
        {
            0x38, 0x47, 0xcf, 0x9f, 0x20, 0xbf, 0xc6, 0x39,
            0x14, 0x45, 0x00, 0x60, 0xed, 0x52, 0x8c, 0x12,
            0x10, 0x4f, 0x74, 0x3d, 0x96, 0xad, 0x23, 0xd6,
            0xe7, 0x6a, 0xbd, 0x17, 0x8d, 0xe8, 0xc8, 0x4f,
        };

        // ------------------------------------------------------------------
        // shared geometry description
        // ------------------------------------------------------------------

        private struct V3
        {
            public float x;
            public float y;
            public float z;

            public static V3 Of(float x, float y, float z)
            {
                V3 v;
                v.x = x;
                v.y = y;
                v.z = z;
                return v;
            }
        }

        /// <summary>One mesh: vertices and faces (vertex triple + material).</summary>
        private sealed class MeshDef
        {
            public List<V3> vertices = new List<V3>();
            /// <summary>(v0, v1, v2, material_index, normal).</summary>
            public List<(uint, uint, uint, ushort, V3)> faces =
                new List<(uint, uint, uint, ushort, V3)>();
        }

        /// <summary>
        /// A quad mesh (two triangles) on the XZ plane at height y, from
        /// (-hx, -hz) to (+hx, +hz) in mesh space. Vertices are wound
        /// counter-clockwise viewed from +y so the geometric winding agrees
        /// with the declared +y normal (the narrowphase derives its side
        /// planes from edge direction x normal — winding must match).
        /// </summary>
        private static MeshDef GroundQuad(float hx, float y, float hz)
        {
            MeshDef m = new MeshDef();
            m.vertices.Add(V3.Of(-hx, y, -hz));
            m.vertices.Add(V3.Of(-hx, y, hz));
            m.vertices.Add(V3.Of(hx, y, hz));
            m.vertices.Add(V3.Of(hx, y, -hz));
            m.faces.Add((0, 1, 2, 0, V3.Of(0.0f, 1.0f, 0.0f)));
            m.faces.Add((0, 2, 3, 0, V3.Of(0.0f, 1.0f, 0.0f)));
            return m;
        }

        /// <summary>
        /// A vertical quad on the XY plane facing -z (from -hx..hx, 0..height),
        /// wound to match the declared -z normal.
        /// </summary>
        private static MeshDef WallQuad(float hx, float height)
        {
            MeshDef m = new MeshDef();
            m.vertices.Add(V3.Of(-hx, 0.0f, 0.0f));
            m.vertices.Add(V3.Of(-hx, height, 0.0f));
            m.vertices.Add(V3.Of(hx, height, 0.0f));
            m.vertices.Add(V3.Of(hx, 0.0f, 0.0f));
            m.faces.Add((0, 1, 2, 0, V3.Of(0.0f, 0.0f, -1.0f)));
            m.faces.Add((0, 2, 3, 0, V3.Of(0.0f, 0.0f, -1.0f)));
            return m;
        }

        /// <summary>Writes one 0x38-byte mesh section record at base.</summary>
        private static void WriteMeshRecord(
            byte[] buf, int b, MeshDef def, ulong vertsOff, ulong facesOff,
            ulong nodesOff)
        {
            PutU32(buf, b + 0x00, 0); // vtable
            PutU32(buf, b + 0x04, 0); // material index + type below
            buf[b + 0x06] = 7; // surf_type = GM_SURF_MESH
            buf[b + 0x07] = 0;
            PutU32(buf, b + 0x08, (uint)def.vertices.Count);
            PutU32(buf, b + 0x0c, 0);
            PutU64(buf, b + 0x10, vertsOff);
            PutU32(buf, b + 0x18, (uint)def.faces.Count);
            PutU32(buf, b + 0x1c, 0);
            PutU64(buf, b + 0x20, facesOff);
            PutU32(buf, b + 0x28, 3); // node_count: root + 2 leaves
            PutU32(buf, b + 0x2c, 0);
            PutU64(buf, b + 0x30, nodesOff);
        }

        /// <summary>
        /// Mesh octree for a two-face mesh: root (skip 3, no face), then one
        /// leaf per face with its AABB computed from the vertices.
        /// </summary>
        private static List<(uint, V3, V3, uint)> MeshNodeRecords(MeshDef def)
        {
            // (skip_count, center, half_extent, face_index)
            V3 lo = V3.Of(float.PositiveInfinity, float.PositiveInfinity,
                float.PositiveInfinity);
            V3 hi = V3.Of(float.NegativeInfinity, float.NegativeInfinity,
                float.NegativeInfinity);
            var leaves = new List<(uint, V3, V3, uint)>();
            for (int f = 0; f < def.faces.Count; f++)
            {
                var (a, bb, c, _, _) = def.faces[f];
                V3 va = def.vertices[(int)a];
                V3 vb = def.vertices[(int)bb];
                V3 vc = def.vertices[(int)c];
                V3 flo = V3.Of(
                    Math.Min(va.x, Math.Min(vb.x, vc.x)),
                    Math.Min(va.y, Math.Min(vb.y, vc.y)),
                    Math.Min(va.z, Math.Min(vb.z, vc.z)));
                V3 fhi = V3.Of(
                    Math.Max(va.x, Math.Max(vb.x, vc.x)),
                    Math.Max(va.y, Math.Max(vb.y, vc.y)),
                    Math.Max(va.z, Math.Max(vb.z, vc.z)));
                lo.x = Math.Min(lo.x, flo.x);
                lo.y = Math.Min(lo.y, flo.y);
                lo.z = Math.Min(lo.z, flo.z);
                hi.x = Math.Max(hi.x, fhi.x);
                hi.y = Math.Max(hi.y, fhi.y);
                hi.z = Math.Max(hi.z, fhi.z);
                leaves.Add((
                    1u,
                    V3.Of((float)(double)(((float)(double)(flo.x + fhi.x)) * 0.5f), (float)(double)(((float)(double)(flo.y + fhi.y)) * 0.5f),
                        (float)(double)(((float)(double)(flo.z + fhi.z)) * 0.5f)),
                    V3.Of((float)(double)(((float)(double)(fhi.x - flo.x)) * 0.5f), (float)(double)(((float)(double)(fhi.y - flo.y)) * 0.5f),
                        (float)(double)(((float)(double)(fhi.z - flo.z)) * 0.5f)),
                    (uint)f));
            }
            var nodes = new List<(uint, V3, V3, uint)>
            {
                (
                    3u,
                    V3.Of((float)(double)(((float)(double)(lo.x + hi.x)) * 0.5f), (float)(double)(((float)(double)(lo.y + hi.y)) * 0.5f),
                        (float)(double)(((float)(double)(lo.z + hi.z)) * 0.5f)),
                    V3.Of((float)(double)(((float)(double)(hi.x - lo.x)) * 0.5f), (float)(double)(((float)(double)(hi.y - lo.y)) * 0.5f),
                        (float)(double)(((float)(double)(hi.z - lo.z)) * 0.5f)),
                    uint.MaxValue)
            };
            nodes.AddRange(leaves);
            return nodes;
        }

        /// <summary>The 31-entry material table (friction, restitution).</summary>
        private static (float, float)[] MaterialTable()
        {
            var t = new (float, float)[31];
            for (int i = 0; i < t.Length; i++)
            {
                t[i] = (1.0f, 0.5f);
            }
            t[0] = (1.0f, 0.5f); // concrete
            t[1] = (1.0f, 0.4f); // pavement
            t[2] = (0.6f, 0.5f); // grass
            t[3] = (0.15f, 0.1f); // ice
            t[4] = (1.0f, 0.3f); // metal
            t[5] = (0.7f, 0.4f); // sand
            t[6] = (0.8f, 0.4f); // dirt
            t[7] = (1.0f, 0.5f); // turbo
            t[8] = (0.9f, 0.4f); // dirt road
            t[9] = (1.2f, 0.2f); // rubber
            t[10] = (0.9f, 0.2f); // sliding rubber
            t[11] = (1.0f, 0.5f); // test
            t[12] = (1.0f, 0.3f); // rock
            t[13] = (0.5f, 0.0f); // water
            t[14] = (0.9f, 0.4f); // wood
            t[15] = (1.0f, 1.0f); // danger
            t[16] = (1.0f, 0.5f); // asphalt
            t[17] = (0.8f, 0.4f); // wet dirt road
            t[18] = (0.9f, 0.4f); // wet asphalt
            t[19] = (0.9f, 0.4f); // wet pavement
            t[20] = (0.55f, 0.45f); // wet grass
            t[21] = (0.4f, 0.3f); // snow
            t[22] = (1.0f, 0.3f); // resonant metal
            t[23] = (1.0f, 0.9f); // golf ball
            t[24] = (1.0f, 0.5f); // golf wall
            t[25] = (1.0f, 0.5f); // golf ground
            t[26] = (1.0f, 0.5f); // turbo2
            t[27] = (1.3f, 0.8f); // bumper
            t[28] = (1.0f, 0.5f); // not collidable
            t[29] = (1.0f, 0.5f); // free wheeling
            t[30] = (1.0f, 0.5f); // turbo roulette
            return t;
        }

        // ------------------------------------------------------------------
        // track
        // ------------------------------------------------------------------

        private sealed class TrackSpec
        {
            /// <summary>Ground tiles: centers on a grid (x, z).</summary>
            public List<(float, float)> tiles = new List<(float, float)>();
            public float tileHalf;
            /// <summary>Turbo strip: (center, half extents) or null.</summary>
            public (V3, V3)? turbo;
            /// <summary>Extra wall quads: (center, half x, half y).</summary>
            public List<(V3, float, float)> walls = new List<(V3, float, float)>();
            /// <summary>Big boundary walls: (center, span, height, axis).</summary>
            public List<(V3, float, float, byte)> boundary =
                new List<(V3, float, float, byte)>();
            /// <summary>Water: (level, floor, width, height, cells).</summary>
            public (float, float, uint, uint, byte[]) water;
        }

        /// <summary>
        /// The "plain" track: a 5x5 field of 96 m tiles (480 m across), a
        /// turbo strip on the drive path, one small crash wall, four
        /// boundary walls around the field.
        /// </summary>
        private static TrackSpec PlainTrackSpec()
        {
            float half = 48.0f;
            var tiles = new List<(float, float)>();
            for (int iz = -2; iz <= 2; iz++)
            {
                for (int ix = -2; ix <= 2; ix++)
                {
                    tiles.Add(((float)(double)(ix * 96.0f), (float)(double)(iz * 96.0f)));
                }
            }
            TrackSpec spec = new TrackSpec();
            spec.tiles = tiles;
            spec.tileHalf = half;
            spec.turbo = (V3.Of(0.0f, 0.06f, 60.0f), V3.Of(12.0f, 0.12f, 6.0f));
            spec.walls.Add((V3.Of(0.0f, 2.5f, 120.0f), 20.0f, 5.0f));
            spec.boundary.Add((V3.Of(0.0f, 25.0f, -240.0f), 240.0f, 25.0f, 0));
            spec.boundary.Add((V3.Of(0.0f, 25.0f, 240.0f), 240.0f, 25.0f, 0));
            spec.boundary.Add((V3.Of(-240.0f, 25.0f, 0.0f), 240.0f, 25.0f, 1));
            spec.boundary.Add((V3.Of(240.0f, 25.0f, 0.0f), 240.0f, 25.0f, 1));
            spec.water = (-1000.0f, -1001.0f, 4, 4, new byte[16]);
            return spec;
        }

        /// <summary>
        /// The "pool" track: the same field with a water pool in the +x/+z
        /// quadrant (exercises the in-water force path — which the original
        /// vehicle decode leaves unbound and panics on; see REPORT.md).
        /// </summary>
        private static TrackSpec PoolTrackSpec()
        {
            TrackSpec spec = PlainTrackSpec();
            byte[] cells = new byte[16];
            cells[5] = 1; // cell (1,1): x in [32,64), z in [32,64)
            cells[6] = 1;
            cells[9] = 1;
            cells[10] = 1;
            spec.water = (2.0f, -992.0f, 4, 4, cells);
            return spec;
        }

        private struct EntryDef
        {
            public uint skip;
            public V3 center;
            public V3 half;
            public float[] iso; // m[9] row-major + t[3]
            public uint flags;
            public uint surface;
            public uint treeId;
            public uint corpus;
        }

        private static float[] IdentIso(float tx, float ty, float tz)
        {
            return new float[]
            {
                1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                tx, ty, tz,
            };
        }

        /// <summary>Builds the TMNFTRK1 bytes for a spec.</summary>
        private static byte[] BuildTrack(TrackSpec spec)
        {
            // Meshes: m0 ground tile, m1 turbo strip, m2 small wall, m3
            // boundary wall. Boundary walls on the x axis get a rotated
            // entry iso (swap x/z), so one mesh serves all four.
            var meshes = new List<MeshDef>
            {
                GroundQuad(spec.tileHalf, 0.0f, spec.tileHalf),
                GroundQuad(12.0f, 0.06f, 6.0f),
                WallQuad(20.0f, 5.0f),
                WallQuad(240.0f, 50.0f),
            };
            // Surfaces: one per mesh; the material remap is the face
            // material id (ground 0, turbo 7, walls 4).
            var surfaceMaterials = new List<byte[]>
            {
                new byte[] { 0 },
                new byte[] { 7 },
                new byte[] { 4 },
                new byte[] { 4 },
            };

            var entries = new List<EntryDef>();
            // World bounds for the root box (the root's iso is identity, so
            // its local box is in world space).
            entries.Add(new EntryDef
            {
                skip = 0, // patched to the total below
                center = V3.Of(0.0f, 12.0f, 0.0f),
                half = V3.Of(241.0f, 26.0f, 241.0f),
                iso = IdentIso(0.0f, 0.0f, 0.0f),
                flags = 0,
                surface = 0,
                treeId = 9000,
                corpus = 1,
            });
            // Tiles: the entry box is the WORLD box (the static scan tests
            // entry boxes directly against world-space query boxes); the
            // entry iso places the tile mesh (authored around its origin).
            foreach (var (tx, tz) in spec.tiles)
            {
                entries.Add(new EntryDef
                {
                    skip = 1,
                    center = V3.Of(tx, 0.0f, tz),
                    half = V3.Of(spec.tileHalf, 0.0f, spec.tileHalf),
                    iso = IdentIso(tx, 0.0f, tz),
                    flags = 0x80,
                    surface = 0,
                    treeId = 1000 + (uint)entries.Count,
                    corpus = 1,
                });
            }
            if (spec.turbo.HasValue)
            {
                var (center, half) = spec.turbo.Value;
                // The turbo mesh is authored at y=0.06 in mesh space; the
                // entry iso places it with translation only.
                entries.Add(new EntryDef
                {
                    skip = 1,
                    center = center,
                    half = half,
                    iso = IdentIso(center.x, 0.0f, center.z),
                    flags = 0x80,
                    surface = 1,
                    treeId = 2000,
                    corpus = 2,
                });
            }
            foreach (var (center, hx, hy) in spec.walls)
            {
                // Wall mesh spans x in [-hx, hx], y in [0, hy], z = 0 in
                // mesh space; the entry places it with translation only.
                entries.Add(new EntryDef
                {
                    skip = 1,
                    center = center,
                    half = V3.Of(hx, hy, 0.0f),
                    iso = IdentIso(center.x, 0.0f, center.z),
                    flags = 0x80,
                    surface = 2,
                    treeId = 3000 + (uint)entries.Count,
                    corpus = 3,
                });
            }
            foreach (var (center, span, wallHeight, axis) in spec.boundary)
            {
                // Axis 0: wall plane at z=center.z (mesh as-is); axis 1:
                // rotated 90 degrees around Y — the world box swaps x/z and
                // the iso carries the rotation.
                V3 half;
                float[] iso;
                if (axis == 0)
                {
                    half = V3.Of(span, (float)(double)(wallHeight * 0.5f), 0.0f);
                    iso = IdentIso(center.x, 0.0f, center.z);
                }
                else
                {
                    half = V3.Of(0.0f, (float)(double)(wallHeight * 0.5f), span);
                    iso = new float[]
                    {
                        0.0f, 0.0f, 1.0f, // row 0
                        0.0f, 1.0f, 0.0f, // row 1
                        -1.0f, 0.0f, 0.0f, // row 2
                        center.x, 0.0f, center.z,
                    };
                }
                entries.Add(new EntryDef
                {
                    skip = 1,
                    center = center,
                    half = half,
                    iso = iso,
                    flags = 0x80,
                    surface = 3,
                    treeId = 4000 + (uint)entries.Count,
                    corpus = 4,
                });
            }
            uint total = (uint)entries.Count;
            entries[0] = new EntryDef
            {
                skip = total,
                center = entries[0].center,
                half = entries[0].half,
                iso = entries[0].iso,
                flags = entries[0].flags,
                surface = entries[0].surface,
                treeId = entries[0].treeId,
                corpus = entries[0].corpus,
            };

            uint corpusCount = 4;

            // --- layout: header 0x150, sections at 8-byte alignment ---
            int headerSize = 0x150;
            int cursor = headerSize;
            int entriesOff = AlignTo(ref cursor);
            cursor += (int)total * 0x60;
            int surfacesOff = AlignTo(ref cursor);
            cursor += 4 * 0x18;
            int meshesOff = AlignTo(ref cursor);
            cursor += meshes.Count * 0x38;
            int verticesOff = AlignTo(ref cursor);
            int vertexTotal = 0;
            foreach (MeshDef m in meshes)
            {
                vertexTotal += m.vertices.Count;
            }
            cursor += vertexTotal * 12;
            int facesOff = AlignTo(ref cursor);
            int faceTotal = 0;
            foreach (MeshDef m in meshes)
            {
                faceTotal += m.faces.Count;
            }
            cursor += faceTotal * 0x20;
            int nodesOff = AlignTo(ref cursor);
            int nodeTotal = 0;
            foreach (MeshDef m in meshes)
            {
                nodeTotal += MeshNodeRecords(m).Count;
            }
            cursor += nodeTotal * 0x20;
            int materialIdsOff = AlignTo(ref cursor);
            int materialIdsLen = 0;
            foreach (byte[] m in surfaceMaterials)
            {
                materialIdsLen += m.Length;
            }
            cursor += materialIdsLen;
            int materialDataOff = AlignTo(ref cursor);
            cursor += 31 * 8;
            int pairsOff = AlignTo(ref cursor);
            cursor += 3 * 0x14;
            int corpusOff = AlignTo(ref cursor);
            cursor += (int)corpusCount * 48;
            int waterOff = AlignTo(ref cursor);
            cursor += 0x28;
            int cellsOff = AlignTo(ref cursor);
            var (level, floor, width, height, cells) = spec.water;
            cursor += cells.Length;
            int fileSize = cursor;

            byte[] buf = new byte[fileSize];

            // --- header ---
            Buffer.BlockCopy(
                System.Text.Encoding.ASCII.GetBytes("TMNFTRK1"), 0, buf, 0, 8);
            PutU32(buf, 0x08, 3); // version
            PutU32(buf, 0x0c, 0x1234_5678); // endian
            PutU32(buf, 0x10, (uint)headerSize);
            PutU32(buf, 0x14, 12); // section count
            PutU64(buf, 0x18, (ulong)fileSize);
            Buffer.BlockCopy(ExeSha256, 0, buf, 0x20, 32);
            // track_sha256: self-referential — the payload digest below is
            // stored here; the loader only checks it against a caller
            // expectation, which stays null in the golden lane.
            // Section table at 0x60: 12 x 16 bytes.
            var sections = new (ulong, uint, uint)[]
            {
                ((ulong)entriesOff, total, 0x60),
                ((ulong)surfacesOff, 4, 0x18),
                ((ulong)meshesOff, (uint)meshes.Count, 0x38),
                ((ulong)verticesOff, (uint)vertexTotal, 12),
                ((ulong)facesOff, (uint)faceTotal, 0x20),
                ((ulong)nodesOff, (uint)nodeTotal, 0x20),
                ((ulong)materialIdsOff, (uint)materialIdsLen, 1),
                ((ulong)materialDataOff, 31, 8),
                ((ulong)pairsOff, 3, 0x14),
                ((ulong)corpusOff, corpusCount, 48),
                ((ulong)waterOff, 1, 0x28),
                ((ulong)cellsOff, width * height, 1),
            };
            for (int i = 0; i < sections.Length; i++)
            {
                int at = 0x60 + i * 16;
                PutU64(buf, at, sections[i].Item1);
                PutU32(buf, at + 8, sections[i].Item2);
                PutU32(buf, at + 12, sections[i].Item3);
            }
            // payload_sha256 at 0x120..0x140, reserved 0x140..0x150.

            // --- entries ---
            for (int i = 0; i < entries.Count; i++)
            {
                EntryDef e = entries[i];
                int at = entriesOff + i * 0x60;
                PutU32(buf, at, e.skip);
                PutF32(buf, at + 4, e.center.x);
                PutF32(buf, at + 8, e.center.y);
                PutF32(buf, at + 12, e.center.z);
                PutF32(buf, at + 16, e.half.x);
                PutF32(buf, at + 20, e.half.y);
                PutF32(buf, at + 24, e.half.z);
                for (int k = 0; k < 12; k++)
                {
                    PutF32(buf, at + 28 + 4 * k, e.iso[k]);
                }
                PutU32(buf, at + 76, e.flags);
                // surface_rel: absolute file offset of the surface record
                PutU64(buf, at + 80,
                    (ulong)(surfacesOff + (int)e.surface * 0x18));
                PutU32(buf, at + 88, e.treeId);
                PutU32(buf, at + 92, e.corpus);
            }

            // --- surfaces (with per-mesh array offsets) ---
            int vertCursor = verticesOff;
            int faceCursor = facesOff;
            int nodeCursor = nodesOff;
            var meshArrays = new List<(int, int, int)>();
            foreach (MeshDef m in meshes)
            {
                meshArrays.Add((vertCursor, faceCursor, nodeCursor));
                vertCursor += m.vertices.Count * 12;
                faceCursor += m.faces.Count * 0x20;
                nodeCursor += MeshNodeRecords(m).Count * 0x20;
            }
            // Material id arrays live contiguously in their section.
            int matCursor = materialIdsOff;
            var surfaceMatOff = new List<int>();
            foreach (byte[] m in surfaceMaterials)
            {
                surfaceMatOff.Add(matCursor);
                foreach (byte id in m)
                {
                    buf[matCursor] = id;
                    matCursor += 1;
                }
            }
            for (int i = 0; i < meshes.Count; i++)
            {
                int at = surfacesOff + i * 0x18;
                PutU64(buf, at, (ulong)(meshesOff + i * 0x38));
                PutU64(buf, at + 8, (ulong)surfaceMatOff[i]);
                PutU32(buf, at + 16, (uint)surfaceMaterials[i].Length);
                PutU32(buf, at + 20, 0);
            }

            // --- meshes ---
            for (int i = 0; i < meshes.Count; i++)
            {
                int at = meshesOff + i * 0x38;
                (int vo, int fo, int no) = meshArrays[i];
                WriteMeshRecord(buf, at, meshes[i],
                    (ulong)vo, (ulong)fo, (ulong)no);
            }

            // --- vertices ---
            for (int i = 0; i < meshes.Count; i++)
            {
                int at = meshArrays[i].Item1;
                foreach (V3 v in meshes[i].vertices)
                {
                    PutF32(buf, at, v.x);
                    PutF32(buf, at + 4, v.y);
                    PutF32(buf, at + 8, v.z);
                    at += 12;
                }
            }

            // --- faces ---
            for (int i = 0; i < meshes.Count; i++)
            {
                int at = meshArrays[i].Item2;
                foreach (var (a, b2, c, mat, normal) in meshes[i].faces)
                {
                    PutF32(buf, at, normal.x);
                    PutF32(buf, at + 4, normal.y);
                    PutF32(buf, at + 8, normal.z);
                    PutU32(buf, at + 12, 0); // reserved
                    PutU32(buf, at + 16, a);
                    PutU32(buf, at + 20, b2);
                    PutU32(buf, at + 24, c);
                    buf[at + 28] = (byte)mat;
                    buf[at + 29] = (byte)(mat >> 8);
                    buf[at + 30] = 0;
                    buf[at + 31] = 0;
                    at += 0x20;
                }
            }

            // --- nodes ---
            for (int i = 0; i < meshes.Count; i++)
            {
                int at = meshArrays[i].Item3;
                foreach (var (skip, center, half, face) in
                    MeshNodeRecords(meshes[i]))
                {
                    PutU32(buf, at, skip);
                    PutF32(buf, at + 4, center.x);
                    PutF32(buf, at + 8, center.y);
                    PutF32(buf, at + 12, center.z);
                    PutF32(buf, at + 16, half.x);
                    PutF32(buf, at + 20, half.y);
                    PutF32(buf, at + 24, half.z);
                    PutU32(buf, at + 28, face);
                    at += 0x20;
                }
            }

            // --- material data ---
            var table = MaterialTable();
            for (int i = 0; i < table.Length; i++)
            {
                int at = materialDataOff + i * 8;
                PutF32(buf, at, table[i].Item1);
                PutF32(buf, at + 4, table[i].Item2);
            }

            // --- collision pairs (payload only; content unused) ---
            for (int i = 0; i < 3; i++)
            {
                int at = pairsOff + i * 0x14;
                PutU32(buf, at, 0);
                PutU32(buf, at + 4, 1);
                PutU32(buf, at + 8, 1);
                PutU32(buf, at + 12, 0);
                PutU32(buf, at + 16, 0);
            }

            // --- corpus isos (identity, translation zero) ---
            for (int i = 0; i < corpusCount; i++)
            {
                int at = corpusOff + i * 48;
                PutF32(buf, at, 1.0f);
                PutF32(buf, at + 16, 1.0f);
                PutF32(buf, at + 32, 1.0f);
                PutF32(buf, at + 44, 0.0f); // t[2]
            }

            // --- water ---
            {
                int at = waterOff;
                PutF32(buf, at + 0x00, 32.0f); // cell_x
                PutF32(buf, at + 0x04, 32.0f); // cell_z
                PutF32(buf, at + 0x08, 0.0f); // origin_x
                PutF32(buf, at + 0x0c, 0.0f); // origin_z
                PutU32(buf, at + 0x10, width);
                PutU32(buf, at + 0x14, height);
                PutU32(buf, at + 0x18, 0); // default_cell
                PutF32(buf, at + 0x1c, level);
                PutF32(buf, at + 0x20, floor);
                PutU32(buf, at + 0x24, 0); // reserved
                for (int i = 0; i < cells.Length; i++)
                {
                    buf[cellsOff + i] = cells[i];
                }
            }

            // --- hashes: payload digest into the header fields ---
            byte[] payloadBytes = new byte[fileSize - headerSize];
            Buffer.BlockCopy(buf, headerSize, payloadBytes, 0, payloadBytes.Length);
            byte[] payload = Sha256.Digest(payloadBytes);
            Buffer.BlockCopy(payload, 0, buf, 0x120, 32);
            Buffer.BlockCopy(payload, 0, buf, 0x40, 32); // track sha == payload
            return buf;
        }

        // ------------------------------------------------------------------
        // route
        // ------------------------------------------------------------------

        /// <summary>
        /// The route: one checkpoint at z=50 (arms the respawn) and the
        /// finish at z=200 — beyond the crash wall at z=120, reachable only
        /// by steering around it.
        /// </summary>
        private static byte[] BuildRoute()
        {
            int headerSize = 0xE0;
            // (position, arc length, half width, legacy flag)
            var centerline = new List<(V3, float, float, uint)>
            {
                (V3.Of(0.0f, 1.0f, 0.0f), 0.0f, 15.0f, 0),
                (V3.Of(0.0f, 1.0f, 50.0f), 50.0f, 15.0f, 0),
                (V3.Of(0.0f, 1.0f, 100.0f), 100.0f, 15.0f, 0),
                (V3.Of(0.0f, 1.0f, 150.0f), 150.0f, 15.0f, 0),
                (V3.Of(0.0f, 1.0f, 200.0f), 200.0f, 15.0f, 0),
                (V3.Of(0.0f, 1.0f, 240.0f), 240.0f, 15.0f, 1),
            };

            int cursor = headerSize;
            int metadataOff = AlignTo(ref cursor);
            cursor += 0x40;
            int startOff = AlignTo(ref cursor);
            cursor += 0x114;
            int checkpointsOff = AlignTo(ref cursor);
            cursor += 0x90; // one checkpoint record
            int finishOff = AlignTo(ref cursor);
            cursor += 0x90;
            int referenceOff = AlignTo(ref cursor);
            cursor += centerline.Count * 0x18;
            int fileSize = cursor;

            byte[] buf = new byte[fileSize];
            Buffer.BlockCopy(
                System.Text.Encoding.ASCII.GetBytes("TMNFROU1"), 0, buf, 0, 8);
            PutU32(buf, 0x08, 3);
            PutU32(buf, 0x0c, 0x1234_5678);
            PutU32(buf, 0x10, (uint)headerSize);
            PutU32(buf, 0x14, 5);
            PutU64(buf, 0x18, (ulong)fileSize);
            Buffer.BlockCopy(ExeSha256, 0, buf, 0x20, 32);
            // track sha: zero (the loader only checks it against a caller
            // expectation, which stays null in the golden lane).
            var sections = new (ulong, uint, uint)[]
            {
                ((ulong)metadataOff, 1, 0x40),
                ((ulong)startOff, 1, 0x114),
                ((ulong)checkpointsOff, 1, 0x90),
                ((ulong)finishOff, 1, 0x90),
                ((ulong)referenceOff, (uint)centerline.Count, 0x18),
            };
            for (int i = 0; i < sections.Length; i++)
            {
                int at = 0x60 + i * 16;
                PutU64(buf, at, sections[i].Item1);
                PutU32(buf, at + 8, sections[i].Item2);
                PutU32(buf, at + 12, sections[i].Item3);
            }

            void IsoIdentityAt(int at, float tx, float ty, float tz)
            {
                PutF32(buf, at, 1.0f);
                PutF32(buf, at + 4, 0.0f);
                PutF32(buf, at + 8, 0.0f);
                PutF32(buf, at + 12, 0.0f);
                PutF32(buf, at + 16, 1.0f);
                PutF32(buf, at + 20, 0.0f);
                PutF32(buf, at + 24, 0.0f);
                PutF32(buf, at + 28, 0.0f);
                PutF32(buf, at + 32, 1.0f);
                PutF32(buf, at + 36, tx);
                PutF32(buf, at + 40, ty);
                PutF32(buf, at + 44, tz);
            }

            // metadata
            {
                int at = metadataOff;
                PutU32(buf, at + 0x00, 1); // lap count
                PutU32(buf, at + 0x04, 1); // checkpoint count
                PutU32(buf, at + 0x08, 1); // finish count
                PutU32(buf, at + 0x0c, 3); // reference count = cp + 2
                PutU32(buf, at + 0x10, 2); // total race checkpoints
                PutU32(buf, at + 0x14, 2); // race checkpoint limit
                PutU32(buf, at + 0x18, 0); // flags
                PutU32(buf, at + 0x1c, (uint)centerline.Count);
                PutU64(buf, at + 0x20, (ulong)startOff);
                PutU64(buf, at + 0x28, (ulong)checkpointsOff);
                PutU64(buf, at + 0x30, (ulong)finishOff);
                PutU64(buf, at + 0x38, (ulong)referenceOff);
            }

            // start: transform (identity at spawn), initial state, spawn
            {
                int at = startOff;
                IsoIdentityAt(at, 0.0f, 1.0f, 0.0f);
                // initial_state (0xAC): quat (w-first bytes: 1,0,0,0), rot
                // identity, pos (0,1,0), everything else zero.
                int isAt = at + 48;
                PutF32(buf, isAt + 0, 1.0f); // quat.x == w (scalar first)
                PutF32(buf, isAt + 16, 1.0f); // rot m[0]
                PutF32(buf, isAt + 32, 1.0f); // rot m[4]
                PutF32(buf, isAt + 48, 1.0f); // rot m[8]
                PutF32(buf, isAt + 52, 0.0f); // pos.x (quat 16 + rot 36)
                PutF32(buf, isAt + 56, 1.0f); // pos.y
                PutF32(buf, isAt + 60, 0.0f); // pos.z
                PutU32(buf, at + 48 + 0xac, 0); // block index
                PutU32(buf, at + 48 + 0xb0, 0); // waypoint type = START
                IsoIdentityAt(at + 48 + 0xb4, 0.0f, 1.0f, 0.0f); // spawn
            }

            // checkpoint trigger (z=50, arms the respawn)
            {
                int at = checkpointsOff;
                PutU32(buf, at + 0x00, 0); // race index
                PutU32(buf, at + 0x04, 42); // block index
                PutU32(buf, at + 0x08, 2); // waypoint type = CHECKPOINT
                PutU32(buf, at + 0x0c, 0x80); // tree flags
                PutF32(buf, at + 0x10, 0.0f); // box center
                PutF32(buf, at + 0x14, 5.0f);
                PutF32(buf, at + 0x18, 50.0f);
                PutF32(buf, at + 0x1c, 12.0f); // half extents
                PutF32(buf, at + 0x20, 10.0f);
                PutF32(buf, at + 0x24, 12.0f);
                IsoIdentityAt(at + 0x28, 0.0f, 0.0f, 0.0f); // transform
                IsoIdentityAt(at + 0x58, 0.0f, 1.0f, 50.0f); // spawn
                PutU32(buf, at + 0x88, 0); // no respawn
                PutU32(buf, at + 0x8c, 0); // reserved
            }

            // finish trigger (z=200, past the crash wall)
            {
                int at = finishOff;
                PutU32(buf, at + 0x00, 0); // race index
                PutU32(buf, at + 0x04, 77); // block index
                PutU32(buf, at + 0x08, 1); // waypoint type = FINISH
                PutU32(buf, at + 0x0c, 0x80); // tree flags
                PutF32(buf, at + 0x10, 0.0f); // box center
                PutF32(buf, at + 0x14, 5.0f);
                PutF32(buf, at + 0x18, 200.0f);
                PutF32(buf, at + 0x1c, 14.0f); // half extents
                PutF32(buf, at + 0x20, 12.0f);
                PutF32(buf, at + 0x24, 14.0f);
                IsoIdentityAt(at + 0x28, 0.0f, 0.0f, 0.0f); // transform
                IsoIdentityAt(at + 0x58, 0.0f, 1.0f, 0.0f); // spawn
                PutU32(buf, at + 0x88, 0); // no respawn
                PutU32(buf, at + 0x8c, 0); // reserved
            }

            // reference points
            for (int i = 0; i < centerline.Count; i++)
            {
                var (p, arc, hw, leg) = centerline[i];
                int at = referenceOff + i * 0x18;
                PutF32(buf, at, p.x);
                PutF32(buf, at + 4, p.y);
                PutF32(buf, at + 8, p.z);
                PutF32(buf, at + 12, arc);
                PutF32(buf, at + 16, hw);
                PutU32(buf, at + 20, leg);
            }

            // payload sha LAST (over all bytes after the 0xE0 header; the
            // layout leaves it at 0xb0..0xd0 with 16 reserved bytes).
            byte[] payloadBytes = new byte[fileSize - headerSize];
            Buffer.BlockCopy(buf, headerSize, payloadBytes, 0, payloadBytes.Length);
            byte[] payload = Sha256.Digest(payloadBytes);
            Buffer.BlockCopy(payload, 0, buf, 0xb0, 32);
            return buf;
        }

        // ------------------------------------------------------------------
        // mask
        // ------------------------------------------------------------------

        /// <summary>
        /// The fixed-seed integer LCG the golden inputs use (PCG-style:
        /// 64-bit state, high 32 bits out). Pure integer arithmetic — the
        /// sequence is a function of the seed alone.
        /// </summary>
        private struct FixtureLcg
        {
            private ulong state;

            public static FixtureLcg Create(ulong seed)
            {
                FixtureLcg l;
                l.state = seed;
                return l;
            }

            public uint NextU32()
            {
                state = unchecked(
                    state * 6364136223846793005UL + 1442695040888963407UL);
                return (uint)(state >> 33);
            }
        }

        /// <summary>
        /// The synthetic 128x128 fake-contact mask: a deterministic pattern
        /// of ~60% flat cells with varied bump strengths, plus a
        /// distinctive cross of strong bumps on the driving path (row/col 6
        /// corresponds to ~48 m with the 8 m material periods).
        /// </summary>
        private static byte[] BuildMask()
        {
            FixtureLcg lcg = FixtureLcg.Create(0xfeed_face_0000_0099UL);
            byte[] mask = new byte[128 * 128];
            for (int y = 0; y < 128; y++)
            {
                for (int x = 0; x < 128; x++)
                {
                    uint v = lcg.NextU32();
                    // ~60% zero cells, the rest varied bump strengths.
                    mask[y * 128 + x] = v % 10 < 6 ? (byte)0 : (byte)(v >> 8);
                }
            }
            for (int x = 0; x < 128; x++)
            {
                mask[6 * 128 + x] = 200;
                mask[x * 128 + 6] = 160;
            }
            return mask;
        }

        // ------------------------------------------------------------------
        // vehicle image
        // ------------------------------------------------------------------

        /// <summary>
        /// The 31-entry ground material id table (from the reference
        /// structure).
        /// </summary>
        private static readonly byte[] GroundMaterialIds = new byte[]
        {
            0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8,
            8, 9, 9, 10, 10, 11, 11, 12, 12,
        };

        /// <summary>
        /// Ground material rows (13): per-surface tire values plus bump
        /// descriptors. Values follow the reference structure; only the
        /// road-like materials carry the shared bump mask (like the real
        /// car, which references the image from few materials).
        /// </summary>
        private static List<(float[], bool, float, float, float, float)>
            GroundMaterialRows()
        {
            float[][] values = new float[13][]
            {
                new float[] { 1.0f, 1.0f, 1.0f, 1.0f },    // 0: asphalt-like
                new float[] { 0.95f, 1.0f, 0.95f, 0.95f }, // 1
                new float[] { 0.7f, 0.8f, 0.8f, 0.7f },    // 2: grass-like
                new float[] { 0.5f, 0.6f, 0.6f, 0.5f },    // 3
                new float[] { 0.9f, 0.95f, 0.9f, 0.9f },   // 4
                new float[] { 0.85f, 0.9f, 0.85f, 0.85f }, // 5
                new float[] { 0.8f, 0.9f, 0.8f, 0.8f },    // 6
                new float[] { 1.0f, 1.0f, 1.0f, 1.0f },    // 7
                new float[] { 0.75f, 0.85f, 0.75f, 0.75f },// 8
                new float[] { 1.1f, 1.2f, 1.0f, 1.1f },    // 9: rubber-like
                new float[] { 0.6f, 0.7f, 0.7f, 0.6f },    // 10
                new float[] { 0.9f, 0.9f, 0.9f, 0.9f },    // 11
                new float[] { 0.8f, 0.8f, 0.8f, 0.8f },    // 12
            };
            var rows =
                new List<(float[], bool, float, float, float, float)>();
            for (int i = 0; i < values.Length; i++)
            {
                bool mask = i == 0 || i == 2 || i == 9;
                rows.Add((values[i], mask, 8.0f, 8.0f, 0.15f, 2.0f));
            }
            return rows;
        }

        /// <summary>Builds the synthetic TMNFM6G1 Stadium vehicle image.</summary>
        private static byte[] BuildVehicle()
        {
            // ---- layout ----
            int headerSize = 320;
            int cursor = headerSize;
            int carOff = AlignTo(ref cursor);
            cursor += 0x878;
            int structOff = AlignTo(ref cursor);
            cursor += 0x50;
            int tuningDescOff = AlignTo(ref cursor);
            cursor += 16;
            int tuningRawOff = AlignTo(ref cursor);
            cursor += 0x3ac;
            int wheelsOff = AlignTo(ref cursor);
            cursor += 4 * 0x2fc;
            int dynaOff = AlignTo(ref cursor);
            cursor += 0x344;
            int paramsOff = AlignTo(ref cursor);
            cursor += 0x5c;
            int stateOff = AlignTo(ref cursor);
            cursor += 0xb4;
            int curveDescOff = AlignTo(ref cursor);
            int curveCount = TmnfFixturesData.CurveOffsets.Length;
            cursor += curveCount * 40;
            int gearboxDescOff = AlignTo(ref cursor);
            cursor += 4 * 24;
            int groundIdsOff = AlignTo(ref cursor);
            cursor += 31 * 4; // ground ids are 31 u32s on disk
            int materialsOff = AlignTo(ref cursor);
            var matRows = GroundMaterialRows();
            cursor += matRows.Count * 40;
            int modelIsoOff = AlignTo(ref cursor);
            cursor += 48;
            int bodyRefOff = AlignTo(ref cursor);
            cursor += 12;
            int curveDataOff = AlignTo(ref cursor);
            // Positions + values for each curve.
            var curveArrays = new List<(int, int, int, int, int)>();
            int keyIdx = 0;
            for (int i = 0; i < curveCount; i++)
            {
                int count = TmnfFixturesData.CurveKeyCounts[i];
                int posOff = cursor;
                cursor += count * 4;
                int valOff = cursor;
                cursor += count * 4;
                curveArrays.Add((
                    TmnfFixturesData.CurveOffsets[i],
                    TmnfFixturesData.CurveInterpolations[i],
                    count, posOff, valOff));
                keyIdx += count;
            }
            int gearboxDataOff = AlignTo(ref cursor);
            var gearboxArrays = new List<(int, int)>();
            for (int i = 0; i < TmnfFixturesData.GearboxOffsets.Length; i++)
            {
                int dataOff = cursor;
                cursor += 6 * 4; // every gearbox table has 6 entries
                gearboxArrays.Add((TmnfFixturesData.GearboxOffsets[i], dataOff));
            }
            int collisionRootOff = AlignTo(ref cursor);
            cursor += 132;
            int collisionChildrenOff = AlignTo(ref cursor);
            int nodeCount = TmnfFixturesData.NodeKind.Length;
            cursor += nodeCount * 132;
            int collisionMatIdsOff = AlignTo(ref cursor);
            cursor += nodeCount; // one material id per surface child
            int fileSize = cursor;

            byte[] b = new byte[fileSize];

            // ---- header ----
            Buffer.BlockCopy(
                System.Text.Encoding.ASCII.GetBytes("TMNFM6G1"), 0, b, 0, 8);
            PutU32(b, 0x08, 4); // version
            PutU32(b, 0x0c, (uint)fileSize); // total size
            PutU32(b, 0x10, 320); // header size
            PutU32(b, 0x14, 0); // phase
            PutU32(b, 0x18, 0); // tick: fresh car
            // ids (must all be nonzero)
            PutU32(b, 0x1c, 0x1001); // car
            PutU32(b, 0x20, 0x1002); // vehicle struct
            PutU32(b, 0x24, 0x1003); // tuning container
            PutU32(b, 0x28, 0x1004); // wheels
            PutU32(b, 0x2c, 0x1005); // item
            PutU32(b, 0x30, 0x1006); // corpus
            PutU32(b, 0x34, 0x1007); // dyna
            PutU32(b, 0x38, 0x1008); // params
            PutU32(b, 0x3c, 0x1009); // state
            PutU32(b, 0x40, 0x100a); // model iso
            PutU32(b, 0x44, 0x100b); // body reference
            PutU32(b, 0x48, 4); // wheel count
            PutU32(b, 0x4c, 31); // ground id count
            PutU32(b, 0x50, (uint)matRows.Count); // ground material count
            PutU32(b, 0x54, 1); // tuning count
            PutU32(b, 0x58, 0); // active tuning key
            PutU32(b, 0x5c, (uint)curveCount);
            PutU32(b, 0x60, 4); // gearbox count = tuning count * 4
            PutU32(b, 0x64, (uint)carOff);
            PutU32(b, 0x68, (uint)structOff);
            PutU32(b, 0x6c, (uint)tuningDescOff);
            PutU32(b, 0x70, (uint)wheelsOff);
            PutU32(b, 0x74, (uint)dynaOff);
            PutU32(b, 0x78, (uint)paramsOff);
            PutU32(b, 0x7c, (uint)stateOff);
            PutU32(b, 0x80, (uint)curveDescOff);
            PutU32(b, 0x84, (uint)gearboxDescOff);
            PutU32(b, 0x88, (uint)groundIdsOff);
            PutU32(b, 0x8c, (uint)materialsOff);
            PutU32(b, 0x90, (uint)modelIsoOff);
            PutU32(b, 0x94, (uint)bodyRefOff);
            PutU32(b, 0x98, (uint)curveDataOff);
            PutU32(b, 0x9c, (uint)gearboxDataOff);
            PutU32(b, 0xa0, (uint)nodeCount); // collision child count
            PutU32(b, 0xa4, (uint)collisionRootOff);
            PutU32(b, 0xa8, (uint)collisionChildrenOff);
            PutU32(b, 0xac, (uint)collisionMatIdsOff);
            // observation words (model value etc.) — informational only
            PutF32(b, 0xb0, 10.0f);
            PutF32(b, 0xb4, 1.0f);
            PutF32(b, 0xb8, 1.0f);
            PutF32(b, 0xbc, 0.0f);
            PutI32(b, 0xc0, 1); // grounded
            PutF32(b, 0xc4, 0.0f); // existing force
            PutF32(b, 0xc8, -10.0f);
            PutF32(b, 0xcc, 0.0f);
            // local speed etc. zeros
            PutI32(b, 0xf8, 0); // sliding
            PutF32(b, 0xfc, 0.0f); // brake force
            Buffer.BlockCopy(ExeSha256, 0, b, 0x100, 32);
            Buffer.BlockCopy(ExeSha256, 0, b, 0x120, 32); // track sha (unchecked)

            // ---- tuning descriptor + raw block ----
            PutU32(b, tuningDescOff, 0); // key
            PutU32(b, tuningDescOff + 4, 0x2001); // object id
            PutU32(b, tuningDescOff + 8, (uint)tuningRawOff);
            PutU32(b, tuningDescOff + 12, 0x3ac);
            {
                int raw = tuningRawOff;
                for (int i = 0; i < TmnfFixturesData.TuningWordOffsets.Length; i++)
                {
                    PutF32(b, raw + TmnfFixturesData.TuningWordOffsets[i],
                        TmnfFixturesData.TuningWordValues[i]);
                }
                for (int i = 0; i < TmnfFixturesData.TuningIntOffsets.Length; i++)
                {
                    PutI32(b, raw + TmnfFixturesData.TuningIntOffsets[i],
                        TmnfFixturesData.TuningIntValues[i]);
                }
                // forced wheel speed source (tuning +0x2bc)
                PutF32(b, raw + 0x2bc, 0.0f);
                // m5 slipping accel scale (tuning +0x1e4): raw-offset-only
                // scalar absent from the reference tables; synthetic 1.5.
                PutF32(b, raw + 0x1e4, 1.5f);
                // curve pointer words: nonzero u32 at every active curve
                // offset except the two primary (vehicle-struct owned) ones
                for (int i = 0; i < TmnfFixturesData.CurveOffsets.Length; i++)
                {
                    int off = TmnfFixturesData.CurveOffsets[i];
                    bool isPrimary = off == 0x044 || off == 0x048;
                    if (!isPrimary)
                    {
                        PutU32(b, raw + off, 0x3000u + (uint)off);
                    }
                }
                // gearbox words: count at the offset, buffer ptr at +4
                for (int i = 0; i < TmnfFixturesData.GearboxOffsets.Length; i++)
                {
                    int off = TmnfFixturesData.GearboxOffsets[i];
                    PutU32(b, raw + off, 6);
                    PutU32(b, raw + off + 4, 0x4000u + (uint)off);
                }
            }

            // ---- vehicle struct (0x50): curve pointer words at 0x44/0x48 ----
            PutU32(b, structOff + 0x44, 0x5001);
            PutU32(b, structOff + 0x48, 0x5002);

            // ---- car block (0x878) ----
            {
                int c = carOff;
                PutF32(b, c + 0x050, 0.0f); // input gas
                PutF32(b, c + 0x054, 0.0f); // input brake
                PutF32(b, c + 0x058, 0.0f); // input steer
                PutF32(b, c + 0x05c, 0.0f); // air effect threshold (disabled)
                // engine @ 0x59c
                PutF32(b, c + 0x59c, 8000.0f); // max rpm
                PutF32(b, c + 0x5b0, 0.0f); // braking factor
                PutF32(b, c + 0x5b4, 0.0f); // rpm
                PutF32(b, c + 0x5b8, 0.0f); // target rpm
                PutF32(b, c + 0x5bc, 1.0f); // clutch
                PutF32(b, c + 0x5c0, 0.0f); // shift timer
                PutI32(b, c + 0x5c4, 0); // reverse (friction selector / model6 mode)
                PutI32(b, c + 0x5c8, 1); // gear
                PutF32(b, c + 0x5cc, -1.0f); // model6 reverse speed threshold
                // aux/compute/model6 state
                PutU32(b, c + 0x2f4, 7); // integration flags: wheels|susp|engine
                PutF32(b, c + 0x1dc, 0.0f); // pivot position (== body box center)
                PutF32(b, c + 0x1e0, 0.6f);
                PutF32(b, c + 0x1e4, 0.0f);
                PutF32(b, c + 0x1e8, 1.0f); // pivot axis x (== simulation gate)
                PutF32(b, c + 0x1ec, 0.5f); // body half y
                PutF32(b, c + 0x1f0, 1.5f); // body half z
                PutF32(b, c + 0x214, 100.0f); // spring c stiffness
                PutF32(b, c + 0x218, 10.0f); // damping
                PutF32(b, c + 0x21c, 0.0f); // value
                PutF32(b, c + 0x220, 0.0f); // target
                PutF32(b, c + 0x224, 0.0f); // velocity
                PutF32(b, c + 0x228, 100.0f); // spring a stiffness
                PutF32(b, c + 0x22c, 10.0f);
                PutF32(b, c + 0x230, 0.0f);
                PutF32(b, c + 0x234, 0.0f);
                PutF32(b, c + 0x238, 0.0f);
                PutF32(b, c + 0x244, 10.0f); // history force limit
                PutF32(b, c + 0x24c, 1.0f); // history force scale
                PutF32(b, c + 0x250, 5.0f); // spring value limit
                PutF32(b, c + 0x2e0, 250.0f); // local speed limit
                PutI32(b, c + 0x2e4, 0); // engine mode
                PutU32(b, c + 0x2e8, 4); // wheel count word
                PutF32(b, c + 0x5a8, 1.0f); // brake input scale
                PutF32(b, c + 0x5ac, 1.0f); // grounded drag scale
                PutF32(b, c + 0x5b0, 0.0f); // computed brake force
                PutF32(b, c + 0x5f4, 0.0f); // turbo factor
                PutI32(b, c + 0x628, 0); // turbo active
                PutI32(b, c + 0x69c, 0); // drive mode
                PutI32(b, c + 0x6a0, 0); // force wheel speed
                PutF32(b, c + 0x840, 1.726f); // axle width
                PutI32(b, c + 0x74c, 0); // air effect mode
                PutI32(b, c + 0x744, 0); // engine limit flag
                PutI32(b, c + 0x748, 0); // gear downshift flag
                PutI32(b, c + 0x73c, 0); // block wheel speed
                PutI32(b, c + 0x60c, 0); // flag 60c
                // model6 model iso @0x6a4: identity (m[4] is spelled at the
                // reference's c+0x6a8+8 — kept verbatim)
                PutF32(b, c + 0x6a4, 1.0f);
                PutF32(b, c + 0x6a8 + 8, 1.0f); // m[4]
                PutF32(b, c + 0x6a4 + 32, 1.0f); // m[8]
            }

            // ---- wheels ----
            for (int i = 0; i < 4; i++)
            {
                int w = wheelsOff + i * 0x2fc;
                int active = TmnfFixturesData.WheelActive[i];
                int steerable = TmnfFixturesData.WheelSteerable[i];
                float radius = TmnfFixturesData.WheelRadius[i];
                float srcTx = TmnfFixturesData.WheelSourceTFlat[i * 3 + 0];
                float srcTy = TmnfFixturesData.WheelSourceTFlat[i * 3 + 1];
                float srcTz = TmnfFixturesData.WheelSourceTFlat[i * 3 + 2];
                float offX = TmnfFixturesData.WheelOffsetFlat[i * 3 + 0];
                float offY = TmnfFixturesData.WheelOffsetFlat[i * 3 + 1];
                float offZ = TmnfFixturesData.WheelOffsetFlat[i * 3 + 2];
                PutI32(b, w + 0x000, active);
                PutI32(b, w + 0x004, steerable);
                PutF32(b, w + 0x008, radius);
                // surface_source iso @0x10 (identity rotation + t)
                PutF32(b, w + 0x10, 1.0f);
                PutF32(b, w + 0x10 + 16, 1.0f); // m[4]
                PutF32(b, w + 0x10 + 32, 1.0f); // m[8]
                PutF32(b, w + 0x10 + 36, srcTx);
                PutF32(b, w + 0x10 + 40, srcTy);
                PutF32(b, w + 0x10 + 44, srcTz);
                // surface_location @0x40 = same
                PutF32(b, w + 0x40, 1.0f);
                PutF32(b, w + 0x40 + 16, 1.0f);
                PutF32(b, w + 0x40 + 32, 1.0f);
                PutF32(b, w + 0x40 + 36, srcTx);
                PutF32(b, w + 0x40 + 40, srcTy);
                PutF32(b, w + 0x40 + 44, srcTz);
                // field70 @0x70: rotation-like floats (reference identity)
                PutF32(b, w + 0x70, 1.0f);
                PutF32(b, w + 0x70 + 16, 1.0f);
                PutF32(b, w + 0x70 + 32, 1.0f);
                PutI32(b, w + 0x0a0, 0x3f80_0000); // fielda0 (1.0f bits)
                PutI32(b, w + 0x0a4, 0x3f80_0000); // fielda4
                PutF32(b, w + 0x0a8, offX);
                PutF32(b, w + 0x0a8 + 4, offY);
                PutF32(b, w + 0x0a8 + 8, offZ);
                // real time @0xb4: damper absorb = rest length
                PutF32(b, w + 0xb4, 0.2f);
                // basis0 @0xc0, basis1 @0xe4: identity
                PutF32(b, w + 0xc0, 1.0f);
                PutF32(b, w + 0xc0 + 16, 1.0f);
                PutF32(b, w + 0xc0 + 32, 1.0f);
                PutF32(b, w + 0xe4, 1.0f);
                PutF32(b, w + 0xe4 + 16, 1.0f);
                PutF32(b, w + 0xe4 + 32, 1.0f);
                // history slots: zeroed with identity matrix at slot+0x30
                int[] slots = { 0x16c, 0x1d0, 0x234, 0x298 };
                foreach (int slot in slots)
                {
                    int at = w + slot;
                    PutF32(b, at + 0x30, 1.0f); // m[0]
                    PutF32(b, at + 0x38, 1.0f); // m[4]
                    PutF32(b, at + 0x40, 1.0f); // m[8]
                }
            }

            // ---- dyna block (0x344) ----
            {
                int d = dynaOff;
                PutI32(b, d + 0x0c0, 1); // clamp angular
                PutF32(b, d + 0x0c4, 100.0f); // max angular speed
                PutI32(b, d + 0x33c, 1); // dirty flag: the car IS active
                PutI32(b, d + 0x340, 1); // mode: full rigid body
                // temp state at dyna+0x274 (180 bytes): spawn state
                int t = d + 0x274;
                PutF32(b, t, 1.0f); // quat w
                PutF32(b, t + 0x10, 1.0f); // rot m[0]
                PutF32(b, t + 0x10 + 16, 1.0f); // m[4]
                PutF32(b, t + 0x10 + 32, 1.0f); // m[8]
                PutF32(b, t + 0x34, 0.0f); // pos x
                PutF32(b, t + 0x34 + 4, 1.0f); // pos y
                PutF32(b, t + 0x34 + 8, 0.0f); // pos z
                // inv inertia world = body diagonal (identity rotation)
                PutF32(b, t + 0x7c, TmnfFixturesData.BodyInverseInertia[0]);
                PutF32(b, t + 0x7c + 16, TmnfFixturesData.BodyInverseInertia[4]);
                PutF32(b, t + 0x7c + 32, TmnfFixturesData.BodyInverseInertia[8]);
            }

            // ---- params (0x5c) ----
            {
                int p = paramsOff;
                PutF32(b, p + 0x00, TmnfFixturesData.BodyMass);
                for (int i = 0; i < 9; i++)
                {
                    PutF32(b, p + 0x04 + 4 * i,
                        TmnfFixturesData.BodyInverseInertia[i]);
                }
                PutF32(b, p + 0x28, TmnfFixturesData.BodyDragLinear);
                PutF32(b, p + 0x2c, TmnfFixturesData.BodyDragAngular);
                PutF32(b, p + 0x30, TmnfFixturesData.BodySubstepLen);
                PutF32(b, p + 0x34, TmnfFixturesData.BodyForceFieldScale);
                PutF32(b, p + 0x38, TmnfFixturesData.BodyCenterOfMass[0]);
                PutF32(b, p + 0x38 + 4, TmnfFixturesData.BodyCenterOfMass[1]);
                PutF32(b, p + 0x38 + 8, TmnfFixturesData.BodyCenterOfMass[2]);
            }

            // ---- state (0xb4): live state = spawn state ----
            {
                int s = stateOff;
                PutF32(b, s, 1.0f); // quat w
                PutF32(b, s + 0x10, 1.0f);
                PutF32(b, s + 0x10 + 16, 1.0f);
                PutF32(b, s + 0x10 + 32, 1.0f);
                PutF32(b, s + 0x34, 0.0f);
                PutF32(b, s + 0x34 + 4, 1.0f);
                PutF32(b, s + 0x34 + 8, 0.0f);
                PutF32(b, s + 0x7c, TmnfFixturesData.BodyInverseInertia[0]);
                PutF32(b, s + 0x7c + 16, TmnfFixturesData.BodyInverseInertia[4]);
                PutF32(b, s + 0x7c + 32, TmnfFixturesData.BodyInverseInertia[8]);
            }

            // ---- curve descriptors + arrays ----
            for (int i = 0; i < curveArrays.Count; i++)
            {
                var (off, interp, count, posOff, valOff) = curveArrays[i];
                int at = curveDescOff + i * 40;
                bool isPrimary = off == 0x044 || off == 0x048;
                PutU32(b, at, isPrimary ? 0u : 1u); // owner kind
                PutU32(b, at + 4, 0); // owner key
                PutU32(b, at + 8, (uint)off); // field offset
                PutU32(b, at + 12, 0x6001u + (uint)i); // object id
                PutU32(b, at + 16, 0x7001u + (uint)i); // positions id
                PutU32(b, at + 20, 0x7801u + (uint)i); // values id
                PutU32(b, at + 24, (uint)count);
                PutI32(b, at + 28, interp);
                PutU32(b, at + 32, (uint)posOff); // positions offset
                PutU32(b, at + 36, (uint)valOff); // values offset
            }
            {
                int flat = 0;
                for (int i = 0; i < curveArrays.Count; i++)
                {
                    var (_, _, count, posOff, valOff) = curveArrays[i];
                    for (int k = 0; k < count; k++)
                    {
                        PutF32(b, posOff + 4 * k,
                            TmnfFixturesData.CurveKeysFlat[flat + 2 * k]);
                        PutF32(b, valOff + 4 * k,
                            TmnfFixturesData.CurveKeysFlat[flat + 2 * k + 1]);
                    }
                    flat += 2 * count;
                }
            }

            // ---- gearbox descriptors + data ----
            for (int i = 0; i < gearboxArrays.Count; i++)
            {
                var (off, dataOff) = gearboxArrays[i];
                int at = gearboxDescOff + i * 24;
                PutU32(b, at, 0); // owner key
                PutU32(b, at + 4, (uint)off); // field offset
                PutU32(b, at + 8, 0x8001u + (uint)i); // buffer id
                PutU32(b, at + 12, 0x8801u + (uint)i); // data id
                PutU32(b, at + 16, 6); // count
                PutU32(b, at + 20, (uint)dataOff); // data offset
                for (int k = 0; k < 6; k++)
                {
                    PutF32(b, dataOff + 4 * k,
                        TmnfFixturesData.GearboxValuesFlat[i * 6 + k]);
                }
            }

            // ---- ground ids + materials ----
            // The ground-id table is 31 u32 words on disk.
            for (int i = 0; i < GroundMaterialIds.Length; i++)
            {
                PutU32(b, groundIdsOff + i * 4, GroundMaterialIds[i]);
            }
            for (int i = 0; i < matRows.Count; i++)
            {
                var (values, mask, px, pz, scale, limit) = matRows[i];
                int at = materialsOff + i * 40;
                PutU32(b, at, 0x9001u + (uint)i); // object id
                for (int k = 0; k < 4; k++)
                {
                    PutF32(b, at + 4 + 4 * k, values[k]);
                }
                PutU32(b, at + 20, mask ? 1u : 0u);
                PutF32(b, at + 24, px);
                PutF32(b, at + 28, pz);
                PutF32(b, at + 32, scale);
                PutF32(b, at + 36, limit);
            }

            // ---- model iso + body reference ----
            PutF32(b, modelIsoOff, 1.0f);
            PutF32(b, modelIsoOff + 16, 1.0f);
            PutF32(b, modelIsoOff + 32, 1.0f);
            PutF32(b, bodyRefOff, TmnfFixturesData.BodyReferencePosition[0]);
            PutF32(b, bodyRefOff + 4, TmnfFixturesData.BodyReferencePosition[1]);
            PutF32(b, bodyRefOff + 8, TmnfFixturesData.BodyReferencePosition[2]);

            // ---- collision tree ----
            {
                int r = collisionRootOff;
                PutU32(b, r + 0x00, 0xa001); // object id
                PutU32(b, r + 0x04, 0x80); // flags (0x80 required)
                PutU32(b, r + 0x08, 0); // surface id
                PutU32(b, r + 0x0c, 0); // geometry id
                PutU32(b, r + 0x10, 2); // kind = root
                PutU32(b, r + 0x14, uint.MaxValue); // wheel index
                PutU32(b, r + 0x18, 0); // material index
                PutU32(b, r + 0x1c, 0); // material count
                // box: car-sized root
                PutF32(b, r + 0x24, 0.0f);
                PutF32(b, r + 0x28, 0.6f);
                PutF32(b, r + 0x2c, 0.0f);
                PutF32(b, r + 0x30, 1.2f);
                PutF32(b, r + 0x34, 0.6f);
                PutF32(b, r + 0x38, 2.0f);
                // local iso at r+0x3c: identity
                PutF32(b, r + 0x3c, 1.0f);
                PutF32(b, r + 0x3c + 16, 1.0f);
                PutF32(b, r + 0x3c + 32, 1.0f);

                uint runningMaterial = 0;
                for (int i = 0; i < nodeCount; i++)
                {
                    int kind = TmnfFixturesData.NodeKind[i];
                    int wheelIndex = TmnfFixturesData.NodeWheelIndex[i];
                    int flags = TmnfFixturesData.NodeFlags[i];
                    int at = collisionChildrenOff + i * 132;
                    uint childCount = 0; // all leaves
                    PutU32(b, at + 0x00, 0xa100u + (uint)i); // object id
                    PutU32(b, at + 0x04, (uint)flags); // 0x1e886: 0x80 + local iso
                    PutU32(b, at + 0x08, 1); // surface id (nonzero = has surface)
                    PutU32(b, at + 0x0c, 1); // geometry id
                    PutU32(b, at + 0x10,
                        (uint)kind | (childCount << 8));
                    PutU32(b, at + 0x14,
                        kind == 1
                            ? (uint)wheelIndex
                            : uint.MaxValue);
                    PutU32(b, at + 0x18, runningMaterial);
                    PutU32(b, at + 0x1c, 1); // material count
                    b[at + 0x20] = 0; // geometry material index (u16 low)
                    b[at + 0x21] = 0; // geometry material index (u16 high)
                    b[at + 0x22] = (byte)TmnfFixturesData.NodeGeometryType[i];
                    b[at + 0x23] = 0;
                    PutF32(b, at + 0x24, TmnfFixturesData.NodeBoxCenterFlat[i * 3 + 0]);
                    PutF32(b, at + 0x28, TmnfFixturesData.NodeBoxCenterFlat[i * 3 + 1]);
                    PutF32(b, at + 0x2c, TmnfFixturesData.NodeBoxCenterFlat[i * 3 + 2]);
                    PutF32(b, at + 0x30, TmnfFixturesData.NodeBoxHalfFlat[i * 3 + 0]);
                    PutF32(b, at + 0x34, TmnfFixturesData.NodeBoxHalfFlat[i * 3 + 1]);
                    PutF32(b, at + 0x38, TmnfFixturesData.NodeBoxHalfFlat[i * 3 + 2]);
                    PutF32(b, at + 0x3c, 1.0f);
                    PutF32(b, at + 0x3c + 16, 1.0f);
                    PutF32(b, at + 0x3c + 32, 1.0f);
                    PutF32(b, at + 0x3c + 36, TmnfFixturesData.NodeLocalTFlat[i * 3 + 0]);
                    PutF32(b, at + 0x3c + 40, TmnfFixturesData.NodeLocalTFlat[i * 3 + 1]);
                    PutF32(b, at + 0x3c + 44, TmnfFixturesData.NodeLocalTFlat[i * 3 + 2]);
                    // shape: radii at offset 0
                    PutF32(b, at + 0x6c, TmnfFixturesData.NodeRadiiFlat[i * 3 + 0]);
                    PutF32(b, at + 0x6c + 4, TmnfFixturesData.NodeRadiiFlat[i * 3 + 1]);
                    PutF32(b, at + 0x6c + 8, TmnfFixturesData.NodeRadiiFlat[i * 3 + 2]);
                    runningMaterial += 1;
                    // material id byte
                    b[collisionMatIdsOff + i] =
                        (byte)TmnfFixturesData.NodeMaterialId[i];
                }
            }

            return b;
        }

        // ------------------------------------------------------------------
        // public entry points
        // ------------------------------------------------------------------

        /// <summary>The "plain" 5x5 field track with turbo strip + walls.</summary>
        public static byte[] PlainTrack()
        {
            return BuildTrack(PlainTrackSpec());
        }

        /// <summary>The "pool" track (water pool in the +x/+z quadrant).</summary>
        public static byte[] PoolTrack()
        {
            return BuildTrack(PoolTrackSpec());
        }

        /// <summary>The one-checkpoint loop route.</summary>
        public static byte[] Route()
        {
            return BuildRoute();
        }

        /// <summary>The synthetic Stadium vehicle image.</summary>
        public static byte[] StadiumVehicle()
        {
            return BuildVehicle();
        }

        /// <summary>The synthetic 128x128 fake-contact mask.</summary>
        public static byte[] GameMask()
        {
            return BuildMask();
        }
    }
}
