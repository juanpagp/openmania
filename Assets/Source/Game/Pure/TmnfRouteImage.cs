using System;
using System.Collections.Generic;

namespace Tmnf.Game.Pure
{
    /// <summary>
    /// Builds a TMNFROU1 route image (the checkpoint/finish format
    /// RouteLoader.TmnfRouteLoadBytes consumes) from a <see cref="GameRouteSpec"/>.
    ///
    /// The route carries the run start, every checkpoint trigger and every
    /// finish trigger, plus a reference centerline the race layer uses for
    /// projection bookkeeping (arc length, corridor). The centerline is
    /// synthesized from the waypoints — start, checkpoints in scene order,
    /// first finish — densified to a regular spacing, with a monotone leg
    /// counter that crosses one boundary per waypoint so the loader's
    /// validation (legIndex 0..checkpointCount, +1 steps, arc length
    /// matching geometry) always holds. Waypoints that coincide are nudged
    /// apart by a centimeter so no zero-length segment can exist.
    ///
    /// Checkpoints are marked no-respawn: the game's reset action restarts
    /// the whole run, and last-checkpoint respawning is intentionally not
    /// part of the gameplay loop yet.
    ///
    /// Pure data in, bytes out: no UnityEngine dependency, no file IO. The
    /// layout follows the on-disk contract of the reference loader exactly
    /// — see TmnfFixtures.BuildRoute in the core, which builds the golden
    /// route with the same layout.
    /// </summary>
    public static class TmnfRouteImage
    {
        private const int HeaderSize = 0xE0;
        private const int SectionCount = 5;
        private const uint WaypointStart = 0;
        private const uint WaypointFinish = 1;
        private const uint WaypointCheckpoint = 2;

        /// <summary>The canonical TMNF 2.11.26 executable SHA-256.</summary>
        private static readonly byte[] ExeSha256 = Tmnf.Sim.TrackLoader.Tmnf21126ExeSha256;

        /// <summary>
        /// Builds the TMNFROU1 bytes. Throws <see cref="InvalidOperationException"/>
        /// with a precise message when the spec cannot produce a valid route
        /// (no finish, too many checkpoints, non-orthonormal start rotation...).
        /// </summary>
        public static byte[] Build(GameRouteSpec spec)
        {
            if (spec == null)
            {
                throw new InvalidOperationException("route image: no route spec");
            }
            if (spec.checkpoints == null || spec.finishes == null)
            {
                throw new InvalidOperationException("route image: incomplete route spec");
            }
            int checkpointCount = spec.checkpoints.Count;
            int finishCount = spec.finishes.Count;
            if (finishCount == 0)
            {
                throw new InvalidOperationException(
                    "route image: the track has no finish trigger");
            }
            if (checkpointCount > 63)
            {
                throw new InvalidOperationException(
                    "route image: more than 63 checkpoints are not supported");
            }
            if ((long)checkpointCount + finishCount > 64)
            {
                throw new InvalidOperationException(
                    "route image: checkpoints + finishes exceed 64 triggers");
            }
            ValidateStartIso(spec.startIso);
            foreach (GameTriggerSpec t in spec.checkpoints)
            {
                ValidateTrigger(t, "checkpoint");
            }
            foreach (GameTriggerSpec t in spec.finishes)
            {
                ValidateTrigger(t, "finish");
            }

            List<CenterPoint> centerline = SynthesizeCenterline(spec);

            return WriteImage(spec, checkpointCount, finishCount, centerline);
        }

        private static void ValidateStartIso(float[] iso)
        {
            if (iso == null || iso.Length != 12)
            {
                throw new InvalidOperationException(
                    "route image: start iso must hold 12 floats (3x3 rotation + xyz)");
            }
            foreach (float v in iso)
            {
                if (!float.IsFinite(v))
                {
                    throw new InvalidOperationException(
                        "route image: start iso has non-finite values");
                }
            }
            // Row norms must be 1 (the loader's ValidSpawn tolerance is 1e-3).
            for (int row = 0; row < 3; row++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    float v = iso[row * 3 + k];
                    sum += v * v;
                }
                if (Math.Abs(sum - 1.0f) > 1e-4f)
                {
                    throw new InvalidOperationException(
                        "route image: start rotation is not orthonormal (scale/shear on the Start object?)");
                }
            }
        }

        private static void ValidateTrigger(GameTriggerSpec t, string what)
        {
            if (t == null)
            {
                throw new InvalidOperationException($"route image: null {what} trigger");
            }
            if (!(t.hx > 0.0f) || !(t.hy > 0.0f) || !(t.hz > 0.0f))
            {
                throw new InvalidOperationException(
                    $"route image: {what} trigger box has a non-positive half extent");
            }
            if (!(float.IsFinite(t.cx) && float.IsFinite(t.cy) && float.IsFinite(t.cz)))
            {
                throw new InvalidOperationException(
                    $"route image: {what} trigger box center is non-finite");
            }
            if (t.rot == null || t.rot.Length != 9)
            {
                throw new InvalidOperationException(
                    $"route image: {what} trigger rotation must hold 9 floats");
            }
            foreach (float v in t.rot)
            {
                if (!float.IsFinite(v))
                {
                    throw new InvalidOperationException(
                        $"route image: {what} trigger rotation is non-finite");
                }
            }
        }

        // ------------------------------------------------------------------
        // centerline synthesis
        // ------------------------------------------------------------------

        private struct CenterPoint
        {
            public float x;
            public float y;
            public float z;
            public float arcLength;
            public float halfWidth;
            public uint legIndex;
        }

        /// <summary>
        /// One synthesized reference point: position, accumulated arc length
        /// (always the exact sum of segment lengths, so the loader's
        /// geometry check holds by construction) and the leg counter.
        /// </summary>
        private sealed class CenterBuilder
        {
            private readonly List<CenterPoint> points = new List<CenterPoint>();
            private readonly float halfWidth;
            private readonly double maxStep;
            private float x;
            private float y;
            private float z;
            private double arc;
            private uint leg;

            public CenterBuilder(float startX, float startY, float startZ,
                float halfWidth, float maxStep)
            {
                x = startX;
                y = startY;
                z = startZ;
                this.halfWidth = halfWidth;
                this.maxStep = Math.Max(0.5, maxStep);
            }

            public int Count => points.Count;

            public List<CenterPoint> Points => points;

            /// <summary>
            /// Emits the first reference point exactly at the start position
            /// (the loader requires it to bit-match the start transform).
            /// </summary>
            public void Seed()
            {
                AddPoint(x, y, z);
            }

            /// <summary>
            /// Moves to (tx, ty, tz) — through intermediate points so no
            /// segment exceeds the step — then optionally marks a leg
            /// boundary. The arrival point carries the pre-increment leg
            /// (the reference line keeps the checkpoint on its own leg),
            /// matching the layout the golden route uses.
            /// </summary>
            public void Waypoint(float tx, float ty, float tz, bool bumpLeg)
            {
                float dx = tx - x;
                float dy = ty - y;
                float dz = tz - z;
                double length = Math.Sqrt((double)dx * dx + (double)dy * dy + (double)dz * dz);
                if (length < 1e-4)
                {
                    // Coincident waypoint: nudge deterministically so the
                    // segment stays positive-length and the leg boundary
                    // still lands on a real point.
                    tx = x + 0.01f;
                    dx = 0.01f;
                    dy = 0.0f;
                    dz = 0.0f;
                    length = 0.01;
                }
                int steps = Math.Max(1, (int)Math.Ceiling(length / maxStep));
                for (int s = 1; s <= steps; s++)
                {
                    float f = (float)((double)s / steps);
                    AddPoint(x + dx * f, y + dy * f, z + dz * f);
                }
                x = tx;
                y = ty;
                z = tz;
                if (bumpLeg && leg != uint.MaxValue)
                {
                    leg += 1;
                }
            }

            private void AddPoint(float px, float py, float pz)
            {
                if (points.Count > 0)
                {
                    CenterPoint last = points[points.Count - 1];
                    float dx = px - last.x;
                    float dy = py - last.y;
                    float dz = pz - last.z;
                    double seg = Math.Sqrt((double)dx * dx + (double)dy * dy + (double)dz * dz);
                    if (seg < 1e-5)
                    {
                        return; // never emit a duplicate point
                    }
                    arc += seg;
                }
                CenterPoint p = new CenterPoint();
                p.x = px;
                p.y = py;
                p.z = pz;
                p.arcLength = (float)arc;
                p.halfWidth = halfWidth;
                p.legIndex = leg;
                points.Add(p);
            }
        }

        private static List<CenterPoint> SynthesizeCenterline(GameRouteSpec spec)
        {
            float startX = spec.startIso[9];
            float startY = spec.startIso[10];
            float startZ = spec.startIso[11];
            float step = Math.Max(0.5f, spec.centerlineSpacing);
            int checkpointCount = spec.checkpoints.Count;

            CenterBuilder builder = new CenterBuilder(
                startX, startY, startZ,
                Math.Max(0.5f, spec.corridorHalfWidth), step);

            // Seed (leg 0) -> each checkpoint bumps the leg after its arrival
            // point -> the finish arrival carries the final leg. The point
            // count is therefore always >= checkpointCount + 2 (one seed +
            // one arrival per waypoint), exactly the loader's minimum, and
            // every leg transition is a +1 between real points.
            builder.Seed();
            foreach (GameTriggerSpec cp in spec.checkpoints)
            {
                builder.Waypoint(cp.cx, cp.cy, cp.cz, true);
            }
            GameTriggerSpec finish = spec.finishes[0];
            builder.Waypoint(finish.cx, finish.cy, finish.cz, false);

            List<CenterPoint> points = builder.Points;
            if (points.Count < checkpointCount + 2)
            {
                throw new InvalidOperationException(
                    "route image: centerline synthesis produced too few points");
            }
            return points;
        }

        // ------------------------------------------------------------------
        // serialization
        // ------------------------------------------------------------------

        private static byte[] WriteImage(
            GameRouteSpec spec, int checkpointCount, int finishCount,
            List<CenterPoint> centerline)
        {
            int cursor = HeaderSize;
            int metadataOff = Align(ref cursor);
            cursor += 0x40;
            int startOff = Align(ref cursor);
            cursor += 0x114;
            int checkpointsOff = Align(ref cursor);
            cursor += checkpointCount * 0x90;
            int finishOff = Align(ref cursor);
            cursor += finishCount * 0x90;
            int referenceOff = Align(ref cursor);
            cursor += centerline.Count * 0x18;
            int fileSize = cursor;

            byte[] buf = new byte[fileSize];

            PutBytes(buf, 0, System.Text.Encoding.ASCII.GetBytes("TMNFROU1"));
            PutU32(buf, 0x08, 3);                // version
            PutU32(buf, 0x0c, 0x1234_5678);      // endian
            PutU32(buf, 0x10, (uint)HeaderSize);  // header size
            PutU32(buf, 0x14, SectionCount);
            PutU64(buf, 0x18, (ulong)fileSize);
            PutBytes(buf, 0x20, ExeSha256);
            PutBytes(buf, 0x40, new byte[32]); // track sha (zero: no expectation)
            (ulong, uint, uint)[] sections =
            {
                ((ulong)metadataOff, 1, 0x40u),
                ((ulong)startOff, 1, 0x114u),
                ((ulong)checkpointsOff, (uint)checkpointCount, 0x90u),
                ((ulong)finishOff, (uint)finishCount, 0x90u),
                ((ulong)referenceOff, (uint)centerline.Count, 0x18u),
            };
            for (int i = 0; i < sections.Length; i++)
            {
                int at = 0x60 + i * 16;
                PutU64(buf, at, sections[i].Item1);
                PutU32(buf, at + 8, sections[i].Item2);
                PutU32(buf, at + 12, sections[i].Item3);
            }

            // -- metadata --
            {
                int at = metadataOff;
                PutU32(buf, at + 0x00, 1);                          // lap count
                PutU32(buf, at + 0x04, (uint)checkpointCount);      // checkpoint count
                PutU32(buf, at + 0x08, (uint)finishCount);          // finish count
                PutU32(buf, at + 0x0c, (uint)(checkpointCount + 2));// reference count
                PutU32(buf, at + 0x10, (uint)(checkpointCount + 1)); // total race checkpoints
                PutU32(buf, at + 0x14, (uint)(checkpointCount + 1)); // race checkpoint limit
                PutU32(buf, at + 0x18, 0);                          // flags: single lap
                PutU32(buf, at + 0x1c, (uint)centerline.Count);     // centerline count
                PutU64(buf, at + 0x20, (ulong)startOff);
                PutU64(buf, at + 0x28, (ulong)checkpointsOff);
                PutU64(buf, at + 0x30, (ulong)finishOff);
                PutU64(buf, at + 0x38, (ulong)referenceOff);
            }

            // -- start record --
            {
                int at = startOff;
                PutIso(buf, at, spec.startIso);          // transform
                // Initial state block (0xAC bytes): quat (4 floats, scalar
                // first), rot (9 floats, row-major), pos — the rot and pos
                // must bit-match the transform above (loader validation).
                int isAt = at + 48;
                PutQuatFromMatrix(buf, isAt + 0, spec.startIso); // quat
                for (int k = 0; k < 9; k++)
                {
                    PutF32(buf, isAt + 16 + 4 * k, spec.startIso[k]); // rot
                }
                PutF32(buf, isAt + 52, spec.startIso[9]);  // pos.x
                PutF32(buf, isAt + 56, spec.startIso[10]); // pos.y
                PutF32(buf, isAt + 60, spec.startIso[11]); // pos.z
                PutU32(buf, at + 48 + 0xac, 0);          // block index
                PutU32(buf, at + 48 + 0xb0, WaypointStart); // waypoint type
                PutIso(buf, at + 48 + 0xb4, spec.startIso); // spawn = start
            }

            // -- checkpoint records --
            for (int i = 0; i < checkpointCount; i++)
            {
                WriteTrigger(buf, checkpointsOff + i * 0x90, spec.checkpoints[i],
                    (uint)i, WaypointCheckpoint);
            }

            // -- finish records --
            for (int i = 0; i < finishCount; i++)
            {
                WriteTrigger(buf, finishOff + i * 0x90, spec.finishes[i],
                    (uint)i, WaypointFinish);
            }

            // -- reference centerline --
            for (int i = 0; i < centerline.Count; i++)
            {
                CenterPoint p = centerline[i];
                int at = referenceOff + i * 0x18;
                PutF32(buf, at + 0, p.x);
                PutF32(buf, at + 4, p.y);
                PutF32(buf, at + 8, p.z);
                PutF32(buf, at + 12, p.arcLength);
                PutF32(buf, at + 16, p.halfWidth);
                PutU32(buf, at + 20, p.legIndex);
            }

            // -- payload digest --
            byte[] payload = new byte[fileSize - HeaderSize];
            Buffer.BlockCopy(buf, HeaderSize, payload, 0, payload.Length);
            PutBytes(buf, 0xb0, Tmnf.Sim.Sha256.Digest(payload));
            return buf;
        }

        private static void WriteTrigger(
            byte[] buf, int at, GameTriggerSpec t, uint index, uint waypointType)
        {
            PutU32(buf, at + 0x00, index);          // race index
            PutU32(buf, at + 0x04, index + 42);     // block index (arbitrary)
            PutU32(buf, at + 0x08, waypointType);
            PutU32(buf, at + 0x0c, 0x80);           // tree flags: enabled
            PutF32(buf, at + 0x10, 0.0f);            // box center (local)
            PutF32(buf, at + 0x14, 0.0f);
            PutF32(buf, at + 0x18, 0.0f);
            PutF32(buf, at + 0x1c, t.hx);            // half extents
            PutF32(buf, at + 0x20, t.hy);
            PutF32(buf, at + 0x24, t.hz);
            // transform: rotation + world center.
            for (int k = 0; k < 9; k++)
            {
                PutF32(buf, at + 0x28 + 4 * k, t.rot[k]);
            }
            PutF32(buf, at + 0x4c, t.cx);
            PutF32(buf, at + 0x50, t.cy);
            PutF32(buf, at + 0x54, t.cz);
            PutIso(buf, at + 0x58, IdentityIso);    // spawn (unused: no-respawn)
            PutU32(buf, at + 0x88, 1);              // no respawn
            PutU32(buf, at + 0x8c, 0);              // reserved
        }

        private static readonly float[] IdentityIso =
        {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f,
        };

        private static void PutIso(byte[] buf, int at, float[] iso)
        {
            for (int k = 0; k < 12; k++)
            {
                PutF32(buf, at + 4 * k, iso[k]);
            }
        }

        /// <summary>
        /// Scalar-first quaternion of the row-major rotation (the route's
        /// byte order), normalized. Mirrors the reference Shepperd schedule
        /// closely enough for a rotation that is already orthonormal.
        /// </summary>
        private static void PutQuatFromMatrix(byte[] buf, int at, float[] iso)
        {
            float m00 = iso[0];
            float m01 = iso[1];
            float m02 = iso[2];
            float m10 = iso[3];
            float m11 = iso[4];
            float m12 = iso[5];
            float m20 = iso[6];
            float m21 = iso[7];
            float m22 = iso[8];
            float w;
            float x;
            float y;
            float z;
            float trace = m00 + m11 + m22;
            if (trace > 0.0f)
            {
                float s = (float)Math.Sqrt(trace + 1.0f);
                w = s * 0.5f;
                float inv = 0.5f / s;
                x = (m21 - m12) * inv;
                y = (m02 - m20) * inv;
                z = (m10 - m01) * inv;
            }
            else
            {
                int i = m00 < m11 ? 1 : 0;
                if ((i == 0 ? m00 : m11) < m22)
                {
                    i = 2;
                }
                int j = (i + 1) % 3;
                int k = (j + 1) % 3;
                float mij = Mat(iso, i, j);
                float mji = Mat(iso, j, i);
                float mii = Mat(iso, i, i);
                float mjj = Mat(iso, j, j);
                float mkk = Mat(iso, k, k);
                float mik = Mat(iso, i, k);
                float mki = Mat(iso, k, i);
                float mjk = Mat(iso, j, k);
                float mkj = Mat(iso, k, j);
                float s = (float)Math.Sqrt(mii - mjj - mkk + 1.0f);
                float[] q = new float[4];
                q[i + 1] = s * 0.5f;
                float inv = 0.5f / s;
                q[0] = (mkj - mjk) * inv;              // w from skew
                q[j + 1] = (mij + mji) * inv;
                q[k + 1] = (mik + mki) * inv;
                w = q[0];
                x = q[1];
                y = q[2];
                z = q[3];
            }
            float norm = (float)Math.Sqrt(
                (double)w * w + (double)x * x + (double)y * y + (double)z * z);
            if (norm > 0.0f)
            {
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;
            }
            // Scalar first in the file's byte order.
            PutF32(buf, at + 0, w);
            PutF32(buf, at + 4, x);
            PutF32(buf, at + 8, y);
            PutF32(buf, at + 12, z);
        }

        private static float Mat(float[] m, int r, int c)
        {
            return m[r * 3 + c];
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
