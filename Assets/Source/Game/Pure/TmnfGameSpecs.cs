using System;
using System.Collections.Generic;

namespace Tmnf.Game.Pure
{
    /// <summary>
    /// One collidable triangle mesh destined for the custom simulation, in
    /// TMNF world space (the frame the simulation core runs in: +Y up, car
    /// drives +Z; convert from Unity through TmnfCoordinates).
    ///
    /// The vertices are already transformed to world space (the Unity-side
    /// extractor bakes position/rotation/scale into them), so the resulting
    /// collision entry uses an identity transform. Winding: each triangle's
    /// vertex order must be consistent with its declared face normal under
    /// the right-hand rule — the narrowphase derives its side planes from
    /// edge x normal and the ellipsoid kernel re-derives the normal from the
    /// winding, so the two must agree. A face collides from the side its
    /// normal points toward.
    /// </summary>
    public sealed class GameMeshSpec
    {
        /// <summary>Editorial name (used in diagnostics only).</summary>
        public string name = "mesh";

        /// <summary>Flattened xyz triplets, TMNF world space.</summary>
        public float[] vertices = Array.Empty<float>();

        /// <summary>Flattened vertex-index triplets (one per triangle).</summary>
        public uint[] triangles = Array.Empty<uint>();

        /// <summary>
        /// Flattened xyz per triangle: unit face normal, winding-consistent.
        /// </summary>
        public float[] faceNormals = Array.Empty<float>();

        /// <summary>Physical material id (0..30) the faces report.</summary>
        public byte materialId = 0;

        public int VertexCount => vertices == null ? 0 : vertices.Length / 3;

        public int FaceCount => triangles == null ? 0 : triangles.Length / 3;
    }

    /// <summary>
    /// The pure track description: everything the TMNFTRK1 image needs.
    /// Materials follow the reference 31-entry physical table.
    /// </summary>
    public sealed class GameTrackSpec
    {
        /// <summary>Every collidable mesh of the track, world-space.</summary>
        public List<GameMeshSpec> meshes = new List<GameMeshSpec>();

        /// <summary>31 friction values, indexed by physical material id.</summary>
        public float[] friction = new float[TmnfTrackImage.MaterialCount];

        /// <summary>31 restitution values, indexed by physical material id.</summary>
        public float[] restitution = new float[TmnfTrackImage.MaterialCount];

        public GameTrackSpec()
        {
            TmnfTrackImage.DefaultMaterialTable(friction, restitution);
        }

        /// <summary>True when the track would give the car nothing to drive on.</summary>
        public bool IsEmpty => meshes == null || meshes.Count == 0;
    }

    /// <summary>
    /// One trigger-only volume (checkpoint or finish) in TMNF world space:
    /// an oriented box (half extents + rotation + center). Triggers never
    /// collide physically; the race layer tests the car's collision tree
    /// against a twelve-face box built from this record.
    /// </summary>
    public sealed class GameTriggerSpec
    {
        /// <summary>Box center, TMNF world space.</summary>
        public float cx;
        public float cy;
        public float cz;

        /// <summary>Box half extents (all strictly positive).</summary>
        public float hx;
        public float hy;
        public float hz;

        /// <summary>
        /// Row-major 3x3 rotation applied to the box (TMNF frame). Identity
        /// for an axis-aligned volume.
        /// </summary>
        public float[] rot =
        {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
        };

        public static GameTriggerSpec Box(float cx, float cy, float cz,
            float hx, float hy, float hz)
        {
            GameTriggerSpec t = new GameTriggerSpec();
            t.cx = cx;
            t.cy = cy;
            t.cz = cz;
            t.hx = hx;
            t.hy = hy;
            t.hz = hz;
            return t;
        }
    }

    /// <summary>
    /// The pure route description: the run start plus every checkpoint and
    /// finish trigger of the track, all in TMNF world space.
    /// </summary>
    public sealed class GameRouteSpec
    {
        /// <summary>
        /// Start transform: 12 floats — row-major 3x3 rotation then x/y/z
        /// translation (the GmIso4 memory layout). The rotation must be
        /// orthonormal; the car spawns here with this orientation.
        /// </summary>
        public float[] startIso =
        {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f,
        };

        /// <summary>Checkpoint triggers (may be empty, max 63).</summary>
        public List<GameTriggerSpec> checkpoints = new List<GameTriggerSpec>();

        /// <summary>Finish triggers (at least one; see route validation).</summary>
        public List<GameTriggerSpec> finishes = new List<GameTriggerSpec>();

        /// <summary>
        /// Half width of the synthesized reference corridor (the game only
        /// uses the corridor for projection bookkeeping, never for rules).
        /// </summary>
        public float corridorHalfWidth = 30.0f;

        /// <summary>Spacing of the synthesized centerline points, meters.</summary>
        public float centerlineSpacing = 8.0f;
    }
}
