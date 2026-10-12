using System.Collections.Generic;
using Tmnf.Game.Pure;
using Tmnf.Sim;
using Tmnf.Sim.Unity;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Tmnf.Game
{
    /// <summary>
    /// The Unity-side half of the collision bridge: walks a loaded track
    /// scene, validates its gameplay objects and converts everything into
    /// the pure <see cref="GameTrackSpec"/>/<see cref="GameRouteSpec"/> the
    /// image builders consume.
    ///
    /// Geometry: every active MeshFilter under a TrackGeometry component
    /// (or a "Track"-tagged object) is baked — world transform applied,
    /// triangle winding aligned with the mesh's own normals so the solid
    /// side is the side you can see.
    ///
    /// Triggers: each Checkpoint/Finish designates one trigger-only
    /// BoxCollider; it is read as an oriented box through the same
    /// handedness conversion the vehicle uses, never as a Unity physics
    /// collider.
    /// </summary>
    public static class TrackSceneExtractor
    {
        /// <summary>
        /// Extracts and validates the track in the given loaded scene.
        /// Never throws for authoring mistakes — they land in
        /// <see cref="TrackDefinition.errors"/> with object names attached.
        /// </summary>
        public static TrackDefinition Extract(Scene scene)
        {
            TrackDefinition definition = new TrackDefinition();
            definition.scenePath = scene.path;

            Track[] tracks = Object.FindObjectsByType<Track>(
                FindObjectsInactive.Include, FindObjectsSortMode.None);
            if (tracks.Length == 0)
            {
                definition.errors.Add(
                    "No Track component found in the scene. Add a Track component " +
                    "to the track's root GameObject.");
                return definition;
            }
            if (tracks.Length > 1)
            {
                definition.errors.Add(
                    $"Multiple Track components found ({tracks.Length}); a track scene " +
                    "must have exactly one. Offenders: " + Names(tracks));
                return definition;
            }
            Track track = tracks[0];
            if (!track.gameObject.activeInHierarchy)
            {
                definition.errors.Add(
                    $"The Track root '{track.gameObject.name}' is inactive.");
                return definition;
            }
            definition.metadata = track;
            if (string.IsNullOrWhiteSpace(track.trackName))
            {
                definition.warnings.Add(
                    "Track name is empty; using the scene name instead.");
            }

            ExtractStart(track, definition);
            ExtractCheckpoints(scene, definition);
            ExtractFinishes(scene, definition);
            ExtractGeometry(track, definition);
            return definition;
        }

        // ------------------------------------------------------------------
        // gameplay objects
        // ------------------------------------------------------------------

        private static void ExtractStart(Track track, TrackDefinition definition)
        {
            StartBlock[] starts = Object.FindObjectsByType<StartBlock>(
                FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            if (starts.Length == 0)
            {
                definition.errors.Add(
                    "No StartBlock found. A valid track needs exactly one Start " +
                    "object defining where the car spawns.");
                return;
            }
            if (starts.Length > 1)
            {
                definition.errors.Add(
                    $"{starts.Length} StartBlock components found; a track must have " +
                    $"exactly one. Offenders: {Names(starts)}");
                return;
            }
            Transform spawn = starts[0].Spawn;
            if (starts[0].spawnPoint == null)
            {
                definition.warnings.Add(
                    $"StartBlock '{Name(starts[0])}' has no spawn point assigned; " +
                    "using its own transform. Point the 'Spawn Point' field at a " +
                    "child transform to control placement and facing.");
            }
            GmIso4 iso = TmnfCoordinates.ToTmnfSpawn(spawn);
            definition.routeSpec.startIso[0] = iso.m0;
            definition.routeSpec.startIso[1] = iso.m1;
            definition.routeSpec.startIso[2] = iso.m2;
            definition.routeSpec.startIso[3] = iso.m3;
            definition.routeSpec.startIso[4] = iso.m4;
            definition.routeSpec.startIso[5] = iso.m5;
            definition.routeSpec.startIso[6] = iso.m6;
            definition.routeSpec.startIso[7] = iso.m7;
            definition.routeSpec.startIso[8] = iso.m8;
            definition.routeSpec.startIso[9] = iso.t0;
            definition.routeSpec.startIso[10] = iso.t1;
            definition.routeSpec.startIso[11] = iso.t2;
        }

        private static void ExtractCheckpoints(Scene scene, TrackDefinition definition)
        {
            CheckpointBlock[] blocks = Object.FindObjectsByType<CheckpointBlock>(
                FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            if (blocks.Length > 63)
            {
                definition.errors.Add(
                    $"{blocks.Length} checkpoints found; the simulation supports " +
                    "at most 63.");
            }
            foreach (CheckpointBlock block in blocks)
            {
                GameTriggerSpec trigger = ExtractTrigger(
                    block.triggerCollider, "Checkpoint", Name(block), definition);
                if (trigger != null)
                {
                    definition.routeSpec.checkpoints.Add(trigger);
                }
            }
        }

        private static void ExtractFinishes(Scene scene, TrackDefinition definition)
        {
            FinishBlock[] blocks = Object.FindObjectsByType<FinishBlock>(
                FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            if (blocks.Length == 0)
            {
                definition.errors.Add(
                    "No FinishBlock found. A valid track needs at least one " +
                    "Finish object.");
                return;
            }
            if (blocks.Length > 64)
            {
                definition.errors.Add(
                    $"{blocks.Length} finishes found; far too many (max 64 " +
                    "triggers total with the checkpoints).");
            }
            foreach (FinishBlock block in blocks)
            {
                GameTriggerSpec trigger = ExtractTrigger(
                    block.triggerCollider, "Finish", Name(block), definition);
                if (trigger != null)
                {
                    definition.routeSpec.finishes.Add(trigger);
                }
            }
        }

        /// <summary>
        /// Reads one designated BoxCollider as an oriented box in TMNF space.
        /// The eight corners go through the collider's full local-to-world
        /// matrix (so rotation, scale and hierarchy offsets are all
        /// respected); the result is checked to be a proper box — a skewed
        /// hierarchy cannot silently produce wrong gameplay volumes.
        /// </summary>
        private static GameTriggerSpec ExtractTrigger(
            BoxCollider collider, string what, string owner, TrackDefinition definition)
        {
            if (collider == null)
            {
                definition.errors.Add(
                    $"{what} '{owner}' has no trigger collider assigned. Add a " +
                    "trigger-only BoxCollider and assign it to the " +
                    "'Trigger Collider' field.");
                return null;
            }
            if (!collider.isTrigger)
            {
                definition.errors.Add(
                    $"{what} '{owner}': the BoxCollider '{collider.name}' is not a " +
                    "trigger. Enable 'Is Trigger' — gameplay volumes must never " +
                    "physically block the car.");
                return null;
            }
            if (!collider.enabled || !collider.gameObject.activeInHierarchy)
            {
                definition.errors.Add(
                    $"{what} '{owner}': the trigger collider '{collider.name}' is " +
                    "disabled.");
                return null;
            }
            if (collider.size.x <= 0.0f || collider.size.y <= 0.0f || collider.size.z <= 0.0f)
            {
                definition.errors.Add(
                    $"{what} '{owner}': the trigger collider '{collider.name}' has " +
                    "a zero size.");
                return null;
            }

            // Corners in collider-local space, through the world matrix.
            Vector3 c = collider.center;
            Vector3 h = collider.size * 0.5f;
            Matrix4x4 l2w = collider.transform.localToWorldMatrix;
            Vector3[] corner = new Vector3[8];
            for (int i = 0; i < 8; i++)
            {
                Vector3 local = new Vector3(
                    c.x + ((i & 1) == 0 ? -h.x : h.x),
                    c.y + ((i & 2) == 0 ? -h.y : h.y),
                    c.z + ((i & 4) == 0 ? -h.z : h.z));
                corner[i] = l2w.MultiplyPoint3x4(local);
            }

            Vector3 e1 = corner[1] - corner[0]; // local x edge
            Vector3 e2 = corner[2] - corner[0]; // local y edge
            Vector3 e3 = corner[4] - corner[0]; // local z edge
            float l1 = e1.magnitude;
            float l2 = e2.magnitude;
            float l3 = e3.magnitude;
            if (l1 < 1e-5f || l2 < 1e-5f || l3 < 1e-5f)
            {
                definition.errors.Add(
                    $"{what} '{owner}': the trigger collider '{collider.name}' " +
                    "degenerates to a flat volume at its world transform.");
                return null;
            }
            float shear = Mathf.Max(
                Mathf.Abs(Vector3.Dot(e1, e2)) / (l1 * l2),
                Mathf.Max(
                    Mathf.Abs(Vector3.Dot(e1, e3)) / (l1 * l3),
                    Mathf.Abs(Vector3.Dot(e2, e3)) / (l2 * l3)));
            if (shear > 0.01f)
            {
                definition.errors.Add(
                    $"{what} '{owner}': the trigger collider '{collider.name}' is " +
                    "SHEARED by its transform hierarchy (a rotated parent with " +
                    "non-uniform scale). Use uniform scale on the collider's " +
                    "ancestors, or bake the scale into the collider size.");
                return null;
            }

            Vector3 centerU = corner[0] + (e1 + e2 + e3) * 0.5f;
            Vector3 a1 = e1 / l1;
            Vector3 a2 = e2 / l2;
            Vector3 a3 = e3 / l3;

            GameTriggerSpec spec = new GameTriggerSpec();
            // TMNF world (mirror Z) — extents are mirror-invariant.
            spec.cx = centerU.x;
            spec.cy = centerU.y;
            spec.cz = TmnfCoordinates.MirrorZ ? -centerU.z : centerU.z;
            spec.hx = l1 * 0.5f;
            spec.hy = l2 * 0.5f;
            spec.hz = l3 * 0.5f;
            // The iso rotation maps local box axes to (a1, a2, a3) — stored
            // row-major, so column j is axis j.
            spec.rot[0] = a1.x; spec.rot[1] = a2.x; spec.rot[2] = a3.x;
            spec.rot[3] = a1.y; spec.rot[4] = a2.y; spec.rot[5] = a3.y;
            spec.rot[6] = a1.z; spec.rot[7] = a2.z; spec.rot[8] = a3.z;
            return spec;
        }

        // ------------------------------------------------------------------
        // geometry
        // ------------------------------------------------------------------

        private static void ExtractGeometry(Track track, TrackDefinition definition)
        {
            HashSet<MeshFilter> found = new HashSet<MeshFilter>();

            // Every active TrackGeometry component in the scene contributes
            // its whole subtree (they may live under the track root or loose).
            TrackGeometry[] markers = Object.FindObjectsByType<TrackGeometry>(
                FindObjectsInactive.Include, FindObjectsSortMode.None);
            foreach (TrackGeometry marker in markers)
            {
                if (!marker.gameObject.activeInHierarchy)
                {
                    definition.warnings.Add(
                        $"TrackGeometry '{Name(marker)}' is inactive; its meshes " +
                        "were skipped.");
                    continue;
                }
                CollectSubtreeMeshes(marker.transform, found, definition);
            }

            // 'Track'-tagged GameObjects under the track root also count.
            CollectTaggedMeshes(track.transform, found, definition);

            if (found.Count == 0)
            {
                definition.errors.Add(
                    "No track geometry found. Parent your meshes under a " +
                    "TrackGeometry object (or tag objects 'Track') so they can " +
                    "collide with the vehicle.");
                return;
            }

            foreach (MeshFilter filter in found)
            {
                ExtractMesh(filter, definition);
            }
        }

        private static void CollectSubtreeMeshes(
            Transform node, HashSet<MeshFilter> found, TrackDefinition definition)
        {
            for (int i = 0; i < node.childCount; i++)
            {
                CollectSubtreeMeshes(node.GetChild(i), found, definition);
            }
            if (node.gameObject.activeInHierarchy)
            {
                MeshFilter filter = node.GetComponent<MeshFilter>();
                if (filter != null && filter.sharedMesh != null)
                {
                    found.Add(filter);
                }
            }
            else
            {
                MeshFilter dormant = node.GetComponent<MeshFilter>();
                if (dormant != null && dormant.sharedMesh != null)
                {
                    definition.warnings.Add(
                        $"Mesh '{Name(node)}' is inactive and was skipped.");
                }
            }
        }

        private static void CollectTaggedMeshes(
            Transform node, HashSet<MeshFilter> found, TrackDefinition definition)
        {
            if (node.CompareTag("Track"))
            {
                CollectSubtreeMeshes(node, found, definition);
                return; // the tagged subtree fully covers its children
            }
            for (int i = 0; i < node.childCount; i++)
            {
                CollectTaggedMeshes(node.GetChild(i), found, definition);
            }
        }

        private static void ExtractMesh(MeshFilter filter, TrackDefinition definition)
        {
            Mesh mesh = filter.sharedMesh;
            string meshName = Name(filter.transform);
            if (!mesh.isReadable)
            {
                definition.errors.Add(
                    $"Mesh '{meshName}' is not readable. Select its asset and " +
                    "enable 'Read/Write' in the import settings (the track " +
                    "converter reads the triangles at load time).");
                return;
            }

            int surfaceMaterial = ResolveSurfaceMaterial(filter.transform);

            Vector3[] localVerts = mesh.vertices;
            Vector3[] localNormals = mesh.normals;
            Transform t = filter.transform;

            // Bake the whole hierarchy into TMNF world space.
            float[] verts = new float[localVerts.Length * 3];
            bool mirror = TmnfCoordinates.MirrorZ;
            for (int i = 0; i < localVerts.Length; i++)
            {
                Vector3 world = t.TransformPoint(localVerts[i]);
                verts[i * 3 + 0] = world.x;
                verts[i * 3 + 1] = world.y;
                verts[i * 3 + 2] = mirror ? -world.z : world.z;
            }

            List<uint> triangles = new List<uint>();
            List<float> normals = new List<float>();
            int degenerate = 0;
            int flipped = 0;
            int subMeshCount = mesh.subMeshCount;
            for (int s = 0; s < subMeshCount; s++)
            {
                int[] subTriangles = mesh.GetTriangles(s);
                for (int f = 0; f < subTriangles.Length; f += 3)
                {
                    uint i0 = (uint)subTriangles[f];
                    uint i1 = (uint)subTriangles[f + 1];
                    uint i2 = (uint)subTriangles[f + 2];
                    float ax = verts[i0 * 3 + 0];
                    float ay = verts[i0 * 3 + 1];
                    float az = verts[i0 * 3 + 2];
                    float bx = verts[i1 * 3 + 0];
                    float by = verts[i1 * 3 + 1];
                    float bz = verts[i1 * 3 + 2];
                    float cx = verts[i2 * 3 + 0];
                    float cy = verts[i2 * 3 + 1];
                    float cz = verts[i2 * 3 + 2];
                    // Geometric normal, right-hand rule in the TMNF frame.
                    float e1x = bx - ax;
                    float e1y = by - ay;
                    float e1z = bz - az;
                    float e2x = cx - ax;
                    float e2y = cy - ay;
                    float e2z = cz - az;
                    float nx = e1y * e2z - e1z * e2y;
                    float ny = e1z * e2x - e1x * e2z;
                    float nz = e1x * e2y - e1y * e2x;
                    float length = Mathf.Sqrt(nx * nx + ny * ny + nz * nz);
                    if (length < 1e-12f)
                    {
                        degenerate++;
                        continue;
                    }
                    nx /= length;
                    ny /= length;
                    nz /= length;

                    // Align the winding with the mesh's authored normals so
                    // the collidable side is the side Unity renders.
                    if (localNormals != null && localNormals.Length == localVerts.Length)
                    {
                        Vector3 authoredU = t.TransformDirection(
                            (localNormals[i0] + localNormals[i1] + localNormals[i2]) / 3.0f);
                        float axn = authoredU.x;
                        float ayn = authoredU.y;
                        float azn = mirror ? -authoredU.z : authoredU.z;
                        float authoredLength = Mathf.Sqrt(
                            axn * axn + ayn * ayn + azn * azn);
                        if (authoredLength > 1e-6f)
                        {
                            float dot = (nx * axn + ny * ayn + nz * azn) / authoredLength;
                            if (dot < 0.0f)
                            {
                                // Flip winding: swap v1/v2 and negate the normal.
                                uint tmp = i1;
                                i1 = i2;
                                i2 = tmp;
                                nx = -nx;
                                ny = -ny;
                                nz = -nz;
                                flipped++;
                            }
                        }
                    }

                    triangles.Add(i0);
                    triangles.Add(i1);
                    triangles.Add(i2);
                    normals.Add(nx);
                    normals.Add(ny);
                    normals.Add(nz);
                }
            }

            if (triangles.Count == 0)
            {
                if (degenerate > 0)
                {
                    definition.errors.Add(
                        $"Mesh '{meshName}' has only degenerate (zero-area) " +
                        "triangles; it cannot collide.");
                }
                else
                {
                    definition.errors.Add(
                        $"Mesh '{meshName}' has no triangles; it cannot collide.");
                }
                return;
            }
            if (degenerate > 0)
            {
                definition.warnings.Add(
                    $"Mesh '{meshName}': skipped {degenerate} degenerate triangles.");
            }
            if (flipped > 0)
            {
                definition.warnings.Add(
                    $"Mesh '{meshName}': flipped {flipped} triangles whose winding " +
                    "opposed their authored normals.");
            }

            GameMeshSpec spec = new GameMeshSpec();
            spec.name = meshName;
            spec.vertices = verts;
            spec.triangles = triangles.ToArray();
            spec.faceNormals = normals.ToArray();
            spec.materialId = (byte)surfaceMaterial;
            definition.trackSpec.meshes.Add(spec);
        }

        private static int ResolveSurfaceMaterial(Transform node)
        {
            Transform t = node;
            while (t != null)
            {
                TrackSurface surface = t.GetComponent<TrackSurface>();
                if (surface != null)
                {
                    return surface.Id;
                }
                TrackGeometry geometry = t.GetComponent<TrackGeometry>();
                if (geometry != null)
                {
                    return geometry.defaultSurfaceMaterial;
                }
                t = t.parent;
            }
            return 0;
        }

        // ------------------------------------------------------------------
        // helpers
        // ------------------------------------------------------------------

        private static string Name(Component component)
        {
            return component != null && component.transform != null
                ? component.transform.name
                : "(null)";
        }

        private static string Names<T>(T[] components) where T : Component
        {
            List<string> names = new List<string>();
            foreach (T component in components)
            {
                names.Add(Name(component));
            }
            return string.Join(", ", names);
        }
    }
}
