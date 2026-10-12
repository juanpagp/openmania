using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// Marks a GameObject whose whole hierarchy is track collision geometry.
    ///
    /// When a race loads, every active MeshFilter under a TrackGeometry
    /// object (or under any GameObject tagged "Track") is baked into the
    /// custom simulation as a collidable triangle mesh: world position,
    /// rotation and scale are respected, the mesh needs Read/Write enabled
    /// in its import settings, and its triangle winding/normals decide which
    /// side is solid — exactly like a rendered surface (drive on the side
    /// you can see).
    ///
    /// Add this component to one object and parent your imported meshes
    /// beneath it. Gameplay objects (Start/Checkpoint/Finish) should NOT
    /// live under it unless you want their meshes to be solid.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class TrackGeometry : MonoBehaviour
    {
        [Tooltip("Physical material every mesh in this hierarchy reports " +
                 "(friction/restitution): 0 concrete, 1 pavement, 2 grass, " +
                 "3 ice, 16 asphalt, 7 turbo, ... Override per mesh with " +
                 "the TrackSurface component.")]
        [Range(0, 30)]
        public int defaultSurfaceMaterial = 0;
    }
}
