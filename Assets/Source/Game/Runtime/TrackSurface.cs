using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// Physical surface override for one track mesh: which of the 31 TMNF
    /// physical materials (friction + restitution) the mesh reports to the
    /// simulation. Attach next to a MeshFilter under a TrackGeometry object;
    /// without it the hierarchy's TrackGeometry default applies.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class TrackSurface : MonoBehaviour
    {
        public enum SurfaceMaterial
        {
            Concrete = 0,
            Pavement = 1,
            Grass = 2,
            Ice = 3,
            Metal = 4,
            Sand = 5,
            Dirt = 6,
            Turbo = 7,
            DirtRoad = 8,
            Rubber = 9,
            SlidingRubber = 10,
            Test = 11,
            Rock = 12,
            Water = 13,
            Wood = 14,
            Danger = 15,
            Asphalt = 16,
            WetDirtRoad = 17,
            WetAsphalt = 18,
            WetPavement = 19,
            WetGrass = 20,
            Snow = 21,
            ResonantMetal = 22,
            GolfBall = 23,
            GolfWall = 24,
            GolfGround = 25,
            Turbo2 = 26,
            Bumper = 27,
            NotCollidable = 28,
            FreeWheeling = 29,
            TurboRoulette = 30,
        }

        [Tooltip("Physical material of every mesh on this GameObject " +
                 "(and children without their own TrackSurface).")]
        public SurfaceMaterial surface = SurfaceMaterial.Asphalt;

        public int Id => (int)surface;
    }
}
