using UnityEngine;

namespace Tmnf.Sim.Unity
{
    /// <summary>
    /// THE one place where TMNF coordinates become Unity coordinates.
    ///
    /// The simulation core is a transliteration of the game's math and keeps
    /// the game's native frame: +Y is up, gravity pulls -Y, and the car
    /// drives along +Z. All game-side code (rendering, cameras, effects)
    /// must go through this class — never scatter conversion math into
    /// gameplay scripts, or the frame convention forks and becomes
    /// undebuggable.
    ///
    /// Handedness: the physics itself is handedness-agnostic (plain matrix
    /// algebra), but the MODEL you import for the car was authored in a
    /// concrete frame. If your car renders mirrored (text on the body
    /// reversed, steering inverted visually), flip <see cref="MirrorZ"/> to
    /// the other value — this is the ONLY line you should ever need to
    /// change. The default mirrors Z, which maps a right-handed authored
    /// model into Unity's left-handed frame; imported left-handed (fbx with
    /// -Z forward converted by Unity to +Z forward) assets usually want
    /// <c>false</c>.
    /// </summary>
    public static class TmnfCoordinates
    {
        /// <summary>
        /// Unity(x, y, z) = TMNF(x, y, MirrorZ ? -z : z).
        /// Change this HERE if your rendering looks mirrored. One line, one
        /// place, applies to every conversion below.
        /// </summary>
        public static readonly bool MirrorZ = true;

        /// <summary>TMNF world position/vector → Unity position/vector.</summary>
        public static Vector3 ToUnity(GmVec3 v)
        {
            return MirrorZ
                ? new Vector3(v.x, v.y, -v.z)
                : new Vector3(v.x, v.y, v.z);
        }

        /// <summary>Unity position/vector → TMNF world vector.</summary>
        public static GmVec3 ToTmnf(Vector3 v)
        {
            return MirrorZ
                ? new GmVec3(v.x, v.y, -v.z)
                : new GmVec3(v.x, v.y, v.z);
        }

        /// <summary>
        /// TMNF car quaternion (exposed by <see cref="VehicleState"/> as
        /// x, y, z, w) → Unity rotation. A mirror similarity transforms a
        /// rotation about axis (x, y, z) by angle θ into a rotation about
        /// the mirrored axis by -θ, which for the quaternion is
        /// (-x, -y, ±z, w) — equivalent up to the global quaternion sign,
        /// which Unity ignores.
        /// </summary>
        public static Quaternion ToUnity(float x, float y, float z, float w)
        {
            return MirrorZ
                ? new Quaternion(-x, -y, z, w)
                : new Quaternion(x, y, z, w);
        }

        /// <summary>Unity rotation → TMNF quaternion (x, y, z, w).</summary>
        public static void ToTmnf(Quaternion q, out float x, out float y, out float z, out float w)
        {
            if (MirrorZ)
            {
                x = -q.x;
                y = -q.y;
                z = q.z;
                w = q.w;
            }
            else
            {
                x = q.x;
                y = q.y;
                z = q.z;
                w = q.w;
            }
        }
    }
}
