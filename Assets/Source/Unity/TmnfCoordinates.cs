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

        /// <summary>
        /// A TMNF isometry (row-major rotation + translation) as a Unity
        /// TRS matrix. The conjugation R_u = M * R_t * M (M = the Z mirror,
        /// so entries with exactly one index on the Z axis flip sign) makes
        /// the matrix act correctly on Unity points; the translation
        /// mirrors like any vector. Consistent with the quaternion path
        /// above (ToUnity on the iso's quaternion yields the same rotation).
        /// Use for gizmos, debug drawing and any place a sim-space
        /// transform must be shown in the Unity scene.
        /// </summary>
        public static Matrix4x4 ToUnity(ref GmIso4 iso)
        {
            Matrix4x4 m = default(Matrix4x4);
            if (MirrorZ)
            {
                m.m00 = iso.m0; m.m01 = iso.m1; m.m02 = -iso.m2;
                m.m10 = iso.m3; m.m11 = iso.m4; m.m12 = -iso.m5;
                m.m20 = -iso.m6; m.m21 = -iso.m7; m.m22 = iso.m8;
                m.m03 = iso.t0; m.m13 = iso.t1; m.m23 = -iso.t2;
            }
            else
            {
                m.m00 = iso.m0; m.m01 = iso.m1; m.m02 = iso.m2;
                m.m10 = iso.m3; m.m11 = iso.m4; m.m12 = iso.m5;
                m.m20 = iso.m6; m.m21 = iso.m7; m.m22 = iso.m8;
                m.m03 = iso.t0; m.m13 = iso.t1; m.m23 = iso.t2;
            }
            m.m33 = 1.0f;
            return m;
        }

        /// <summary>
        /// The TMNF spawn isometry for a Unity transform, with the Unity
        /// convention that the transform's +Z (blue axis) is the direction
        /// the car must drive toward.
        ///
        /// The simulation's car drives along its local +Z in the TMNF frame;
        /// the mirror maps that axis onto Unity -Z, so a plain conversion of
        /// the transform's rotation would make the car drive toward the
        /// spawn's BACKWARD. Composing a 180-degree yaw before the mirror
        /// conversion aligns the car's forward with the spawn's Unity +Z,
        /// its up with the spawn's up, and its right with the spawn's right —
        /// everything a track author expects from orienting a Start block.
        /// </summary>
        public static GmIso4 ToTmnfSpawn(Transform spawn)
        {
            Quaternion facing = spawn.rotation * Quaternion.Euler(0.0f, 180.0f, 0.0f);
            float x;
            float y;
            float z;
            float w;
            if (MirrorZ)
            {
                x = -facing.x;
                y = -facing.y;
                z = facing.z;
                w = facing.w;
            }
            else
            {
                x = facing.x;
                y = facing.y;
                z = facing.z;
                w = facing.w;
            }
            // Standard quaternion -> row-major rotation (the same schedule
            // the core's GmMat3.SetFromQuat uses, with the scalar first).
            GmIso4 iso = default(GmIso4);
            iso.m0 = 1.0f - 2.0f * (y * y + z * z);
            iso.m1 = 2.0f * (x * y - z * w);
            iso.m2 = 2.0f * (x * z + y * w);
            iso.m3 = 2.0f * (x * y + z * w);
            iso.m4 = 1.0f - 2.0f * (x * x + z * z);
            iso.m5 = 2.0f * (y * z - x * w);
            iso.m6 = 2.0f * (x * z - y * w);
            iso.m7 = 2.0f * (y * z + x * w);
            iso.m8 = 1.0f - 2.0f * (x * x + y * y);
            iso.t0 = spawn.position.x;
            iso.t1 = spawn.position.y;
            iso.t2 = MirrorZ ? -spawn.position.z : spawn.position.z;
            return iso;
        }
    }
}
