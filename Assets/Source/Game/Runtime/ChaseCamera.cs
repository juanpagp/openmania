using Tmnf.Sim.Unity;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// A smooth chase camera for the simulated car: sits behind and above
    /// the interpolated render state and looks ahead of it. Purely
    /// presentational — it never feeds anything back into the simulation.
    /// </summary>
    public sealed class ChaseCamera : MonoBehaviour
    {
        [Tooltip("Distance behind the car (along the car's forward).")]
        public float distance = 7.0f;

        [Tooltip("Height above the car.")]
        public float height = 2.8f;

        [Tooltip("Look-at height above the car.")]
        public float lookHeight = 1.2f;

        [Tooltip("Position smoothing (seconds of lag; 0 = hard lock).")]
        public float positionSmoothing = 0.12f;

        [Tooltip("Degrees of yaw the car must turn before the camera follows.")]
        public float followYawSpeed = 10.0f;

        private TmnfSim driver;
        private Vector3 velocity;
        private bool snapped;

        public void Initialize(TmnfSim driver)
        {
            this.driver = driver;
            snapped = false;
        }

        private void LateUpdate()
        {
            if (driver == null || driver.Sim == null)
            {
                return;
            }
            RenderState render = driver.Render;
            Quaternion rotation = render.rotation;
            // The visual car faces its local -Z (the handedness-converted
            // forward), so the render transform's +Z is BEHIND the car.
            Vector3 behind = rotation * Vector3.forward;
            if (behind.sqrMagnitude < 1e-6f)
            {
                behind = Vector3.forward;
            }
            behind = Flatten(behind);
            if (behind.sqrMagnitude < 1e-6f)
            {
                behind = Vector3.forward;
            }
            behind.Normalize();

            Vector3 target = render.position + Vector3.up * height + behind * distance;
            if (!snapped)
            {
                transform.position = target;
                velocity = Vector3.zero;
                snapped = true;
            }
            else if (positionSmoothing > 0.0f)
            {
                transform.position = Vector3.SmoothDamp(
                    transform.position, target, ref velocity, positionSmoothing);
            }
            else
            {
                transform.position = target;
            }

            // Look at a point ahead of the car (opposite of behind).
            Vector3 look = render.position + Vector3.up * lookHeight - behind * 3.0f;
            transform.rotation = Quaternion.LookRotation(look - transform.position, Vector3.up);
        }

        private static Vector3 Flatten(Vector3 v)
        {
            v.y = 0.0f;
            return v;
        }
    }
}
