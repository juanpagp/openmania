using Tmnf.Sim;
using Tmnf.Sim.Unity;
using UnityEngine;

namespace Tmnf.Game
{
    /// <summary>
    /// The placeholder vehicle: a box body with a cabin, a headlight bar,
    /// a spoiler and four cylinder wheels. Purely presentational — the
    /// custom simulation owns every aspect of the car's motion; this
    /// component only reads the state it produces (including the live wheel
    /// positions and spin from the simulation's own wheel state).
    ///
    /// The model is authored facing the visual car's -Z (the direction the
    /// simulation's car moves toward after the handedness conversion); the
    /// wheels follow <c>World.nodes[wheelNodeOf[i]].localIso</c>, so they
    /// track the simulated suspension exactly.
    ///
    /// <see cref="showVisuals"/> (plus the runtime V key) hides the whole
    /// visual hierarchy without touching the simulation or the collision
    /// debug rendering.
    /// </summary>
    public sealed class VehicleVisual : MonoBehaviour
    {
        [Tooltip("Show the visible car model. The simulation and the F3 " +
                 "collision debug rendering are unaffected by this flag.")]
        public bool showVisuals = true;

        private TmnfSim driver;
        private Transform visualRoot;
        private Transform[] wheels;
        private Transform[] wheelSpin;
        private float wheelRadius = 0.364f;

        /// <summary>Builds the placeholder under this component.</summary>
        public void Initialize(TmnfSim driver)
        {
            this.driver = driver;
            World world = driver.Sim != null ? driver.Sim.World : null;
            if (world != null)
            {
                for (int w = 0; w < 4; w++)
                {
                    float r = world.vehicle.wheels[w].radius;
                    if (r > 0.01f)
                    {
                        wheelRadius = r;
                    }
                }
            }
            Build();
            SyncLatest();
        }

        private void Build()
        {
            visualRoot = new GameObject("VehicleVisualRoot").transform;
            visualRoot.transform.SetParent(transform, false);

            Material body = GameMaterials.Lit(new Color(0.86f, 0.86f, 0.90f));
            Material cabin = GameMaterials.Lit(new Color(0.16f, 0.19f, 0.24f), 0.6f);
            Material accent = GameMaterials.Lit(new Color(1.0f, 0.75f, 0.10f), 0.4f);
            Material spoiler = GameMaterials.Lit(new Color(0.80f, 0.12f, 0.10f), 0.5f);

            // Body — sized to the Stadium car's collision ellipsoid union.
            AddBox(visualRoot, "Body", new Vector3(0f, 0.55f, -0.15f),
                new Vector3(1.95f, 0.80f, 3.90f), body);
            AddBox(visualRoot, "Cabin", new Vector3(0f, 1.05f, -0.05f),
                new Vector3(1.35f, 0.45f, 1.70f), cabin);
            // Headlight bar marks the front (-Z of the visual car).
            AddBox(visualRoot, "Headlights", new Vector3(0f, 0.55f, -2.03f),
                new Vector3(1.30f, 0.16f, 0.10f), accent);
            AddBox(visualRoot, "Spoiler", new Vector3(0f, 1.02f, 1.78f),
                new Vector3(1.70f, 0.07f, 0.38f), spoiler);
            AddBox(visualRoot, "SpoilerLeft", new Vector3(-0.72f, 0.86f, 1.78f),
                new Vector3(0.08f, 0.30f, 0.34f), spoiler);
            AddBox(visualRoot, "SpoilerRight", new Vector3(0.72f, 0.86f, 1.78f),
                new Vector3(0.08f, 0.30f, 0.34f), spoiler);

            wheels = new Transform[4];
            wheelSpin = new Transform[4];
            Material wheelMat = GameMaterials.Lit(new Color(0.10f, 0.10f, 0.11f), 0.25f);
            Material rimMat = GameMaterials.Lit(new Color(0.75f, 0.78f, 0.82f), 0.5f);
            for (int i = 0; i < 4; i++)
            {
                Transform holder = new GameObject("Wheel" + i).transform;
                holder.SetParent(visualRoot, false);
                Transform spin = new GameObject("Spin").transform;
                spin.SetParent(holder, false);
                GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                // Primitives come with a Unity collider — useless (and
                // misleading) for the custom simulation.
                Destroy(cylinder.GetComponent<Collider>());
                cylinder.name = "Tire";
                cylinder.transform.SetParent(spin, false);
                cylinder.transform.localScale = new Vector3(wheelRadius, 0.18f, wheelRadius);
                cylinder.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
                cylinder.GetComponent<MeshRenderer>().sharedMaterial = wheelMat;

                GameObject rim = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                Destroy(rim.GetComponent<Collider>());
                rim.name = "Rim";
                rim.transform.SetParent(spin, false);
                rim.transform.localScale = new Vector3(wheelRadius * 0.55f, 0.19f, wheelRadius * 0.55f);
                rim.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
                rim.GetComponent<MeshRenderer>().sharedMaterial = rimMat;

                wheels[i] = holder;
                wheelSpin[i] = spin;
            }
            ApplyVisibility();
        }

        private static void AddBox(
            Transform parent, string name, Vector3 localPosition, Vector3 size, Material material)
        {
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(box.GetComponent<Collider>());
            box.name = name;
            box.transform.SetParent(parent, false);
            box.transform.localPosition = localPosition;
            box.transform.localScale = size;
            box.GetComponent<MeshRenderer>().sharedMaterial = material;
        }

        /// <summary>Places the visuals at the latest interpolated state.</summary>
        public void SyncLatest()
        {
            if (driver == null || visualRoot == null)
            {
                return;
            }
            RenderState render = driver.Render;
            visualRoot.SetPositionAndRotation(render.position, render.rotation);

            World world = driver.Sim != null ? driver.Sim.World : null;
            if (world != null)
            {
                bool mirror = TmnfCoordinates.MirrorZ;
                for (int w = 0; w < 4; w++)
                {
                    int node = world.wheelNodeOf[w];
                    if (node < 0 || node >= world.nodes.Count)
                    {
                        continue;
                    }
                    GmIso4 local = world.nodes[node].localIso;
                    // Car-local TMNF -> Unity car-local: (x, y, -z).
                    Vector3 localPosition = new Vector3(local.t0, local.t1,
                        mirror ? -local.t2 : local.t2);
                    wheels[w].localPosition = localPosition;

                    // Wheel orientation: the sim iso rotation conjugated into
                    // Unity. The rolling spin lives on the Spin child below
                    // (rotating about the axle, holder-local X).
                    GmMat3 rot = local.Rotation();
                    Quaternion rotation = Quaternion.LookRotation(
                        new Vector3(-rot.m2, -rot.m5, rot.m8),
                        new Vector3(rot.m1, rot.m4, -rot.m7));
                    wheels[w].localRotation = rotation;

                    // TMNF RotX(phase) conjugates to a negated Unity spin:
                    // rotate the visible wheel the opposite way about X.
                    float phase = world.vehicle.wheels[w].realTime.rotationPhase;
                    wheelSpin[w].localRotation =
                        Quaternion.Euler(-phase * Mathf.Rad2Deg, 0f, 0f);
                }
            }
        }

        private void ApplyVisibility()
        {
            if (visualRoot != null)
            {
                visualRoot.gameObject.SetActive(showVisuals);
            }
        }

        /// <summary>Toggles the visible model (the V key).</summary>
        public void ToggleVisuals()
        {
            showVisuals = !showVisuals;
            ApplyVisibility();
        }

        private void LateUpdate()
        {
            SyncLatest();
        }
    }
}
