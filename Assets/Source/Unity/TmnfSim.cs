using System;
using UnityEngine;

namespace Tmnf.Sim.Unity
{
    /// <summary>
    /// The game-facing vehicle driver: one <see cref="Sim"/> ticking at a
    /// FIXED 100 Hz, decoupled from the render frame.
    ///
    /// Correctness contract (see UNITY_SETUP.md):
    /// - One physics step is ALWAYS exactly one 10 ms tick inside the core.
    ///   Frame timing only decides HOW MANY steps run this frame — the step
    ///   size is never Time.deltaTime, never scaled, never paused-mid-tick.
    ///   Per-step results are therefore bit-identical to the golden traces
    ///   no matter the frame rate.
    /// - The visual transform is an INTERPOLATION between the last two sim
    ///   states (render-only; it is never fed back into the simulation).
    /// - This component never touches Unity physics: no Rigidbody, no
    ///   Collider, no FixedUpdate forces. The TMNF core owns all dynamics.
    ///
    /// Input contract: write <see cref="Input"/> once per rendered frame
    /// (before Update). The same button state is sampled for every fixed
    /// step that runs in that frame — exactly like the game's input packet.
    /// </summary>
    public sealed class TmnfSim : MonoBehaviour
    {
        /// <summary>One simulation tick is 10 ms — fixed by construction.</summary>
        public const float TickSeconds = 0.01f;

        [Header("Fixtures (relative to StreamingAssets)")]
        [Tooltip("TMNFTRK1 track image, e.g. Tmnf.Sim.GoldenData/fixtures/plain.tmnftrack")]
        public string trackFile = "Tmnf.Sim.GoldenData/fixtures/plain.tmnftrack";

        [Tooltip("TMNFM6G1 vehicle snapshot, e.g. Tmnf.Sim.GoldenData/fixtures/stadium.tmnfvehicle")]
        public string vehicleFile = "Tmnf.Sim.GoldenData/fixtures/stadium.tmnfvehicle";

        [Tooltip("128x128 fake-contact mask, e.g. Tmnf.Sim.GoldenData/fixtures/game-mask.bin")]
        public string maskFile = "Tmnf.Sim.GoldenData/fixtures/game-mask.bin";

        [Tooltip("Optional TMNFROU1 route (checkpoints/finish). Empty = sandbox.")]
        public string routeFile = "";

        [Header("Catch-up policy")]
        [Tooltip("Maximum simulation steps executed in one rendered frame (spiral-of-death guard).")]
        public int maxStepsPerFrame = 8;

        [Tooltip("Freezes the simulation: no steps run and no time accumulates. " +
                 "Used for the pre-race countdown and the pause menu — the vehicle " +
                 "state is held exactly, not merely hidden.")]
        public bool paused;

        /// <summary>
        /// The current frame's driving input. Write once per rendered frame;
        /// it is sampled for every fixed step of that frame.
        /// </summary>
        [NonSerialized] public TmnfInput Input;

        /// <summary>Raised after every fixed simulation step.</summary>
        public event Action<VehicleState> OnFixedStep;

        /// <summary>
        /// Raised once when the bound route's race finishes. The facade's
        /// State() observes canRespawn, which refuses a finished race (the
        /// upstream quirk documented in TmnfSimPureData); instead of letting
        /// that escape from the scheduler, the driver surfaces it here and
        /// keeps the last stepped state.
        /// </summary>
        public event Action<uint> OnRaceFinished;

        private bool finishNotified;

        /// <summary>The simulation. Null until loaded / after Dispose.</summary>
        public Sim Sim { get; private set; }

        /// <summary>Total fixed steps executed since load.</summary>
        public long Ticks { get; private set; }

        /// <summary>
        /// The interpolated render state (between the last two sim states).
        /// Valid after the first step.
        /// </summary>
        public RenderState Render
        {
            get
            {
                if (Sim == null || !hasState)
                {
                    return new RenderState();
                }
                float alpha = Mathf.Clamp01(accumulator / TickSeconds);
                VehicleState from = hasPrevious ? previous : current;
                return new RenderState
                {
                    position = Vector3.Lerp(
                        TmnfCoordinates.ToUnity(from.position),
                        TmnfCoordinates.ToUnity(current.position),
                        alpha),
                    rotation = Quaternion.Slerp(
                        TmnfCoordinates.ToUnity(from.quaternion[0], from.quaternion[1], from.quaternion[2], from.quaternion[3]),
                        TmnfCoordinates.ToUnity(current.quaternion[0], current.quaternion[1], current.quaternion[2], current.quaternion[3]),
                        alpha),
                    linearSpeed = TmnfCoordinates.ToUnity(current.linearSpeed),
                    angularSpeed = TmnfCoordinates.ToUnity(current.angularSpeed),
                    state = current,
                    alpha = alpha,
                };
            }
        }

        private float accumulator;
        private VehicleState previous;
        private VehicleState current;
        private bool hasPrevious;
        private bool hasState;

        private void Start()
        {
            // Auto-load only when fixture paths are configured; a driver
            // that will Attach() its own Sim leaves them empty.
            if (Sim == null
                && !string.IsNullOrEmpty(trackFile)
                && !string.IsNullOrEmpty(vehicleFile))
            {
                Load();
            }
        }

        /// <summary>
        /// Loads the fixtures and builds the simulation. Called from Start;
        /// call it yourself for explicit lifetime control.
        /// </summary>
        public void Load()
        {
            string streaming = Application.streamingAssetsPath;
            SimOptions options = new SimOptions
            {
                fakeContactMask = System.IO.Path.Combine(streaming, maskFile),
                route = string.IsNullOrEmpty(routeFile)
                    ? null
                    : System.IO.Path.Combine(streaming, routeFile),
                pinProvenance = false,
            };
            Sim = Sim.Load(
                System.IO.Path.Combine(streaming, trackFile),
                System.IO.Path.Combine(streaming, vehicleFile),
                options);
            current = Sim.State();
            hasState = true;
            accumulator = 0.0f;
            Ticks = 0;
        }

        /// <summary>
        /// Adopts an externally built simulation (a track converted from
        /// Unity geometry, for instance) and drives it with this component's
        /// fixed-rate scheduler and render interpolation. The sim must not
        /// be finished (its State() is read once here).
        /// </summary>
        public void Attach(Sim sim)
        {
            if (sim == null)
            {
                throw new InvalidOperationException("TmnfSim.Attach: null sim");
            }
            Sim = sim;
            current = sim.State();
            hasState = true;
            hasPrevious = false;
            accumulator = 0.0f;
            Ticks = 0;
            paused = false;
            finishNotified = false;
        }

        private void Update()
        {
            if (Sim == null || paused)
            {
                return;
            }
            // Frame time decides the step COUNT only (clamped so a hitch
            // cannot spiral): each executed step is one exact 10 ms tick.
            accumulator += Mathf.Min(Time.deltaTime, TickSeconds * maxStepsPerFrame);
            int steps = 0;
            while (accumulator >= TickSeconds && steps < maxStepsPerFrame)
            {
                StepOnce();
                accumulator -= TickSeconds;
                steps++;
            }
            if (steps == maxStepsPerFrame && accumulator >= TickSeconds)
            {
                // Over budget: drop the backlog rather than slow the sim —
                // the simulation itself must never run behind real time.
                accumulator = 0.0f;
            }
        }

        /// <summary>
        /// Executes exactly one fixed 10 ms step with the current
        /// <see cref="Input"/>. Use this to drive the sim from your own
        /// scheduler (deterministic replays, tests) instead of Update.
        /// When the race finishes on this step, the finishing quirk is
        /// swallowed and reported through <see cref="OnRaceFinished"/>.
        /// </summary>
        public VehicleState StepOnce()
        {
            if (Sim == null)
            {
                throw new InvalidOperationException("TmnfSim: not loaded");
            }
            previous = current;
            hasPrevious = true;
            TmnfInput input = Input;
            try
            {
                current = Sim.Step(ref input);
            }
            catch (InvalidOperationException) when (!finishNotified && IsRaceFinished())
            {
                // TmnfRaceStep already recorded the result; State() then
                // refused to observe the finished race (upstream quirk).
                // Report and keep the last good state.
                finishNotified = true;
                OnRaceFinished?.Invoke(Sim.Race().Value.finishTimeMs);
                return current;
            }
            hasState = true;
            Ticks++;
            OnFixedStep?.Invoke(current);
            return current;
        }

        private bool IsRaceFinished()
        {
            Race.TmnfRaceState? race = Sim.Race();
            return race.HasValue && race.Value.finished != 0;
        }

        /// <summary>The last stepped state without stepping.</summary>
        public VehicleState State()
        {
            if (Sim == null)
            {
                throw new InvalidOperationException("TmnfSim: not loaded");
            }
            return Sim.State();
        }

        /// <summary>
        /// Tears the simulation down. Safe to call repeatedly; Load() or
        /// Attach() may be called again afterwards (a full reset).
        /// </summary>
        public void Dispose()
        {
            Sim = null;
            hasState = false;
            hasPrevious = false;
            accumulator = 0.0f;
            Ticks = 0;
            paused = false;
            finishNotified = false;
        }

        private void OnDestroy()
        {
            Dispose();
        }
    }

    /// <summary>
    /// The render-facing state: interpolated transform plus the raw last
    /// simulation observation. Interpolation is visual only — the
    /// simulation states themselves are never blended.
    /// </summary>
    public struct RenderState
    {
        /// <summary>Interpolated world position (Unity frame).</summary>
        public Vector3 position;

        /// <summary>Interpolated world rotation (Unity frame).</summary>
        public Quaternion rotation;

        /// <summary>Current linear speed (Unity frame).</summary>
        public Vector3 linearSpeed;

        /// <summary>Current angular speed (Unity frame).</summary>
        public Vector3 angularSpeed;

        /// <summary>The last raw simulation observation (TMNF frame).</summary>
        public VehicleState state;

        /// <summary>The interpolation fraction used for this render frame.</summary>
        public float alpha;
    }
}
