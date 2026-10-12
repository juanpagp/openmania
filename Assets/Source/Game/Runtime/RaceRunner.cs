using System.Collections;
using System.Collections.Generic;
using Tmnf.Game.Pure;
using Tmnf.Sim;
using Tmnf.Sim.Unity;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;

namespace Tmnf.Game
{
    // 'Sim' inside any Tmnf.* namespace resolves to the sibling namespace
    // Tmnf.Sim before the imported class; alias the facade explicitly.
    using Sim = global::Tmnf.Sim.Sim;

    /// <summary>
    /// The race lifecycle: load the selected track scene, validate it,
    /// convert its geometry into the custom simulation, spawn the vehicle at
    /// the Start, run the countdown, race, detect checkpoints/finish, time,
    /// show results, and restart cleanly — all around the untouched
    /// physics core.
    ///
    /// State machine: LoadingTrack -> (Failed | Countdown -> Racing ->
    /// Finished), with Paused reachable from Countdown/Racing. Every (re)
    /// start builds a fresh Sim from the cached track/route images, so a
    /// restart can never inherit stale race or vehicle state.
    /// </summary>
    public sealed class RaceRunner : MonoBehaviour
    {
        private enum State
        {
            LoadingTrack,
            Countdown,
            Racing,
            Finished,
            Paused,
            Failed,
        }

        private const string MainMenuScene = "Assets/Scenes/MainMenu.unity";
        private const float CountdownSeconds = 1.5f;

        [Header("Wired at runtime")]
        [Tooltip("Camera the race uses (auto: the scene's main camera).")]
        public Camera raceCamera;

        private TmnfSim simDriver;
        private VehicleVisual vehicleVisual;
        private ChaseCamera chaseCamera;
        private CollisionDebugVisualizer debugVisualizer;
        private RaceHud hud;
        private readonly RaceInput input = new RaceInput();

        private TrackDefinition definition;
        private byte[] trackBytes;
        private byte[] routeBytes;
        private byte[] vehicleBlob;
        private byte[] maskBlob;
        private Scene loadedTrackScene;

        private State state = State.LoadingTrack;
        private float countdownRemaining = CountdownSeconds;
        private bool visualsCreated;
        private bool hudBuilt;
        private bool resultsShown;

        private int personalBestBefore;
        private int authorTimeMs;
        private int referenceMs = -1;
        private string personalBestKey;

        private uint finishedTimeMs;

        private void Awake()
        {
            if (raceCamera == null)
            {
                raceCamera = Camera.main;
            }
            if (raceCamera == null)
            {
                raceCamera = GetComponentInChildren<Camera>();
            }
            if (raceCamera == null)
            {
                Debug.LogError("[Tmnf.Game] no camera in the Race scene; add one.");
            }

            simDriver = gameObject.AddComponent<TmnfSim>();
            simDriver.trackFile = ""; // everything is attached explicitly
            simDriver.vehicleFile = "";
            simDriver.maskFile = "";
            simDriver.OnFixedStep += OnSimFixedStep;
            simDriver.OnRaceFinished += OnSimRaceFinished;

            vehicleVisual = gameObject.AddComponent<VehicleVisual>();
            chaseCamera = raceCamera != null
                ? raceCamera.gameObject.AddComponent<ChaseCamera>()
                : gameObject.AddComponent<ChaseCamera>();
            debugVisualizer = gameObject.AddComponent<CollisionDebugVisualizer>();
            hud = gameObject.AddComponent<RaceHud>();
            hud.onRetry = RestartRun;
            hud.onResume = ResumeFromPause;
            hud.onRestartFromPause = RestartRun;
            hud.onTrackSelect = GoToTrackSelect;
            hud.onMainMenu = GoToMainMenu;
            hud.onErrorBack = GoToTrackSelect;

            EnsureEventSystem();
        }

        private void Start()
        {
            StartCoroutine(LoadTrackAndStart());
        }

        private IEnumerator LoadTrackAndStart()
        {
            string scenePath = GameSession.selectedTrackScene;
            if (string.IsNullOrEmpty(scenePath))
            {
                Fail("No track selected. Pick a track from the track selection screen.");
                yield break;
            }

            AsyncOperation load = SceneManager.LoadSceneAsync(scenePath, LoadSceneMode.Additive);
            if (load == null)
            {
                Fail($"Could not load the track scene '{scenePath}'. Is it added to the " +
                    "track catalog and present in the project?");
                yield break;
            }
            while (!load.isDone)
            {
                yield return null;
            }

            loadedTrackScene = SceneManager.GetSceneByPath(scenePath);
            if (!loadedTrackScene.IsValid() || !loadedTrackScene.isLoaded)
            {
                Fail($"The track scene '{scenePath}' did not load.");
                yield break;
            }
            DisableSceneCameras(loadedTrackScene);

            TrackDefinition extracted;
            try
            {
                extracted = TrackSceneExtractor.Extract(loadedTrackScene);
            }
            catch (System.Exception e)
            {
                Fail($"Track extraction crashed: {e.Message}");
                yield break;
            }
            definition = extracted;
            foreach (string warning in definition.warnings)
            {
                Debug.LogWarning($"[Tmnf.Game] track: {warning}");
            }
            if (!definition.Valid)
            {
                state = State.Failed;
                BuildHudOnce();
                hud.ShowErrors(definition.errors);
                Debug.LogError("[Tmnf.Game] the track is invalid — the race cannot start:\n" +
                    string.Join("\n", definition.errors));
                yield break;
            }

            try
            {
                // Build both simulation images and validate them through the
                // reference loaders (throws with precise messages).
                trackBytes = TmnfTrackImage.Build(definition.trackSpec);
                routeBytes = TmnfRouteImage.Build(definition.routeSpec);
                RouteLoader.TmnfRouteLoadBytes(routeBytes, null);
                TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
                vehicleBlob = VehicleFactory.LoadVehicleBlob();
                maskBlob = VehicleFactory.LoadMask();
            }
            catch (System.Exception e)
            {
                Fail($"Building the simulation from the track failed: {e.Message}");
                yield break;
            }

            if (definition.metadata != null && definition.metadata.applyDefaultMaterialOnLoad)
            {
                ApplyDefaultMaterial(loadedTrackScene);
            }

            BuildHudOnce();
            StartRace();
        }

        // ------------------------------------------------------------------
        // race lifecycle
        // ------------------------------------------------------------------

        private void StartRace()
        {
            simDriver.Dispose();
            TrackLoader.TmnfTrack track = TrackLoader.TmnfTrackLoadBytes(trackBytes, null);
            Sim sim = new Sim(track, vehicleBlob, maskBlob, new SimOptions
            {
                fakeContactMask = null,
                route = null,
                routeBytes = routeBytes,
                pinProvenance = false,
            });

            // Place the vehicle at the track's Start transform (the Sim
            // leaves it at the snapshot's authored origin until respawned).
            GmIso4 startIso = default(GmIso4);
            startIso.m0 = definition.routeSpec.startIso[0];
            startIso.m1 = definition.routeSpec.startIso[1];
            startIso.m2 = definition.routeSpec.startIso[2];
            startIso.m3 = definition.routeSpec.startIso[3];
            startIso.m4 = definition.routeSpec.startIso[4];
            startIso.m5 = definition.routeSpec.startIso[5];
            startIso.m6 = definition.routeSpec.startIso[6];
            startIso.m7 = definition.routeSpec.startIso[7];
            startIso.m8 = definition.routeSpec.startIso[8];
            startIso.t0 = definition.routeSpec.startIso[9];
            startIso.t1 = definition.routeSpec.startIso[10];
            startIso.t2 = definition.routeSpec.startIso[11];
            sim.World.Respawn(ref startIso);

            simDriver.Attach(sim);
            simDriver.paused = true; // held until the countdown ends

            if (!visualsCreated)
            {
                vehicleVisual.Initialize(simDriver);
                chaseCamera.Initialize(simDriver);
                debugVisualizer.Initialize(simDriver, definition);
                visualsCreated = true;
            }
            else
            {
                vehicleVisual.SyncLatest();
            }

            personalBestKey = definition.scenePath;
            personalBestBefore = PersonalBestStore.Get(personalBestKey);
            authorTimeMs = definition.metadata != null ? definition.metadata.AuthorTimeOrNone : -1;
            referenceMs = personalBestBefore > 0 ? personalBestBefore : authorTimeMs;

            hud.HideResults();
            hud.HidePause();
            hud.SetProgress(0, definition.routeSpec.checkpoints.Count);
            hud.SetTime(0, referenceMs, referenceMs > 0);
            resultsShown = false;
            finishedTimeMs = 0;

            countdownRemaining = CountdownSeconds;
            state = State.Countdown;
        }

        /// <summary>Restart the run from the beginning (R, Retry, pause menu).</summary>
        private void RestartRun()
        {
            if (state == State.Failed || trackBytes == null)
            {
                return;
            }
            StartRace();
        }

        private void Update()
        {
            if (!hudBuilt)
            {
                return; // still loading the track scene / building the race
            }
            input.Sample();

            if (input.DebugPressed)
            {
                debugVisualizer.enabled = !debugVisualizer.enabled;
            }
            if (input.VisualsPressed)
            {
                vehicleVisual.ToggleVisuals();
            }
            hud.SetDebug(debugVisualizer.enabled, vehicleVisual.showVisuals);

            switch (state)
            {
                case State.Countdown:
                    StepCountdown();
                    if (input.PausePressed)
                    {
                        EnterPause();
                    }
                    break;

                case State.Racing:
                    simDriver.Input = input.Drive;
                    UpdateLiveHud();
                    if (input.ResetPressed)
                    {
                        RestartRun();
                    }
                    else if (input.PausePressed)
                    {
                        EnterPause();
                    }
                    break;

                case State.Paused:
                    if (input.PausePressed)
                    {
                        ResumeFromPause();
                    }
                    break;

                case State.Finished:
                    if (input.ResetPressed)
                    {
                        RestartRun();
                    }
                    break;
            }
        }

        private void StepCountdown()
        {
            countdownRemaining -= Time.unscaledDeltaTime;
            string label;
            if (countdownRemaining > CountdownSeconds * 0.75f)
            {
                label = "3";
            }
            else if (countdownRemaining > CountdownSeconds * 0.5f)
            {
                label = "2";
            }
            else if (countdownRemaining > CountdownSeconds * 0.25f)
            {
                label = "1";
            }
            else if (countdownRemaining > 0.0f)
            {
                label = "GO!";
            }
            else
            {
                label = "";
                state = State.Racing;
                simDriver.paused = false;
            }
            hud.SetCountdown(label);
        }

        private void UpdateLiveHud()
        {
            if (simDriver.Sim == null)
            {
                return;
            }
            Race.TmnfRaceState? race = simDriver.Sim.Race();
            VehicleState stateSnapshot = simDriver.State();
            int milliseconds = race.HasValue ? (int)race.Value.elapsedTicks * 10 : 0;
            hud.SetTime(milliseconds, referenceMs, referenceMs > 0);
            hud.SetProgress(
                race.HasValue ? (int)race.Value.visitedCount : 0,
                definition.routeSpec.checkpoints.Count);
            float speed = TmnfCoordinates.ToUnity(stateSnapshot.linearSpeed).magnitude;
            hud.SetSpeed(speed);
        }

        // ------------------------------------------------------------------
        // simulation events
        // ------------------------------------------------------------------

        private void OnSimFixedStep(VehicleState state)
        {
            // Nothing per-step is required beyond the HUD refresh in Update;
            // kept as the extension point for effects/replays.
        }

        private void OnSimRaceFinished(uint finishTimeMs)
        {
            if (state != State.Racing)
            {
                return;
            }
            state = State.Finished;
            simDriver.paused = true;
            finishedTimeMs = finishTimeMs;

            int milliseconds = (int)finishTimeMs;
            bool improved = false;
            if (!string.IsNullOrEmpty(personalBestKey))
            {
                improved = PersonalBestStore.Submit(personalBestKey, milliseconds);
            }
            int personalBestAfter = improved ? milliseconds : personalBestBefore;
            hud.SetTime(milliseconds, referenceMs, referenceMs > 0);
            hud.ShowResults(
                milliseconds, personalBestBefore, personalBestAfter,
                authorTimeMs,
                definition.metadata != null ? definition.metadata.trackName : "Track");
            resultsShown = true;
            Debug.Log($"[Tmnf.Game] finished '{(definition.metadata != null ? definition.metadata.trackName : "Track")}' " +
                $"in {RaceTimeFormat.Time(milliseconds)}" +
                (improved ? " (new personal best)" : "") + ".");
        }

        // ------------------------------------------------------------------
        // pause
        // ------------------------------------------------------------------

        private void EnterPause()
        {
            if (state != State.Countdown && state != State.Racing)
            {
                return;
            }
            if (state == State.Countdown)
            {
                // The countdown resumes from where it paused.
            }
            state = State.Paused;
            simDriver.paused = true;
            hud.ShowPause();
        }

        private void ResumeFromPause()
        {
            if (state != State.Paused)
            {
                return;
            }
            hud.HidePause();
            state = countdownRemaining > 0.0f ? State.Countdown : State.Racing;
            simDriver.paused = state == State.Countdown;
        }

        // ------------------------------------------------------------------
        // navigation
        // ------------------------------------------------------------------

        private void GoToTrackSelect()
        {
            GameSession.showTrackSelect = true;
            SceneManager.LoadScene(MainMenuScene);
        }

        private void GoToMainMenu()
        {
            GameSession.showTrackSelect = false;
            SceneManager.LoadScene(MainMenuScene);
        }

        private void Fail(string message)
        {
            state = State.Failed;
            BuildHudOnce();
            hud.ShowErrors(new List<string> { message });
            Debug.LogError("[Tmnf.Game] " + message);
        }

        // ------------------------------------------------------------------
        // helpers
        // ------------------------------------------------------------------

        private void BuildHudOnce()
        {
            if (hudBuilt)
            {
                return;
            }
            hudBuilt = true;
            string name = definition != null && definition.metadata != null
                ? definition.metadata.trackName
                : GameSession.selectedTrackName;
            string author = definition != null && definition.metadata != null
                ? definition.metadata.author
                : "Unknown";
            int authorTime = definition != null && definition.metadata != null
                ? definition.metadata.AuthorTimeOrNone
                : -1;
            hud.Build(name, author, authorTime, -1);
        }

        /// <summary>
        /// Disables cameras and audio listeners the track scene may carry so
        /// the race scene's own camera stays the one rendering.
        /// </summary>
        private static void DisableSceneCameras(Scene scene)
        {
            if (!scene.IsValid() || !scene.isLoaded)
            {
                return;
            }
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Camera camera in root.GetComponentsInChildren<Camera>(true))
                {
                    camera.enabled = false;
                }
                foreach (AudioListener listener in root.GetComponentsInChildren<AudioListener>(true))
                {
                    listener.enabled = false;
                }
            }
        }

        private static void ApplyDefaultMaterial(Scene scene)
        {
            Material material = null;
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (MeshRenderer renderer in root.GetComponentsInChildren<MeshRenderer>(true))
                {
                    if (renderer.sharedMaterial == null || renderer.sharedMaterial.name == "Default-Material")
                    {
                        if (material == null)
                        {
                            material = GameMaterials.Lit(new Color(0.62f, 0.64f, 0.68f));
                            material.name = "TrackDefault";
                        }
                        renderer.sharedMaterial = material;
                    }
                }
            }
        }

        /// <summary>uGUI buttons need an EventSystem; create one on demand.</summary>
        public static void EnsureEventSystem()
        {
            UiBuilder.EnsureEventSystem();
        }
    }
}
